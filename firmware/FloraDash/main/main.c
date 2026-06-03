#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "esp_log.h"

// Drivers ADC nouveaux (ESP-IDF v5+)
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

// Driver de l'ecran OLED (notre composant local)
#include "ssd1306.h"

// Bus I2C + driver du capteur d'humidite MBR3
#include "driver/i2c_master.h"
#include "soil_sensor.h"

static void oled_rotate_180(SSD1306_t *dev);

#define LED_ORANGE GPIO_NUM_4

static const char *TAG = "FLORADASH";

// Multi-echantillonnage ADC
#define NB_SAMPLES        16
// Rafraichissement de l'affichage
#define REFRESH_PERIOD_MS 100

// ════════════════════════════════════════════════════════
// ADRESSES I2C DES PERIPHERIQUES FLORADASH (7 bits)
// ════════════════════════════════════════════════════════
#define I2C_ADDR_OLED       0x3C  // Ecran SSD1306
#define I2C_ADDR_SOIL       0x37  // Capteur d'humidite (CY8CMBR3102)
#define I2C_ADDR_FUELGAUGE  0x36  // Jauge batterie (absente si pas de batterie)

void app_main(void)
{
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);

    ESP_LOGI(TAG, "=== Demarrage FloraDash ===");

    // ── LED orange ─────────────────────────────────────────
    gpio_reset_pin(LED_ORANGE);
    gpio_set_direction(LED_ORANGE, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_ORANGE, 0);

    // ── OLED : c'est lui qui cree le bus I2C_NUM_0 ─────────
    SSD1306_t dev;
    i2c_master_init(&dev, CONFIG_SDA_GPIO, CONFIG_SCL_GPIO, CONFIG_RESET_GPIO);
    ssd1306_init(&dev, 128, 64);
    oled_rotate_180(&dev);  
    ssd1306_clear_screen(&dev, false);
    ssd1306_contrast(&dev, 0xff);
    ssd1306_display_text(&dev, 0, "FloraDash", 9, false);

    // ── Recuperation du bus partage cree par l'OLED ────────
    // On ne recree PAS de bus : on demande le handle deja
    // existant sur I2C_NUM_0 et on le partage avec le MBR3.
    i2c_master_bus_handle_t bus = NULL;
    esp_err_t err = i2c_master_get_bus_handle(I2C_NUM_0, &bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Impossible de recuperer le bus I2C : %s",
                 esp_err_to_name(err));
    }

    // ── Init + validation du capteur d'humidite du sol ───────
    bool soil_ok = false;
    if (bus != NULL && soil_sensor_begin(bus) == ESP_OK) {
        uint16_t id = 0;
        if (soil_sensor_check_id(&id) == ESP_OK) {
            soil_ok = true;   // communication validee (DEVICE_ID = 0x0A01)
        }
    }

    // ── ADC oneshot (test potentiometre, inchange) ─────────
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc1_handle));

    adc_oneshot_chan_cfg_t chan_config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_0, &chan_config));
    ESP_LOGI(TAG, "ADC initialise : ADC1 canal 0");

    adc_cali_handle_t cali_handle = NULL;
    bool cali_ok = false;
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id  = ADC_UNIT_1,
        .atten    = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_curve_fitting(&cali_config, &cali_handle) == ESP_OK) {
        cali_ok = true;
        ESP_LOGI(TAG, "Calibration : curve fitting active");
    } else {
        ESP_LOGW(TAG, "Calibration indisponible, estimation utilisee");
    }

    printf("\n=== FloraDash - Test ESP-IDF ===\n");
    printf("Chip       : %s\n",
           (chip_info.model == CHIP_ESP32C3) ? "ESP32-C3" : "Autre");
    printf("Heap libre : %lu octets\n", esp_get_free_heap_size());
    printf("Soil sensor: %s\n", soil_ok ? "OK" : "absent");
    printf("================================\n\n");

    // ════════════════════════════════════════════════════════
    // BOUCLE PRINCIPALE
    // ════════════════════════════════════════════════════════
    char line_buf[32];
    while (1) {
        // ── Lecture ADC moyennee (potentiometre) ───────────
        int raw_sum = 0;
        for (int i = 0; i < NB_SAMPLES; i++) {
            int raw;
            if (adc_oneshot_read(adc1_handle, ADC_CHANNEL_0, &raw) != ESP_OK) {
                continue;
            }
            raw_sum += raw;
        }
        int raw_avg = raw_sum / NB_SAMPLES;

        int voltage_mv;
        if (cali_ok) {
            adc_cali_raw_to_voltage(cali_handle, raw_avg, &voltage_mv);
        } else {
            voltage_mv = (raw_avg * 3300) / 4095;
        }

        snprintf(line_buf, sizeof(line_buf), "Volt: %d.%02d V",
                 voltage_mv / 1000, (voltage_mv % 1000) / 10);
        ssd1306_display_text(&dev, 3, line_buf, strlen(line_buf), false);

        // ── Lecture capteur d'humidite MBR3 (capacite pF) ──
        // On lit la CAPACITE en pF et on la convertit en %.
        if (soil_ok) {
            uint8_t soil_pf = 0;
            esp_err_t serr = soil_sensor_read_pf(&soil_pf);
            if (serr == ESP_OK) {
                int hum = soil_sensor_pf_to_percent(soil_pf);

                snprintf(line_buf, sizeof(line_buf), "Soil: %3u pF", soil_pf);
                ssd1306_display_text(&dev, 5, line_buf, strlen(line_buf), false);

                snprintf(line_buf, sizeof(line_buf), "Humid: %3d %%", hum);
                ssd1306_display_text(&dev, 6, line_buf, strlen(line_buf), false);

                // Barre de progression (16 caracteres)
                char bar[17];
                int filled = (hum * 16) / 100;
                for (int i = 0; i < 16; i++) bar[i] = (i < filled) ? '#' : '.';
                bar[16] = '\0';
                ssd1306_display_text(&dev, 7, bar, 16, true);

                ESP_LOGI(TAG, "Soil Cp = %u pF  ->  %d %%", soil_pf, hum);
            } else {
                ESP_LOGW(TAG, "Lecture Cp echouee : %s", esp_err_to_name(serr));
            }
        }

        vTaskDelay(pdMS_TO_TICKS(REFRESH_PERIOD_MS));
    }
}

// Rotation 180 materielle propre (pas de miroir) : inverse
// segment remap (A0) ET COM scan (C0) au niveau du controleur.
static void oled_rotate_180(SSD1306_t *dev)
{
    uint8_t cmd[3] = {
        OLED_CONTROL_BYTE_CMD_STREAM,  // 0x00
        OLED_CMD_SET_SEGMENT_REMAP_0,  // 0xA0 : miroir X
        0xC0                            // COM scan normal : miroir Y
    };
    i2c_master_transmit(dev->_i2c_dev_handle, cmd, sizeof(cmd), 100);
}