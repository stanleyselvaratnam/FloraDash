#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <limits.h>

// Drivers ADC nouveaux (ESP-IDF v5+)
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

// Driver de l'ecran OLED (notre composant local)
#include "ssd1306.h"

// Bus I2C + drivers capteurs
#include "driver/i2c_master.h"
#include "soil_sensor.h"
#include "dht11.h"

// ════════════════════════════════════════════════════════
// CONFIGURATION
// ════════════════════════════════════════════════════════
static const char *TAG = "FLORADASH";

#define LED_ORANGE        GPIO_NUM_4
#define DHT_GPIO          GPIO_NUM_3

#define NB_SAMPLES        16        // Multi-echantillonnage ADC

// Cadences des timers (en microsecondes)
#define ADC_PERIOD_US     250000    // 250 ms (4Hz)
#define SOIL_PERIOD_US    100000    // 100 ms (10Hz)
#define DHT_PERIOD_US     1500000   // 1.5 s : datasheet DHT11 = periode >= 1 s

// Bits de notification : un par capteur. La boucle se reveille quand un
// timer signale, et lit les bits pour savoir quelle(s) tache(s) executer.
#define NOTIFY_ADC        (1 << 0)
#define NOTIFY_SOIL       (1 << 1)
#define NOTIFY_DHT        (1 << 2)

// Adresses I2C des peripheriques (7 bits) — reference de cablage
#define I2C_ADDR_OLED       0x3C    // Ecran SSD1306
#define I2C_ADDR_SOIL       0x37    // Capteur d'humidite (CY8CMBR3102)
#define I2C_ADDR_FUELGAUGE  0x36    // Jauge batterie (absente si pas de batterie)

// Configuration UNIQUE du CY8CMBR3102 : decommenter pour flasher une fois,
// verifier "CRC accepte" dans les logs, PUIS recommenter et reflasher.
// #define FLORADASH_CONFIGURE_SOIL_ONCE

// Lignes OLED utilisees (evite les collisions d'affichage)
#define OLED_LINE_TITLE   0
#define OLED_LINE_VOLT    3
#define OLED_LINE_DHT     4
#define OLED_LINE_SOIL    5
#define OLED_LINE_HUMID   6
#define OLED_LINE_BAR     7

// ════════════════════════════════════════════════════════
// CONTEXTE : tout l'etat vivant du systeme en un seul endroit
// ════════════════════════════════════════════════════════
typedef struct {
    SSD1306_t                 oled;
    adc_oneshot_unit_handle_t adc;
    adc_cali_handle_t         cali;
    bool                      cali_ok;
    bool                      soil_ok;

    // Etat DHT11 : derniere lecture valide affichee a chaque rafraichissement
    dht11_reading_t dht;
    bool            dht_valid;
} floradash_t;

// Tache principale a reveiller depuis les callbacks timer.
static TaskHandle_t s_main_task;

// ════════════════════════════════════════════════════════
// PROTOTYPES
// ════════════════════════════════════════════════════════
static void led_init(void);
static void oled_init(floradash_t *fd);
static void oled_rotate_180(SSD1306_t *dev);
static void soil_init(floradash_t *fd);
static void adc_init(floradash_t *fd);
static void timers_init(void);
static void print_banner(const floradash_t *fd);

static void task_read_adc(floradash_t *fd, char *buf, size_t buflen);
static void task_read_soil(floradash_t *fd, char *buf, size_t buflen);
static void task_read_dht(floradash_t *fd, char *buf, size_t buflen);

// ════════════════════════════════════════════════════════
// CALLBACKS TIMER — signalent uniquement, aucun travail
// Contexte = tache esp_timer (dispatch par defaut, pas ISR),
// donc xTaskNotify simple suffit (pas de variante FromISR).
// ════════════════════════════════════════════════════════
static void cb_adc(void *arg)  { xTaskNotify(s_main_task, NOTIFY_ADC,  eSetBits); }
static void cb_soil(void *arg) { xTaskNotify(s_main_task, NOTIFY_SOIL, eSetBits); }
static void cb_dht(void *arg)  { xTaskNotify(s_main_task, NOTIFY_DHT,  eSetBits); }

// ════════════════════════════════════════════════════════
// POINT D'ENTREE — se lit comme un sommaire
// ════════════════════════════════════════════════════════
void app_main(void)
{
    static floradash_t fd = {0};
    char line_buf[32];

    ESP_LOGI(TAG, "=== Demarrage FloraDash ===");

    led_init();
    oled_init(&fd);
    soil_init(&fd);
    adc_init(&fd);

    dht11_init(DHT_GPIO);
    vTaskDelay(pdMS_TO_TICKS(1000));   // stabilisation DHT11 au boot (datasheet)

    print_banner(&fd);

    // On memorise la tache courante : les timers la reveilleront.
    s_main_task = xTaskGetCurrentTaskHandle();
    timers_init();

    while (1) {
        uint32_t bits;
        // Dort sans consommer de CPU jusqu'a ce qu'un timer signale.
        // Le 0 : n'efface rien en entrant. ULONG_MAX : efface tout en sortant.
        xTaskNotifyWait(0, ULONG_MAX, &bits, portMAX_DELAY);

        if (bits & NOTIFY_ADC)  task_read_adc(&fd,  line_buf, sizeof(line_buf));
        if (bits & NOTIFY_SOIL) task_read_soil(&fd, line_buf, sizeof(line_buf));
        if (bits & NOTIFY_DHT)  task_read_dht(&fd,  line_buf, sizeof(line_buf));
    }
}

// ════════════════════════════════════════════════════════
// INITIALISATION
// ════════════════════════════════════════════════════════
static void led_init(void)
{
    gpio_reset_pin(LED_ORANGE);
    gpio_set_direction(LED_ORANGE, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_ORANGE, 0);
}

// L'OLED cree le bus I2C_NUM_0 ; les autres peripheriques le partagent.
static void oled_init(floradash_t *fd)
{
    i2c_master_init(&fd->oled, CONFIG_SDA_GPIO, CONFIG_SCL_GPIO, CONFIG_RESET_GPIO);
    ssd1306_init(&fd->oled, 128, 64);
    oled_rotate_180(&fd->oled);
    ssd1306_clear_screen(&fd->oled, false);
    ssd1306_contrast(&fd->oled, 0xff);
    ssd1306_display_text(&fd->oled, OLED_LINE_TITLE, "FloraDash", 9, false);
}

// Rotation 180 materielle propre (pas de miroir) : inverse segment remap (A0)
// ET COM scan (C0) au niveau du controleur.
static void oled_rotate_180(SSD1306_t *dev)
{
    uint8_t cmd[3] = {
        OLED_CONTROL_BYTE_CMD_STREAM,  // 0x00
        OLED_CMD_SET_SEGMENT_REMAP_0,  // 0xA0 : miroir X
        0xC0                            // COM scan inverse : miroir Y
    };
    i2c_master_transmit(dev->_i2c_dev_handle, cmd, sizeof(cmd), 100);
}

// Recupere le bus partage cree par l'OLED, initialise et valide le capteur de sol.
static void soil_init(floradash_t *fd)
{
    fd->soil_ok = false;

    i2c_master_bus_handle_t bus = NULL;
    if (i2c_master_get_bus_handle(I2C_NUM_0, &bus) != ESP_OK || bus == NULL) {
        ESP_LOGE(TAG, "Impossible de recuperer le bus I2C");
        return;
    }

    if (soil_sensor_begin(bus) == ESP_OK) {
        uint16_t id = 0;
        if (soil_sensor_check_id(&id) == ESP_OK) {
            fd->soil_ok = true;   // communication validee (DEVICE_ID = 0x0A01)
        }
    }

#ifdef FLORADASH_CONFIGURE_SOIL_ONCE
    if (fd->soil_ok) {
        if (soil_sensor_apply_ezclick_config() == ESP_OK) {
            ESP_LOGI(TAG, "Capteur configure. Recommente le define maintenant.");
        } else {
            ESP_LOGE(TAG, "Echec de la configuration du capteur !");
        }
    } else {
        ESP_LOGW(TAG, "Capteur absent : config sautee.");
    }
#endif
}

static void adc_init(floradash_t *fd)
{
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &fd->adc));

    adc_oneshot_chan_cfg_t chan_config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten    = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(fd->adc, ADC_CHANNEL_0, &chan_config));
    ESP_LOGI(TAG, "ADC initialise : ADC1 canal 0");

    fd->cali_ok = false;
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id  = ADC_UNIT_1,
        .atten    = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_curve_fitting(&cali_config, &fd->cali) == ESP_OK) {
        fd->cali_ok = true;
        ESP_LOGI(TAG, "Calibration : curve fitting active");
    } else {
        ESP_LOGW(TAG, "Calibration indisponible, estimation utilisee");
    }
}

// Cree un timer periodique unique a partir d'un callback et d'une cadence.
// ESP_ERROR_CHECK acceptable ici : un echec au boot = bug de config.
static void start_timer(esp_timer_cb_t cb, const char *name, uint64_t period_us)
{
    const esp_timer_create_args_t args = {
        .callback = cb,
        .name     = name,
        .dispatch_method = ESP_TIMER_TASK,   // defaut : execute dans une tache
    };
    esp_timer_handle_t h;
    ESP_ERROR_CHECK(esp_timer_create(&args, &h));
    ESP_ERROR_CHECK(esp_timer_start_periodic(h, period_us));
}

static void timers_init(void)
{
    start_timer(cb_adc,  "adc",  ADC_PERIOD_US);
    start_timer(cb_soil, "soil", SOIL_PERIOD_US);
    start_timer(cb_dht,  "dht",  DHT_PERIOD_US);
    ESP_LOGI(TAG, "Timers demarres (adc/soil/dht)");
}

static void print_banner(const floradash_t *fd)
{
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);

    printf("\n=== FloraDash - Test ESP-IDF ===\n");
    printf("Chip       : %s\n",
           (chip_info.model == CHIP_ESP32C3) ? "ESP32-C3" : "Autre");
    printf("Heap libre : %lu octets\n", esp_get_free_heap_size());
    printf("Soil sensor: %s\n", fd->soil_ok ? "OK" : "absent");
    printf("================================\n\n");
}

// ════════════════════════════════════════════════════════
// TACHES DE LA BOUCLE (un bloc = une fonction)
// ════════════════════════════════════════════════════════

// Lecture ADC moyennee (potentiometre de test) + affichage tension.
static void task_read_adc(floradash_t *fd, char *buf, size_t buflen)
{
    int raw_sum = 0;
    for (int i = 0; i < NB_SAMPLES; i++) {
        int raw;
        if (adc_oneshot_read(fd->adc, ADC_CHANNEL_0, &raw) == ESP_OK) {
            raw_sum += raw;
        }
    }
    int raw_avg = raw_sum / NB_SAMPLES;

    int voltage_mv;
    if (fd->cali_ok) {
        adc_cali_raw_to_voltage(fd->cali, raw_avg, &voltage_mv);
    } else {
        voltage_mv = (raw_avg * 3300) / 4095;
    }

    snprintf(buf, buflen, "Volt: %d.%02d V",
             voltage_mv / 1000, (voltage_mv % 1000) / 10);
    ssd1306_display_text(&fd->oled, OLED_LINE_VOLT, buf, strlen(buf), false);
}

// Lecture capteur de sol (capacite pF) -> conversion en % + barre de progression.
static void task_read_soil(floradash_t *fd, char *buf, size_t buflen)
{
    if (!fd->soil_ok) {
        return;
    }

    uint8_t soil_pf = 0;
    esp_err_t serr = soil_sensor_read_pf(&soil_pf);
    if (serr != ESP_OK) {
        ESP_LOGW(TAG, "Lecture Cp echouee : %s", esp_err_to_name(serr));
        return;
    }

    int hum = soil_sensor_pf_to_percent(soil_pf);

    snprintf(buf, buflen, "Soil: %3u pF", soil_pf);
    ssd1306_display_text(&fd->oled, OLED_LINE_SOIL, buf, strlen(buf), false);

    snprintf(buf, buflen, "Humid: %3d %%", hum);
    ssd1306_display_text(&fd->oled, OLED_LINE_HUMID, buf, strlen(buf), false);

    // Barre de progression (16 caracteres)
    char bar[17];
    int filled = (hum * 16) / 100;
    for (int i = 0; i < 16; i++) {
        bar[i] = (i < filled) ? '#' : '.';
    }
    bar[16] = '\0';
    ssd1306_display_text(&fd->oled, OLED_LINE_BAR, bar, 16, true);

    ESP_LOGI(TAG, "Soil Cp = %u pF  ->  %d %%", soil_pf, hum);
}

// Lecture DHT11 : la cadence (>= 1 s) est portee par le timer dedie,
// plus besoin de comparer les timestamps ici.
// NOTE : dht11_read() bloque ~5 ms (section critique bit-bang). Ce blocage
// retarde les autres reveils tant que le driver n'est pas porte sur RMT.
static void task_read_dht(floradash_t *fd, char *buf, size_t buflen)
{
    if (dht11_read(&fd->dht) == ESP_OK) {
        fd->dht_valid = true;
        ESP_LOGI("DHT11", "T=%.0f C  RH=%.0f %%",
                 fd->dht.temperature, fd->dht.humidity);
    } else {
        ESP_LOGW("DHT11", "Lecture DHT11 echouee");
    }

    if (fd->dht_valid) {
        snprintf(buf, buflen, "T:%2.0fC  H:%2.0f%%",
                 fd->dht.temperature, fd->dht.humidity);
        ssd1306_display_text(&fd->oled, OLED_LINE_DHT, buf, strlen(buf), false);
    }
}