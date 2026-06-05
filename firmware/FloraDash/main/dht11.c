#include "dht11.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"   // esp_rom_delay_us
#include "esp_timer.h"     // esp_timer_get_time
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ════════════════════════════════════════════════════════
// TIMINGS DHT11 (confirmes datasheet Aosong / OSEPP)
// ════════════════════════════════════════════════════════
// Start    : ligne basse >= 18 ms, puis MCU relache et attend
//            20-40 us la reponse du capteur.
// Reponse  : capteur tire bas ~80 us puis haut ~80 us.
// Bit      : 50 us bas, puis haut court 26-28 us (=0) ou 70 us (=1).
// On distingue 0/1 avec un seuil a 40 us (entre 28 et 70).
#define DHT_START_LOW_MS      20    // >= 18 ms exige par la datasheet
#define DHT_START_RELEASE_US  30    // dans la fenetre 20-40 us
#define DHT_RESP_TIMEOUT_US   100   // marge sur les phases ~80 us
#define DHT_BIT_LOW_TIMEOUT   70    // marge sur les 50 us bas
#define DHT_BIT_HIGH_TIMEOUT  90    // marge sur le haut (max 70 us)
#define DHT_BIT_THRESHOLD_US  40    // > seuil => bit a 1

#define DHT_DATA_BYTES        5     // 40 bits : RH int/dec, T int/dec, checksum

static int s_gpio = -1;

esp_err_t dht11_init(int gpio_num)
{
    s_gpio = gpio_num;
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << gpio_num,
        .mode         = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
    };
    return gpio_config(&cfg);
}

// Attend que la ligne atteigne 'level' ; renvoie la duree ecoulee
// en us, ou -1 si le timeout est atteint avant.
static int wait_level(int level, int timeout_us)
{
    int64_t start = esp_timer_get_time();
    while (gpio_get_level(s_gpio) != level) {
        if (esp_timer_get_time() - start > timeout_us) {
            return -1;
        }
    }
    return (int)(esp_timer_get_time() - start);
}

esp_err_t dht11_read(dht11_reading_t *out)
{
    if (out == NULL) return ESP_ERR_INVALID_ARG;

    uint8_t data[DHT_DATA_BYTES] = {0};

    // ── Signal de start ────────────────────────────────────
    gpio_set_level(s_gpio, 0);
    vTaskDelay(pdMS_TO_TICKS(DHT_START_LOW_MS));
    gpio_set_level(s_gpio, 1);
    esp_rom_delay_us(DHT_START_RELEASE_US);

    // ── Section critique : timing serre en us ──────────────
    // RESERVE A VERIFIER (ESP-IDF v6.0.1, ESP32-C3) : portDISABLE/
    // ENABLE_INTERRUPTS est-il la bonne primitive ici, ou faut-il
    // portENTER_CRITICAL/portEXIT_CRITICAL avec spinlock ? A valider
    // contre la doc avant de considerer ce driver comme fiable.
    portDISABLE_INTERRUPTS();

    // Reponse du capteur : ~80 us bas puis ~80 us haut
    if (wait_level(0, DHT_RESP_TIMEOUT_US) < 0) goto fail;
    if (wait_level(1, DHT_RESP_TIMEOUT_US) < 0) goto fail;
    if (wait_level(0, DHT_RESP_TIMEOUT_US) < 0) goto fail;

    // 40 bits : chaque bit = 50 us bas, puis haut dont la duree code 0/1
    for (int i = 0; i < 40; i++) {
        if (wait_level(1, DHT_BIT_LOW_TIMEOUT) < 0) goto fail;
        int high = wait_level(0, DHT_BIT_HIGH_TIMEOUT);
        if (high < 0) goto fail;
        if (high > DHT_BIT_THRESHOLD_US) {
            data[i / 8] |= (1 << (7 - (i % 8)));
        }
    }

    portENABLE_INTERRUPTS();

    // ── Checksum : 8 bits de poids faible de la somme des 4 octets ──
    if ((uint8_t)(data[0] + data[1] + data[2] + data[3]) != data[4]) {
        return ESP_ERR_INVALID_CRC;
    }

    // DHT11 : les octets decimaux (data[1], data[3]) valent toujours 0
    out->humidity    = (float)data[0];
    out->temperature = (float)data[2];
    return ESP_OK;

fail:
    portENABLE_INTERRUPTS();
    return ESP_ERR_TIMEOUT;
}