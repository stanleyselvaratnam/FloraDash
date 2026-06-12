#include "dht11.h"
#include "driver/gpio.h"
#include "driver/rmt_rx.h"
#include "esp_rom_sys.h"   // esp_rom_delay_us
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h" 

// ════════════════════════════════════════════════════════
// TIMINGS DHT11 (confirmes datasheet Aosong / OSEPP)
// ════════════════════════════════════════════════════════
// Start    : ligne basse >= 18 ms, puis MCU relache.
// Reponse  : capteur tire bas ~80 us puis haut ~80 us.
// Bit      : 50 us bas, puis haut court 26-28 us (=0) ou 70 us (=1).
// On distingue 0/1 avec un seuil a 50 us (entre 28 et 70).
#define DHT_START_LOW_MS        20    // >= 18 ms exige par la datasheet
#define DHT_BIT_THRESHOLD_US    50    // duree du HAUT > seuil => bit a 1

#define DHT_DATA_BYTES          5     // 40 bits : RH int/dec, T int/dec, checksum

// ── Parametres RMT ──────────────────────────────────────
// Resolution 1 MHz : 1 tick = 1 us. Suffisant pour distinguer
// 28 us (bit 0) de 70 us (bit 1) avec une large marge.
#define DHT_RMT_RESOLUTION_HZ   1000000

// Nombre de symboles a capturer. Le capteur envoie : 1 reponse de
// presence + 40 bits. Chaque "symbole" RMT encode 2 fronts (un niveau
// haut + un niveau bas). On prevoit large : ~48 symboles couvrent la
// reponse + les 40 bits avec marge.
#define DHT_RMT_SYMBOLS         64

// Filtre de glitch : un front plus court que 1 us est du bruit.
#define DHT_SIGNAL_MIN_NS       1000        // 1 us
// Seuil de fin de trame : si la ligne reste stable plus longtemps que
// ca, le RMT considere la trame finie. Apres le dernier bit, le DHT11
// relache la ligne (repos haut) : ce repos long > 200 us declenche la
// fin de reception. On met 1 ms pour etre tranquille.
#define DHT_SIGNAL_MAX_NS       1000000     // 1 ms

static int                  s_gpio = -1;
static rmt_channel_handle_t s_rx_chan = NULL;
static QueueHandle_t        s_recv_queue = NULL;
static rmt_symbol_word_t    s_symbols[DHT_RMT_SYMBOLS];

// ════════════════════════════════════════════════════════
// CALLBACK — contexte ISR. Ne fait que transferer le resultat
// vers la tache via une queue. Aucun decodage ici.
// ════════════════════════════════════════════════════════
static bool IRAM_ATTR rmt_rx_done_cb(rmt_channel_handle_t chan,
                                     const rmt_rx_done_event_data_t *edata,
                                     void *user_ctx)
{
    BaseType_t high_task_wakeup = pdFALSE;
    QueueHandle_t q = (QueueHandle_t)user_ctx;
    // On copie la structure d'evenement (pointeur vers symboles + nombre).
    xQueueSendFromISR(q, edata, &high_task_wakeup);
    return high_task_wakeup == pdTRUE;
}

// ════════════════════════════════════════════════════════
// INIT — cree le canal RX, enregistre le callback, arme le canal.
// ════════════════════════════════════════════════════════
esp_err_t dht11_init(int gpio_num)
{
    s_gpio = gpio_num;

    // La ligne data est en open-drain avec pull-up : au repos elle est
    // haute (tiree par le pull-up). Le capteur et le MCU ne font que
    // tirer bas. On configure la GPIO en open-drain pour generer le start,
    // mais le RMT lira la meme broche.
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << gpio_num,
        .mode         = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
    };
    esp_err_t err = gpio_config(&cfg);
    if (err != ESP_OK) return err;

    rmt_rx_channel_config_t rx_cfg = {
        .gpio_num       = gpio_num,
        .clk_src        = RMT_CLK_SRC_DEFAULT,
        .resolution_hz  = DHT_RMT_RESOLUTION_HZ,
        .mem_block_symbols = DHT_RMT_SYMBOLS,
    };
    err = rmt_new_rx_channel(&rx_cfg, &s_rx_chan);
    if (err != ESP_OK) return err;

    s_recv_queue = xQueueCreate(1, sizeof(rmt_rx_done_event_data_t));
    if (s_recv_queue == NULL) return ESP_ERR_NO_MEM;

    rmt_rx_event_callbacks_t cbs = {
        .on_recv_done = rmt_rx_done_cb,
    };
    err = rmt_rx_register_event_callbacks(s_rx_chan, &cbs, s_recv_queue);
    if (err != ESP_OK) return err;

    return rmt_enable(s_rx_chan);
}

// ════════════════════════════════════════════════════════
// DECODAGE — purement algorithmique, AUCUNE contrainte temps reel.
// On recoit un tableau de symboles ; pour chaque bit on regarde la
// duree du niveau HAUT et on compare au seuil.
// ════════════════════════════════════════════════════════
static esp_err_t decode_frame(const rmt_symbol_word_t *sym, size_t num,
                              dht11_reading_t *out)
{

    // Le premier symbole est la reponse de presence du capteur
    // (~80 us bas + ~80 us haut) : on le saute. Restent 40 symboles
    // de donnees, un par bit.
    if (num < 41) {
        return ESP_ERR_INVALID_SIZE;   // trame incomplete
    }

    uint8_t data[DHT_DATA_BYTES] = {0};

    // sym[0] = presence. Les bits utiles commencent a sym[1].
    for (int i = 0; i < 40; i++) {
        const rmt_symbol_word_t *s = &sym[i + 1];
        // Pour le DHT11 : chaque bit = un creux (50 us bas) suivi d'un
        // pic haut dont la DUREE code la valeur. Dans un symbole RMT,
        // level0/duration0 est le 1er front, level1/duration1 le 2nd.
        // Le niveau HAUT est celui dont level == 1.
        uint32_t high_us = (s->level0 == 1) ? s->duration0 : s->duration1;

        if (high_us > DHT_BIT_THRESHOLD_US) {
            data[i / 8] |= (1 << (7 - (i % 8)));
        }
    }

    // Checksum : 8 bits faibles de la somme des 4 premiers octets.
    if ((uint8_t)(data[0] + data[1] + data[2] + data[3]) != data[4]) {
        return ESP_ERR_INVALID_CRC;
    }

    out->humidity    = (float)data[0];
    out->temperature = (float)data[2];
    return ESP_OK;
}

// ════════════════════════════════════════════════════════
// LECTURE — genere le start, lance la capture RMT, DORT en attendant
// le callback. Plus aucune section critique, plus de blocage CPU.
// ════════════════════════════════════════════════════════
esp_err_t dht11_read(dht11_reading_t *out)
{
    if (out == NULL) return ESP_ERR_INVALID_ARG;
    if (s_rx_chan == NULL) return ESP_ERR_INVALID_STATE;

    // ── 1. Reprendre la broche en sortie GPIO pour generer le start ──
    // Le canal RX a route la broche vers son entree ; on la reconfigure
    // explicitement en sortie open-drain pour piloter la ligne nous-memes.
    gpio_set_direction(s_gpio, GPIO_MODE_OUTPUT_OD);
    gpio_set_level(s_gpio, 0);
    vTaskDelay(pdMS_TO_TICKS(DHT_START_LOW_MS));   // pull-down >= 18 ms
    gpio_set_level(s_gpio, 1);                      // relache

    // ── 2. Rendre la broche au RMT et armer immediatement ──
    // On re-route la broche vers l'entree RMT. Le capteur va repondre
    // dans 20-40 us : on doit etre arme avant.
    gpio_set_direction(s_gpio, GPIO_MODE_INPUT);

    rmt_receive_config_t rx_config = {
        .signal_range_min_ns = DHT_SIGNAL_MIN_NS,
        .signal_range_max_ns = DHT_SIGNAL_MAX_NS,
    };
    esp_err_t err = rmt_receive(s_rx_chan, s_symbols, sizeof(s_symbols), &rx_config);
    if (err != ESP_OK) return err;

    // ── 3. Dormir jusqu'au callback ──
    rmt_rx_done_event_data_t rx_data;
    if (xQueueReceive(s_recv_queue, &rx_data, pdMS_TO_TICKS(200)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    return decode_frame(rx_data.received_symbols, rx_data.num_symbols, out);
}