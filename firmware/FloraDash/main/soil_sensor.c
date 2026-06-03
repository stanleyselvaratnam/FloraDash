#include "soil_sensor.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "SOIL";

// ════════════════════════════════════════════════════════
// REGISTRES CY8CMBR3102 (lib SparkFun / TRM Infineon)
// ════════════════════════════════════════════════════════
#define REG_SENSOR_ID         0x82  // selection du capteur a debugguer
#define REG_FAMILY_ID         0x8F  // 1 octet  -> 0x9A
#define REG_DEVICE_ID         0x90  // 2 octets LE -> 0x0A01
#define REG_SYNC_COUNTER0     0xB9
#define REG_SYNC_COUNTER1     0xDB
#define REG_DEBUG_SENSOR_ID   0xDC  // echo du SENSOR_ID applique
#define REG_DEBUG_CP          0xDD  // capacite en pF (1 octet) <-- humidite

#define SID_0                 0x00
#define SID_1                 0x01

#define SOIL_TIMEOUT_MS       100
#define SOIL_SYNC_RETRIES     5     // relectures pour coherence sync
#define SOIL_WAKE_RETRIES     10    // reessais reveil (NACK au boot)

static i2c_master_dev_handle_t s_dev = NULL;

// ── E/S registre de base ───────────────────────────────────
static esp_err_t read_reg(uint8_t reg, uint8_t *buf, size_t len)
{
    if (s_dev == NULL) return ESP_ERR_INVALID_STATE;
    return i2c_master_transmit_receive(s_dev, &reg, 1, buf, len, SOIL_TIMEOUT_MS);
}

static esp_err_t read_u8(uint8_t reg, uint8_t *val)
{
    return read_reg(reg, val, 1);
}

static esp_err_t write_u8(uint8_t reg, uint8_t val)
{
    if (s_dev == NULL) return ESP_ERR_INVALID_STATE;
    uint8_t payload[2] = { reg, val };
    return i2c_master_transmit(s_dev, payload, sizeof(payload), SOIL_TIMEOUT_MS);
}

// ── Lecture coherente d'un octet (entre SYNC0 et SYNC1) ────
// La donnee de debug n'est valide que si SYNC0 == SYNC1, sinon
// elle a change pendant la lecture. C'est la methode SparkFun.
static esp_err_t read_synced_u8(uint8_t reg, uint8_t *val)
{
    for (int i = 0; i < SOIL_SYNC_RETRIES; i++) {
        uint8_t s0 = 0, s1 = 0, data = 0;
        if (read_u8(REG_SYNC_COUNTER0, &s0)   != ESP_OK) return ESP_FAIL;
        if (read_u8(reg, &data)               != ESP_OK) return ESP_FAIL;
        if (read_u8(REG_SYNC_COUNTER1, &s1)   != ESP_OK) return ESP_FAIL;
        if (s0 == s1) { *val = data; return ESP_OK; }
    }
    return ESP_ERR_INVALID_STATE;
}

// ── Selection d'un capteur, avec confirmation (methode SparkFun)
// On ecrit SENSOR_ID puis on attend que DEBUG_SENSOR_ID (lu en
// synchro) confirme la valeur. C'est ce qui garantit qu'on lira
// le bon capteur, et ce qui evite l'alternance de valeurs.
static esp_err_t set_sensor_id(uint8_t sid)
{
    if (write_u8(REG_SENSOR_ID, sid) != ESP_OK) return ESP_FAIL;

    for (int i = 0; i < 20; i++) {
        uint8_t echo = 0xFF;
        if (read_synced_u8(REG_DEBUG_SENSOR_ID, &echo) == ESP_OK && echo == sid) {
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    return ESP_ERR_TIMEOUT;
}

esp_err_t soil_sensor_begin(i2c_master_bus_handle_t bus)
{
    if (bus == NULL) return ESP_ERR_INVALID_ARG;

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = SOIL_I2C_ADDR,
        .scl_speed_hz    = 100000,
    };
    esp_err_t err = i2c_master_bus_add_device(bus, &dev_cfg, &s_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Ajout device echoue : %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "Capteur ajoute au bus a 0x%02X", SOIL_I2C_ADDR);

    // Valide la communication (reveille la puce au passage).
    // On NE reconfigure PAS : la config d'usine SparkFun marche.
    return soil_sensor_check_id(NULL);
}

esp_err_t soil_sensor_check_id(uint16_t *id_out)
{
    uint8_t raw[2] = {0};
    esp_err_t err = ESP_FAIL;

    // La puce NACK la 1ere transaction si elle dort : on reessaye.
    for (int i = 0; i < SOIL_WAKE_RETRIES; i++) {
        err = read_reg(REG_DEVICE_ID, raw, sizeof(raw));
        if (err == ESP_OK) break;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Lecture DEVICE_ID echouee : %s", esp_err_to_name(err));
        return err;
    }

    uint16_t id = (uint16_t)raw[0] | ((uint16_t)raw[1] << 8);
    if (id_out) *id_out = id;

    uint8_t family = 0;
    read_u8(REG_FAMILY_ID, &family);

    if (id != SOIL_DEVICE_ID || family != SOIL_FAMILY_ID) {
        ESP_LOGW(TAG, "ID inattendu : DEVICE=0x%04X FAMILY=0x%02X", id, family);
        return ESP_ERR_INVALID_RESPONSE;
    }

    ESP_LOGI(TAG, "DEVICE_ID=0x%04X FAMILY=0x%02X -> OK", id, family);
    return ESP_OK;
}

esp_err_t soil_sensor_read_pf(uint8_t *cp_pf)
{
    if (cp_pf == NULL) return ESP_ERR_INVALID_ARG;

    // Sequence SparkFun exacte (readCapacitancePF) :
    // DEBUG_CP ne se met a jour que quand SENSOR_ID change. On
    // bascule donc sur l'autre capteur, puis on revient sur SID_0
    // AVEC confirmation via DEBUG_SENSOR_ID, avant de lire.
    if (set_sensor_id(SID_1) != ESP_OK) return ESP_FAIL;
    if (set_sensor_id(SID_0) != ESP_OK) return ESP_FAIL;

    return read_synced_u8(REG_DEBUG_CP, cp_pf);
}

int soil_sensor_pf_to_percent(uint8_t cp_pf)
{
    int pct = ((int)cp_pf - SOIL_CP_DRY) * 100 / (SOIL_CP_WET - SOIL_CP_DRY);
    if (pct < 0)   pct = 0;
    if (pct > 100) pct = 100;
    return pct;
}