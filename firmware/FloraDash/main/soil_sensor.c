#include "soil_sensor.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soil_config_ezclick.h"

static const char *TAG = "SOIL";

// ════════════════════════════════════════════════════════
// REGISTRES CY8CMBR3102 (TRM Infineon / lib SparkFun)
// Ordonnes par adresse.
// ════════════════════════════════════════════════════════
#define REG_SENSOR_ID         0x82  // selection du capteur a debugguer
#define REG_CTRL_CMD          0x86  // registre de commande
#define REG_CTRL_CMD_STATUS   0x88  // statut de la derniere commande
#define REG_CTRL_CMD_ERR      0x89  // code d'erreur eventuel
#define REG_FAMILY_ID         0x8F  // 1 octet     -> 0x9A
#define REG_DEVICE_ID         0x90  // 2 octets LE -> 0x0A01
#define REG_SYNC_COUNTER1     0xDB  // compteur sync bas  (encadre les debug)
#define REG_DEBUG_SENSOR_ID   0xDC  // echo du SENSOR_ID applique
#define REG_DEBUG_CP          0xDD  // capacite en pF (1 octet) <-- humidite
#define REG_SYNC_COUNTER2     0xE7  // compteur sync haut (encadre les debug)

// ── Commandes du registre REG_CTRL_CMD ─────────────────────
#define CMD_SAVE_CHECK_CRC    0x02  // calcule + verifie CRC, sauve en NVM
#define CMD_SW_RESET          0xFF  // reset logiciel

// ── Identifiants de capteur (SENSOR_ID) ────────────────────
#define SID_0                 0x00
#define SID_1                 0x01

// ── Parametres ─────────────────────────────────────────────
#define CONFIG_BLOCK_LEN      128   // 0x00..0x7F
#define SOIL_TIMEOUT_MS       100
#define SOIL_SYNC_RETRIES     5     // relectures pour coherence sync
#define SOIL_WAKE_RETRIES     10    // reessais reveil (NACK au boot)
#define SOIL_ID_CONFIRM_TRIES 20    // confirmations de set_sensor_id

static i2c_master_dev_handle_t s_dev = NULL;

// ════════════════════════════════════════════════════════
// E/S REGISTRE DE BASE
// ════════════════════════════════════════════════════════
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

// ════════════════════════════════════════════════════════
// LECTURE SYNCHRONISEE DES REGISTRES DE DEBUG
// ════════════════════════════════════════════════════════
// Les registres de debug (DEBUG_SENSOR_ID 0xDC, DEBUG_CP 0xDD)
// sont encadres par SYNC_COUNTER1 (0xDB) et SYNC_COUNTER2 (0xE7).
// Le TRM precise qu'ils ne sont valides que lorsque les deux
// compteurs qui les encadrent sont egaux. Si un rafraichissement
// tombe pendant la lecture, les compteurs different -> on retente.
static esp_err_t read_synced_u8(uint8_t reg, uint8_t *val)
{
    for (int i = 0; i < SOIL_SYNC_RETRIES; i++) {
        uint8_t s1 = 0, s2 = 0, data = 0;
        if (read_u8(REG_SYNC_COUNTER1, &s1) != ESP_OK) return ESP_FAIL;
        if (read_u8(reg, &data)             != ESP_OK) return ESP_FAIL;
        if (read_u8(REG_SYNC_COUNTER2, &s2) != ESP_OK) return ESP_FAIL;
        if (s1 == s2) {
            *val = data;
            return ESP_OK;
        }
    }
    return ESP_ERR_INVALID_STATE;
}

// ════════════════════════════════════════════════════════
// SELECTION D'UN CAPTEUR AVEC CONFIRMATION (methode SparkFun)
// ════════════════════════════════════════════════════════
// On ecrit SENSOR_ID puis on attend que DEBUG_SENSOR_ID (lu en
// synchro) confirme la valeur. C'est ce qui garantit la lecture
// du bon capteur et evite l'alternance de valeurs.
static esp_err_t set_sensor_id(uint8_t sid)
{
    if (write_u8(REG_SENSOR_ID, sid) != ESP_OK) return ESP_FAIL;

    for (int i = 0; i < SOIL_ID_CONFIRM_TRIES; i++) {
        uint8_t echo = 0xFF;
        if (read_synced_u8(REG_DEBUG_SENSOR_ID, &echo) == ESP_OK && echo == sid) {
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    return ESP_ERR_TIMEOUT;
}

// ════════════════════════════════════════════════════════
// CONFIGURATION UNIQUE DE LA PUCE (bloc EZ-Click)
// ════════════════════════════════════════════════════════
// Sequence TRM : ecrire les 128 octets de config, puis envoyer
// SAVE_CHECK_CRC. La puce recalcule le CRC de son cote et ne
// sauve en flash QUE s'il correspond aux 2 derniers octets.
// Enfin, reset pour appliquer. A n'executer qu'UNE fois.
esp_err_t soil_sensor_apply_ezclick_config(void)
{
    if (s_dev == NULL) return ESP_ERR_INVALID_STATE;

    // 1. Ecrire la config registre par registre (offset = adresse
    //    0x00..0x7F, valeur = octet correspondant).
    ESP_LOGI(TAG, "Ecriture du bloc de config (128 octets)...");
    for (uint8_t off = 0; off < CONFIG_BLOCK_LEN; off++) {
        esp_err_t err = write_u8(off, SOIL_EZCLICK_CONFIG[off]);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Echec ecriture offset 0x%02X : %s",
                     off, esp_err_to_name(err));
            return err;
        }
    }

    // 2. Commande SAVE_CHECK_CRC : la puce verifie le CRC et sauve.
    ESP_LOGI(TAG, "Envoi SAVE_CHECK_CRC...");
    esp_err_t err = write_u8(REG_CTRL_CMD, CMD_SAVE_CHECK_CRC);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Echec envoi commande : %s", esp_err_to_name(err));
        return err;
    }

    // 3. Laisser le temps a l'ecriture flash (~220 ms typique).
    vTaskDelay(pdMS_TO_TICKS(300));

    // 4. Verifier le statut. 0 = succes ; sinon lire le code erreur.
    uint8_t status = 0xFF, errcode = 0xFF;
    if (read_u8(REG_CTRL_CMD_STATUS, &status) != ESP_OK) {
        ESP_LOGW(TAG, "Statut illisible (la puce a peut-etre reset)");
    }
    if (status != 0x00) {
        read_u8(REG_CTRL_CMD_ERR, &errcode);
        ESP_LOGE(TAG, "CRC refuse ! status=0x%02X err=0x%02X", status, errcode);
        return ESP_ERR_INVALID_CRC;
    }
    ESP_LOGI(TAG, "Config sauvegardee, CRC accepte.");

    // 5. Reset logiciel pour appliquer la nouvelle config.
    write_u8(REG_CTRL_CMD, CMD_SW_RESET);
    vTaskDelay(pdMS_TO_TICKS(300));   // attendre le reboot

    ESP_LOGI(TAG, "Puce reconfiguree et redemarree.");
    return ESP_OK;
}

// ════════════════════════════════════════════════════════
// API PUBLIQUE
// ════════════════════════════════════════════════════════
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

// Reveille la puce avant la lecture : relit DEVICE_ID avec retry sur NACK.
// Necessaire car la puce se rendort entre deux lectures espacees.
static esp_err_t soil_wake(void)
{
    uint8_t raw[2] = {0};
    for (int i = 0; i < SOIL_WAKE_RETRIES; i++) {
        if (read_reg(REG_DEVICE_ID, raw, sizeof(raw)) == ESP_OK) {
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    return ESP_ERR_TIMEOUT;
}

esp_err_t soil_sensor_read_pf(uint8_t *cp_pf)
{
    if (cp_pf == NULL) return ESP_ERR_INVALID_ARG;

    // Etape 0 : reveiller la puce si elle s'est rendormie.
    if (soil_wake() != ESP_OK) {
        ESP_LOGW(TAG, "Reveil capteur echoue");
        return ESP_FAIL;
    }

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