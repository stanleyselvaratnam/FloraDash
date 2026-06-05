#ifndef SOIL_SENSOR_H
#define SOIL_SENSOR_H

#include "driver/i2c_master.h"
#include "esp_err.h"
#include <stdint.h>

// ════════════════════════════════════════════════════════
// Capteur d'humidite de sol SparkFun Qwiic (SEN-30480)
// base sur le controleur CapSense CY8CMBR3102.
//
// Porte fidelement depuis la lib officielle SparkFun :
//   https://github.com/sparkfun/SparkFun_CY8CMBR3_Arduino_Library
//
// L'humidite est mesuree via la CAPACITE en pF (registre DEBUG_CP).
// La capacite augmente avec l'humidite du sol.
//
// La puce arrive deja configuree d'usine : on ne reconfigure PAS
// la flash, on se contente de lire (comme l'exemple de base
// SparkFun). La lecture suit leur sequence exacte :
//   basculer SENSOR_ID pour forcer la mise a jour de DEBUG_CP,
//   confirmer le SENSOR_ID via DEBUG_SENSOR_ID (lecture synchro),
//   puis lire DEBUG_CP (lecture synchro).
// ════════════════════════════════════════════════════════

#define SOIL_I2C_ADDR        0x37
#define SOIL_DEVICE_ID       0x0A01   // registre DEVICE_ID (0x90)
#define SOIL_FAMILY_ID       0x9A     // registre FAMILY_ID (0x8F)

// Points de calibration (en pF) mesures sur TON capteur.
// La capacite monte avec l'humidite : sec = bas, humide = haut.
#define SOIL_CP_DRY          7        // sol sec
#define SOIL_CP_WET          27       // sol bien humide

// Ajoute le capteur sur un bus I2C existant (partage avec l'OLED)
// et valide la communication. Ne reconfigure PAS la puce.
esp_err_t soil_sensor_begin(i2c_master_bus_handle_t bus);

// Valide la presence : DEVICE_ID == 0x0A01 et FAMILY_ID == 0x9A.
esp_err_t soil_sensor_check_id(uint16_t *id_out);

// Lit la capacite du capteur en picofarads (0..255).
esp_err_t soil_sensor_read_pf(uint8_t *cp_pf);

// Convertit une capacite pF en pourcentage d'humidite (0..100)
// selon les bornes SOIL_CP_DRY / SOIL_CP_WET.
int soil_sensor_pf_to_percent(uint8_t cp_pf);

// Configure la puce UNE SEULE FOIS avec le bloc EZ-Click, le
// sauvegarde en flash (NVM), puis reset. A appeler manuellement
// une fois, PAS a chaque boot.
esp_err_t soil_sensor_apply_ezclick_config(void);

#endif // SOIL_SENSOR_H