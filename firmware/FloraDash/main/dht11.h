#ifndef DHT11_H
#define DHT11_H

#include "esp_err.h"

typedef struct {
    float temperature;  // °C
    float humidity;     // %RH
} dht11_reading_t;

esp_err_t dht11_init(int gpio_num);
esp_err_t dht11_read(dht11_reading_t *out);

#endif // DHT11_H