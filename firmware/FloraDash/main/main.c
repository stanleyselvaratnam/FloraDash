#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_system.h"

void app_main(void)
{
    // Infos sur la puce pour confirmer qu'on parle bien au bon chip
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);

    printf("\n=== FloraDash - Test ESP-IDF ===\n");
    printf("Chip       : %s\n",
           (chip_info.model == CHIP_ESP32C3) ? "ESP32-C3" : "Autre");
    printf("Coeurs     : %d\n", chip_info.cores);
    printf("Revision   : %d\n", chip_info.revision);
    printf("Heap libre : %lu octets\n", esp_get_free_heap_size());
    printf("================================\n\n");

    int compteur = 0;
    while (1) {
        printf("FloraDash vivant ! tick = %d\n", compteur++);
        vTaskDelay(pdMS_TO_TICKS(1000));  // 1 seconde
    }
}