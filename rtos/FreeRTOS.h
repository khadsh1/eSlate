#ifndef FREERTOS_H
#define FREERTOS_H

#include <stdint.h>

/**
 * @brief Tick period in milliseconds
 */
#define portTICK_PERIOD_MS      1

/**
 * @brief Task delay in milliseconds
 * 
 * @param xTicksToDelay Number of ticks to delay
 */
void vTaskDelay(uint32_t xTicksToDelay);

/**
 * @brief Get the current tick count
 * 
 * @return Current tick count
 */
uint32_t xTaskGetTickCount(void);

#endif // FREERTOS_H
