#ifndef TASK_H
#define TASK_H

#include <stdint.h>
#include <stdbool.h>

// FreeRTOS base types
typedef unsigned int UBaseType_t;

// FreeRTOS task types and constants
typedef void * TaskHandle_t;
typedef void (*TaskFunction_t)(void *);

// Task priorities
#define tskIDLE_PRIORITY         0

// Tick type
typedef uint32_t TickType_t;

// Base type for return values
typedef int32_t BaseType_t;

// Task creation
BaseType_t xTaskCreate(TaskFunction_t pxTaskCode,
                      const char * const pcName,
                      const uint16_t usStackDepth,
                      void * const pvParameters,
                      UBaseType_t uxPriority,
                      TaskHandle_t * const pxCreatedTask);

// Task deletion
void vTaskDelete(TaskHandle_t xTaskToDelete);

// Task delay
void vTaskDelay(const TickType_t xTicksToDelay);

// Task delay until
void vTaskDelayUntil(TickType_t * const pxPreviousWakeTime, const TickType_t xTimeIncrement);

// Get tick count
TickType_t xTaskGetTickCount(void);
TickType_t xTaskGetTickCountFromISR(void);

// Task yield
void taskYIELD(void);

// Scheduler control
void vTaskStartScheduler(void);

// Critical sections
void vPortEnterCritical(void);
void vPortExitCritical(void);

// Timer setup
void vPortSetupTimerInterrupt(void);

// FreeRTOS constants
#define pdPASS                  (1)
#define pdFAIL                  (0)
#define pdTRUE                  (1)
#define pdFALSE                 (0)

// Milliseconds to ticks conversion
#define pdMS_TO_TICKS(xTimeInMs) ((TickType_t)((xTimeInMs) * 1000 / 1000))

#endif // TASK_H