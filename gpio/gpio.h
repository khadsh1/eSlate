#ifndef DRIVER_GPIO_H
#define DRIVER_GPIO_H

#include <stdint.h>
#include "../esp/esp_err.h"

/**
 * @brief GPIO pin enumeration
 */
typedef enum {
    GPIO_NUM_0 = 0,
    GPIO_NUM_1 = 1,
    GPIO_NUM_2 = 2,
    GPIO_NUM_3 = 3,
    GPIO_NUM_4 = 4,
    GPIO_NUM_5 = 5,
    GPIO_NUM_6 = 6,
    GPIO_NUM_7 = 7,
    GPIO_NUM_8 = 8,
    GPIO_NUM_9 = 9,
    GPIO_NUM_10 = 10,
    GPIO_NUM_11 = 11,
    GPIO_NUM_12 = 12,
    GPIO_NUM_13 = 13,
    GPIO_NUM_14 = 14,
    GPIO_NUM_15 = 15,
    GPIO_NUM_16 = 16,
    GPIO_NUM_17 = 17,
    GPIO_NUM_18 = 18,
    GPIO_NUM_19 = 19,
    GPIO_NUM_20 = 20,
    GPIO_NUM_21 = 21,
    GPIO_NUM_MAX = 48,
} gpio_num_t;

/**
 * @brief GPIO interrupt type
 */
typedef enum {
    GPIO_INTR_DISABLE = 0,
    GPIO_INTR_POSEDGE = 1,
    GPIO_INTR_NEGEDGE = 2,
    GPIO_INTR_ANYEDGE = 3,
    GPIO_INTR_LOW_LEVEL = 4,
    GPIO_INTR_HIGH_LEVEL = 5,
} gpio_int_type_t;

/**
 * @brief GPIO mode
 */
typedef enum {
    GPIO_MODE_DISABLE = 0,
    GPIO_MODE_INPUT = 1,
    GPIO_MODE_OUTPUT = 2,
    GPIO_MODE_OUTPUT_OD = 3,
} gpio_mode_t;

/**
 * @brief GPIO pull configuration
 */
typedef enum {
    GPIO_PULLUP_ONLY,
    GPIO_PULLDOWN_ONLY,
    GPIO_PULLUP_PULLDOWN,
    GPIO_FLOATING,
} gpio_pull_mode_t;

/**
 * @brief GPIO configuration structure
 */
typedef struct {
    uint64_t pin_bit_mask;          /**< Bitmask of GPIO pins */
    gpio_mode_t mode;               /**< GPIO mode */
    gpio_int_type_t intr_type;      /**< GPIO interrupt type */
    uint8_t pull_up_en;             /**< Enable internal pull-up */
    uint8_t pull_down_en;           /**< Enable internal pull-down */
} gpio_config_t;

#define GPIO_PULLUP_DISABLE     0
#define GPIO_PULLUP_ENABLE      1
#define GPIO_PULLDOWN_DISABLE   0
#define GPIO_PULLDOWN_ENABLE    1

/**
 * @brief Configure GPIO pin
 * 
 * @param pGPIOConfig GPIO configuration
 * @return ESP_OK on success
 */
esp_err_t gpio_config(const gpio_config_t *pGPIOConfig);

/**
 * @brief Set GPIO output level
 * 
 * @param gpio_num GPIO number
 * @param level Level (0 or 1)
 * @return ESP_OK on success
 */
esp_err_t gpio_set_level(gpio_num_t gpio_num, uint32_t level);

/**
 * @brief Get GPIO input level
 * 
 * @param gpio_num GPIO number
 * @return 0 or 1
 */
int gpio_get_level(gpio_num_t gpio_num);

#endif // DRIVER_GPIO_H
