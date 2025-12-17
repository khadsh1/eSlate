#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include "gpio.h"
#include "../esp/esp_err.h"

// ESP32-S3 GPIO hardware register definitions
#define DR_REG_GPIO_BASE           0x60004000
#define GPIO_OUT_REG               (DR_REG_GPIO_BASE + 0x0004)
#define GPIO_OUT_W1TS_REG          (DR_REG_GPIO_BASE + 0x0008)
#define GPIO_OUT_W1TC_REG          (DR_REG_GPIO_BASE + 0x000C)
#define GPIO_IN_REG                (DR_REG_GPIO_BASE + 0x003C)
#define GPIO_ENABLE_REG            (DR_REG_GPIO_BASE + 0x0020)
#define GPIO_ENABLE_W1TS_REG       (DR_REG_GPIO_BASE + 0x0024)
#define GPIO_ENABLE_W1TC_REG       (DR_REG_GPIO_BASE + 0x0028)

// GPIO pin configuration registers (GPIO_PIN0_REG to GPIO_PIN39_REG)
#define GPIO_PIN_REG(pin)          (DR_REG_GPIO_BASE + 0x0088 + (pin * 4))

// GPIO pin register fields
#define GPIO_PIN_PAD_DRIVER        (1 << 2)  // 0: Normal, 1: Open drain
#define GPIO_PIN_INT_ENA_MASK      (0x7 << 7)  // Interrupt enable mask
#define GPIO_PIN_INT_TYPE_MASK     (0x7 << 9)  // Interrupt type mask

// GPIO function select registers
#define IO_MUX_REG_BASE            0x60009000
#define IO_MUX_REG(pin)            (IO_MUX_REG_BASE + (pin * 4))
#define IO_MUX_FUNC_MASK           0x1FF
#define IO_MUX_PULLUP              (1 << 9)
#define IO_MUX_PULLDOWN            (1 << 10)

// Register access macros
#define REG_READ(reg)              (*(volatile uint32_t *)(reg))
#define REG_WRITE(reg, val)        (*(volatile uint32_t *)(reg) = (val))
#define REG_SET_BIT(reg, bit)      REG_WRITE(reg, REG_READ(reg) | (1 << bit))
#define REG_CLR_BIT(reg, bit)      REG_WRITE(reg, REG_READ(reg) & ~(1 << bit))

/**
 * @brief Configure GPIO pin with direct register access
 * Real ESP32-S3 hardware register implementation
 */
esp_err_t gpio_config(const gpio_config_t *pGPIOConfig) {
    if (pGPIOConfig == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Configure each pin in the bitmask
    for (int pin = 0; pin < GPIO_NUM_MAX; pin++) {
        if (pGPIOConfig->pin_bit_mask & (1ULL << pin)) {
            // Configure pin function (GPIO function = 2 for most pins)
            uint32_t mux_reg = IO_MUX_REG(pin);
            uint32_t mux_val = REG_READ(mux_reg);
            mux_val &= ~IO_MUX_FUNC_MASK;
            mux_val |= 2; // GPIO function

            // Configure pull-up/pull-down
            if (pGPIOConfig->pull_up_en) {
                mux_val |= IO_MUX_PULLUP;
            } else {
                mux_val &= ~IO_MUX_PULLUP;
            }

            if (pGPIOConfig->pull_down_en) {
                mux_val |= IO_MUX_PULLDOWN;
            } else {
                mux_val &= ~IO_MUX_PULLDOWN;
            }

            REG_WRITE(mux_reg, mux_val);

            // Configure GPIO pin register
            uint32_t pin_reg = GPIO_PIN_REG(pin);
            uint32_t pin_val = REG_READ(pin_reg);

            // Clear interrupt configuration
            pin_val &= ~GPIO_PIN_INT_ENA_MASK;
            pin_val &= ~GPIO_PIN_INT_TYPE_MASK;

            // Configure drive mode (open drain vs push-pull)
            if (pGPIOConfig->mode & GPIO_MODE_OUTPUT_OD) {
                pin_val |= GPIO_PIN_PAD_DRIVER;  // Open drain
            } else {
                pin_val &= ~GPIO_PIN_PAD_DRIVER; // Push-pull
            }

            REG_WRITE(pin_reg, pin_val);

            // Configure input/output enable
            if (pGPIOConfig->mode & GPIO_MODE_INPUT) {
                // Input is always enabled for GPIO pins, controlled by IO_MUX
            }

            if (pGPIOConfig->mode & GPIO_MODE_OUTPUT) {
                // Enable output
                REG_SET_BIT(GPIO_ENABLE_REG, pin);
            } else {
                // Disable output
                REG_CLR_BIT(GPIO_ENABLE_REG, pin);
            }
        }
    }

    return ESP_OK;
}

/**
 * @brief Set GPIO output level with direct register access
 * Real ESP32-S3 hardware register implementation
 */
esp_err_t gpio_set_level(gpio_num_t gpio_num, uint32_t level) {
    if (gpio_num >= GPIO_NUM_MAX) {
        return ESP_ERR_INVALID_ARG;
    }

    if (level) {
        // Set output high using W1TS register
        REG_SET_BIT(GPIO_OUT_W1TS_REG, gpio_num);
    } else {
        // Set output low using W1TC register
        REG_SET_BIT(GPIO_OUT_W1TC_REG, gpio_num);
    }

    return ESP_OK;
}

/**
 * @brief Get GPIO input level with direct register access
 * Real ESP32-S3 hardware register implementation
 */
int gpio_get_level(gpio_num_t gpio_num) {
    if (gpio_num >= GPIO_NUM_MAX) {
        return -1;
    }

    // Read from GPIO input register
    uint32_t in_reg = REG_READ(GPIO_IN_REG);
    return (in_reg & (1 << gpio_num)) ? 1 : 0;
}