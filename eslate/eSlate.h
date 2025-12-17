#ifndef ESLATE_H
#define ESLATE_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/**
 * @brief LCD Display Interface with Capacitive Touchscreen
 * 
 * This module provides functions to control an LCD display with
 * capacitive touchscreen connected to ESP32-S3 via SPI (display)
 * and I2C (touchscreen).
 */

// Canvas dimensions
#define CANVAS_WIDTH    320
#define CANVAS_HEIGHT   200  // Leave space for GUI at bottom
#define CANVAS_START_Y  40   // GUI area at top

/**
 * @brief Touch point structure
 */
typedef struct {
    uint16_t x;          /**< X coordinate */
    uint16_t y;          /**< Y coordinate */
    uint16_t pressure;   /**< Touch pressure (0-255, 0 = no touch) */
    uint8_t id;          /**< Touch point ID (for multi-touch) */
} touch_point_t;

/**
 * @brief Touch event types
 */
typedef enum {
    TOUCH_EVENT_NONE = 0,
    TOUCH_EVENT_DOWN = 1,
    TOUCH_EVENT_UP = 2,
    TOUCH_EVENT_MOVE = 3,
} touch_event_t;

/**
 * @brief Touch event structure
 */
typedef struct {
    touch_event_t event;     /**< Type of touch event */
    touch_point_t point;     /**< Touch point data */
    uint32_t timestamp;      /**< Event timestamp */
} touch_event_data_t;

/**
 * @brief Establish connection to LCD display on ESP32-S3
 * 
 * This function initializes:
 * - GPIO pins for control signals (DC, RST, BL)
 * - SPI bus and device configuration for LCD
 * - LCD hardware reset
 * 
 * @return ESP_OK on successful initialization, error code otherwise
 */
esp_err_t lcd_init_connection(void);

/**
 * @brief Initialize capacitive touchscreen
 * 
 * This function initializes:
 * - I2C bus for touchscreen communication
 * - Touch controller configuration
 * - Touch interrupt handling
 * 
 * @return ESP_OK on successful initialization, error code otherwise
 */
esp_err_t touch_init(void);

/**
 * @brief Initialize both LCD display and touchscreen
 * 
 * @return ESP_OK on successful initialization, error code otherwise
 */
esp_err_t eslate_init(void);

/**
 * @brief Send command to LCD display
 * 
 * @param cmd Command byte to send
 * @return ESP_OK on success
 */
esp_err_t lcd_send_command(uint8_t cmd);

/**
 * @brief Send data to LCD display
 * 
 * @param data Data byte to send
 * @return ESP_OK on success
 */
esp_err_t lcd_send_data(uint8_t data);

/**
 * @brief Set LCD backlight brightness
 * 
 * @param brightness Backlight level (0 = OFF, 1 = ON)
 */
void lcd_set_backlight(uint8_t brightness);

/**
 * @brief Get current touch point data
 * 
 * @param point Pointer to touch_point_t structure to fill
 * @return ESP_OK if touch data is valid, ESP_FAIL if no touch detected
 */
esp_err_t touch_get_point(touch_point_t *point);

/**
 * @brief Get touch event (blocking)
 * 
 * @param event Pointer to touch_event_data_t structure to fill
 * @param timeout_ms Timeout in milliseconds (0 = no timeout)
 * @return ESP_OK on success, ESP_ERR_TIMEOUT on timeout
 */
esp_err_t touch_get_event(touch_event_data_t *event, uint32_t timeout_ms);

/**
 * @brief Check if touchscreen is currently being touched
 * 
 * @return true if touched, false otherwise
 */
bool touch_is_touched(void);

/**
 * @brief Calibrate touchscreen coordinates
 * 
 * @param display_width Display width in pixels
 * @param display_height Display height in pixels
 * @return ESP_OK on success
 */
esp_err_t touch_calibrate(uint16_t display_width, uint16_t display_height);

/**
 * @brief Set touchscreen sensitivity
 * 
 * @param sensitivity Sensitivity level (0-255, higher = more sensitive)
 * @return ESP_OK on success
 */
esp_err_t touch_set_sensitivity(uint8_t sensitivity);

/**
 * @brief Initialize LCD display controller
 *
 * @return ESP_OK on success
 */
esp_err_t lcd_init_display(void);

/**
 * @brief Draw pixel on LCD display
 *
 * @param x X coordinate
 * @param y Y coordinate
 * @param color 16-bit RGB565 color
 */
void lcd_draw_pixel(uint16_t x, uint16_t y, uint16_t color);

/**
 * @brief Fill rectangle on LCD display
 *
 * @param x Start X coordinate
 * @param y Start Y coordinate
 * @param width Rectangle width
 * @param height Rectangle height
 * @param color 16-bit RGB565 color
 */
void lcd_fill_rect(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color);

/**
 * @brief Draw line on LCD display
 *
 * @param x0 Start X coordinate
 * @param y0 Start Y coordinate
 * @param x1 End X coordinate
 * @param y1 End Y coordinate
 * @param color 16-bit RGB565 color
 */
void lcd_draw_line(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color);

/**
 * @brief Draw circle on LCD display
 *
 * @param center_x Center X coordinate
 * @param center_y Center Y coordinate
 * @param radius Circle radius
 * @param color 16-bit RGB565 color
 */
void lcd_draw_circle(uint16_t center_x, uint16_t center_y, uint16_t radius, uint16_t color);

/**
 * @brief Draw filled circle on LCD display
 *
 * @param center_x Center X coordinate
 * @param center_y Center Y coordinate
 * @param radius Circle radius
 * @param color 16-bit RGB565 color
 */
void lcd_fill_circle(uint16_t center_x, uint16_t center_y, uint16_t radius, uint16_t color);

/**
 * @brief Draw rectangle on LCD display
 *
 * @param x Start X coordinate
 * @param y Start Y coordinate
 * @param width Rectangle width
 * @param height Rectangle height
 * @param color 16-bit RGB565 color
 */
void lcd_draw_rect(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color);

/**
 * @brief Clear LCD display
 *
 * @param color 16-bit RGB565 color to fill with
 */
void lcd_clear(uint16_t color);

/**
 * @brief Draw character on LCD display
 *
 * @param x Start X coordinate
 * @param y Start Y coordinate
 * @param c Character to draw
 * @param color 16-bit RGB565 color
 */
void lcd_draw_char(uint16_t x, uint16_t y, char c, uint16_t color);

/**
 * @brief Draw string on LCD display
 *
 * @param x Start X coordinate
 * @param y Start Y coordinate
 * @param str String to draw
 * @param color 16-bit RGB565 color
 */
void lcd_draw_string(uint16_t x, uint16_t y, const char* str, uint16_t color);

// Color definitions (RGB565)
#define COLOR_BLACK     0x0000
#define COLOR_WHITE     0xFFFF
#define COLOR_RED       0xF800
#define COLOR_GREEN     0x07E0
#define COLOR_BLUE      0x001F
#define COLOR_YELLOW    0xFFE0
#define COLOR_CYAN      0x07FF
#define COLOR_MAGENTA   0xF81F
#define COLOR_GRAY      0x8410

#endif // ESLATE_H
