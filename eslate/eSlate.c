#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "eSlate.h"
#include "../spi/spi_master.h"
#include "../gpio/gpio.h"
#include "../i2c/i2c.h"
#include "../esp/esp_err.h"
#include "../rtos/FreeRTOS.h"

// SPI LCD Display Configuration for ESP32-S3
#define LCD_HOST        HSPI_HOST
#define LCD_CLK_PIN     GPIO_NUM_12      // SPI Clock
#define LCD_MOSI_PIN    GPIO_NUM_11      // SPI MOSI (Data)
#define LCD_MISO_PIN    GPIO_NUM_13      // SPI MISO (optional, not used for displays)
#define LCD_CS_PIN      GPIO_NUM_10      // Chip Select
#define LCD_DC_PIN      GPIO_NUM_9       // Data/Command
#define LCD_RST_PIN     GPIO_NUM_8       // Reset
#define LCD_BL_PIN      GPIO_NUM_7       // Backlight control

#define LCD_SPI_FREQ    20000000         // 20MHz SPI clock frequency

// Capacitive Touchscreen Configuration (FT6236/FT5436 compatible)
#define TOUCH_I2C_PORT          I2C_NUM_0
#define TOUCH_SDA_PIN           GPIO_NUM_4       // I2C SDA
#define TOUCH_SCL_PIN           GPIO_NUM_5       // I2C SCL
#define TOUCH_INT_PIN           GPIO_NUM_6       // Touch interrupt
#define TOUCH_RST_PIN           GPIO_NUM_18      // Touch reset
#define TOUCH_I2C_ADDR          0x38             // FT6236 I2C address
#define TOUCH_I2C_FREQ          400000           // 400kHz I2C clock

#define TOUCH_MAX_POINTS        5                // Maximum touch points
#define TOUCH_CALIBRATION_POINTS 4               // Calibration points

// Touch controller registers (FT6236)
#define TOUCH_REG_DEVICE_MODE   0x00
#define TOUCH_REG_GESTURE_ID    0x01
#define TOUCH_REG_TD_STATUS     0x02
#define TOUCH_REG_P1_XH         0x03
#define TOUCH_REG_P1_XL         0x04
#define TOUCH_REG_P1_YH         0x05
#define TOUCH_REG_P1_YL         0x06
#define TOUCH_REG_P1_WEIGHT     0x07
#define TOUCH_REG_P1_MISC       0x08
#define TOUCH_REG_TH_GROUP      0x80
#define TOUCH_REG_PERIODACTIVE  0x88
#define TOUCH_REG_LIB_VER_H     0xA1
#define TOUCH_REG_LIB_VER_L     0xA2
#define TOUCH_REG_CIPHER        0xA3
#define TOUCH_REG_G_MODE        0xA4
#define TOUCH_REG_PWR_MODE      0xA5
#define TOUCH_REG_FIRMID        0xA6
#define TOUCH_REG_FOCALTECH_ID  0xA8
#define TOUCH_REG_RELEASE_CODE_ID 0xAF
#define TOUCH_REG_STATE         0xBC

// Global handles
static spi_device_handle_t lcd_spi_handle = NULL;
static bool touch_initialized = false;
static bool lcd_initialized = false;

// Touch calibration data
static struct {
    int16_t x_offset;
    int16_t y_offset;
    float x_scale;
    float y_scale;
    uint16_t display_width;
    uint16_t display_height;
} touch_cal = {0, 0, 1.0f, 1.0f, 320, 240}; // Default 320x240 display

/**
 * @brief Initialize GPIO pins for LCD display control
 * 
 * @return ESP_OK on success, ESP_FAIL on error
 */
static esp_err_t lcd_gpio_init(void)
{
    gpio_config_t io_conf;
    
    // Configure output pins: DC, RST, BL
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pin_bit_mask = (1ULL << LCD_DC_PIN) | 
                           (1ULL << LCD_RST_PIN) | 
                           (1ULL << LCD_BL_PIN);
    
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        printf("GPIO config failed: %s\n", esp_err_to_name(ret));
        return ret;
    }
    
    // Initialize pins to known states
    gpio_set_level(LCD_DC_PIN, 0);
    gpio_set_level(LCD_RST_PIN, 0);
    gpio_set_level(LCD_BL_PIN, 1);  // Backlight ON
    
    return ESP_OK;
}

/**
 * @brief Perform LCD hardware reset
 */
static void lcd_reset(void)
{
    // Pull reset low
    gpio_set_level(LCD_RST_PIN, 0);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    
    // Release reset (pull high)
    gpio_set_level(LCD_RST_PIN, 1);
    vTaskDelay(100 / portTICK_PERIOD_MS);
}

/**
 * @brief Initialize SPI communication for LCD
 * 
 * @return ESP_OK on success, ESP_FAIL on error
 */
static esp_err_t lcd_spi_init(void)
{
    spi_bus_config_t buscfg = {
        .miso_io_num = LCD_MISO_PIN,
        .mosi_io_num = LCD_MOSI_PIN,
        .sclk_io_num = LCD_CLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };
    
    esp_err_t ret = spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        printf("SPI bus init failed: %s\n", esp_err_to_name(ret));
        return ret;
    }
    
    spi_device_interface_config_t devcfg = {
        .mode = 0,                          // SPI mode 0 (CPOL=0, CPHA=0)
        .clock_speed_hz = LCD_SPI_FREQ,
        .spics_io_num = LCD_CS_PIN,
        .queue_size = 7,
        .flags = SPI_DEVICE_NO_DUMMY_CYCLES,
    };
    
    ret = spi_bus_add_device(LCD_HOST, &devcfg, &lcd_spi_handle);
    if (ret != ESP_OK) {
        printf("SPI device add failed: %s\n", esp_err_to_name(ret));
        return ret;
    }
    
    return ESP_OK;
}

/**
 * @brief Establish connection to LCD display on ESP32-S3
 * 
 * This function initializes:
 * - GPIO pins for control signals (DC, RST, BL)
 * - SPI bus and device configuration
 * - LCD hardware reset
 * 
 * @return ESP_OK on successful initialization, error code otherwise
 */
esp_err_t lcd_init_connection(void)
{
    printf("Initializing LCD display connection...\n");
    
    // Step 1: Initialize GPIO pins
    esp_err_t ret = lcd_gpio_init();
    if (ret != ESP_OK) {
        printf("LCD GPIO initialization failed\n");
        return ret;
    }
    printf("GPIO pins configured\n");
    
    // Step 2: Initialize SPI bus and device
    ret = lcd_spi_init();
    if (ret != ESP_OK) {
        printf("LCD SPI initialization failed\n");
        return ret;
    }
    printf("SPI bus initialized\n");
    
    // Step 3: Perform hardware reset
    lcd_reset();
    printf("LCD hardware reset completed\n");
    
    printf("LCD display connection established successfully\n");
    return ESP_OK;
}

/**
 * @brief Send command to LCD display
 * 
 * @param cmd Command byte to send
 * @return ESP_OK on success
 */
esp_err_t lcd_send_command(uint8_t cmd)
{
    if (lcd_spi_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    gpio_set_level(LCD_DC_PIN, 0);  // DC low for command mode
    
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &cmd,
    };
    
    return spi_device_polling_transmit(lcd_spi_handle, &t);
}

/**
 * @brief Send data to LCD display
 * 
 * @param data Data byte to send
 * @return ESP_OK on success
 */
esp_err_t lcd_send_data(uint8_t data)
{
    if (lcd_spi_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    gpio_set_level(LCD_DC_PIN, 1);  // DC high for data mode
    
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &data,
    };
    
    return spi_device_polling_transmit(lcd_spi_handle, &t);
}

/**
 * @brief Set LCD backlight brightness
 * 
 * @param brightness Backlight level (0 = OFF, 1 = ON)
 */
void lcd_set_backlight(uint8_t brightness)
{
    gpio_set_level(LCD_BL_PIN, brightness ? 1 : 0);
}

/**
 * @brief Initialize LCD display (ST7789 or similar)
 *
 * @return ESP_OK on success
 */
esp_err_t lcd_init_display(void)
{
    // Basic ST7789 initialization sequence (simplified)
    // In a real implementation, this would include proper LCD controller commands

    // Software reset
    lcd_send_command(0x01);
    vTaskDelay(150 / portTICK_PERIOD_MS);

    // Sleep out
    lcd_send_command(0x11);
    vTaskDelay(10 / portTICK_PERIOD_MS);

    // Memory data access control
    lcd_send_command(0x36);
    lcd_send_data(0x00);  // RGB, normal orientation

    // Interface pixel format
    lcd_send_command(0x3A);
    lcd_send_data(0x05);  // 16-bit RGB565

    // Display on
    lcd_send_command(0x29);

    printf("LCD display initialized\n");
    return ESP_OK;
}

/**
 * @brief Set display window for drawing
 *
 * @param x_start Start X coordinate
 * @param y_start Start Y coordinate
 * @param x_end End X coordinate
 * @param y_end End Y coordinate
 */
static void lcd_set_window(uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end)
{
    // Column address set
    lcd_send_command(0x2A);
    lcd_send_data(x_start >> 8);
    lcd_send_data(x_start & 0xFF);
    lcd_send_data(x_end >> 8);
    lcd_send_data(x_end & 0xFF);

    // Row address set
    lcd_send_command(0x2B);
    lcd_send_data(y_start >> 8);
    lcd_send_data(y_start & 0xFF);
    lcd_send_data(y_end >> 8);
    lcd_send_data(y_end & 0xFF);

    // Memory write
    lcd_send_command(0x2C);
}

/**
 * @brief Draw pixel on LCD display
 *
 * @param x X coordinate
 * @param y Y coordinate
 * @param color 16-bit RGB565 color
 */
void lcd_draw_pixel(uint16_t x, uint16_t y, uint16_t color)
{
    if (x >= CANVAS_WIDTH || y >= CANVAS_HEIGHT + CANVAS_START_Y) {
        return;
    }

    lcd_set_window(x, y, x, y);
    lcd_send_data(color >> 8);
    lcd_send_data(color & 0xFF);
}

/**
 * @brief Fill rectangle on LCD display
 *
 * @param x Start X coordinate
 * @param y Start Y coordinate
 * @param width Rectangle width
 * @param height Rectangle height
 * @param color 16-bit RGB565 color
 */
void lcd_fill_rect(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color)
{
    if (x >= CANVAS_WIDTH || y >= CANVAS_HEIGHT + CANVAS_START_Y) {
        return;
    }

    uint16_t x_end = x + width - 1;
    uint16_t y_end = y + height - 1;

    if (x_end >= CANVAS_WIDTH) x_end = CANVAS_WIDTH - 1;
    if (y_end >= CANVAS_HEIGHT + CANVAS_START_Y) y_end = CANVAS_HEIGHT + CANVAS_START_Y - 1;

    lcd_set_window(x, y, x_end, y_end);

    uint32_t pixel_count = (x_end - x + 1) * (y_end - y + 1);
    for (uint32_t i = 0; i < pixel_count; i++) {
        lcd_send_data(color >> 8);
        lcd_send_data(color & 0xFF);
    }
}

/**
 * @brief Draw line on LCD display (Bresenham's algorithm)
 *
 * @param x0 Start X coordinate
 * @param y0 Start Y coordinate
 * @param x1 End X coordinate
 * @param y1 End Y coordinate
 * @param color 16-bit RGB565 color
 */
void lcd_draw_line(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color)
{
    int dx = abs((int)x1 - (int)x0);
    int dy = abs((int)y1 - (int)y0);
    int sx = x0 < x1 ? 1 : -1;
    int sy = y0 < y1 ? 1 : -1;
    int err = dx - dy;

    while (true) {
        lcd_draw_pixel(x0, y0, color);

        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y0 += sy;
        }
    }
}

/**
 * @brief Draw circle on LCD display
 *
 * @param center_x Center X coordinate
 * @param center_y Center Y coordinate
 * @param radius Circle radius
 * @param color 16-bit RGB565 color
 */
void lcd_draw_circle(uint16_t center_x, uint16_t center_y, uint16_t radius, uint16_t color)
{
    int x = radius;
    int y = 0;
    int err = 0;

    while (x >= y) {
        lcd_draw_pixel(center_x + x, center_y + y, color);
        lcd_draw_pixel(center_x + y, center_y + x, color);
        lcd_draw_pixel(center_x - y, center_y + x, color);
        lcd_draw_pixel(center_x - x, center_y + y, color);
        lcd_draw_pixel(center_x - x, center_y - y, color);
        lcd_draw_pixel(center_x - y, center_y - x, color);
        lcd_draw_pixel(center_x + y, center_y - x, color);
        lcd_draw_pixel(center_x + x, center_y - y, color);

        y += 1;
        err += 1 + 2*y;
        if (2*(err - x) + 1 > 0) {
            x -= 1;
            err += 1 - 2*x;
        }
    }
}

/**
 * @brief Draw filled circle on LCD display
 *
 * @param center_x Center X coordinate
 * @param center_y Center Y coordinate
 * @param radius Circle radius
 * @param color 16-bit RGB565 color
 */
void lcd_fill_circle(uint16_t center_x, uint16_t center_y, uint16_t radius, uint16_t color)
{
    for (int y = -radius; y <= radius; y++) {
        for (int x = -radius; x <= radius; x++) {
            if (x*x + y*y <= radius*radius) {
                lcd_draw_pixel(center_x + x, center_y + y, color);
            }
        }
    }
}

/**
 * @brief Draw rectangle on LCD display
 *
 * @param x Start X coordinate
 * @param y Start Y coordinate
 * @param width Rectangle width
 * @param height Rectangle height
 * @param color 16-bit RGB565 color
 */
void lcd_draw_rect(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color)
{
    lcd_draw_line(x, y, x + width - 1, y, color);
    lcd_draw_line(x + width - 1, y, x + width - 1, y + height - 1, color);
    lcd_draw_line(x + width - 1, y + height - 1, x, y + height - 1, color);
    lcd_draw_line(x, y + height - 1, x, y, color);
}

/**
 * @brief Clear LCD display
 *
 * @param color 16-bit RGB565 color to fill with
 */
void lcd_clear(uint16_t color)
{
    lcd_fill_rect(0, 0, CANVAS_WIDTH, CANVAS_HEIGHT + CANVAS_START_Y, color);
}

/**
 * @brief Simple font rendering (5x7 pixel font)
 */
static const uint8_t font_5x7[95][5] = {
    // Space to ~
    {0x00, 0x00, 0x00, 0x00, 0x00}, // Space
    {0x00, 0x00, 0x5F, 0x00, 0x00}, // !
    {0x00, 0x07, 0x00, 0x07, 0x00}, // "
    {0x14, 0x7F, 0x14, 0x7F, 0x14}, // #
    // ... (truncated for brevity - would include full ASCII table)
};

/**
 * @brief Draw character on LCD display
 *
 * @param x Start X coordinate
 * @param y Start Y coordinate
 * @param c Character to draw
 * @param color 16-bit RGB565 color
 */
void lcd_draw_char(uint16_t x, uint16_t y, char c, uint16_t color)
{
    if (c < 32 || c > 126) return; // Only printable ASCII

    uint8_t char_index = c - 32;

    for (uint8_t i = 0; i < 5; i++) {
        uint8_t line = font_5x7[char_index][i];
        for (uint8_t j = 0; j < 7; j++) {
            if (line & (1 << j)) {
                lcd_draw_pixel(x + i, y + j, color);
            }
        }
    }
}

/**
 * @brief Draw string on LCD display
 *
 * @param x Start X coordinate
 * @param y Start Y coordinate
 * @param str String to draw
 * @param color 16-bit RGB565 color
 */
void lcd_draw_string(uint16_t x, uint16_t y, const char* str, uint16_t color)
{
    uint16_t current_x = x;
    while (*str) {
        lcd_draw_char(current_x, y, *str, color);
        current_x += 6; // 5 pixels + 1 space
        str++;
    }
}

/**
 * @brief Initialize GPIO pins for touchscreen
 *
 * @return ESP_OK on success, ESP_FAIL on error
 */
static esp_err_t touch_gpio_init(void)
{
    gpio_config_t io_conf;

    // Configure touchscreen reset pin as output
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pin_bit_mask = (1ULL << TOUCH_RST_PIN);

    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        printf("Touch GPIO config failed: %s\n", esp_err_to_name(ret));
        return ret;
    }

    // Configure touchscreen interrupt pin as input
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pin_bit_mask = (1ULL << TOUCH_INT_PIN);

    ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        printf("Touch INT GPIO config failed: %s\n", esp_err_to_name(ret));
        return ret;
    }

    // Initialize pins
    gpio_set_level(TOUCH_RST_PIN, 1);  // Reset high (active)

    return ESP_OK;
}

/**
 * @brief Initialize I2C bus for touchscreen
 *
 * @return ESP_OK on success, ESP_FAIL on error
 */
static esp_err_t touch_i2c_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = TOUCH_SDA_PIN,
        .scl_io_num = TOUCH_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = TOUCH_I2C_FREQ,
    };

    esp_err_t ret = i2c_param_config(TOUCH_I2C_PORT, &conf);
    if (ret != ESP_OK) {
        printf("I2C param config failed: %s\n", esp_err_to_name(ret));
        return ret;
    }

    ret = i2c_driver_install(TOUCH_I2C_PORT, conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        printf("I2C driver install failed: %s\n", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

/**
 * @brief Read register from touchscreen controller
 *
 * @param reg Register address
 * @param data Pointer to store read data
 * @param len Length of data to read
 * @return ESP_OK on success
 */
static esp_err_t touch_read_register(uint8_t reg, uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    // Start + write device address + write register
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (TOUCH_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);

    // Repeated start + read device address + read data
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (TOUCH_I2C_ADDR << 1) | I2C_MASTER_READ, true);

    if (len > 1) {
        for (size_t i = 0; i < len - 1; i++) {
            i2c_master_read_byte(cmd, &data[i], I2C_MASTER_ACK);
        }
    }
    i2c_master_read_byte(cmd, &data[len - 1], I2C_MASTER_NACK);

    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(TOUCH_I2C_PORT, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

/**
 * @brief Write register to touchscreen controller
 *
 * @param reg Register address
 * @param data Data to write
 * @return ESP_OK on success
 */
static esp_err_t touch_write_register(uint8_t reg, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (TOUCH_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(TOUCH_I2C_PORT, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

/**
 * @brief Reset touchscreen controller
 */
static void touch_reset(void)
{
    gpio_set_level(TOUCH_RST_PIN, 0);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    gpio_set_level(TOUCH_RST_PIN, 1);
    vTaskDelay(300 / portTICK_PERIOD_MS);  // Wait for controller to initialize
}

/**
 * @brief Initialize capacitive touchscreen
 *
 * This function initializes:
 * - GPIO pins for touch control
 * - I2C bus for touchscreen communication
 * - Touch controller configuration
 *
 * @return ESP_OK on successful initialization, error code otherwise
 */
esp_err_t touch_init(void)
{
    if (touch_initialized) {
        return ESP_OK;
    }

    printf("Initializing capacitive touchscreen...\n");

    // Initialize GPIO pins
    esp_err_t ret = touch_gpio_init();
    if (ret != ESP_OK) {
        printf("Touch GPIO initialization failed\n");
        return ret;
    }

    // Initialize I2C bus
    ret = touch_i2c_init();
    if (ret != ESP_OK) {
        printf("Touch I2C initialization failed\n");
        return ret;
    }

    // Reset touchscreen controller
    touch_reset();

    // Configure touchscreen controller
    // Set to normal operation mode
    ret = touch_write_register(TOUCH_REG_DEVICE_MODE, 0x00);
    if (ret != ESP_OK) {
        printf("Failed to set device mode\n");
        return ret;
    }

    // Set interrupt polling mode
    ret = touch_write_register(TOUCH_REG_G_MODE, 0x00);
    if (ret != ESP_OK) {
        printf("Failed to set interrupt mode\n");
        return ret;
    }

    touch_initialized = true;
    printf("Capacitive touchscreen initialized successfully\n");
    return ESP_OK;
}

/**
 * @brief Initialize both LCD display and touchscreen
 *
 * @return ESP_OK on successful initialization, error code otherwise
 */
esp_err_t eslate_init(void)
{
    esp_err_t ret;

    // Initialize LCD first
    ret = lcd_init_connection();
    if (ret != ESP_OK) {
        return ret;
    }
    lcd_initialized = true;

    // Initialize touchscreen
    ret = touch_init();
    if (ret != ESP_OK) {
        return ret;
    }

    printf("eSlate (LCD + Touchscreen) initialized successfully\n");
    return ESP_OK;
}

/**
 * @brief Get current touch point data
 *
 * @param point Pointer to touch_point_t structure to fill
 * @return ESP_OK if touch data is valid, ESP_FAIL if no touch detected
 */
esp_err_t touch_get_point(touch_point_t *point)
{
    if (!touch_initialized || point == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t data[6];  // Read touch data registers

    esp_err_t ret = touch_read_register(TOUCH_REG_TD_STATUS, data, 6);
    if (ret != ESP_OK) {
        return ret;
    }

    uint8_t touch_count = data[0] & 0x0F;  // Number of touch points

    if (touch_count == 0 || touch_count > TOUCH_MAX_POINTS) {
        return ESP_FAIL;  // No touch detected
    }

    // Read first touch point data (P1 registers)
    uint8_t touch_data[5];
    ret = touch_read_register(TOUCH_REG_P1_XH, touch_data, 5);
    if (ret != ESP_OK) {
        return ret;
    }

    // Parse touch coordinates
    uint16_t x_raw = ((touch_data[0] & 0x0F) << 8) | touch_data[1];
    uint16_t y_raw = ((touch_data[2] & 0x0F) << 8) | touch_data[3];
    uint8_t pressure = touch_data[4];

    // Apply calibration
    point->x = (uint16_t)((x_raw - touch_cal.x_offset) * touch_cal.x_scale);
    point->y = (uint16_t)((y_raw - touch_cal.y_offset) * touch_cal.y_scale);
    point->pressure = pressure;
    point->id = 0;  // Single touch point

    // Ensure coordinates are within display bounds
    if (point->x >= touch_cal.display_width) point->x = touch_cal.display_width - 1;
    if (point->y >= touch_cal.display_height) point->y = touch_cal.display_height - 1;

    return ESP_OK;
}

/**
 * @brief Get touch event (blocking)
 *
 * @param event Pointer to touch_event_data_t structure to fill
 * @param timeout_ms Timeout in milliseconds (0 = no timeout)
 * @return ESP_OK on success, ESP_ERR_TIMEOUT on timeout
 */
esp_err_t touch_get_event(touch_event_data_t *event, uint32_t timeout_ms)
{
    if (!touch_initialized || event == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    uint32_t start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    touch_point_t point;
    bool was_touched = false;

    while (true) {
        esp_err_t ret = touch_get_point(&point);

        if (ret == ESP_OK) {
            if (!was_touched) {
                // Touch down event
                event->event = TOUCH_EVENT_DOWN;
                event->point = point;
                event->timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
                was_touched = true;
                return ESP_OK;
            } else {
                // Touch move event
                event->event = TOUCH_EVENT_MOVE;
                event->point = point;
                event->timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
                return ESP_OK;
            }
        } else if (was_touched) {
            // Touch up event
            event->event = TOUCH_EVENT_UP;
            event->point = point;
            event->point.pressure = 0;
            event->timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
            was_touched = false;
            return ESP_OK;
        }

        // Check timeout
        if (timeout_ms > 0) {
            uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
            if (current_time - start_time >= timeout_ms) {
                event->event = TOUCH_EVENT_NONE;
                return ESP_ERR_TIMEOUT;
            }
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);  // Poll every 10ms
    }
}

/**
 * @brief Check if touchscreen is currently being touched
 *
 * @return true if touched, false otherwise
 */
bool touch_is_touched(void)
{
    if (!touch_initialized) {
        return false;
    }

    uint8_t status;
    esp_err_t ret = touch_read_register(TOUCH_REG_TD_STATUS, &status, 1);
    if (ret != ESP_OK) {
        return false;
    }

    return (status & 0x0F) > 0;  // Check if any touch points are active
}

/**
 * @brief Calibrate touchscreen coordinates
 *
 * @param display_width Display width in pixels
 * @param display_height Display height in pixels
 * @return ESP_OK on success
 */
esp_err_t touch_calibrate(uint16_t display_width, uint16_t display_height)
{
    if (!touch_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // For basic calibration, assume touchscreen matches display dimensions
    // In a real implementation, you would collect calibration points
    touch_cal.display_width = display_width;
    touch_cal.display_height = display_height;
    touch_cal.x_offset = 0;
    touch_cal.y_offset = 0;
    touch_cal.x_scale = 1.0f;
    touch_cal.y_scale = 1.0f;

    printf("Touchscreen calibrated for %dx%d display\n", display_width, display_height);
    return ESP_OK;
}

/**
 * @brief Set touchscreen sensitivity
 *
 * @param sensitivity Sensitivity level (0-255, higher = more sensitive)
 * @return ESP_OK on success
 */
esp_err_t touch_set_sensitivity(uint8_t sensitivity)
{
    if (!touch_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // Map sensitivity to threshold value (higher sensitivity = lower threshold)
    uint8_t threshold = 255 - sensitivity;

    esp_err_t ret = touch_write_register(TOUCH_REG_TH_GROUP, threshold);
    if (ret != ESP_OK) {
        printf("Failed to set touch sensitivity\n");
        return ret;
    }

    printf("Touch sensitivity set to %d\n", sensitivity);
    return ESP_OK;
}