# eSlate - Capacitive Touchscreen LCD for ESP32-S3

A comprehensive C library for controlling LCD displays with capacitive touchscreens on ESP32-S3 microcontrollers. This project provides both LCD display control via SPI and capacitive touchscreen input via I2C.

## Features

### LCD Display Control
- SPI-based communication with LCD displays
- Support for various LCD controllers (ILI9341, ST7789, etc.)
- Configurable GPIO pin assignments
- Hardware reset and backlight control
- Command and data transmission functions

### Capacitive Touchscreen
- I2C-based touchscreen communication
- Compatible with FT6236/FT5436 touch controllers
- Multi-touch support (up to 5 touch points)
- Pressure sensitivity detection
- Touch calibration and sensitivity adjustment
- Event-driven touch input (touch down, move, up)

### Hardware Configuration

#### LCD Display Pins (SPI)
- **MOSI**: GPIO 11 (Data)
- **MISO**: GPIO 13 (Not used for displays)
- **CLK**: GPIO 12 (SPI Clock)
- **CS**: GPIO 10 (Chip Select)
- **DC**: GPIO 9 (Data/Command)
- **RST**: GPIO 8 (Reset)
- **BL**: GPIO 7 (Backlight)

#### Touchscreen Pins (I2C)
- **SDA**: GPIO 4 (I2C Data)
- **SCL**: GPIO 5 (I2C Clock)
- **INT**: GPIO 6 (Touch Interrupt)
- **RST**: GPIO 18 (Touch Reset)

## API Reference

### Initialization Functions

```c
// Initialize LCD display only
esp_err_t lcd_init_connection(void);

// Initialize touchscreen only
esp_err_t touch_init(void);

// Initialize both LCD and touchscreen
esp_err_t eslate_init(void);
```

### LCD Control Functions

```c
// Send command to LCD
esp_err_t lcd_send_command(uint8_t cmd);

// Send data to LCD
esp_err_t lcd_send_data(uint8_t data);

// Control backlight (0 = OFF, 1 = ON)
void lcd_set_backlight(uint8_t brightness);
```

### Touchscreen Functions

```c
// Get current touch point
esp_err_t touch_get_point(touch_point_t *point);

// Get touch event (blocking with timeout)
esp_err_t touch_get_event(touch_event_data_t *event, uint32_t timeout_ms);

// Check if screen is currently touched
bool touch_is_touched(void);

// Calibrate touchscreen coordinates
esp_err_t touch_calibrate(uint16_t display_width, uint16_t display_height);

// Set touch sensitivity (0-255)
esp_err_t touch_set_sensitivity(uint8_t sensitivity);
```

### Data Structures

```c
// Touch point information
typedef struct {
    uint16_t x;          // X coordinate
    uint16_t y;          // Y coordinate
    uint16_t pressure;   // Touch pressure (0-255)
    uint8_t id;          // Touch point ID
} touch_point_t;

// Touch event information
typedef struct {
    touch_event_t event;     // Event type (DOWN, UP, MOVE)
    touch_point_t point;     // Touch point data
    uint32_t timestamp;      // Event timestamp
} touch_event_data_t;
```

## Usage Example

```c
#include "eSlate.h"

int main(void) {
    // Initialize LCD and touchscreen
    if (eslate_init() != ESP_OK) {
        return -1;
    }

    // Turn on backlight
    lcd_set_backlight(1);

    // Calibrate for 320x240 display
    touch_calibrate(320, 240);

    // Set medium sensitivity
    touch_set_sensitivity(128);

    // Main touch event loop
    while (1) {
        touch_event_data_t event;

        if (touch_get_event(&event, 100) == ESP_OK) {
            switch (event.event) {
                case TOUCH_EVENT_DOWN:
                    printf("Touch at (%d, %d)\n", event.point.x, event.point.y);
                    break;
                case TOUCH_EVENT_MOVE:
                    // Handle drag
                    break;
                case TOUCH_EVENT_UP:
                    // Handle release
                    break;
            }
        }
    }

    return 0;
}
```

## Building the Project

### Prerequisites
- GCC compiler
- Make build system

### Build Commands
```bash
# Clean and build
make clean && make

# Run the demo
./bin/main
```

## Hardware Setup

### ESP32-S3 Pin Connections

#### LCD Display
```
ESP32-S3    | LCD Display
------------|-------------
GPIO 11     | MOSI/SDA
GPIO 12     | SCK/SCL
GPIO 10     | CS
GPIO 9      | DC/RS
GPIO 8      | RESET
GPIO 7      | Backlight
GND         | GND
3.3V        | VCC
```

#### Capacitive Touchscreen
```
ESP32-S3    | Touch Controller
------------|-----------------
GPIO 4      | SDA
GPIO 5      | SCL
GPIO 6      | INT (optional)
GPIO 18     | RESET
GND         | GND
3.3V        | VCC
```

### Supported Touch Controllers
- FocalTech FT6236
- FocalTech FT5436
- Compatible I2C capacitive touch controllers

## Project Structure

```
eSlate/
├── eSlate.c          # Main implementation
├── eSlate.h          # Public API header
├── main.c            # Demo application
├── esp_err.h         # ESP-IDF error codes
├── driver/
│   ├── gpio.h        # GPIO driver
│   ├── spi_master.h  # SPI master driver
│   └── i2c.h         # I2C driver
├── freertos/
│   └── FreeRTOS.h    # FreeRTOS task functions
├── Makefile          # Build configuration
└── README.md         # This file
```

## Notes

- This implementation uses mock ESP-IDF headers for development
- For production use on ESP32-S3, replace with actual ESP-IDF SDK
- Touch calibration may need adjustment for specific hardware
- SPI frequency is set to 20MHz, adjust if needed for your display
- I2C frequency is set to 400kHz for touchscreen communication

## License

This project is provided as-is for educational and development purposes.