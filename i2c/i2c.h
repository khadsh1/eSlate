#ifndef DRIVER_I2C_H
#define DRIVER_I2C_H

#include <stdint.h>
#include <stdbool.h>
#include "../esp/esp_err.h"
#include "../gpio/gpio.h"

/**
 * @brief I2C port enumeration
 */
typedef enum {
    I2C_NUM_0 = 0,
    I2C_NUM_1 = 1,
} i2c_port_t;

/**
 * @brief I2C mode
 */
typedef enum {
    I2C_MODE_MASTER = 0,
    I2C_MODE_SLAVE = 1,
} i2c_mode_t;

/**
 * @brief I2C ACK types
 */
typedef enum {
    I2C_MASTER_ACK = 0,
    I2C_MASTER_NACK = 1,
} i2c_ack_type_t;

/**
 * @brief I2C master configuration
 */
typedef struct {
    uint32_t clk_speed;    /**< I2C clock frequency for master mode */
} i2c_master_config_t;

/**
 * @brief I2C configuration structure
 */
typedef struct {
    i2c_mode_t mode;             /**< I2C mode */
    uint32_t sda_io_num;         /**< GPIO number for SDA */
    uint32_t scl_io_num;         /**< GPIO number for SCL */
    uint32_t sda_pullup_en;      /**< Enable SDA internal pull-up */
    uint32_t scl_pullup_en;      /**< Enable SCL internal pull-up */
    i2c_master_config_t master;  /**< I2C master configuration */
} i2c_config_t;

/**
 * @brief I2C command link handle
 */
typedef void *i2c_cmd_handle_t;

/**
 * @brief I2C master write/read flags
 */
#define I2C_MASTER_WRITE 0  /**< I2C write flag */
#define I2C_MASTER_READ  1  /**< I2C read flag */

/**
 * @brief Initialize I2C bus
 * 
 * @param port I2C port number
 * @param config I2C configuration
 * @return ESP_OK on success
 */
esp_err_t i2c_param_config(i2c_port_t port, const i2c_config_t *config);

/**
 * @brief Install I2C driver
 * 
 * @param port I2C port number
 * @param mode I2C mode
 * @param slv_rx_buf_len Slave receive buffer length (slave mode only)
 * @param slv_tx_buf_len Slave transmit buffer length (slave mode only)
 * @param intr_alloc_flags Interrupt allocation flags
 * @return ESP_OK on success
 */
esp_err_t i2c_driver_install(i2c_port_t port, i2c_mode_t mode, size_t slv_rx_buf_len, size_t slv_tx_buf_len, int intr_alloc_flags);

/**
 * @brief Create I2C command link
 * 
 * @return Command link handle
 */
i2c_cmd_handle_t i2c_cmd_link_create(void);

/**
 * @brief Delete I2C command link
 * 
 * @param cmd_handle Command link handle
 */
void i2c_cmd_link_delete(i2c_cmd_handle_t cmd_handle);

/**
 * @brief Queue start signal in I2C command
 * 
 * @param cmd_handle Command link handle
 * @return ESP_OK on success
 */
esp_err_t i2c_master_start(i2c_cmd_handle_t cmd_handle);

/**
 * @brief Queue stop signal in I2C command
 * 
 * @param cmd_handle Command link handle
 * @return ESP_OK on success
 */
esp_err_t i2c_master_stop(i2c_cmd_handle_t cmd_handle);

/**
 * @brief Queue write byte command
 * 
 * @param cmd_handle Command link handle
 * @param data Data byte to write
 * @param ack_en Enable ACK check
 * @return ESP_OK on success
 */
esp_err_t i2c_master_write_byte(i2c_cmd_handle_t cmd_handle, uint8_t data, bool ack_en);

/**
 * @brief Queue read byte command
 * 
 * @param cmd_handle Command link handle
 * @param data Pointer to store read data
 * @param ack ACK value to send after reading
 * @return ESP_OK on success
 */
esp_err_t i2c_master_read_byte(i2c_cmd_handle_t cmd_handle, uint8_t *data, i2c_ack_type_t ack);

/**
 * @brief Execute I2C commands
 * 
 * @param port I2C port number
 * @param cmd_handle Command link handle
 * @param ticks_to_wait Timeout in ticks
 * @return ESP_OK on success
 */
esp_err_t i2c_master_cmd_begin(i2c_port_t port, i2c_cmd_handle_t cmd_handle, uint32_t ticks_to_wait);

#endif // DRIVER_I2C_H