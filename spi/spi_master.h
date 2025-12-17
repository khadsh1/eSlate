#ifndef DRIVER_SPI_MASTER_H
#define DRIVER_SPI_MASTER_H

#include <stdint.h>
#include "../esp/esp_err.h"
#include "../gpio/gpio.h"

/**
 * @brief SPI host enumeration
 */
typedef enum {
    SPI1_HOST = 0,
    SPI2_HOST = 1,
    SPI3_HOST = 2,
    HSPI_HOST = SPI2_HOST,
    VSPI_HOST = SPI3_HOST,
} spi_host_device_t;

/**
 * @brief SPI DMA channel
 */
typedef enum {
    SPI_DMA_DISABLED = 0,
    SPI_DMA_CH1 = 1,
    SPI_DMA_CH2 = 2,
    SPI_DMA_CH_AUTO = 3,
} spi_dma_chan_t;

/**
 * @brief Opaque SPI device handle
 */
typedef void *spi_device_handle_t;

/**
 * @brief SPI bus configuration
 */
typedef struct {
    int mosi_io_num;            /**< GPIO pin for MOSI signal, or -1 if not used */
    int miso_io_num;            /**< GPIO pin for MISO signal, or -1 if not used */
    int sclk_io_num;            /**< GPIO pin for SPI clock signal */
    int quadwp_io_num;          /**< GPIO pin for SPI WP signal (quad mode), -1 to disable */
    int quadhd_io_num;          /**< GPIO pin for SPI HD signal (quad mode), -1 to disable */
    int max_transfer_sz;        /**< Maximum transfer size in bytes */
    uint32_t flags;             /**< Flags for SPI bus configuration */
    uint32_t intr_flags;        /**< Interrupt flags for SPI bus */
} spi_bus_config_t;

/**
 * @brief SPI device configuration
 */
typedef struct {
    uint8_t mode;               /**< SPI mode (0-3) */
    uint32_t clock_speed_hz;    /**< SPI clock speed in Hz */
    uint16_t spics_io_num;      /**< Chip select GPIO pin, or -1 to disable */
    uint32_t spics_ext_io_num;  /**< External chip select GPIO pin (quad mode), or -1 */
    uint8_t cs_ena_pretrans;    /**< Amount of SPI bit-times CS is activated before transmission */
    uint8_t cs_ena_posttrans;   /**< Amount of SPI bit-times CS is kept active after transmission */
    int input_delay_ns;         /**< Input delay in nanoseconds */
    uint8_t duty_cycle_pos;     /**< Duty cycle for high clock pulse in percent (0-100) */
    uint8_t use_txfifo;         /**< Whether to use TXFIFO for transmission */
    uint8_t use_rxfifo;         /**< Whether to use RXFIFO for reception */
    int queue_size;             /**< Transaction queue size */
    uint32_t flags;             /**< Flags for SPI device configuration */
} spi_device_interface_config_t;

/**
 * @brief SPI transaction configuration
 */
typedef struct {
    uint32_t flags;             /**< Flags for transaction */
    uint16_t cmd;               /**< Command phase data (for command mode) */
    uint8_t cmd_bitlen;         /**< Length of command phase in bits */
    uint16_t addr;              /**< Address phase data */
    uint8_t addr_bitlen;        /**< Length of address phase in bits */
    uint16_t padding_between_addr_and_data;  /**< Padding between address and data phase */
    uint16_t tx_data_length;    /**< Length of tx data in bits (0 for full send) */
    uint16_t rx_data_length;    /**< Length of rx data in bits */
    uint16_t length;            /**< Total data length in bits */
    const void *tx_buffer;      /**< Pointer to transmit buffer */
    void *rx_buffer;            /**< Pointer to receive buffer */
    void *user;                 /**< User-defined variable */
} spi_transaction_t;

#define SPI_DEVICE_NO_DUMMY_CYCLES     0x0001  /**< Disable dummy cycles */
#define SPI_DEVICE_USE_DMA             0x0004  /**< Use DMA for transmission */

/**
 * @brief Initialize SPI bus
 * 
 * @param host SPI host device
 * @param bus_config SPI bus configuration
 * @param dma_chan DMA channel to use
 * @return ESP_OK on success
 */
esp_err_t spi_bus_initialize(spi_host_device_t host, const spi_bus_config_t *bus_config, spi_dma_chan_t dma_chan);

/**
 * @brief Add SPI device
 * 
 * @param host SPI host device
 * @param dev_config Device configuration
 * @param handle Output handle for device
 * @return ESP_OK on success
 */
esp_err_t spi_bus_add_device(spi_host_device_t host, const spi_device_interface_config_t *dev_config, spi_device_handle_t *handle);

/**
 * @brief Remove SPI device
 * 
 * @param handle SPI device handle
 * @return ESP_OK on success
 */
esp_err_t spi_bus_remove_device(spi_device_handle_t handle);

/**
 * @brief Transmit data via SPI (polling mode, blocks until completion)
 * 
 * @param handle SPI device handle
 * @param trans_desc SPI transaction configuration
 * @return ESP_OK on success
 */
esp_err_t spi_device_polling_transmit(spi_device_handle_t handle, spi_transaction_t *trans_desc);

/**
 * @brief Transmit and receive data via SPI (polling mode)
 * 
 * @param handle SPI device handle
 * @param trans_desc SPI transaction configuration
 * @return ESP_OK on success
 */
esp_err_t spi_device_polling_transceive(spi_device_handle_t handle, spi_transaction_t *trans_desc);

#endif // DRIVER_SPI_MASTER_H
