#include "storage.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

// Simulated storage (in real ESP32, this would use NVS or SPIFFS)
static uint8_t simulated_flash[STORAGE_MAX_SIZE];
static bool flash_initialized = false;
static uint32_t flash_used = 0;

// Storage data header
typedef struct {
    uint32_t magic;        // Magic number to verify data integrity
    uint32_t version;      // Data format version
    uint32_t data_size;    // Size of stored data
    uint32_t checksum;     // Simple checksum
} storage_header_t;

#define STORAGE_MAGIC 0x45534C54  // "ESLT"
#define STORAGE_VERSION 1

/**
 * @brief Calculate simple checksum
 */
static uint32_t calculate_checksum(const uint8_t* data, uint32_t size)
{
    uint32_t checksum = 0;
    for (uint32_t i = 0; i < size; i++) {
        checksum += data[i];
        checksum = (checksum << 1) | (checksum >> 31);  // Rotate left
    }
    return checksum;
}

/**
 * @brief Initialize storage system
 */
void storage_init(void)
{
    if (!flash_initialized) {
        memset(simulated_flash, 0xFF, STORAGE_MAX_SIZE);
        flash_used = 0;
        flash_initialized = true;
        printf("Storage system initialized (simulated)\n");
    }
}

/**
 * @brief Save canvas to flash storage
 */
storage_result_t storage_save_canvas(drawing_canvas_t* canvas)
{
    if (!canvas) {
        return STORAGE_ERR_INVALID_DATA;
    }

    // Calculate required storage size
    uint32_t data_size = sizeof(drawing_canvas_t);
    uint32_t total_size = sizeof(storage_header_t) + data_size;

    if (total_size > STORAGE_MAX_SIZE) {
        return STORAGE_ERR_NO_SPACE;
    }

    // Create storage header
    storage_header_t header;
    header.magic = STORAGE_MAGIC;
    header.version = STORAGE_VERSION;
    header.data_size = data_size;
    header.checksum = calculate_checksum((uint8_t*)canvas, data_size);

    // Write header and data to simulated flash
    memcpy(simulated_flash, &header, sizeof(storage_header_t));
    memcpy(simulated_flash + sizeof(storage_header_t), canvas, data_size);

    flash_used = total_size;

    printf("Canvas saved to storage (%d bytes)\n", total_size);
    return STORAGE_OK;
}

/**
 * @brief Load canvas from flash storage
 */
storage_result_t storage_load_canvas(drawing_canvas_t* canvas)
{
    if (!canvas) {
        return STORAGE_ERR_INVALID_DATA;
    }

    // Read header
    storage_header_t header;
    memcpy(&header, simulated_flash, sizeof(storage_header_t));

    // Verify header
    if (header.magic != STORAGE_MAGIC) {
        return STORAGE_ERR_NOT_FOUND;
    }

    if (header.version != STORAGE_VERSION) {
        return STORAGE_ERR_INVALID_DATA;
    }

    uint32_t total_size = sizeof(storage_header_t) + header.data_size;
    if (total_size > STORAGE_MAX_SIZE) {
        return STORAGE_ERR_INVALID_DATA;
    }

    // Read canvas data
    drawing_canvas_t temp_canvas;
    memcpy(&temp_canvas, simulated_flash + sizeof(storage_header_t), header.data_size);

    // Verify checksum
    uint32_t calculated_checksum = calculate_checksum((uint8_t*)&temp_canvas, header.data_size);
    if (calculated_checksum != header.checksum) {
        return STORAGE_ERR_INVALID_DATA;
    }

    // Copy to output
    *canvas = temp_canvas;

    printf("Canvas loaded from storage (%d bytes)\n", total_size);
    return STORAGE_OK;
}

/**
 * @brief Check if canvas data exists in storage
 */
bool storage_canvas_exists(void)
{
    storage_header_t header;
    memcpy(&header, simulated_flash, sizeof(storage_header_t));

    return (header.magic == STORAGE_MAGIC && header.version == STORAGE_VERSION);
}

/**
 * @brief Delete stored canvas data
 */
storage_result_t storage_delete_canvas(void)
{
    memset(simulated_flash, 0xFF, STORAGE_MAX_SIZE);
    flash_used = 0;
    printf("Canvas data deleted from storage\n");
    return STORAGE_OK;
}

/**
 * @brief Get storage statistics
 */
void storage_get_stats(uint32_t* used_bytes, uint32_t* free_bytes)
{
    if (used_bytes) *used_bytes = flash_used;
    if (free_bytes) *free_bytes = STORAGE_MAX_SIZE - flash_used;
}

/**
 * @brief Format storage (erase all data)
 */
storage_result_t storage_format(void)
{
    memset(simulated_flash, 0xFF, STORAGE_MAX_SIZE);
    flash_used = 0;
    printf("Storage formatted\n");
    return STORAGE_OK;
}