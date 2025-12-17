#ifndef STORAGE_H
#define STORAGE_H

#include <stdint.h>
#include <stdbool.h>
#include "../drawing/drawing.h"

// Storage configuration
#define STORAGE_NAMESPACE "eSlate"
#define STORAGE_KEY_CANVAS "canvas"
#define STORAGE_MAX_SIZE 4096  // Maximum size for stored data

// Storage result codes
typedef enum {
    STORAGE_OK = 0,
    STORAGE_ERR_NOT_FOUND = 1,
    STORAGE_ERR_INVALID_DATA = 2,
    STORAGE_ERR_NO_SPACE = 3,
    STORAGE_ERR_WRITE_FAILED = 4,
    STORAGE_ERR_READ_FAILED = 5,
} storage_result_t;

/**
 * @brief Initialize storage system
 */
void storage_init(void);

/**
 * @brief Save canvas to flash storage
 *
 * @param canvas Canvas to save
 * @return Storage result code
 */
storage_result_t storage_save_canvas(drawing_canvas_t* canvas);

/**
 * @brief Load canvas from flash storage
 *
 * @param canvas Canvas structure to fill
 * @return Storage result code
 */
storage_result_t storage_load_canvas(drawing_canvas_t* canvas);

/**
 * @brief Check if canvas data exists in storage
 *
 * @return true if data exists, false otherwise
 */
bool storage_canvas_exists(void);

/**
 * @brief Delete stored canvas data
 *
 * @return Storage result code
 */
storage_result_t storage_delete_canvas(void);

/**
 * @brief Get storage statistics
 *
 * @param used_bytes Bytes used in storage
 * @param free_bytes Bytes free in storage
 */
void storage_get_stats(uint32_t* used_bytes, uint32_t* free_bytes);

/**
 * @brief Format storage (erase all data)
 *
 * @return Storage result code
 */
storage_result_t storage_format(void);

#endif // STORAGE_H