#ifndef ESP_ERR_H
#define ESP_ERR_H

#include <stdint.h>

/**
 * @brief ESP Error Codes
 */
typedef int32_t esp_err_t;

#define ESP_OK                      0x00    /**< Success */
#define ESP_FAIL                    0x01    /**< Generic failure */
#define ESP_ERR_INVALID_ARG         0x02    /**< Invalid argument */
#define ESP_ERR_INVALID_STATE       0x03    /**< Invalid state */
#define ESP_ERR_NO_MEM              0x04    /**< Out of memory */
#define ESP_ERR_TIMEOUT             0x05    /**< Timeout */

/**
 * @brief Convert error code to error name string
 * 
 * @param code Error code
 * @return String representation of error code
 */
const char *esp_err_to_name(esp_err_t code);

#endif // ESP_ERR_H
