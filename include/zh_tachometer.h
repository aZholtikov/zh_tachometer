/**
 * @file zh_tachometer.h
 *
 * @brief ESP-IDF driver for a quadrature encoder-based tachometer using PCNT
 *        and ESP-Timer to measure rotational speed in RPM.
 *
 * The module leverages the ESP-IDF PCNT peripheral in quadrature decoder mode
 * to count encoder pulses and an ESP-Timer running at 10 Hz to sample the
 * accumulated count and compute RPM. The result is a non-blocking, RTOS-friendly
 * interface suitable for motor speed monitoring.
 *
 * Key features:
 * - Quadrature encoder support via two GPIO lines (A/B phases)
 * - Automatic RPM calculation based on configurable pulses-per-revolution
 * - Configurable GPIO pull-up resistors
 * - Thread-safe access through FreeRTOS-compatible handles
 *
 * @note The internal PCNT unit uses accumulation mode with watch points at
 *       ±32767. RPM values above 32767 may overflow — ensure the encoder
 *       pulses-per-revolution and expected RPM range stay within safe bounds.
 */

#pragma once

#include "math.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define ZH_TACHOMETER_INIT_CONFIG_DEFAULT() \
    {                                       \
        .a_gpio_number = GPIO_NUM_MAX,      \
        .b_gpio_number = GPIO_NUM_MAX,      \
        .pullup = true,                     \
        .encoder_pulses = 0}

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief Opaque handle to the tachometer instance.
     *
     * Used to identify and access a specific tachometer throughout its
     * lifetime. The internal structure is defined in the implementation
     * file and must not be accessed directly.
     */
    typedef struct _zh_tachometer_handle_t zh_tachometer_handle_t;

    /**
     * @brief Initialization configuration for the quadrature encoder tachometer.
     *
     * This structure is passed to `zh_tachometer_init()` to configure the GPIO
     * pins, pull-up resistors, and the encoder's pulses-per-revolution ratio.
     *
     * @note Both GPIO pins must be assigned before initialization.
     *       Use GPIO_NUM_MAX to indicate an unassigned pin.
     */
    typedef struct
    {
        uint8_t a_gpio_number;   /*!< Encoder A phase GPIO number */
        uint8_t b_gpio_number;   /*!< Encoder B phase GPIO number */
        bool pullup;             /*!< Enable or disable GPIO pull-up resistors */
        uint16_t encoder_pulses; /*!< Number of pulses per one full rotation */
    } zh_tachometer_init_config_t;

    /**
     * @brief Initialize the tachometer with the provided configuration.
     *
     * Allocates a handle, configures the ESP-Timer (10 Hz sampling), and sets up
     * the PCNT peripheral in quadrature decoder mode with glitch filtering.
     *
     * @param[in] config Pointer to the initialization configuration (must not be NULL)
     * @param[out] handle Pointer to receive the created tachometer handle (must be NULL)
     *
     * @return ESP_OK on success
     * @return ESP_ERR_INVALID_ARG if `config` or `handle` is NULL, or `encoder_pulses` is zero
     * @return ESP_ERR_INVALID_STATE if the handle is already initialized.
     * @return ESP_ERR_NO_MEM if memory allocation fails
     * @return ESP_FAIL if PCNT initialization or configuration failed
     *
     */
    esp_err_t zh_tachometer_init(const zh_tachometer_init_config_t *config, zh_tachometer_handle_t **handle);

    /**
     * @brief Deinitialize the tachometer and release all resources.
     *
     * Stops the PCNT unit, removes channels and watch points, deletes the
     * ESP-Timer, and frees the handle memory. The handle pointer is set to
     * NULL on success.
     *
     * @param[in,out] handle Pointer to the tachometer handle (must not be NULL)
     *
     * @note After deinitialization, the handle is invalidated and must not be
     *       used. Call zh_tachometer_init() again to reinitialize.
     *
     * @return ESP_OK on success
     * @return ESP_ERR_INVALID_ARG if `handle` is NULL or `*handle` is NULL
     * @return ESP_FAIL if PCNT stop, disable, or deletion failed
     */
    esp_err_t zh_tachometer_deinit(zh_tachometer_handle_t **handle);

    /**
     * @brief Retrieve the current RPM value from the tachometer.
     *
     * Returns the last sampled RPM value computed by the timer callback.
     * The value represents absolute RPM (direction is discarded); negative
     * rotation is reported as a positive value.
     *
     * @param[in] handle Pointer to the tachometer handle (must not be NULL)
     * @param[out] value Pointer to receive the RPM value (must not be NULL)
     *
     * @return ESP_OK on success
     * @return ESP_ERR_INVALID_ARG if `handle` or `value` is NULL
     *
     * @note This function is non-blocking and returns the most recent value
     *       without accessing the PCNT peripheral directly.
     */
    esp_err_t zh_tachometer_get(zh_tachometer_handle_t **handle, uint16_t *value);

#ifdef __cplusplus
}
#endif