/**
 * @file zh_tachometer.h
 */

#pragma once

#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/**
 * @brief Tachometer initial default values.
 */
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
     * @brief Structure for initial initialization of tachometer.
     */
    typedef struct
    {
        uint8_t a_gpio_number;   /*!< Encoder A GPIO number. */
        uint8_t b_gpio_number;   /*!< Encoder B GPIO number. */
        bool pullup;             /*!< Pullup GPIO enable/disable. */
        uint16_t encoder_pulses; /*!< Number of pulses per one rotation. */
    } zh_tachometer_init_config_t;

    /**
     * @brief Tachometer handle.
     */
    typedef struct
    {
        pcnt_unit_handle_t pcnt_unit_handle;         /*!< Tachometer unique pcnt unit handle. */
        pcnt_channel_handle_t pcnt_channel_a_handle; /*!< Tachometer unique pcnt channel handle. */
        pcnt_channel_handle_t pcnt_channel_b_handle; /*!< Tachometer unique pcnt channel handle. */
        esp_timer_handle_t esp_timer_handle;         /*!< Tachometer unique timer handle. */
        uint16_t value;                              /*!< Tachometer value. */
        uint16_t encoder_pulses;                     /*!< Number of pulses per one rotation. */
        bool is_initialized;                         /*!< Tachometer initialization flag. */
    } zh_tachometer_handle_t;

    /**
     * @brief Initialize tachometer.
     *
     * @param[in] config Pointer to tachometer initialized configuration structure. Can point to a temporary variable.
     * @param[out] handle Pointer to unique tachometer handle.
     *
     * @note Before initialize the tachometer recommend initialize zh_tachometer_init_config_t structure with default values.
     *
     * @code zh_tachometer_init_config_t config = ZH_TACHOMETER_INIT_CONFIG_DEFAULT() @endcode
     *
     * @return ESP_OK if success or an error code otherwise.
     */
    esp_err_t zh_tachometer_init(const zh_tachometer_init_config_t *config, zh_tachometer_handle_t *handle);

    /**
     * @brief Deinitialize tachometer.
     *
     * @param[in, out] handle Pointer to unique tachometer handle.
     *
     * @return ESP_OK if success or an error code otherwise.
     */
    esp_err_t zh_tachometer_deinit(zh_tachometer_handle_t *handle);

    /**
     * @brief Get tachometer value.
     *
     * @param[in] handle Pointer to unique tachometer handle.
     * @param[out] value Tachometer value.
     *
     * @return ESP_OK if success or an error code otherwise.
     */
    esp_err_t zh_tachometer_get(const zh_tachometer_handle_t *handle, uint16_t *value);

#ifdef __cplusplus
}
#endif