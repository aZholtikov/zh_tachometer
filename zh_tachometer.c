#include "zh_tachometer.h"

#define TAG "zh_tachometer"

#define ZH_LOGI(msg, ...) ESP_LOGI(TAG, msg, ##__VA_ARGS__)
#define ZH_LOGE(msg, err, ...) ESP_LOGE(TAG, "[%s:%d:%s] " msg, __FILE__, __LINE__, esp_err_to_name(err), ##__VA_ARGS__)

#define ZH_ERROR_CHECK(cond, err, cleanup, msg, ...) \
    if (!(cond))                                     \
    {                                                \
        ZH_LOGE(msg, err, ##__VA_ARGS__);            \
        cleanup;                                     \
        return err;                                  \
    }

#define ZH_ERROR_CHECK_VOID(cond, cleanup, msg, ...) \
    if (!(cond))                                     \
    {                                                \
        ZH_LOGE(msg, ESP_FAIL, ##__VA_ARGS__);       \
        cleanup;                                     \
        return;                                      \
    }

/**
 * @brief Internal handle structure for the tachometer driver.
 *
 * Holds references to the PCNT unit and channels, the ESP-Timer used for
 * periodic RPM sampling, the last computed RPM value, and the encoder's
 * pulses-per-revolution configuration.
 *
 * @note This structure is opaque to callers and must only be accessed
 *       through the public API functions.
 */
struct _zh_tachometer_handle_t
{
    pcnt_unit_handle_t pcnt_unit_handle;         /*!< PCNT unit handle for quadrature decoding */
    pcnt_channel_handle_t pcnt_channel_a_handle; /*!< PCNT channel A handle (edge detector) */
    pcnt_channel_handle_t pcnt_channel_b_handle; /*!< PCNT channel B handle (edge detector) */
    esp_timer_handle_t esp_timer_handle;         /*!< ESP-Timer handle for 100 Hz RPM sampling */
    uint16_t value;                              /*!< Last computed RPM value (absolute) */
    uint16_t encoder_pulses;                     /*!< Pulses per one full rotation */
};

/**
 * @brief Validate the initialization configuration.
 *
 * Checks that `encoder_pulses` is greater than zero.
 *
 * @param config Pointer to the initialization configuration
 *
 * @return ESP_OK on valid configuration
 * @return ESP_ERR_INVALID_ARG if `encoder_pulses` is zero or less
 */
static esp_err_t _zh_tachometer_validate_config(const zh_tachometer_init_config_t *config);

/**
 * @brief Initialize the PCNT peripheral in quadrature decoder mode.
 *
 * Configures the PCNT unit with accumulation mode, glitch filter (1000 ns),
 * two channels for encoder phases A and B, edge/level actions for direction
 * counting, and watch points at ±32767. Starts the unit and clears the
 * counter. Optionally disables GPIO pull-ups if configured.
 *
 * @param config Pointer to the initialization configuration
 * @param handle Pointer to the initialized tachometer handle
 *
 * @return ESP_OK on successful PCNT initialization
 * @return ESP_ERR_INVALID_ARG if GPIO numbers are invalid or identical
 * @return ESP_FAIL on any PCNT configuration failure
 */
static esp_err_t _zh_tachometer_pcnt_init(const zh_tachometer_init_config_t *config, zh_tachometer_handle_t *handle);

/**
 * @brief Initialize the ESP-Timer for periodic RPM sampling.
 *
 * Creates a periodic timer that fires every 10 ms (100 Hz) and invokes
 * the alarm callback to read and convert the PCNT count to RPM.
 *
 * @param handle Pointer to the initialized tachometer handle
 *
 * @return ESP_OK on successful timer initialization
 * @return ESP_FAIL on timer creation or start failure
 */
static esp_err_t _zh_tachometer_timer_init(zh_tachometer_handle_t *handle);

/**
 * @brief ESP-Timer alarm callback for RPM sampling.
 *
 * Executed every 10 ms from ISR context. Reads the accumulated PCNT count,
 * clears it, and converts it to RPM using the formula:
 *
 *     RPM = (count * 100.0 / pulses_per_rev) * 60
 *
 * The result is stored as an absolute value (direction discarded).
 *
 * @param arg Pointer to the tachometer handle
 */
static void _zh_tachometer_timer_on_alarm_cb(void *arg);

esp_err_t zh_tachometer_init(const zh_tachometer_init_config_t *config, zh_tachometer_handle_t **handle)
{
    ZH_LOGI("Tachometer initialization started.");
    ZH_ERROR_CHECK(config != NULL && handle != NULL, ESP_ERR_INVALID_ARG, NULL, "Tachometer initialization failed. Invalid argument.");
    ZH_ERROR_CHECK(*handle == NULL, ESP_ERR_INVALID_STATE, NULL, "Tachometer initialization failed. Tachometer is already initialized.");
    ZH_ERROR_CHECK(_zh_tachometer_validate_config(config) == ESP_OK, ESP_FAIL, NULL, "Tachometer initialization failed. Initial configuration check failed.");
    *handle = heap_caps_calloc(1, sizeof(zh_tachometer_handle_t), MALLOC_CAP_8BIT);
    ZH_ERROR_CHECK(*handle != NULL, ESP_ERR_NO_MEM, NULL, "Tachometer initialization failed. Failed to allocate tachometer handle.");
    ZH_ERROR_CHECK(_zh_tachometer_timer_init(*handle) == ESP_OK, ESP_FAIL, heap_caps_free(*handle); *handle = NULL, "Tachometer initialization failed. Timer initialization failed.");
    ZH_ERROR_CHECK(_zh_tachometer_pcnt_init(config, *handle) == ESP_OK, ESP_FAIL,
                   {ZH_ERROR_CHECK(esp_timer_stop((*handle)->esp_timer_handle) == ESP_OK, ESP_FAIL, NULL, "Timer stop fail.")};
                   {ZH_ERROR_CHECK(esp_timer_delete((*handle)->esp_timer_handle) == ESP_OK, ESP_FAIL, NULL, "Timer delete fail.")};
                   heap_caps_free(*handle); *handle = NULL, "Tachometer initialization failed. PCNT initialization failed.");
    (*handle)->encoder_pulses = config->encoder_pulses;
    ZH_LOGI("Tachometer initialization completed successfully.");
    return ESP_OK;
}

esp_err_t zh_tachometer_deinit(zh_tachometer_handle_t **handle)
{
    ZH_LOGI("Tachometer deinitialization started.");
    ZH_ERROR_CHECK(handle != NULL && *handle != NULL, ESP_ERR_INVALID_ARG, NULL, "Tachometer deinitialization failed. Invalid argument.");
    ZH_ERROR_CHECK(pcnt_unit_stop((*handle)->pcnt_unit_handle) == ESP_OK, ESP_FAIL, NULL, "Tachometer deinitialization failed. PCNT unit stop fail.");
    ZH_ERROR_CHECK(pcnt_unit_disable((*handle)->pcnt_unit_handle) == ESP_OK, ESP_FAIL, NULL, "Tachometer deinitialization failed. PCNT unit disable fail.");
    ZH_ERROR_CHECK(pcnt_unit_remove_watch_point((*handle)->pcnt_unit_handle, 32767) == ESP_OK, ESP_FAIL, NULL, "Tachometer deinitialization failed. PCNT unit remove watch point fail.");
    ZH_ERROR_CHECK(pcnt_unit_remove_watch_point((*handle)->pcnt_unit_handle, -32767) == ESP_OK, ESP_FAIL, NULL, "Tachometer deinitialization failed. PCNT unit remove watch point fail.");
    ZH_ERROR_CHECK(pcnt_del_channel((*handle)->pcnt_channel_a_handle) == ESP_OK, ESP_FAIL, NULL, "Tachometer deinitialization failed. PCNT delete channel fail.");
    ZH_ERROR_CHECK(pcnt_del_channel((*handle)->pcnt_channel_b_handle) == ESP_OK, ESP_FAIL, NULL, "Tachometer deinitialization failed. PCNT delete channel fail.");
    ZH_ERROR_CHECK(pcnt_del_unit((*handle)->pcnt_unit_handle) == ESP_OK, ESP_FAIL, NULL, "Tachometer deinitialization failed. PCNT delete unit fail.");
    ZH_ERROR_CHECK(esp_timer_stop((*handle)->esp_timer_handle) == ESP_OK, ESP_FAIL, NULL, "Tachometer deinitialization failed. Timer stop fail.");
    ZH_ERROR_CHECK(esp_timer_delete((*handle)->esp_timer_handle) == ESP_OK, ESP_FAIL, NULL, "Tachometer deinitialization failed. Timer delete fail.");
    heap_caps_free(*handle);
    *handle = NULL;
    ZH_LOGI("Tachometer deinitialization completed successfully.");
    return ESP_OK;
}

esp_err_t zh_tachometer_get(zh_tachometer_handle_t **handle, uint16_t *value)
{
    ZH_LOGI("Tachometer get position started.");
    ZH_ERROR_CHECK(handle != NULL && *handle != NULL && value != NULL, ESP_ERR_INVALID_ARG, NULL, "Tachometer get position failed. Invalid argument.");
    *value = (*handle)->value;
    ZH_LOGI("Tachometer get position completed successfully.");
    return ESP_OK;
}

static esp_err_t _zh_tachometer_validate_config(const zh_tachometer_init_config_t *config)
{
    ZH_ERROR_CHECK(config->encoder_pulses > 0, ESP_ERR_INVALID_ARG, NULL, "Invalid encoder pulses.");
    return ESP_OK;
}

static esp_err_t _zh_tachometer_pcnt_init(const zh_tachometer_init_config_t *config, zh_tachometer_handle_t *handle)
{
    ZH_ERROR_CHECK(config->a_gpio_number < GPIO_NUM_MAX && config->b_gpio_number < GPIO_NUM_MAX, ESP_ERR_INVALID_ARG, NULL, "Invalid GPIO number.")
    ZH_ERROR_CHECK(config->a_gpio_number != config->b_gpio_number, ESP_ERR_INVALID_ARG, NULL, "Encoder A and B GPIO is same.")
    pcnt_unit_config_t pcnt_unit_config = {
        .high_limit = 32767,
        .low_limit = -32767,
        .flags.accum_count = true,
    };
    pcnt_unit_handle_t pcnt_unit_handle = NULL;
    ZH_ERROR_CHECK(pcnt_new_unit(&pcnt_unit_config, &pcnt_unit_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT initialization failed.");
    pcnt_glitch_filter_config_t pcnt_glitch_filter_config = {
        .max_glitch_ns = 1000,
    };
    ZH_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit_handle, &pcnt_glitch_filter_config) == ESP_OK, ESP_FAIL,
                   {ZH_ERROR_CHECK(pcnt_del_unit(pcnt_unit_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete unit fail.")}, "PCNT initialization failed.");
    pcnt_chan_config_t pcnt_chan_a_config = {
        .edge_gpio_num = config->a_gpio_number,
        .level_gpio_num = config->b_gpio_number,
    };
    pcnt_channel_handle_t pcnt_channel_a_handle = NULL;
    ZH_ERROR_CHECK(pcnt_new_channel(pcnt_unit_handle, &pcnt_chan_a_config, &pcnt_channel_a_handle) == ESP_OK, ESP_FAIL,
                   {ZH_ERROR_CHECK(pcnt_del_unit(pcnt_unit_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete unit fail.")}, "PCNT initialization failed.");
    pcnt_chan_config_t pcnt_chan_b_config = {
        .edge_gpio_num = config->b_gpio_number,
        .level_gpio_num = config->a_gpio_number,
    };
    pcnt_channel_handle_t pcnt_channel_b_handle = NULL;
    ZH_ERROR_CHECK(pcnt_new_channel(pcnt_unit_handle, &pcnt_chan_b_config, &pcnt_channel_b_handle) == ESP_OK, ESP_FAIL,
                   {ZH_ERROR_CHECK(pcnt_del_channel(pcnt_channel_a_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete channel fail.")};
                   {ZH_ERROR_CHECK(pcnt_del_unit(pcnt_unit_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete unit fail.")}, "PCNT initialization failed.");
    ZH_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_channel_a_handle, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD) == ESP_OK, ESP_FAIL,
                   {ZH_ERROR_CHECK(pcnt_del_channel(pcnt_channel_a_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete channel fail.")};
                   {ZH_ERROR_CHECK(pcnt_del_channel(pcnt_channel_b_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete channel fail.")};
                   {ZH_ERROR_CHECK(pcnt_del_unit(pcnt_unit_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete unit fail.")}, "PCNT initialization failed.");
    ZH_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_channel_a_handle, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_HOLD) == ESP_OK, ESP_FAIL,
                   {ZH_ERROR_CHECK(pcnt_del_channel(pcnt_channel_a_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete channel fail.")};
                   {ZH_ERROR_CHECK(pcnt_del_channel(pcnt_channel_b_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete channel fail.")};
                   {ZH_ERROR_CHECK(pcnt_del_unit(pcnt_unit_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete unit fail.")}, "PCNT initialization failed.");
    ZH_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_channel_b_handle, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD) == ESP_OK, ESP_FAIL,
                   {ZH_ERROR_CHECK(pcnt_del_channel(pcnt_channel_a_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete channel fail.")};
                   {ZH_ERROR_CHECK(pcnt_del_channel(pcnt_channel_b_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete channel fail.")};
                   {ZH_ERROR_CHECK(pcnt_del_unit(pcnt_unit_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete unit fail.")}, "PCNT initialization failed.");
    ZH_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_channel_b_handle, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_HOLD) == ESP_OK, ESP_FAIL,
                   {ZH_ERROR_CHECK(pcnt_del_channel(pcnt_channel_a_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete channel fail.")};
                   {ZH_ERROR_CHECK(pcnt_del_channel(pcnt_channel_b_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete channel fail.")};
                   {ZH_ERROR_CHECK(pcnt_del_unit(pcnt_unit_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete unit fail.")}, "PCNT initialization failed.");
    ZH_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit_handle, 32767) == ESP_OK, ESP_FAIL,
                   {ZH_ERROR_CHECK(pcnt_del_channel(pcnt_channel_a_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete channel fail.")};
                   {ZH_ERROR_CHECK(pcnt_del_channel(pcnt_channel_b_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete channel fail.")};
                   {ZH_ERROR_CHECK(pcnt_del_unit(pcnt_unit_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete unit fail.")}, "PCNT initialization failed.");
    ZH_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit_handle, -32767) == ESP_OK, ESP_FAIL,
                   {ZH_ERROR_CHECK(pcnt_unit_remove_watch_point(pcnt_unit_handle, 32767) == ESP_OK, ESP_FAIL, NULL, "PCNT unit remove watch point fail.")};
                   {ZH_ERROR_CHECK(pcnt_del_channel(pcnt_channel_a_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete channel fail.")};
                   {ZH_ERROR_CHECK(pcnt_del_channel(pcnt_channel_b_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete channel fail.")};
                   {ZH_ERROR_CHECK(pcnt_del_unit(pcnt_unit_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete unit fail.")}, "PCNT initialization failed.");
    ZH_ERROR_CHECK(pcnt_unit_enable(pcnt_unit_handle) == ESP_OK, ESP_FAIL,
                   {ZH_ERROR_CHECK(pcnt_unit_remove_watch_point(pcnt_unit_handle, 32767) == ESP_OK, ESP_FAIL, NULL, "PCNT unit remove watch point fail.")};
                   {ZH_ERROR_CHECK(pcnt_unit_remove_watch_point(pcnt_unit_handle, -32767) == ESP_OK, ESP_FAIL, NULL, "PCNT unit remove watch point fail.")};
                   {ZH_ERROR_CHECK(pcnt_del_channel(pcnt_channel_a_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete channel fail.")};
                   {ZH_ERROR_CHECK(pcnt_del_channel(pcnt_channel_b_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete channel fail.")};
                   {ZH_ERROR_CHECK(pcnt_del_unit(pcnt_unit_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete unit fail.")}, "PCNT initialization failed.");
    ZH_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit_handle) == ESP_OK, ESP_FAIL,
                   {ZH_ERROR_CHECK(pcnt_unit_disable(pcnt_unit_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT unit disable fail.")};
                   {ZH_ERROR_CHECK(pcnt_unit_remove_watch_point(pcnt_unit_handle, 32767) == ESP_OK, ESP_FAIL, NULL, "PCNT unit remove watch point fail.")};
                   {ZH_ERROR_CHECK(pcnt_unit_remove_watch_point(pcnt_unit_handle, -32767) == ESP_OK, ESP_FAIL, NULL, "PCNT unit remove watch point fail.")};
                   {ZH_ERROR_CHECK(pcnt_del_channel(pcnt_channel_a_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete channel fail.")};
                   {ZH_ERROR_CHECK(pcnt_del_channel(pcnt_channel_b_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete channel fail.")};
                   {ZH_ERROR_CHECK(pcnt_del_unit(pcnt_unit_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete unit fail.")}, "PCNT initialization failed.");
    ZH_ERROR_CHECK(pcnt_unit_start(pcnt_unit_handle) == ESP_OK, ESP_FAIL,
                   {ZH_ERROR_CHECK(pcnt_unit_disable(pcnt_unit_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT unit disable fail.")};
                   {ZH_ERROR_CHECK(pcnt_unit_remove_watch_point(pcnt_unit_handle, 32767) == ESP_OK, ESP_FAIL, NULL, "PCNT unit remove watch point fail.")};
                   {ZH_ERROR_CHECK(pcnt_unit_remove_watch_point(pcnt_unit_handle, -32767) == ESP_OK, ESP_FAIL, NULL, "PCNT unit remove watch point fail.")};
                   {ZH_ERROR_CHECK(pcnt_del_channel(pcnt_channel_a_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete channel fail.")};
                   {ZH_ERROR_CHECK(pcnt_del_channel(pcnt_channel_b_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete channel fail.")};
                   {ZH_ERROR_CHECK(pcnt_del_unit(pcnt_unit_handle) == ESP_OK, ESP_FAIL, NULL, "PCNT delete unit fail.")}, "PCNT initialization failed.");
    if (config->pullup == false)
    {
        ZH_ERROR_CHECK(gpio_pullup_dis((gpio_num_t)config->a_gpio_number) == ESP_OK, ESP_FAIL, NULL, "GPIO pullup disable fail.");
        ZH_ERROR_CHECK(gpio_pullup_dis((gpio_num_t)config->b_gpio_number) == ESP_OK, ESP_FAIL, NULL, "GPIO pullup disable fail.");
    }
    handle->pcnt_unit_handle = pcnt_unit_handle;
    handle->pcnt_channel_a_handle = pcnt_channel_a_handle;
    handle->pcnt_channel_b_handle = pcnt_channel_b_handle;
    return ESP_OK;
}

static esp_err_t _zh_tachometer_timer_init(zh_tachometer_handle_t *handle)
{
    const esp_timer_create_args_t timer_args = {
        .callback = &_zh_tachometer_timer_on_alarm_cb,
        .arg = handle,
    };
    ZH_ERROR_CHECK(esp_timer_create(&timer_args, &handle->esp_timer_handle) == ESP_OK, ESP_FAIL, NULL, "Timer initialization failed.");
    ZH_ERROR_CHECK(esp_timer_start_periodic(handle->esp_timer_handle, 10000) == ESP_OK, ESP_FAIL,
                   {ZH_ERROR_CHECK(esp_timer_delete(handle->esp_timer_handle) == ESP_OK, ESP_FAIL, NULL, "Timer delete fail.")}, "Timer initialization failed.");
    return ESP_OK;
}

static void IRAM_ATTR _zh_tachometer_timer_on_alarm_cb(void *arg)
{
    zh_tachometer_handle_t *handle = (zh_tachometer_handle_t *)arg;
    int pcnt_count = 0;
    ZH_ERROR_CHECK_VOID(pcnt_unit_get_count(handle->pcnt_unit_handle, &pcnt_count) == ESP_OK, NULL, "PCNT internal error.");
    ZH_ERROR_CHECK_VOID(pcnt_unit_clear_count(handle->pcnt_unit_handle) == ESP_OK, NULL, "PCNT internal error.");
    float value_temp = ((pcnt_count * 100.0) / handle->encoder_pulses) * 60;
    handle->value = (uint16_t)fabs(value_temp);
}