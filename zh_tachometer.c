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

static esp_err_t _zh_tachometer_validate_config(const zh_tachometer_init_config_t *config);
static esp_err_t _zh_tachometer_pcnt_init(const zh_tachometer_init_config_t *config, zh_tachometer_handle_t *handle);
static esp_err_t _zh_tachometer_timer_init(zh_tachometer_handle_t *handle);
static void _zh_tachometer_timer_on_alarm_cb(void *arg);

esp_err_t zh_tachometer_init(const zh_tachometer_init_config_t *config, zh_tachometer_handle_t *handle) // -V2008
{
    ZH_LOGI("Tachometer initialization started.");
    ZH_ERROR_CHECK(config != NULL && handle != NULL, ESP_ERR_INVALID_ARG, NULL, "Tachometer initialization failed. Invalid argument.");
    ZH_ERROR_CHECK(handle->is_initialized == false, ESP_ERR_INVALID_STATE, NULL, "Tachometer initialization failed. Tachometer is already initialized.");
    esp_err_t err = _zh_tachometer_validate_config(config);
    ZH_ERROR_CHECK(err == ESP_OK, err, NULL, "Tachometer initialization failed. Initial configuration check failed.");
    err = _zh_tachometer_timer_init(handle);
    ZH_ERROR_CHECK(err == ESP_OK, err, NULL, "Tachometer initialization failed. Timer initialization failed.");
    err = _zh_tachometer_pcnt_init(config, handle);
    ZH_ERROR_CHECK(err == ESP_OK, err, esp_timer_stop(handle->esp_timer_handle); esp_timer_delete(handle->esp_timer_handle), "Tachometer initialization failed. PCNT initialization failed.");
    handle->encoder_pulses = config->encoder_pulses;
    handle->is_initialized = true;
    ZH_LOGI("Tachometer initialization completed successfully.");
    return ESP_OK;
}

esp_err_t zh_tachometer_deinit(zh_tachometer_handle_t *handle)
{
    ZH_LOGI("Tachometer deinitialization started.");
    ZH_ERROR_CHECK(handle != NULL, ESP_ERR_INVALID_ARG, NULL, "Tachometer deinitialization failed. Invalid argument.");
    ZH_ERROR_CHECK(handle->is_initialized == true, ESP_FAIL, NULL, "Tachometer deinitialization failed. Tachometer not initialized.");
    pcnt_unit_stop(handle->pcnt_unit_handle);
    pcnt_unit_disable(handle->pcnt_unit_handle);
    pcnt_unit_remove_watch_point(handle->pcnt_unit_handle, 32767);
    pcnt_unit_remove_watch_point(handle->pcnt_unit_handle, -32767);
    pcnt_del_channel(handle->pcnt_channel_a_handle);
    pcnt_del_channel(handle->pcnt_channel_b_handle);
    pcnt_del_unit(handle->pcnt_unit_handle);
    esp_timer_stop(handle->esp_timer_handle);
    esp_timer_delete(handle->esp_timer_handle);
    handle->is_initialized = false;
    ZH_LOGI("Tachometer deinitialization completed successfully.");
    return ESP_OK;
}

esp_err_t zh_tachometer_get(const zh_tachometer_handle_t *handle, uint16_t *value)
{
    ZH_LOGI("Tachometer get position started.");
    ZH_ERROR_CHECK(handle != NULL && value != NULL, ESP_ERR_INVALID_ARG, NULL, "Tachometer get position failed. Invalid argument.");
    ZH_ERROR_CHECK(handle->is_initialized == true, ESP_FAIL, NULL, "Tachometer get position failed. Tachometer not initialized.");
    *value = handle->value;
    ZH_LOGI("Tachometer get position completed successfully.");
    return ESP_OK;
}

static esp_err_t _zh_tachometer_validate_config(const zh_tachometer_init_config_t *config)
{
    ZH_ERROR_CHECK(config->encoder_pulses > 0, ESP_ERR_INVALID_ARG, NULL, "Invalid encoder pulses.");
    return ESP_OK;
}

static esp_err_t _zh_tachometer_pcnt_init(const zh_tachometer_init_config_t *config, zh_tachometer_handle_t *handle) // -V2008
{
    ZH_ERROR_CHECK(config->a_gpio_number < GPIO_NUM_MAX && config->b_gpio_number < GPIO_NUM_MAX, ESP_ERR_INVALID_ARG, NULL, "Invalid GPIO number.")
    ZH_ERROR_CHECK(config->a_gpio_number != config->b_gpio_number, ESP_ERR_INVALID_ARG, NULL, "Encoder A and B GPIO is same.")
    pcnt_unit_config_t pcnt_unit_config = {
        .high_limit = 32767,
        .low_limit = -32767,
        .flags.accum_count = true,
    };
    pcnt_unit_handle_t pcnt_unit_handle = NULL;
    esp_err_t err = pcnt_new_unit(&pcnt_unit_config, &pcnt_unit_handle);
    ZH_ERROR_CHECK(err == ESP_OK, err, NULL, "PCNT initialization failed.");
    pcnt_glitch_filter_config_t pcnt_glitch_filter_config = {
        .max_glitch_ns = 1000,
    };
    err = pcnt_unit_set_glitch_filter(pcnt_unit_handle, &pcnt_glitch_filter_config);
    ZH_ERROR_CHECK(err == ESP_OK, err, pcnt_del_unit(pcnt_unit_handle), "PCNT initialization failed.");
    pcnt_chan_config_t pcnt_chan_a_config = {
        .edge_gpio_num = config->a_gpio_number,
        .level_gpio_num = config->b_gpio_number,
    };
    pcnt_channel_handle_t pcnt_channel_a_handle = NULL;
    err = pcnt_new_channel(pcnt_unit_handle, &pcnt_chan_a_config, &pcnt_channel_a_handle);
    ZH_ERROR_CHECK(err == ESP_OK, err, pcnt_del_unit(pcnt_unit_handle), "PCNT initialization failed.");
    pcnt_chan_config_t pcnt_chan_b_config = {
        .edge_gpio_num = config->b_gpio_number,
        .level_gpio_num = config->a_gpio_number,
    };
    pcnt_channel_handle_t pcnt_channel_b_handle = NULL;
    err = pcnt_new_channel(pcnt_unit_handle, &pcnt_chan_b_config, &pcnt_channel_b_handle);
    ZH_ERROR_CHECK(err == ESP_OK, err, pcnt_del_channel(pcnt_channel_a_handle); pcnt_del_unit(pcnt_unit_handle), "PCNT initialization failed.");
    err = pcnt_channel_set_edge_action(pcnt_channel_a_handle, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD);
    ZH_ERROR_CHECK(err == ESP_OK, err, pcnt_del_channel(pcnt_channel_a_handle); pcnt_del_channel(pcnt_channel_b_handle); pcnt_del_unit(pcnt_unit_handle), "PCNT initialization failed.");
    err = pcnt_channel_set_level_action(pcnt_channel_a_handle, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_HOLD);
    ZH_ERROR_CHECK(err == ESP_OK, err, pcnt_del_channel(pcnt_channel_a_handle); pcnt_del_channel(pcnt_channel_b_handle); pcnt_del_unit(pcnt_unit_handle), "PCNT initialization failed.");
    err = pcnt_channel_set_edge_action(pcnt_channel_b_handle, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD);
    ZH_ERROR_CHECK(err == ESP_OK, err, pcnt_del_channel(pcnt_channel_a_handle); pcnt_del_channel(pcnt_channel_b_handle); pcnt_del_unit(pcnt_unit_handle), "PCNT initialization failed.");
    err = pcnt_channel_set_level_action(pcnt_channel_b_handle, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_HOLD);
    ZH_ERROR_CHECK(err == ESP_OK, err, pcnt_del_channel(pcnt_channel_a_handle); pcnt_del_channel(pcnt_channel_b_handle); pcnt_del_unit(pcnt_unit_handle), "PCNT initialization failed.");
    err = pcnt_unit_add_watch_point(pcnt_unit_handle, 32767);
    ZH_ERROR_CHECK(err == ESP_OK, err, pcnt_del_channel(pcnt_channel_a_handle); pcnt_del_channel(pcnt_channel_b_handle); pcnt_del_unit(pcnt_unit_handle), "PCNT initialization failed.");
    err = pcnt_unit_add_watch_point(pcnt_unit_handle, -32767);
    ZH_ERROR_CHECK(err == ESP_OK, err, pcnt_unit_remove_watch_point(pcnt_unit_handle, 32767); pcnt_del_channel(pcnt_channel_a_handle); pcnt_del_channel(pcnt_channel_b_handle);
                   pcnt_del_unit(pcnt_unit_handle), "PCNT initialization failed.");
    err = pcnt_unit_enable(pcnt_unit_handle);
    ZH_ERROR_CHECK(err == ESP_OK, err, pcnt_unit_remove_watch_point(pcnt_unit_handle, 32767); pcnt_unit_remove_watch_point(pcnt_unit_handle, -32767); pcnt_del_channel(pcnt_channel_a_handle);
                   pcnt_del_channel(pcnt_channel_b_handle); pcnt_del_unit(pcnt_unit_handle), "PCNT initialization failed.");
    err = pcnt_unit_clear_count(pcnt_unit_handle);
    ZH_ERROR_CHECK(err == ESP_OK, err, pcnt_unit_disable(pcnt_unit_handle); pcnt_unit_remove_watch_point(pcnt_unit_handle, 32767); pcnt_unit_remove_watch_point(pcnt_unit_handle, -32767);
                   pcnt_del_channel(pcnt_channel_a_handle); pcnt_del_channel(pcnt_channel_b_handle); pcnt_del_unit(pcnt_unit_handle), "PCNT initialization failed.");
    err = pcnt_unit_start(pcnt_unit_handle);
    ZH_ERROR_CHECK(err == ESP_OK, err, pcnt_unit_disable(pcnt_unit_handle); pcnt_unit_remove_watch_point(pcnt_unit_handle, 32767); pcnt_unit_remove_watch_point(pcnt_unit_handle, -32767);
                   pcnt_del_channel(pcnt_channel_a_handle); pcnt_del_channel(pcnt_channel_b_handle); pcnt_del_unit(pcnt_unit_handle), "PCNT initialization failed.");
    if (config->pullup == false)
    {
        gpio_pullup_dis((gpio_num_t)config->a_gpio_number);
        gpio_pullup_dis((gpio_num_t)config->b_gpio_number);
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
    esp_err_t err = esp_timer_create(&timer_args, &handle->esp_timer_handle);
    ZH_ERROR_CHECK(err == ESP_OK, err, NULL, "Timer initialization failed.");
    err = esp_timer_start_periodic(handle->esp_timer_handle, 10000);
    ZH_ERROR_CHECK(err == ESP_OK, err, esp_timer_delete(handle->esp_timer_handle), "Timer initialization failed.");
    return ESP_OK;
}

static void IRAM_ATTR _zh_tachometer_timer_on_alarm_cb(void *arg)
{
    zh_tachometer_handle_t *handle = (zh_tachometer_handle_t *)arg;
    int pcnt_count = 0;
    pcnt_unit_get_count(handle->pcnt_unit_handle, &pcnt_count);
    pcnt_unit_clear_count(handle->pcnt_unit_handle);
    float value_temp = ((pcnt_count * 100.0) / handle->encoder_pulses) * 60;
    handle->value = (value_temp < 0) ? -(uint16_t)value_temp : (uint16_t)value_temp; // -V2004
}