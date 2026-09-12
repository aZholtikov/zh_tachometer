# ESP32 ESP-IDF component for tachometer (via rotary optical encoder)

## Features

1. Quadrature encoder support via ESP-IDF PCNT peripheral in quadrature decoder mode.
2. Automatic RPM calculation based on configurable pulses-per-revolution.
3. Periodic RPM sampling via ESP-Timer at 100 Hz (10 ms interval).
4. Configurable GPIO pull-up resistors for encoder A/B phases.
5. PCNT glitch filter (1000 ns) to reject spurious transitions.
6. Watch points at ±32767 for overflow detection.
7. Multiple tachometers supported — each requires its own PCNT unit and timer.

## Note

Enable the following settings in menuconfig:

```text
PCNT_CTRL_FUNC_IN_IRAM
PCNT_ISR_IRAM_SAF
```

## Using

In an existing project, run the following command to install the components:

```bash
cd ../your_project/components
git clone https://github.com/aZholtikov/zh_tachometer
```

In the application, add the component:

```c
#include "zh_tachometer.h"
```

## Example

```c
#include "zh_tachometer.h"

static zh_tachometer_handle_t *tachometer_handle = NULL;

void app_main(void)
{
    esp_log_level_set("zh_tachometer", ESP_LOG_ERROR);

    zh_tachometer_init_config_t config = ZH_TACHOMETER_INIT_CONFIG_DEFAULT();
    config.a_gpio_number = GPIO_NUM_26;
    config.b_gpio_number = GPIO_NUM_27;
    config.encoder_pulses = 3600;

    esp_err_t ret = zh_tachometer_init(&config, &tachometer_handle);
    if (ret != ESP_OK) {
        ESP_LOG_ERROR("zh_tachometer", "Initialization failed: %s", esp_err_to_name(ret));
        return;
    }

    for (;;) {
        uint16_t rpm = 0;
        ret = zh_tachometer_get(&tachometer_handle, &rpm);
        if (ret == ESP_OK) {
            printf("Tachometer value is %d rpm.\n", rpm);
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    zh_tachometer_deinit(&tachometer_handle);
}
```
