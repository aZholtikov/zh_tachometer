# ESP32 ESP-IDF component for tachometer (via rotary optical encoder)

## Tested on

1. [ESP32 ESP-IDF v5.5.3](https://docs.espressif.com/projects/esp-idf/en/v5.5.3/esp32/index.html)

## SAST Tools

[PVS-Studio](https://pvs-studio.com/pvs-studio/?utm_source=website&utm_medium=github&utm_campaign=open_source) - static analyzer for C, C++, C#, and Java code.

## Attention

1. For correct operation, please enable the following settings in the menuconfig:

```text
PCNT_CTRL_FUNC_IN_IRAM
PCNT_ISR_IRAM_SAF
```

## Using

In an existing project, run the following command to install the components:

```text
cd ../your_project/components
git clone https://github.com/aZholtikov/zh_tachometer
```

In the application, add the component:

```c
#include "zh_tachometer.h"
```

## Examples

```c
#include "zh_tachometer.h"

zh_tachometer_handle_t tachometer_handle = {0};

void app_main(void)
{
    esp_log_level_set("zh_tachometer", ESP_LOG_ERROR);
    zh_tachometer_init_config_t config = ZH_TACHOMETER_INIT_CONFIG_DEFAULT();
    config.a_gpio_number = GPIO_NUM_26;
    config.b_gpio_number = GPIO_NUM_27;
    config.encoder_pulses = 3600;
    zh_tachometer_init(&config, &tachometer_handle);
    for (;;)
    {
        uint16_t value = 0;
        zh_tachometer_get(&tachometer_handle, &value);
        printf("Tachometer value is %d rpm.\n", value);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
```
