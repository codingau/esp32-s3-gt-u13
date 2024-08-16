/**
 * @brief   GPIO 初始化，执行模块。
 *
 * @author  nyx
 * @date    2024-07-18
 */
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "driver/gpio.h"

#include "app_config.h"

 /**
 * @brief 日志 TAG。
 */
static const char* TAG = "app_gpio";

/**
 * @brief GPIO 操作函数，改变返回 1，未改变返回 0。
 * @param level
 */
int app_gpio_set_level(gpio_num_t gpio_num, uint32_t level) {
    int cur_level = gpio_get_level(gpio_num);
    if (cur_level == level) {
        return 0;
    } else {
        esp_err_t ret = gpio_set_level(gpio_num, level);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "------ GPIO 电平状态改变: %lu", level);
            return 1;
        } else {
            return 0;
        }
    }
}

/**
 * @brief 返回 GPIO 电平字符串。
 * @param buffer
 * @param size
 */
void app_gpio_get_string(char* buffer, size_t size) {
    int level = gpio_get_level(APP_GPIO_NUM_BLE);
    int length = snprintf(buffer, size, "%d%d", APP_GPIO_NUM_BLE, level);
    if (length >= size) {
        ESP_LOGE(TAG, "------ GPIO 电平字符串被截断。");
    }
}

/**
 * @brief 初始化函数。
 * @return
 */
esp_err_t app_gpio_init(void) {
    gpio_config_t gpio_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT_OUTPUT,
        .pin_bit_mask = APP_GPIO_PIN_BIT_MASK,
        .pull_down_en = 0,
        .pull_up_en = 0
    };
    esp_err_t gpio_ret = gpio_config(&gpio_conf);
    return gpio_ret;
}
