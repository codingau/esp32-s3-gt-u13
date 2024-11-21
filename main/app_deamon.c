/**
 * @brief   守护线程。
 *
 * @author  nyx
 * @date    2024-07-27
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <pthread.h>
#include <stdatomic.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "mqtt_client.h"

#include "app_config.h"
#include "app_ping.h"
#include "app_mqtt.h"
#include "app_gnss.h"
#include "app_main.h"

 /**
 * @brief 日志 TAG。
 */
static const char* TAG = "app_deamon";

/**
 * @brief APP 当前状态。0 = 启动期间，1 = 运行期间。
 */
int app_status = 0;

/**
 * @brief 网络状态的守护任务。
 * @param param
 */
static void app_deamon_network_task(void* param) {
    uint32_t count = 0;
    while (1) {
        if (count % 30 == 0) {
            ESP_LOGI(TAG, "------ app_deamon_network_task() 守护任务，执行次数：%lu，APP 状态：%d", count, app_status);
        }

        if (app_status == 1) {
            uint32_t cur_ts = esp_log_timestamp();
            uint32_t mqtt_last_ts = atomic_load(&app_mqtt_last_ts);
            if (cur_ts - mqtt_last_ts > 10000) {// 最后一次 mqtt 提交数据时间，大于 10 秒。

                esp_err_t ping_start_ret = app_ping_start();
                if (ping_start_ret != ESP_OK) {
                    vTaskDelay(pdMS_TO_TICKS(10000));// 等待 10 秒，减少 PING 次数。
                    int level = gpio_get_level(APP_GPIO_NUM_BLE);
                    if (level == 1) {
                        ESP_LOGE(TAG, "------ PING 函数执行结果：失败！蓝牙开关状态：打开，暂不重启。");
                    } else {
                        ESP_LOGE(TAG, "------ PING 函数执行结果：失败！执行：esp_restart()");
                        esp_restart();
                    }
                }
                int ping_ret = 0;
                for (int i = 0; i < 11; i++) {
                    vTaskDelay(pdMS_TO_TICKS(100));// 每个循环等待 100 毫秒。
                    ping_ret = atomic_load(&app_ping_ret);
                    if (ping_ret > 0) {// 如果有返回值。
                        break;
                    }
                    if (i == 10) {// 最后一次循环，还没有结果的情况，等同于超时。
                        ping_ret = -1;
                    }
                }
                vTaskDelay(pdMS_TO_TICKS(10000));// 等待 10 秒，减少 PING 次数。
                if (ping_ret == -1) {// 断网状态，直接检测信号。
                    int level = gpio_get_level(APP_GPIO_NUM_BLE);
                    if (level == 1) {
                        ESP_LOGE(TAG, "------ PING 函数执行结果：超时！蓝牙开关状态：打开，暂不重启。");
                    } else {
                        ESP_LOGE(TAG, "------ PING 函数执行结果：超时！执行：esp_restart()");
                        esp_restart();
                    }
                } else {
                    ESP_LOGI(TAG, "------ PING 函数执行结果：正常。返回 time 值：%d。", ping_ret);
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1000));// 每秒执行一次。
        count++;
    }
}

/**
 * @brief 主循环任务的守护任务。
 * @param param
 */
static void app_deamon_loop_task(void* param) {
    uint32_t count = 0;
    while (1) {
        ESP_LOGI(TAG, "------ app_deamon_loop_task() 守护任务，执行次数：%lu", count);

        uint32_t cur_ts = esp_log_timestamp();
        uint32_t loop_last_ts = atomic_load(&app_main_loop_last_ts);
        if (cur_ts > 86400000) {// 每天重启一次。
            ESP_LOGE(TAG, "------ app_deamon_loop_task() 定时重启。执行：esp_restart()");
            esp_restart();
        }
        if (cur_ts - loop_last_ts > 60000) {// 大于 60 秒。
            int level = gpio_get_level(APP_GPIO_NUM_BLE);
            if (level == 1) {
                ESP_LOGE(TAG, "------ app_main_loop_task() 执行超时！！！蓝牙开关状态：打开，暂不重启。");
            } else {
                ESP_LOGE(TAG, "------ app_main_loop_task() 执行超时！！！立即执行：esp_restart()");
                esp_restart();
            }
        }

        vTaskDelay(pdMS_TO_TICKS(30000));// 每 30 秒执行一次。
        count++;
    }
}

/**
 * @brief 初始化函数。
 * @return
 */
esp_err_t app_deamon_init(void) {
    xTaskCreate(app_deamon_loop_task, "app_dm_loop_task", 4096, NULL, 1, NULL);// 主循环任务守护任务，优先级 1。
    xTaskCreate(app_deamon_network_task, "app_dm_network_task", 4096, NULL, 2, NULL);// 网络状态守护任务，优先级 2。
    return ESP_OK;
}