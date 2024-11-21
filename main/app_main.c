/**
 * @brief   开发板主函数。
 *
 * @author  nyx
 * @date    2024-07-11
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdatomic.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "mqtt_client.h"
#include "nmea.h"

#include "app_led.h"
#include "app_deamon.h"
#include "app_sd.h"
#include "app_wifi.h"
#include "app_sntp.h"
#include "app_mqtt.h"
#include "app_gpio.h"
#include "app_ble.h"
#include "app_gnss.h"
#include "app_json.h"
#include "app_ping.h"
#include "app_main.h"
#include "app_config.h"

 /**
 * @brief 日志 TAG。
 */
static const char* TAG = "app_main";

/**
 * @brief 最近一次 LOOP 的时间戳。
 */
_Atomic uint32_t app_main_loop_last_ts = ATOMIC_VAR_INIT(0);

/**
 * @brief APP 主任务运行期间的数据。
 */
app_main_data_t app_main_data = {
    .dev_addr = "00:00:00:00:00:00",        // 初始全部为 0。
    .dev_time = "19700101000000000",        // 初始化为起始时间。
    .log_ts = 0,                            // 初始为 0。
    .ble_ts = 0,                            // 初始为 0。
    .gpios = {0},                           // GPIO 电平字符串。
    .gnss_time = "19700101000000000",       // 初始化为起始时间。
    .gnss_valid = false,                    // 有效性为 false。
    .sat = 0,                               // 初始卫星数为 0。
    .alt = 0.0,                             // 初始高度设为 0.0 米。
    .lat = 0.0,                             // 纬度。
    .lon = 0.0,                             // 经度。
    .spd = 0.0,                             // 速度。
    .trk = 0.0,                             // 航向角度。
    .f = 0,                                 // 初始化标记为 0。
};

/**
 * @brief 获取当前 UTC 时间字符串，并使用 ISO 8601 标准格式化字符串。
 * @param buffer
 * @param buffer_size
 */
void get_cur_utc_time(char* buffer, size_t buffer_size) {
    struct timeval tv;
    struct tm timeinfo;
    gettimeofday(&tv, NULL);// 获取当前时间，秒和微秒。
    gmtime_r(&tv.tv_sec, &timeinfo); // 转换数据格式，秒的部分。
    strftime(buffer, buffer_size - 1, "%Y%m%d%H%M%S", &timeinfo);// 格式化时间，精确到秒。当前的格式是：20240711024955
    int millisec = tv.tv_usec / 1000;// 计算毫秒。
    snprintf(buffer + strlen(buffer), buffer_size - strlen(buffer) - 1, "%03d", millisec);// 追加毫秒字符串。返回时间格式：20240711024955148
}

/**
 * @brief 获取 GNSS UTC 时间字符串，并使用 ISO 8601 标准格式化字符串。
 * @param buffer
 */
void get_gnss_utc_time(char* buffer, size_t buffer_size) {
    struct tm timeinfo = app_gnss_data.date_time;
    strftime(buffer, buffer_size, "%Y%m%d%H%M%S000", &timeinfo);// GNSS 时间没有毫秒数。
}

/**
 * @brief 循环任务。
 * @param
 */
void app_main_loop_task(void) {
    uint32_t esp_log_ts = esp_log_timestamp();
    atomic_store(&app_main_loop_last_ts, esp_log_ts);

    get_cur_utc_time(app_main_data.dev_time, sizeof(app_main_data.dev_time));// 设备时间。
    app_main_data.log_ts = esp_log_ts / 1000;// 系统启动以后的秒数。
    app_main_data.ble_ts = atomic_load(&app_ble_disc_ts) / 1000;// 最后一次扫描到蓝牙开关的秒数。
    app_gpio_get_string(app_main_data.gpios, sizeof(app_main_data.gpios));

    pthread_mutex_lock(&app_gnss_data.mutex);
    get_gnss_utc_time(app_main_data.gnss_time, sizeof(app_main_data.gnss_time));// GNSS 时间。
    app_main_data.gnss_valid = app_gnss_data.valid;// 有效性。
    app_main_data.sat = app_gnss_data.sat;// 卫星数。
    app_main_data.alt = app_gnss_data.alt;// 高度，默认单位：M。
    app_main_data.lat = app_gnss_data.lat;// 纬度。
    app_main_data.lon = app_gnss_data.lon;// 经度。
    app_main_data.spd = app_gnss_data.spd;// 速度，默认单位：节。
    app_main_data.trk = app_gnss_data.trk;// 航向角度。
    app_main_data.mag = app_gnss_data.mag;// 磁偏角度。
    pthread_mutex_unlock(&app_gnss_data.mutex);

    char json[512];
    app_json_serialize(json, sizeof(json), &app_main_data);

    // 如果有 MQTT，则 MQTT 推送到服务器。
    if (app_mqtt_5_client != NULL) {

        int pub_ret = app_mqtt_publish_msg(json);
        if (pub_ret >= 0) {// 推送成功。
            app_led_set_value(0, 10, 0, 0, 10, 0, app_main_data.gnss_valid);// 只闪绿色。

        } else {// 推送失败，写入缓存。
            app_sd_write_cache_file(json);
            app_led_set_value(10, 0, 0, 0, 10, 0, app_main_data.gnss_valid);// 红绿交替闪烁。
        }

    } else {// 没有 MQTT，直接写入缓存文件。
        app_sd_write_cache_file(json);
        app_led_set_value(10, 10, 0, 10, 10, 0, app_main_data.gnss_valid);// 只闪黄色。
    }

    app_sd_fsync_log_file();// 把日志写入 SD 卡。
}

/**
 * @brief 主函数，系统启动，开始循环任务。
 * @param
 */
void app_main(void) {

    // 初始化 LED，失败不终止运行。
    esp_err_t led_ret = app_led_init();
    if (led_ret != ESP_OK) {// 如果失败，大爷就不闪灯了，其它程序继续运行。
        ESP_LOGE(TAG, "------ 初始化 LED：失败！");
    } else {
        ESP_LOGI(TAG, "------ 初始化 LED：OK。");
    }

    // 初始化 SD，并且创建日志文件。
    esp_err_t sd_ret = app_sd_init();
    if (sd_ret != ESP_OK) {// 如果 SD 卡初始化失败，闪灯但不停止工作。
        ESP_LOGE(TAG, "------ 初始化 SD 卡：失败！");
    } else {
        ESP_LOGI(TAG, "------ 初始化 SD 卡：OK。");
    }

    app_sd_fsync_log_file();// 把日志写入 SD 卡。

    // 初始化守护任务。
    esp_err_t deamon_ret = app_deamon_init();
    if (deamon_ret != ESP_OK) {
        app_led_set_value(10, 10, 0, 10, 0, 0, 0);// 黄红交替闪烁。
        ESP_LOGE(TAG, "------ 初始化守护任务：失败！");
    } else {
        ESP_LOGI(TAG, "------ 初始化守护任务：OK。");
    }

    app_sd_fsync_log_file();// 把日志写入 SD 卡。

    // 初始化 GPIO 执行模块。
    esp_err_t gpio_ret = app_gpio_init();
    if (gpio_ret != ESP_OK) {
        app_led_set_value(10, 10, 0, 10, 0, 0, 0);// 黄红交替闪烁。
        ESP_LOGE(TAG, "------ 初始化 GPIO：失败！");
    } else {
        ESP_LOGI(TAG, "------ 初始化 GPIO：OK。");
    }

    app_sd_fsync_log_file();// 把日志写入 SD 卡。

    // 初始化 NVS，失败则终止运行。因为其它功能依赖于 NVS。
    esp_err_t nvs_ret = nvs_flash_init();
    if (nvs_ret == ESP_ERR_NVS_NO_FREE_PAGES || nvs_ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {// 如果 NVS 分区空间不足或者发现新版本，需要擦除 NVS 分区并重试初始化。
        nvs_ret = nvs_flash_erase();
        if (nvs_ret != ESP_OK) {
            app_led_set_value(10, 10, 0, 10, 0, 0, 0);// 黄红交替闪烁。
            ESP_LOGE(TAG, "------ 擦除 NVS：失败！");
            return;
        }
        nvs_ret = nvs_flash_init();
        if (nvs_ret != ESP_OK) {
            app_led_set_value(10, 10, 0, 10, 0, 0, 0);// 黄红交替闪烁。
            ESP_LOGE(TAG, "------ 初始化 NVS：失败！");
            return;
        }
    } else {
        ESP_LOGI(TAG, "------ 初始化 NVS：OK。");
    }

    app_sd_fsync_log_file();// 把日志写入 SD 卡。

    // 初始化事件循环，主要用于网络接口。
    esp_err_t event_loop_ret = esp_event_loop_create_default();
    if (event_loop_ret != ESP_OK) {
        app_led_set_value(10, 10, 0, 10, 0, 0, 0);// 黄红交替闪烁。
        ESP_LOGE(TAG, "------ 初始化 EVENT_LOOP：失败！");
    } else {
        ESP_LOGI(TAG, "------ 初始化 EVENT_LOOP：OK。");
    }

    app_sd_fsync_log_file();// 把日志写入 SD 卡。

    // 初始化 NETIF 网络接口。
    esp_err_t netif_ret = ESP_FAIL;
    if (event_loop_ret == ESP_OK) {
        netif_ret = esp_netif_init();
        if (netif_ret != ESP_OK) {
            app_led_set_value(10, 10, 0, 10, 0, 0, 0);// 黄红交替闪烁。
            ESP_LOGE(TAG, "------ 初始化 NETIF：失败！");
        } else {
            ESP_LOGI(TAG, "------ 初始化 NETIF：OK。");
        }
    }

    app_sd_fsync_log_file();// 把日志写入 SD 卡。

    // 初始化 BLE，失败不终止运行。
    if (gpio_ret == ESP_OK) {
        esp_err_t ble_ret = app_ble_init();
        if (ble_ret != ESP_OK) {
            app_led_set_value(10, 10, 0, 10, 0, 0, 0);// 黄红交替闪烁。
            ESP_LOGE(TAG, "------ 初始化 BLE：失败！");
        } else {
            ESP_LOGI(TAG, "------ 初始化 BLE：OK。");
        }
    }

    app_sd_fsync_log_file();// 把日志写入 SD 卡。

    // 初始化 WIFI。
    esp_err_t wifi_ret = ESP_FAIL;
    if (netif_ret == ESP_OK) {
        wifi_ret = app_wifi_init(app_main_data.dev_addr);
        if (wifi_ret != ESP_OK) {
            app_led_set_value(10, 10, 0, 10, 0, 0, 0);// 黄红交替闪烁。
            ESP_LOGE(TAG, "------ 初始化 WIFI：失败！");
        } else {
            ESP_LOGI(TAG, "------ 初始化 WIFI：OK。");
        }
    }

    app_sd_fsync_log_file();// 把日志写入 SD 卡。

    // 初始化 SNTP。
    esp_err_t sntp_ret = ESP_FAIL;
    if (wifi_ret == ESP_OK) {
        sntp_ret = app_sntp_init();
        if (sntp_ret != ESP_OK) {
            app_led_set_value(10, 10, 0, 10, 0, 0, 0);// 黄红交替闪烁。
            ESP_LOGE(TAG, "------ 初始化 SNTP：失败！");
        } else {
            ESP_LOGI(TAG, "------ 初始化 SNTP：OK。");
        }
    }

    app_sd_fsync_log_file();// 把日志写入 SD 卡。

    // 初始化 MQTT，失败不终止运行。可以写数据到本地。
    esp_err_t mqtt_ret = ESP_FAIL;
    if (wifi_ret == ESP_OK) {
        mqtt_ret = app_mqtt_init(app_main_data.dev_addr, strlen(app_main_data.dev_addr));
        if (mqtt_ret != ESP_OK) {
            app_led_set_value(10, 10, 0, 10, 0, 0, 0);// 黄红交替闪烁。
            ESP_LOGE(TAG, "------ 初始化 MQTT：失败！");
        } else {
            ESP_LOGI(TAG, "------ 初始化 MQTT：OK。");
        }
    }

    app_sd_fsync_log_file();// 把日志写入 SD 卡。

    // 初始化 PING 功能。
    if (wifi_ret == ESP_OK) {
        esp_err_t ping_ret = app_ping_init();
        if (ping_ret != ESP_OK) {
            app_led_set_value(10, 10, 0, 10, 0, 0, 0);// 黄红交替闪烁。
            ESP_LOGE(TAG, "------ 初始化 PING：失败！");
        } else {
            ESP_LOGI(TAG, "------ 初始化 PING：OK。");
        }
    }

    app_sd_fsync_log_file();// 把日志写入 SD 卡。

    // 初始化 GNSS。
    esp_err_t gnss_ret = app_gnss_init();
    if (gnss_ret != ESP_OK) {
        app_led_set_value(10, 10, 0, 10, 0, 0, 0);// 黄红交替闪烁。
        ESP_LOGE(TAG, "------ 初始化 GNSS：失败！");
    } else {
        ESP_LOGI(TAG, "------ 初始化 GNSS：OK。");
    }

    app_sd_fsync_log_file();// 把日志写入 SD 卡。
    app_status = 1;

    for (int i = 10; i > 0; i--) {
        ESP_LOGI(TAG, "------ APP MAIN 启动倒计时: %d", i);// 主要用于等待 MQTT 连接。
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_LOGI(TAG, "------ APP MAIN 启动主任务循环......");
    const TickType_t task_period = pdMS_TO_TICKS(1000);
    while (1) {
        TickType_t start_tick = xTaskGetTickCount();// 开始时间。

        app_main_loop_task();// 循环任务。

        // char buffer[1024];
        // vTaskGetRunTimeStats(buffer);
        // printf("---------------------------------------------\n%s", buffer);

        TickType_t end_tick = xTaskGetTickCount();// 结束时间。
        TickType_t task_duration = end_tick - start_tick;
        if (task_duration > task_period) {// 是否需要延时至下一个周期。
            vTaskDelay(task_period - (task_duration % task_period));
        } else {
            vTaskDelay(task_period - task_duration);
        }
        if (app_main_data.gnss_valid == false) {// 如果数据无效，延迟 4 秒，5 秒一次。
            vTaskDelay(pdMS_TO_TICKS(4000));
        } else if (app_main_data.spd < 5) {// 停止未移动，速度小于 9.26 公里，5 秒一次。
            vTaskDelay(pdMS_TO_TICKS(4000));
        } else if (app_main_data.spd < 30) {// 低速移动，速度小于 55.56 公里，2 秒一次。
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}
