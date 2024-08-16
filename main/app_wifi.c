/**
 * @brief   WIFI 初始化。
 *
 * @author  nyx
 * @date    2024-08-18
 */
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "lwip/err.h"
#include "lwip/sys.h"

#include "app_config.h"

#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_FAIL_BIT       BIT1

 /**
  * @brief WIFI 事件组。
  */
static EventGroupHandle_t app_wifi_event_group;

/**
* @brief 日志 TAG。
*/
static const char* TAG = "app_wifi";

/**
* @brief 重试 WIFI 连接次数。
*/
static int app_wifi_retry_count = 0;

/**
* @brief WIFI 事件句柄。
*/
static void app_wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT) {
        if (event_id == WIFI_EVENT_STA_START) {
            ESP_LOGI(TAG, "------ 执行 WIFI 连接。");
            esp_wifi_connect();

        } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
            app_wifi_retry_count++;
            ESP_LOGI(TAG, "------ WIFI 连接已断开，等待 10 秒后重试。重试次数：%d", app_wifi_retry_count);
            vTaskDelay(pdMS_TO_TICKS(10000));// 等待 10 秒，自动重连。
            esp_wifi_connect();
        }

    } else if (event_base == IP_EVENT) {
        if (event_id == IP_EVENT_STA_GOT_IP) {
            ip_event_got_ip_t* event = (ip_event_got_ip_t*)event_data;
            ESP_LOGI(TAG, "------ WIFI 已连接。SSID：" APP_WIFI_SSID "，获取 IP：" IPSTR, IP2STR(&event->ip_info.ip));
            app_wifi_retry_count = 0;
            xEventGroupSetBits(app_wifi_event_group, WIFI_CONNECTED_BIT);
        }
    }
}

/**
 * @brief 初始化函数。
 * @param
 * @return
 */
esp_err_t app_wifi_init(char* dev_addr) {

    uint8_t mac_addr_t[6] = { 0 };
    esp_read_mac(mac_addr_t, ESP_MAC_WIFI_STA);// MAC 地址，当芯片的硬件 ID 使用。
    sprintf(dev_addr, "%02X:%02X:%02X:%02X:%02X:%02X",
        mac_addr_t[0], mac_addr_t[1], mac_addr_t[2], mac_addr_t[3], mac_addr_t[4], mac_addr_t[5]);
    ESP_LOGI(TAG, "------ 获取 MAC 地址：%s", dev_addr);

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_err_t ret = esp_wifi_init(&cfg);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, app_wifi_event_handler, NULL, NULL);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, app_wifi_event_handler, NULL, NULL);
    if (ret != ESP_OK) {
        return ret;
    }

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = APP_WIFI_SSID,
            .password = APP_WIFI_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ret = esp_wifi_set_mode(WIFI_MODE_STA);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = esp_wifi_start();
    if (ret != ESP_OK) {
        return ret;
    }

    ESP_LOGI(TAG, "------ WIFI 启动：完成。");

    app_wifi_event_group = xEventGroupCreate();
    EventBits_t bits = xEventGroupWaitBits(app_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
    if (bits & WIFI_CONNECTED_BIT) {
        return ESP_OK;
    }

    return ESP_FAIL;
}
