/**
 * @brief   MQTT 初始化，上传数据功能。
 *
 * @author  nyx
 * @date    2024-07-08
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdatomic.h>
#include <pthread.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "mqtt_client.h"

#include "app_sd.h"
#include "app_config.h"

 /**
  * @brief 日志 TAG。
  */
static const char* TAG = "app_mqtt";

/**
 * @brief MQTT 初始化状态。
 */
static int app_mqtt_init_status = 0;

/**
 * @brief 最近一次发送 MQTT 的时间戳。
 */
_Atomic uint32_t app_mqtt_last_ts = ATOMIC_VAR_INIT(0);

/**
 * @brief MQTT 客户端。
 */
esp_mqtt_client_handle_t app_mqtt_5_client;

/**
 * @brief 是否推送日志备份文件。
 */
static int app_mqtt_pub_bak_count = 0;

/**
 * @brief MQTT 发消息给服务器。
 * @param msg
 * @return message_id of the publish message (for QoS 0 message_id will always
 *          be zero) on success. -1 on failure, -2 in case of full outbox.
 */
int app_mqtt_publish_msg(char* msg) {
    if (app_mqtt_init_status == 0) {
        ESP_LOGE(TAG, "------ MQTT 初始化失败，MQTT 客户端状态：不可用！");
        return -1;
    }
    int ret = esp_mqtt_client_publish(app_mqtt_5_client, APP_MQTT_PUB_MGS_TOPIC, msg, strlen(msg), APP_MQTT_QOS, 0);
    if (ret >= 0) {
        atomic_store(&app_mqtt_last_ts, esp_log_timestamp());
    }
    return ret;
}

/**
 * @brief MQTT 发日志给服务器。
 * @param topic
 * @param msg
 * @return
 */
int app_mqtt_publish_log(char* topic, char* log) {
    if (app_mqtt_init_status == 0) {
        ESP_LOGE(TAG, "------ MQTT 初始化失败，MQTT 客户端状态：不可用！");
        return -1;
    }
    int ret = esp_mqtt_client_publish(app_mqtt_5_client, topic, log, strlen(log), APP_MQTT_QOS, 0);
    return ret;
}

/**
 * @brief MQTT 事件回调函数。
 * @param handler_args
 * @param base
 * @param event_id
 * @param event_data
 * @return
 */
static void app_mqtt_event_handler(void* handler_args, esp_event_base_t base, int32_t event_id, void* event_data) {
    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "------ MQTT 事件：已连接。");
            if (app_mqtt_pub_bak_count == 0) {
                app_sd_pub_log_bak_file();
                app_sd_pub_cache_bak_file();
                app_mqtt_pub_bak_count = 1;
            }
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "------ MQTT 事件：断开连接！");
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "------ MQTT 事件：发布完成！");
            break;
        case MQTT_EVENT_BEFORE_CONNECT:
            ESP_LOGI(TAG, "------ MQTT 事件：连接之前！");
            break;
        default:
            ESP_LOGI(TAG, "------ MQTT 其它事件。EVENT ID：%ld", event_id);
            break;
    }
}

/**
 * @brief 初始化函数。
 * @param will_msg
 * @param msg_len
 * @return
 */
esp_err_t app_mqtt_init(char* will_msg, size_t will_msg_len) {

    esp_mqtt_client_config_t mqtt5_cfg = {
        .session.protocol_ver = MQTT_PROTOCOL_V_5,
        .broker.address.uri = APP_MQTT_URI,
        .credentials.username = APP_MQTT_USERNAME,
        .credentials.authentication.password = APP_MQTT_PASSWORD,

        .network.timeout_ms = 2000,// 网络操作超时为 2 秒。MQTT_NETWORK_TIMEOUT_MS 默认 10 秒。
        .network.reconnect_timeout_ms = 2000,// 设置重连间隔为 2 秒。MQTT_RECON_DEFAULT_MS 默认 10 秒。
        .network.disable_auto_reconnect = false,    // 自动连接！

        .session.last_will.topic = APP_MQTT_WILL_TOPIC,
        .session.last_will.msg = will_msg,
        .session.last_will.msg_len = will_msg_len,
        .session.last_will.qos = APP_MQTT_QOS,
        .session.last_will.retain = true,
    };

    app_mqtt_5_client = esp_mqtt_client_init(&mqtt5_cfg);
    esp_mqtt_client_register_event(app_mqtt_5_client, ESP_EVENT_ANY_ID, app_mqtt_event_handler, NULL);
    esp_err_t mqtt_ret = esp_mqtt_client_start(app_mqtt_5_client);
    if (mqtt_ret == ESP_OK) {
        app_mqtt_init_status = 1;
    }
    return mqtt_ret;
}