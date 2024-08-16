/**
 * @brief   MQTT 初始化，上传数据功能。
 *
 * @author  nyx
 * @date    2024-07-08
 */
#pragma once

#include "mqtt_client.h"

 /**
  * @brief 最近一次发送 MQTT 的时间戳。
  */
extern _Atomic uint32_t app_mqtt_last_ts;

/**
 * @brief MQTT 5 客户端。
 */
extern esp_mqtt_client_handle_t app_mqtt_5_client;

/**
 * @brief MQTT 发消息给服务器。
 * @param msg
 * @return message_id of the publish message (for QoS 0 message_id will always
 *          be zero) on success. -1 on failure, -2 in case of full outbox.
 */
int app_mqtt_publish_msg(char* msg);

/**
 * @brief MQTT 发日志给服务器。
 * @param topic
 * @param msg
 * @return
 */
int app_mqtt_publish_log(char* topic, char* log);

/**
 * @brief 初始化函数。
 * @param will_msg
 * @param msg_len
 * @return
 */
esp_err_t app_mqtt_init(char* will_msg, size_t will_msg_len);
