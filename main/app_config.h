/**
 * @brief   本应用的配置参数，自定义参数。
 *
 * @author  nyx
 * @date    2024-07-10
 */
#pragma once

 /**
  * @brief 蓝牙接近开关地址，设置为蓝牙白名单。
  */
#define APP_BLE_WHITE_LIST { \
    {.type = BLE_ADDR_PUBLIC, .val = {0x37, 0xb0, 0x01, 0x78, 0x90, 0xdf}}, \
    {.type = BLE_ADDR_PUBLIC, .val = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}}, \
}

  /*
  * WIFI 热点配置。
  */
#define APP_WIFI_SSID                   "ldv"
#define APP_WIFI_PASSWORD               "Yxxxy123!"

  /*
  * GPIO 输出针脚。
  */
#define APP_GPIO_NUM_BLE                21  // 蓝牙接近开关。
#define APP_GPIO_PIN_BIT_MASK           (1ULL << APP_GPIO_NUM_BLE)

  /*
  * 如果蓝牙接近开关离开 60 秒，则关闭。
  */
#define APP_BLE_LEAVE_TIMEOUT           60

  /*
   * MQTT 服务器配置。
   */
#define APP_MQTT_URI                    "mqtt://codingau.i234.me"
#define APP_MQTT_USERNAME               "iot001"
#define APP_MQTT_PASSWORD               "iot001esp32s3"
#define APP_MQTT_PUB_MGS_TOPIC          "topic/iotmsg"
#define APP_MQTT_PUB_LOG_TOPIC          "topic/iotlog"
#define APP_MQTT_WILL_TOPIC             "topic/will"
#define APP_MQTT_QOS                    0                   // 实际测试连续发送 1000 条 200 个字符，QOS = 0 耗时 2.5 秒，QOS = 1 耗时 9 秒左右。


   /*
    * AT 命令发送与数据接收的 UART 端口配置。
    */
#define APP_UART_PORT_NUM            UART_NUM_1
#define APP_UART_BAUD_RATE           115200
#define APP_UART_TX_PIN              20
#define APP_UART_RX_PIN              19
#define APP_UART_BUF_SIZE            1024