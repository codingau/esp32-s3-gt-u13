/**
 * @brief   GNSS 初始化，获取定位数据。
 *
 * @author  nyx
 * @date    2024-06-28
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

#include "nmea.h"
#include "gpgga.h"
#include "gprmc.h"

#include "app_gnss.h"
#include "app_config.h"

 /**
 * @brief 日志 TAG。
 */
static const char* TAG = "app_gnss";

/**
 * @brief 初始化 GNSS 数据。
 */
app_gnss_data_t app_gnss_data = {
    .date_time = {
        .tm_year = 70,
        .tm_mon = 0,
        .tm_mday = 1,
        .tm_hour = 0,
        .tm_min = 0,
        .tm_sec = 0
     },                                     // 日期时间。
    .valid = false,                         // 有效性。
    .sat = 0,                               // 卫星数。
    .alt = 0.0,                             // 高度。
    .lat = 0.0,                             // 纬度。
    .lon = 0.0,                             // 经度。
    .spd = 0.0,                             // 速度。
    .trk = 0.0,                             // 航向角度。
    .mag = 0.0,                             // 磁偏角度。
    .mutex = PTHREAD_MUTEX_INITIALIZER      // 互斥锁。
};

static char s_buf[APP_UART_BUF_SIZE + 1];
static size_t s_total_bytes;
static char* s_last_buf_end;

/**
 * @brief 从字符串中，查找子字符串。
 * @param buffer
 * @param buffer_length
 * @param substring
 * @return
 */
static char* find_substring(const char* buffer, size_t buffer_length, const char* substring) {
    size_t substring_length = strlen(substring);
    for (size_t i = 0; i <= buffer_length - substring_length; i++) {
        if (memcmp(buffer + i, substring, substring_length) == 0) {
            return (char*)(buffer + i);
        }
    }
    return NULL;
}


/**
 * @brief 从 UART 读取一行数据。
 * @param out_line_buf
 * @param out_line_len
 * @param timeout_ms
 */
static void app_gnss_read_uart_line(char** out_line_buf, size_t* out_line_len, int timeout_ms) {
    *out_line_buf = NULL;
    *out_line_len = 0;

    if (s_last_buf_end != NULL) {
        /* Data left at the end of the buffer after the last call;
         * copy it to the beginning.
         */
        size_t len_remaining = s_total_bytes - (s_last_buf_end - s_buf);
        memmove(s_buf, s_last_buf_end, len_remaining);
        s_last_buf_end = NULL;
        s_total_bytes = len_remaining;
    }

    /* Read data from the UART */
    int read_bytes = uart_read_bytes(APP_UART_PORT_NUM, (uint8_t*)s_buf + s_total_bytes, APP_UART_BUF_SIZE - s_total_bytes, pdMS_TO_TICKS(timeout_ms));
    if (read_bytes <= 0) {
        return;
    }

    s_total_bytes += read_bytes;

    /* find start (a dollar sign) */
    char* start = memchr(s_buf, '$', s_total_bytes);

    /* find end of line */
    char* end = memchr(start, '\r', s_total_bytes - (start - s_buf));
    if (end == NULL || *(++end) != '\n') {
        return;
    }
    end++;

    end[-2] = NMEA_END_CHAR_1;
    end[-1] = NMEA_END_CHAR_2;

    *out_line_buf = start;
    *out_line_len = end - start;
    if (end < s_buf + s_total_bytes) {
        /* some data left at the end of the buffer, record its position until the next call */
        s_last_buf_end = end;
    } else {
        s_total_bytes = 0;
    }
}

/**
 * @brief 循环读取 UART 的任务。
 * @param param
 */
static void app_gnss_read_task(void* param) {

    while (1) {
        char* start;
        size_t length;
        app_gnss_read_uart_line(&start, &length, 200 /* ms */);// A7670E 和 GT-U13 模块，输出频率是 1 秒 1 次，每次输出 N 条记录。
        if (length == 0) {
            continue;
        }
        // ESP_LOGW(TAG, "%d ------ %.*s", length, length, start);

        char print_str[length];
        strncpy(print_str, start, length - 2);// 不 copy 尾部的 \r\n 换行符。
        print_str[length - 2] = '\0';

        nmea_s* data = nmea_parse(start, length, 0);
        if (data == NULL) {// 没有解析器的数据，不处理。
            free(data);
            continue;
        }
        if (data->errors != 0) {// 如果有错误，直接丢弃数据，进入下一次循环。
            free(data);
            continue;
        }

        if (NMEA_GPGGA == data->type) {// 只处理 gga 和 rmc，其它类型不需要。

            ESP_LOGW(TAG, " ------ %s", print_str);
            pthread_mutex_lock(&app_gnss_data.mutex);
            nmea_gpgga_s* gga = (nmea_gpgga_s*)data;
            app_gnss_data.sat = gga->n_satellites;
            app_gnss_data.alt = gga->altitude;
            pthread_mutex_unlock(&app_gnss_data.mutex);

        } else if (NMEA_GPRMC == data->type) {

            ESP_LOGW(TAG, " ------ %s", print_str);
            pthread_mutex_lock(&app_gnss_data.mutex);
            nmea_gprmc_s* rmc = (nmea_gprmc_s*)data;
            app_gnss_data.valid = rmc->valid;
            if (app_gnss_data.valid) {// false 的时候，以下数据全部为 0。
                app_gnss_data.date_time = rmc->date_time;
                app_gnss_data.lat = rmc->latitude.degrees + (rmc->latitude.minutes / 60.0);
                app_gnss_data.lat = round(app_gnss_data.lat * 1000000) / 1000000;// 四舍五入 6 位小数。
                if (rmc->latitude.cardinal == NMEA_CARDINAL_DIR_SOUTH) {// 南经是负数。
                    app_gnss_data.lat = -app_gnss_data.lat;
                }
                app_gnss_data.lon = rmc->longitude.degrees + (rmc->longitude.minutes / 60.0);
                app_gnss_data.lon = round(app_gnss_data.lon * 1000000) / 1000000;// 四舍五入 6 位小数。
                if (rmc->longitude.cardinal == NMEA_CARDINAL_DIR_WEST) {// 西经是负数。
                    app_gnss_data.lon = -app_gnss_data.lon;
                }
                app_gnss_data.spd = rmc->gndspd_knots;
                app_gnss_data.trk = rmc->track_deg;
                app_gnss_data.mag = rmc->magvar_deg;
                if (rmc->magvar_cardinal == NMEA_CARDINAL_DIR_WEST) {// 向西的磁偏角是负数。
                    app_gnss_data.mag = -app_gnss_data.mag;
                }
            }
            pthread_mutex_unlock(&app_gnss_data.mutex);
        }
        if (data != NULL) {
            free(data);
        }
    }
}

/**
 * @brief 初始化函数。
 * @return
 */
esp_err_t app_gnss_init(void) {
    uart_config_t uart_config = {
        .baud_rate = APP_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    esp_err_t ret = uart_param_config(APP_UART_PORT_NUM, &uart_config);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = uart_set_pin(APP_UART_PORT_NUM, APP_UART_TX_PIN, APP_UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = uart_driver_install(APP_UART_PORT_NUM, APP_UART_BUF_SIZE, 0, 0, NULL, 0);
    if (ret != ESP_OK) {
        return ret;
    }
    ESP_LOGI(TAG, " ------ UART 驱动安装完成，启动 GNSS 接收任务。");
    xTaskCreate(app_gnss_read_task, "app_gnss_read_task", 4096, NULL, 8, NULL);// 启动接收任务。
    return ESP_OK;
}