/**
 * @brief   SD 卡初始化，本地文件读写。
 *
 * @author  nyx
 * @date    2024-06-28
 */
#include <stdio.h>
#include <string.h>
#include <dirent.h>
#include <errno.h>
#include <unistd.h>
#include <sys/stat.h>
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"

#include "app_main.h"
#include "app_mqtt.h"
#include "app_config.h"

 /**
 * @brief SD 卡引脚配置，这个板只有1条数据脚。
 */
#define APP_SD_WIDTH         1
#define APP_SD_CMD           38
#define APP_SD_CLK           39
#define APP_SD_DATA          40

 /**
 * @brief 挂载点。
 */
#define APP_SD_MOUNT_POINT   "/sdcard"

 /**
 * @brief 日志目录。
 *        必须大写！为啥啊！
 *        实际测试，传参是小写，建立的文件名还是大写！
 *        坑死我啊，目录名必须大写，文件名也必须大写，并且不能太长！
 */
#define APP_SD_LOG_DIR              APP_SD_MOUNT_POINT"/LOG"

 /**
 * @brief 日志文件名。
 */
#define APP_SD_LOG_TXT              APP_SD_LOG_DIR"/LOG.TXT"

 /**
 * @brief 日志备份文件名，稍后按时间复制为文件。
 */
#define APP_SD_LOG_FILE_TXT         APP_SD_LOG_DIR"/FILE.TXT"

 /**
 * @brief 日志备份文件名，稍后推送给 MQTT 服务器。
 */
#define APP_SD_LOG_MQTT_TXT         APP_SD_LOG_DIR"/MQTT.TXT"

 /**
 * @brief 缓存目录。
 */
#define APP_SD_CACHE_DIR            APP_SD_MOUNT_POINT"/CACHE"

 /**
 * @brief 缓存文件名。
 */
#define APP_SD_CACHE_TXT            APP_SD_CACHE_DIR"/CACHE.TXT"

 /**
 * @brief 缓存备份文件名，稍后按时间复制为文件。
 */
#define APP_SD_CACHE_FILE_TXT       APP_SD_CACHE_DIR"/FILE.TXT"

 /**
 * @brief 缓存备份文件名，稍后推送给 MQTT 服务器。
 */
#define APP_SD_CACHE_MQTT_TXT       APP_SD_CACHE_DIR"/MQTT.TXT"

 /**
 * @brief 日志 TAG。
 */
static const char* TAG = "app_sd";

/**
 * @brief SD 卡初始化状态。
 */
static int app_sd_init_status = 0;

/**
* @brief 日志文件。
*/
static FILE* app_sd_log_file = NULL;

/**
* @brief 缓存文件。
*/
static FILE* app_sd_cache_file = NULL;

/**
* @brief 日志文件写入计数。
*/
static uint32_t app_sd_log_file_write_count = 0;

/**
* @brief 缓存 MQTT 已推送行数。
*/
static int app_sd_cache_mqtt_pub_line = 0;

/**
* @brief 输出数据到缓存文件。
*/
void app_sd_write_cache_file(char* json) {
    if (app_sd_init_status == 0) {
        ESP_LOGE(TAG, "------ SD 卡初始化失败，SD 卡状态：不可用！");
        return;
    }
    if (app_sd_cache_file == NULL) {
        ESP_LOGE(TAG, "------ SD 卡写入缓存文件：失败！app_sd_cache_file == NULL");
        return;
    }
    size_t len = strlen(json);
    json[len - 2] = '1';// 替换 json 中标记字段值为 1，标记为缓存数据。
    json[len] = '\n';// 追加换行符。
    json[len + 1] = '\0'; // 添加字符串终止符。
    size_t write_len = fwrite(json, 1, strlen(json), app_sd_cache_file);
    fflush(app_sd_cache_file);
    fsync(fileno(app_sd_cache_file));
    ESP_LOGI(TAG, "------ SD 卡写入缓存，字节数：%d --> %s", write_len, json);
}

/**
* @brief 确保写出日志内容到 SD 卡。
*        fsync() 执行比较消耗性能，所以由外部调用，隔一段时间执行一次。
*/
void app_sd_fsync_log_file(void) {
    if (app_sd_init_status == 0) {
        ESP_LOGE(TAG, "------ SD 卡初始化失败，SD 卡状态：不可用！");
        return;
    }
    if (app_sd_log_file != NULL && app_sd_log_file_write_count > 0) {
        fsync(fileno(app_sd_log_file));
    }
}

/**
* @brief 增加写日志到文件的功能，保留日志输出到 UART。
*/
static int app_sd_write_log_file(const char* fmt, va_list args) {
    int ret_uart = vprintf(fmt, args);// 先写 UART。
    int ret_file = 0;
    if (app_sd_log_file != NULL) {
        ret_file = vfprintf(app_sd_log_file, fmt, args);// 再写文件。
        fflush(app_sd_log_file);
        app_sd_log_file_write_count++;
    }
    return ret_uart < 0 ? ret_uart : ret_file;
}

/**
* @brief 计算备份文件数量。
*/
static int app_sd_count_bak_files(const char* path) {
    DIR* dp = opendir(path);
    if (dp == NULL) {
        ESP_LOGE(TAG, "------ SD 卡打开目录：失败！目录：%s", path);
        return -1;
    }
    int file_count = 0;
    struct dirent* entry;
    while ((entry = readdir(dp))) {
        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) {
            continue;
        }
        if (strcmp(entry->d_name, APP_SD_LOG_TXT) == 0 || // 排除文件名。
            strcmp(entry->d_name, APP_SD_LOG_FILE_TXT) == 0 ||
            strcmp(entry->d_name, APP_SD_LOG_MQTT_TXT) == 0 ||
            strcmp(entry->d_name, APP_SD_CACHE_TXT) == 0 ||
            strcmp(entry->d_name, APP_SD_CACHE_FILE_TXT) == 0 ||
            strcmp(entry->d_name, APP_SD_CACHE_MQTT_TXT) == 0
            ) {
            continue;
        }
        file_count++;
    }
    closedir(dp);
    return file_count;
}

/**
 * @brief 删除全部备份文件。
 */
static void app_sd_delete_bak_files(const char* path) {
    DIR* log_dir = opendir(path);
    if (log_dir == NULL) {
        ESP_LOGE(TAG, "------ SD 卡打开目录：失败！目录：%s", path);
        return;
    }
    struct dirent* entry;
    while ((entry = readdir(log_dir)) != NULL) {// 遍历目录。
        char file_path[256];
        if (strlen(path) + strlen(entry->d_name) + 1 < sizeof(file_path)) {
            if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) {// 忽略 "." 和 ".."
                continue;
            }
            if (strcmp(entry->d_name, APP_SD_LOG_TXT) == 0 ||// 排除文件名。
                strcmp(entry->d_name, APP_SD_LOG_FILE_TXT) == 0 ||
                strcmp(entry->d_name, APP_SD_LOG_MQTT_TXT) == 0 ||
                strcmp(entry->d_name, APP_SD_CACHE_TXT) == 0 ||
                strcmp(entry->d_name, APP_SD_CACHE_FILE_TXT) == 0 ||
                strcmp(entry->d_name, APP_SD_CACHE_MQTT_TXT) == 0
                ) {
                continue;
            }
            int path_length = snprintf(file_path, sizeof(file_path), "%s/%s", path, entry->d_name);
            if (path_length < 0 || path_length >= sizeof(file_path)) {
                continue; // 文件名长度超过缓冲区，跳过这个文件。
            }
            remove(file_path);// 删除文件。 
        }
    }
    closedir(log_dir);
}

/**
 * @brief 复制文件，追加模式。
 * @param source
 * @param destination
 */
static void app_sd_copy_file(const char* source, const char* destination) {
    FILE* src = fopen(source, "rb");
    if (src == NULL) {
        ESP_LOGE(TAG, "------ SD 卡复制文件，无法打开源文件！文件名：%s", source);
        return;
    }
    FILE* dest = fopen(destination, "a");// 追加模式。
    if (dest == NULL) {
        ESP_LOGE(TAG, "------ SD 卡复制文件，无法打开目标文件！文件名：%s", destination);
        fclose(src);
        return;
    }
    char buffer[1024];
    size_t bytesRead;
    while ((bytesRead = fread(buffer, 1, sizeof(buffer), src)) > 0) {
        fwrite(buffer, 1, bytesRead, dest);
    }
    fclose(src);
    fclose(dest);
}

/**
* @brief 备份日志文件。
*/
void app_sd_bak_log_file(void) {
    if (app_sd_init_status == 0) {
        ESP_LOGE(TAG, "------ SD 卡初始化失败，SD 卡状态：不可用！");
        return;
    }
    ESP_LOGI(TAG, "------ SD 卡备份日志文件：开始。");
    if (access(APP_SD_LOG_FILE_TXT, F_OK) == -1) {
        ESP_LOGE(TAG, "------ SD 卡备份日志文件：失败。文件不存在，文件名：%s", APP_SD_LOG_FILE_TXT);
        return;
    }
    time_t now;
    time(&now); // 获取当前时间（秒）。
    struct tm timeinfo;
    gmtime_r(&now, &timeinfo); // 将时间转换为 UTC 时间。
    char new_bak_name[64];
    strftime(new_bak_name, sizeof(new_bak_name), APP_SD_LOG_DIR"/%m%d%H%M.TXT", &timeinfo);// 月日时分.TXT
    app_sd_copy_file(APP_SD_LOG_FILE_TXT, new_bak_name);
    ESP_LOGI(TAG, "------ SD 卡备份日志文件：完成。文件名：%s", new_bak_name);
    remove(APP_SD_LOG_FILE_TXT);// 文件备份完，删除 FILE.TXT
}

/**
* @brief 备份缓存文件。
*/
void app_sd_bak_cache_file(void) {
    if (app_sd_init_status == 0) {
        ESP_LOGE(TAG, "------ SD 卡初始化失败，SD 卡状态：不可用！");
        return;
    }
    ESP_LOGI(TAG, "------ SD 卡备份缓存文件：开始。");
    if (access(APP_SD_CACHE_FILE_TXT, F_OK) == -1) {
        ESP_LOGE(TAG, "------ SD 卡备份缓存文件：失败。文件不存在，文件名：%s", APP_SD_CACHE_FILE_TXT);
        return;
    }
    time_t now;
    time(&now); // 获取当前时间（秒）。
    struct tm timeinfo;
    gmtime_r(&now, &timeinfo); // 将时间转换为 UTC 时间。
    char new_bak_name[64];
    strftime(new_bak_name, sizeof(new_bak_name), APP_SD_CACHE_DIR"/%m%d%H%M.TXT", &timeinfo);// 月日时分.TXT
    app_sd_copy_file(APP_SD_CACHE_FILE_TXT, new_bak_name);
    ESP_LOGI(TAG, "------ SD 卡备份缓存文件：完成。文件名：%s", new_bak_name);
    remove(APP_SD_CACHE_FILE_TXT);// 文件备份完，删除 FILE.TXT
}


/**
* @brief 推送日志备份文件。
*/
void app_sd_pub_log_bak_file(void) {
    if (app_sd_init_status == 0) {
        ESP_LOGE(TAG, "------ SD 卡初始化失败，SD 卡状态：不可用！");
        return;
    }
    ESP_LOGI(TAG, "------ SD 卡推送日志备份文件：开始。文件名：%s", APP_SD_LOG_MQTT_TXT);
    if (access(APP_SD_LOG_MQTT_TXT, F_OK) == -1) {// 检查 LOG.BAK 是否存在，存在则发送给服务器。
        ESP_LOGE(TAG, "------ SD 卡推送日志备份文件：失败。文件不存在，文件名：%s", APP_SD_LOG_MQTT_TXT);
        return;
    }
    FILE* file = fopen(APP_SD_LOG_MQTT_TXT, "r");
    if (file == NULL) {
        ESP_LOGE(TAG, "------ SD 卡推送日志备份文件：失败。打开文件失败，文件名：%s", APP_SD_LOG_MQTT_TXT);
        return;
    }
    int line_count = 0;
    char line[1000];
    while (fgets(line, sizeof(line), file) != NULL) {// 逐行读取文件内容。
        size_t len = strlen(line);// 去除行尾的换行符。
        if (len > 0 && line[len - 1] == '\n') {
            line[len - 1] = '\0'; // 将换行符替换为 NULL 终止符。
        }
        char topic[100];
        snprintf(topic, sizeof(topic), "%s/%s", APP_MQTT_PUB_LOG_TOPIC, app_main_data.dev_addr);
        int pub_ret = app_mqtt_publish_log(topic, line);
        if (pub_ret < 0) {// 只要有一次发送失败，就跳出循环，不再继续执行。
            ESP_LOGW(TAG, "------ SD 卡推送日志备份文件：中断。文件名：%s，推送行数：%d", APP_SD_LOG_MQTT_TXT, line_count);
            break;
        }
        line_count++;

    }
    fclose(file);
    remove(APP_SD_LOG_MQTT_TXT);// 推送完成，删除 MQTT.TXT
    ESP_LOGI(TAG, "------ SD 卡推送日志备份文件：完成。文件名：%s，推送行数：%d", APP_SD_LOG_MQTT_TXT, line_count);
}

/**
* @brief 推送缓存备份文件。
*/
void app_sd_pub_cache_bak_file(void) {
    if (app_sd_init_status == 0) {
        ESP_LOGE(TAG, "------ SD 卡初始化失败，SD 卡状态：不可用！");
        return;
    }
    ESP_LOGI(TAG, "------ SD 卡推送缓存备份文件：开始。文件名：%s", APP_SD_CACHE_MQTT_TXT);
    if (access(APP_SD_CACHE_MQTT_TXT, F_OK) == -1) {// 检查 LOG.BAK 是否存在，存在则发送给服务器。
        ESP_LOGE(TAG, "------ SD 卡推送缓存备份文件：失败。文件不存在，文件名：%s", APP_SD_CACHE_MQTT_TXT);
        return;
    }
    FILE* file = fopen(APP_SD_CACHE_MQTT_TXT, "r");
    if (file == NULL) {
        ESP_LOGE(TAG, "------ SD 卡推送缓存备份文件：失败。打开文件失败，文件名：%s", APP_SD_CACHE_MQTT_TXT);
        return;
    }

    char line[1024];
    ESP_LOGI(TAG, "------ SD 卡推送缓存文件：开始。文件名：%s，跳过行数：%d", APP_SD_CACHE_MQTT_TXT, app_sd_cache_mqtt_pub_line);
    for (int i = 0; i < app_sd_cache_mqtt_pub_line; ++i) {
        fgets(line, sizeof(line), file);// 跳过 N 行。
    }
    while (fgets(line, sizeof(line), file) != NULL) {// 逐行读取文件内容。
        size_t len = strlen(line);// 去除行尾的换行符。
        if (len > 0 && line[len - 1] == '\n') {
            line[len - 1] = '\0'; // 将换行符替换为 NULL 终止符。
        }
        int pub_ret = app_mqtt_publish_msg(line);
        if (pub_ret < 0) {// 只要有一次发送失败，就跳出循环，不再继续执行。
            ESP_LOGW(TAG, "------ SD 卡推送缓存备份文件：中断。下次启动时重试。文件名：%s，推送行数：%d", APP_SD_CACHE_MQTT_TXT, app_sd_cache_mqtt_pub_line);
            return;
        }
        app_sd_cache_mqtt_pub_line++;// 记录已发条数。
    }
    fclose(file);// 关闭文件。
    remove(APP_SD_CACHE_MQTT_TXT);// 推送完成，删除 MQTT.TXT
    app_sd_cache_mqtt_pub_line = 0;
    ESP_LOGI(TAG, "------ SD 卡推送缓存备份文件：完成。文件名：%s，推送行数：%d", APP_SD_CACHE_MQTT_TXT, app_sd_cache_mqtt_pub_line);
}

/**
* @brief 创建日志文件。
*/
static void app_sd_create_log_file(void) {
    if (access(APP_SD_LOG_TXT, F_OK) != -1) {// 检查文件是否存在。
        app_sd_copy_file(APP_SD_LOG_TXT, APP_SD_LOG_MQTT_TXT);// 先复制一份到 MQTT.TXT
        if (access(APP_SD_LOG_FILE_TXT, F_OK) != -1) {// 检查 FILE.TXT 文件是否存在，存在则删除。
            remove(APP_SD_LOG_FILE_TXT);
        }
        rename(APP_SD_LOG_TXT, APP_SD_LOG_FILE_TXT);// 再重命名为 FILE.TXT
    }
    app_sd_log_file = fopen(APP_SD_LOG_TXT, "a");
    if (app_sd_log_file == NULL) {
        ESP_LOGE(TAG, "------ SD 卡创建日志文件：失败！文件名：%s", APP_SD_LOG_TXT);
    } else {
        ESP_LOGI(TAG, "------ SD 卡创建日志文件：完成。文件名：%s", APP_SD_LOG_TXT);
        esp_log_set_vprintf(app_sd_write_log_file);// 重定向输出 LOG 到文件。
    }
}

/**
* @brief 创建缓存文件。
*/
static void app_sd_create_cache_file(void) {
    if (access(APP_SD_CACHE_TXT, F_OK) != -1) {// 检查文件是否存在。
        app_sd_copy_file(APP_SD_CACHE_TXT, APP_SD_CACHE_MQTT_TXT);// 先复制一份到 MQTT.TXT
        if (access(APP_SD_CACHE_FILE_TXT, F_OK) != -1) {// 检查 FILE.TXT 文件是否存在，存在则删除。
            remove(APP_SD_CACHE_FILE_TXT);
        }
        rename(APP_SD_CACHE_TXT, APP_SD_CACHE_FILE_TXT);// 再重命名为 FILE.TXT
    }
    app_sd_cache_file = fopen(APP_SD_CACHE_TXT, "a");// 创建一个新文件。
    if (app_sd_cache_file == NULL) {
        ESP_LOGE(TAG, "------ SD 卡创建缓存文件：失败！文件名：%s", APP_SD_CACHE_TXT);
    } else {
        ESP_LOGI(TAG, "------ SD 卡创建缓存文件：完成。文件名：%s", APP_SD_CACHE_TXT);
    }
}

/**
* @brief SD 卡创建目录。
*/
static int app_sd_mkdir(const char* path) {
    if (access(path, F_OK) == -1) {// Check if the directory exists
        if (mkdir(path, 0700) == -1) {// Create the directory
            ESP_LOGI(TAG, "------ SD 卡创建目录：失败！%s: %s", path, strerror(errno));
            return -1;
        }
    }
    return 1;
}

/**
 * @brief 初始化函数。
 * @param
 * @return
 */
esp_err_t app_sd_init(void) {

    const char mount_point[] = APP_SD_MOUNT_POINT;

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();

    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = APP_SD_WIDTH;
    slot_config.cmd = APP_SD_CMD;
    slot_config.clk = APP_SD_CLK;
    slot_config.d0 = APP_SD_DATA;

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = true,     // 挂载失败格式化。
        .max_files = 10,                    // 最多打开文件数。
        .allocation_unit_size = 16 * 1024   // 格式化单元大小。
    };

    sdmmc_card_t* card;
    esp_err_t ret = esp_vfs_fat_sdmmc_mount(mount_point, &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "------ SD 卡挂载：失败！%s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "------ SD 卡挂载：完成。挂载点：%s", mount_point);
    sdmmc_card_print_info(stdout, card);

    int log_dir_ret = app_sd_mkdir(APP_SD_LOG_DIR);// 创建日志目录。
    if (log_dir_ret == -1) {
        return ESP_FAIL;
    }
    int cache_dir_ret = app_sd_mkdir(APP_SD_CACHE_DIR);// 创建缓存目录。
    if (cache_dir_ret == -1) {
        return ESP_FAIL;
    }

    int log_bak_count = app_sd_count_bak_files(APP_SD_LOG_DIR);// 清理多余的日志备份文件。
    if (log_bak_count > 100) {
        app_sd_delete_bak_files(APP_SD_LOG_DIR);
    }
    int cache_bak_count = app_sd_count_bak_files(APP_SD_CACHE_DIR);// 清理多余的缓存备份文件。
    if (cache_bak_count > 100) {
        app_sd_delete_bak_files(APP_SD_CACHE_DIR);
    }

    app_sd_create_log_file();
    app_sd_create_cache_file();
    app_sd_init_status = 1;
    return ESP_OK;
}