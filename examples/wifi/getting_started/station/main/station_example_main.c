/* WiFi Station with UART to TCP Bridge (Optimized for ESP8266)

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <fcntl.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "tcpip_adapter.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"
#include <lwip/netdb.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include <regex.h>
#include "esp_timer.h"
#include "esp_task_wdt.h"  // TWDT 支持

#define DEFAULT_ESP_WIFI_SSID      "FUNLIGHT"
#define DEFAULT_ESP_WIFI_PASS      "funlight"
#define EXAMPLE_ESP_MAXIMUM_RETRY  10
#define NVS_NAMESPACE              "config"
#define BOOT_COUNT_KEY             "boot_count"
#define BOOT_COUNT_THRESHOLD       6
#define BOOT_COUNT_RESET_DELAY_MS  10000 // 10 seconds

static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
#define HOST_IP_ADDR "192.168.4.1"
static const char *TAG = "slave";

#define PORT 12345
#define CONFIG_EXAMPLE_IPV4 1
#define UART_BUF_SIZE (512)
#define TCP_RETRY_DELAY_MS 2000
#define TCP_CONNECT_TIMEOUT_MS 10000
#define TCP_KEEPALIVE_MS 10000
#define RESPONSE_TIMEOUT_MS 5000

#define LED_PIN GPIO_NUM_2

static uint32_t current_baud_rate = 115200;
static uint8_t current_stop_bits = UART_STOP_BITS_1;
static uint8_t current_parity = UART_PARITY_DISABLE;
static uint8_t current_data_bits = UART_DATA_8_BITS;
static bool is_tcp_connected = false;
static int tcp_sock = -1;
static uint8_t debug_mode = 0;

/* Initialize UART0 */
static void uart_init(uint32_t baud_rate, uint8_t stop_bits, uint8_t parity, uint8_t data_bits)
{
    uart_config_t uart_config = {
        .baud_rate = baud_rate,
        .data_bits = data_bits,
        .parity = parity,
        .stop_bits = stop_bits,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    esp_err_t err = uart_param_config(UART_NUM_0, &uart_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART param config failed: %s", esp_err_to_name(err));
    }
    // 增大 TX buffer 到 2048，避免 queue 满阻塞
    err = uart_driver_install(UART_NUM_0, UART_BUF_SIZE * 2, UART_BUF_SIZE * 4, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART driver install failed: %s", esp_err_to_name(err));
    }
}

/* Check WiFi connection status */
static bool is_wifi_connected(void)
{
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, 100 / portTICK_PERIOD_MS);
    return (bits & WIFI_CONNECTED_BIT) != 0;
}

/* Initialize LED on GPIO2 */
static void led_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    esp_err_t err = gpio_config(&io_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "LED GPIO config failed: %s", esp_err_to_name(err));
    }
    gpio_set_level(LED_PIN, 1); // LED off (active-low) by default
}

/* Load configurations from NVS */
static esp_err_t load_nvs_config(uint32_t *baud_rate, uint8_t *stop_bits, uint8_t *parity, uint8_t *data_bits, char *ssid, size_t ssid_len, char *password, size_t pass_len, uint8_t *debug)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        *baud_rate = 115200;
        *stop_bits = UART_STOP_BITS_1;
        *parity = UART_PARITY_DISABLE;
        *data_bits = UART_DATA_8_BITS;
        strlcpy(ssid, DEFAULT_ESP_WIFI_SSID, ssid_len);
        strlcpy(password, DEFAULT_ESP_WIFI_PASS, pass_len);
        *debug = 0;
        return err;
    }

    size_t len = ssid_len;
    err = nvs_get_str(nvs_handle, "wifi_ssid", ssid, &len);
    if (err != ESP_OK || len == 0) {
        strlcpy(ssid, DEFAULT_ESP_WIFI_SSID, ssid_len);
    }
    len = pass_len;
    err = nvs_get_str(nvs_handle, "wifi_pass", password, &len);
    if (err != ESP_OK || len == 0) {
        strlcpy(password, DEFAULT_ESP_WIFI_PASS, pass_len);
    }
    err = nvs_get_u32(nvs_handle, "baud_rate", baud_rate);
    if (err != ESP_OK) {
        *baud_rate = 115200;
    }
    err = nvs_get_u8(nvs_handle, "stop_bits", stop_bits);
    if (err != ESP_OK) {
        *stop_bits = UART_STOP_BITS_1;
    }
    err = nvs_get_u8(nvs_handle, "parity", parity);
    if (err != ESP_OK) {
        *parity = UART_PARITY_DISABLE;
    }
    err = nvs_get_u8(nvs_handle, "data_bits", data_bits);
    if (err != ESP_OK) {
        *data_bits = UART_DATA_8_BITS;
    }
    err = nvs_get_u8(nvs_handle, "debug_mode", debug);
    if (err != ESP_OK) {
        *debug = 0;
    }
    nvs_close(nvs_handle);

    if (ssid[0] == '\0' || strlen(ssid) > 32 || !isprint((unsigned char)ssid[0])) {
        strlcpy(ssid, DEFAULT_ESP_WIFI_SSID, ssid_len);
    }
    size_t pass_length = strlen(password);
    if (pass_length > 0 && (pass_length < 8 || pass_length > 64 || !isprint((unsigned char)password[0]))) {
        strlcpy(password, DEFAULT_ESP_WIFI_PASS, pass_len);
    }

    esp_log_level_set(TAG, *debug ? ESP_LOG_INFO : ESP_LOG_NONE);
    return ESP_OK;
}

/* Save configurations to NVS */
static void save_nvs_config(uint32_t baud_rate, uint8_t stop_bits, uint8_t parity, uint8_t data_bits, const char *ssid, const char *password, uint8_t debug)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS for saving config: %s", esp_err_to_name(err));
        return;
    }
    if (ssid) {
        nvs_set_str(nvs_handle, "wifi_ssid", ssid);
    }
    if (password) {
        nvs_set_str(nvs_handle, "wifi_pass", password);
    }
    nvs_set_u32(nvs_handle, "baud_rate", baud_rate);
    nvs_set_u8(nvs_handle, "stop_bits", stop_bits);
    nvs_set_u8(nvs_handle, "parity", parity);
    nvs_set_u8(nvs_handle, "data_bits", data_bits);
    nvs_set_u8(nvs_handle, "debug_mode", debug);
    err = nvs_commit(nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to commit NVS config: %s", esp_err_to_name(err));
    }
    nvs_close(nvs_handle);
}

/* Print raw data (binary-safe) to UART0 */
static SemaphoreHandle_t raw_log_mutex = NULL;

static void raw_log_init(void)
{
    if (!raw_log_mutex) {
        raw_log_mutex = xSemaphoreCreateMutex();
        if (!raw_log_mutex) {
            ESP_LOGE(TAG, "Failed to create raw log mutex");
        }
    }
}

static void print_raw_data(const uint8_t *data, size_t len)
{
    if (!data || len == 0) return;

    // 喂 TWDT 前
    esp_task_wdt_reset();

    static bool initialized = false;
    if (!initialized) {
        raw_log_init();
        initialized = true;
    }

    if (raw_log_mutex && xSemaphoreTake(raw_log_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        int written = uart_write_bytes(UART_NUM_0, (const char *)data, len);
        if (written != len) {
            ESP_LOGW(TAG, "UART write incomplete: %d/%zu bytes", written, len);
        }
        // 移除 uart_wait_tx_done()：避免阻塞，允许异步 TX
        xSemaphoreGive(raw_log_mutex);
    } else {
        uart_write_bytes(UART_NUM_0, (const char *)data, len);
        // 移除 uart_wait_tx_done()
    }

    // 喂 TWDT 后
    esp_task_wdt_reset();
}

/* TCP send data with optional response expectation */
static esp_err_t tcp_send_data(int sock, const char *payload, size_t payload_len, char *rx_buffer, size_t rx_buffer_size, bool expect_response)
{
    int err = send(sock, payload, payload_len, 0);
    if (err < 0) {
        ESP_LOGE(TAG, "Error during sending: errno %d", errno);
        return ESP_FAIL;
    }

    if (!expect_response) {
        return ESP_OK;
    }

    struct timeval timeout;
    timeout.tv_sec = RESPONSE_TIMEOUT_MS / 1000;
    timeout.tv_usec = (RESPONSE_TIMEOUT_MS % 1000) * 1000;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    int len = recv(sock, rx_buffer, rx_buffer_size - 1, 0);
    if (len < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            ESP_LOGE(TAG, "No response from server within %d ms", RESPONSE_TIMEOUT_MS);
            return ESP_FAIL;
        }
        ESP_LOGE(TAG, "recv failed: errno %d", errno);
        return ESP_FAIL;
    }

    print_raw_data((uint8_t *)rx_buffer, len);
    return ESP_OK;
}

static esp_err_t parse_tcp_command(int sock, const uint8_t *rx_buffer, size_t len)
{
    if (!rx_buffer || len == 0) {
        return ESP_FAIL;
    }

    // 快速路径：如果含 \r\n，假设命令；否则 passthrough
    bool is_likely_command = (memchr(rx_buffer, '\r', len) != NULL || memchr(rx_buffer, '\n', len) != NULL);
    if (!is_likely_command) {
        print_raw_data(rx_buffer, len);
        return ESP_OK;
    }

    // Check if potential BAUD command
    if (len >= 5 && memcmp(rx_buffer, "BAUD=", 5) == 0) {
        // Check for embedded 0x00; if present, treat as transparent binary data
        if (memchr(rx_buffer, 0, len) != NULL) {
            print_raw_data(rx_buffer, len);
            return ESP_OK;
        }

        // Safe to make null-terminated copy
        char *buf = (char *)malloc(len + 1);
        if (!buf) {
            ESP_LOGE(TAG, "Failed to allocate buf for command");
            return ESP_FAIL;
        }
        memcpy(buf, rx_buffer, len);
        buf[len] = 0;

        // Trim to first \r\n if present
        char *cmd_end = strstr(buf, "\r\n");
        if (cmd_end) {
            *cmd_end = 0;
        }

        // Regex pattern
        const char *pattern = "^BAUD=([0-9]+),STOPBIT=([0-9]+),PARITY=([0-9]+),DATABIT=([0-9]+)(\r\n)?$";
        regex_t regex;
        regmatch_t matches[5];
        int ret = regcomp(&regex, pattern, REG_EXTENDED);
        if (ret != 0) {
            ESP_LOGE(TAG, "Failed to compile regex: %d", ret);
            const char *response = "Internal error: regex compilation failed\r\n";
            send(sock, response, strlen(response), 0);
            free(buf);
            regfree(&regex);
            return ESP_FAIL;
        }

        ret = regexec(&regex, buf, 5, matches, 0);
        if (ret != 0) {
            ESP_LOGE(TAG, "Invalid BAUD command format: no match for '%s'", buf);
            const char *response = "Invalid BAUD command format\r\n";
            send(sock, response, strlen(response), 0);
            free(buf);
            regfree(&regex);
            return ESP_FAIL;
        }

        // Extract and convert each field
        char *endptr;
        char baud_str[32] = {0};
        size_t baud_len = matches[1].rm_eo - matches[1].rm_so;
        if (baud_len >= sizeof(baud_str)) {
            ESP_LOGE(TAG, "BAUD value too long");
            const char *response = "Invalid baud rate\r\n";
            send(sock, response, strlen(response), 0);
            free(buf);
            regfree(&regex);
            return ESP_FAIL;
        }
        strncpy(baud_str, buf + matches[1].rm_so, baud_len);
        uint32_t new_baud = strtoul(baud_str, &endptr, 10);
        if (*endptr != 0) {
            ESP_LOGE(TAG, "Invalid BAUD value: non-numeric '%s'", baud_str);
            const char *response = "Invalid baud rate\r\n";
            send(sock, response, strlen(response), 0);
            free(buf);
            regfree(&regex);
            return ESP_FAIL;
        }

        // STOPBIT
        char stopbit_str[32] = {0};
        size_t stopbit_len = matches[2].rm_eo - matches[2].rm_so;
        if (stopbit_len >= sizeof(stopbit_str)) {
            ESP_LOGE(TAG, "STOPBIT value too long");
            const char *response = "Invalid stop bits\r\n";
            send(sock, response, strlen(response), 0);
            free(buf);
            regfree(&regex);
            return ESP_FAIL;
        }
        strncpy(stopbit_str, buf + matches[2].rm_so, stopbit_len);
        uint8_t raw_stop_bits = strtoul(stopbit_str, &endptr, 10);
        if (*endptr != 0) {
            ESP_LOGE(TAG, "Invalid STOPBIT value: non-numeric '%s'", stopbit_str);
            const char *response = "Invalid stop bits\r\n";
            send(sock, response, strlen(response), 0);
            free(buf);
            regfree(&regex);
            return ESP_FAIL;
        }
        uint8_t new_stop_bits = raw_stop_bits + 1;

        // PARITY
        char parity_str[32] = {0};
        size_t parity_len = matches[3].rm_eo - matches[3].rm_so;
        if (parity_len >= sizeof(parity_str)) {
            ESP_LOGE(TAG, "PARITY value too long");
            const char *response = "Invalid parity\r\n";
            send(sock, response, strlen(response), 0);
            free(buf);
            regfree(&regex);
            return ESP_FAIL;
        }
        strncpy(parity_str, buf + matches[3].rm_so, parity_len);
        uint8_t new_parity = strtoul(parity_str, &endptr, 10);
        if (*endptr != 0) {
            ESP_LOGE(TAG, "Invalid PARITY value: non-numeric '%s'", parity_str);
            const char *response = "Invalid parity\r\n";
            send(sock, response, strlen(response), 0);
            free(buf);
            regfree(&regex);
            return ESP_FAIL;
        }
        if (new_parity == 1) new_parity = 3;

        // DATABIT
        char databit_str[32] = {0};
        size_t databit_len = matches[4].rm_eo - matches[4].rm_so;
        if (databit_len >= sizeof(databit_str)) {
            ESP_LOGE(TAG, "DATABIT value too long");
            const char *response = "Invalid data bits\r\n";
            send(sock, response, strlen(response), 0);
            free(buf);
            regfree(&regex);
            return ESP_FAIL;
        }
        strncpy(databit_str, buf + matches[4].rm_so, databit_len);
        uint8_t raw_data_bits = strtoul(databit_str, &endptr, 10);
        if (*endptr != 0) {
            ESP_LOGE(TAG, "Invalid DATABIT value: non-numeric '%s'", databit_str);
            const char *response = "Invalid data bits\r\n";
            send(sock, response, strlen(response), 0);
            free(buf);
            regfree(&regex);
            return ESP_FAIL;
        }

        regfree(&regex);
        free(buf);

        // Validate parameters
        if (new_baud < 110 || new_baud > 2000000) {
            ESP_LOGE(TAG, "Invalid baud rate: %u", new_baud);
            const char *response = "Invalid baud rate\r\n";
            send(sock, response, strlen(response), 0);
            return ESP_FAIL;
        }

        if (new_stop_bits != UART_STOP_BITS_1 && new_stop_bits != UART_STOP_BITS_2) {
            ESP_LOGE(TAG, "Invalid stop bits: %u (raw value: %u)", new_stop_bits, raw_stop_bits);
            const char *response = "Invalid stop bits\r\n";
            send(sock, response, strlen(response), 0);
            return ESP_FAIL;
        }

        if (new_parity != UART_PARITY_DISABLE && new_parity != UART_PARITY_ODD && new_parity != UART_PARITY_EVEN) {
            ESP_LOGE(TAG, "Invalid parity: %u", new_parity);
            const char *response = "Invalid parity\r\n";
            send(sock, response, strlen(response), 0);
            return ESP_FAIL;
        }

        // raw_data_bits is from strtoul (unsigned), so check range before subtracting
        if (raw_data_bits < 5 || raw_data_bits > 8) {
            ESP_LOGE(TAG, "Invalid data bits: %u", raw_data_bits);
            const char *response = "Invalid data bits\r\n";
            send(sock, response, strlen(response), 0);
            return ESP_FAIL;
        }
        uint8_t new_data_bits = raw_data_bits - 5;

        // Apply UART configuration
        uart_set_baudrate(UART_NUM_0, new_baud);
        uart_set_stop_bits(UART_NUM_0, new_stop_bits);
        uart_set_parity(UART_NUM_0, new_parity);
        uart_set_word_length(UART_NUM_0, new_data_bits);

        // Update global variables and save to NVS
        current_baud_rate = new_baud;
        current_stop_bits = new_stop_bits;
        current_parity = new_parity;
        current_data_bits = new_data_bits;
        save_nvs_config(new_baud, new_stop_bits, new_parity, new_data_bits, NULL, NULL, debug_mode);
        return ESP_OK;
    } else if (len >= 5 && memcmp(rx_buffer, "WIFI=", 5) == 0) {
        // Similar fix for WIFI command: check for 0x00
        if (memchr(rx_buffer, 0, len) != NULL) {
            print_raw_data(rx_buffer, len);
            return ESP_OK;
        }

        // Safe to make null-terminated copy
        char *buf = (char *)malloc(len + 1);
        if (!buf) {
            ESP_LOGE(TAG, "Failed to allocate buf for command");
            return ESP_FAIL;
        }
        memcpy(buf, rx_buffer, len);
        buf[len] = 0;

        char *ssid = buf + 5;
        char *password = strchr(ssid, ',');
        if (password) {
            *password = 0;
            password++;
            char *end = strchr(password, '\r');
            if (end) *end = 0;

            if (strlen(ssid) == 0 || strlen(ssid) > 32 || !isprint((unsigned char)ssid[0])) {
                ESP_LOGE(TAG, "Invalid SSID from server: %s", ssid);
                const char *response = "Invalid SSID\r\n";
                send(sock, response, strlen(response), 0);
                free(buf);
                return ESP_FAIL;
            } else if (strlen(password) > 0 && (strlen(password) < 8 || strlen(password) > 64 || !isprint((unsigned char)password[0]))) {
                ESP_LOGE(TAG, "Invalid password from server");
                const char *response = "Invalid password\r\n";
                send(sock, response, strlen(response), 0);
                free(buf);
                return ESP_FAIL;
            } else {
                save_nvs_config(current_baud_rate, current_stop_bits, current_parity, current_data_bits, ssid, password, debug_mode);
                ESP_LOGI(TAG, "WiFi configuration from server successful: SSID=%s", ssid);
                free(buf);
                esp_restart();
                return ESP_OK;
            }
        } else {
            ESP_LOGE(TAG, "Invalid WIFI command format from server");
            const char *response = "Invalid WIFI command format\r\n";
            send(sock, response, strlen(response), 0);
            free(buf);
            return ESP_FAIL;
        }
    } else {
        // Not a command: transparent binary passthrough, send full len to UART
        print_raw_data(rx_buffer, len);
        return ESP_OK;
    }
}

static void tcp_receive_task(void *pvParameters)
{
    uint8_t *rx_buffer = (uint8_t *) malloc(UART_BUF_SIZE);  // FIX: Changed to uint8_t* for binary
    if (!rx_buffer) {
        ESP_LOGE(TAG, "Failed to allocate memory for receive buffer");
        vTaskDelete(NULL);
    }

    while (1) {
        if (!is_tcp_connected || tcp_sock == -1) {
            vTaskDelay(500 / portTICK_PERIOD_MS);  // 延长无连接 delay，避免 CPU 空转
            esp_task_wdt_reset();  // 喂狗
            continue;
        }

        int len = recv(tcp_sock, rx_buffer, UART_BUF_SIZE - 1, 0);
        if (len < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                vTaskDelay(5 / portTICK_PERIOD_MS);  // 缩短到 5ms，更及时 poll
                esp_task_wdt_reset();
                continue;
            }
            ESP_LOGW(TAG, "Receive failed: errno %d, reconnecting...", errno);  // 启用警告日志
            is_tcp_connected = false;
            close(tcp_sock);
            tcp_sock = -1;
            vTaskDelay(TCP_RETRY_DELAY_MS / portTICK_PERIOD_MS);
            esp_task_wdt_reset();
            continue;
        } else if (len == 0) {
            ESP_LOGI(TAG, "Connection closed by server, reconnecting...");
            is_tcp_connected = false;
            close(tcp_sock);
            tcp_sock = -1;
            vTaskDelay(TCP_RETRY_DELAY_MS / portTICK_PERIOD_MS);
            esp_task_wdt_reset();
            continue;
        }

        // 优化 debug hex log：len > 200 时仅摘要，避免慢循环
        if (debug_mode) {
            if (len <= 200) {
                char hex_buf[UART_BUF_SIZE * 3 + 1] = {0};  // +1 for null
                char *ptr = hex_buf;
                for (int i = 0; i < len; i++) {
                    ptr += sprintf(ptr, "%02X ", rx_buffer[i]);  // sprintf 直接追加，无 strlen
                }
                ESP_LOGI(TAG, "Raw TCP receive buffer (hex): %s (length: %d)", hex_buf, len);
            } else {
                ESP_LOGI(TAG, "Large TCP packet received: %d bytes (hex log skipped)", len);
            }
        }

        // Parse and process the received data
        esp_err_t result = parse_tcp_command(tcp_sock, rx_buffer, len);
        if (result != ESP_OK) {
            ESP_LOGW(TAG, "Failed to parse TCP command (len=%d)", len);
        }

        vTaskDelay(1 / portTICK_PERIOD_MS);  // 缩短到 1ms，提高 yield 频率
        esp_task_wdt_reset();  // 每循环喂狗
    }

    free(rx_buffer);
    vTaskDelete(NULL);
}

/* UART command and TCP bridge task */
static void uart_tcp_bridge_task(void *pvParameters)
{
    uint8_t *uart_data = (uint8_t *) malloc(UART_BUF_SIZE);
    char *rx_buffer = (char *) malloc(UART_BUF_SIZE);

    if (!uart_data || !rx_buffer) {
        ESP_LOGE(TAG, "Failed to allocate memory for buffers");
        vTaskDelete(NULL);
    }

    while (1) {
        int len = uart_read_bytes(UART_NUM_0, uart_data, UART_BUF_SIZE - 1, 20 / portTICK_PERIOD_MS);
        if (len > 0) {
            uart_data[len] = 0;
            char *cmd = (char *)uart_data;
            if (debug_mode) ESP_LOGI(TAG, "Raw UART input: %s", cmd);

            if (tcp_sock != -1) {
                esp_err_t send_result = tcp_send_data(tcp_sock, cmd, len, rx_buffer, UART_BUF_SIZE, false);
                if (send_result != ESP_OK) {
                    ESP_LOGW(TAG, "Failed to send UART data over TCP, reconnecting...");
                    close(tcp_sock);
                    tcp_sock = -1;
                    is_tcp_connected = false;
                } else if (debug_mode) {
                    ESP_LOGI(TAG, "Transparent message sent: %s", cmd);
                }
            } else {
                ESP_LOGE(TAG, "TCP not connected, cannot send data");
                // print_raw_log("TCP not connected\r\n");
            }
        }

        if (!is_wifi_connected()) {
            if (debug_mode) ESP_LOGW(TAG, "WiFi not connected, closing socket");
            if (tcp_sock != -1) {
                close(tcp_sock);
                tcp_sock = -1;
                is_tcp_connected = false;
            }
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            esp_task_wdt_reset();
            continue;
        }

        if (tcp_sock == -1 && is_wifi_connected()) {
            struct sockaddr_in dest_addr;
            dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
            dest_addr.sin_family = AF_INET;
            dest_addr.sin_port = htons(PORT);

            tcp_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
            if (tcp_sock < 0) {
                ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
                vTaskDelay(TCP_RETRY_DELAY_MS / portTICK_PERIOD_MS);
                esp_task_wdt_reset();
                continue;
            }

            struct timeval timeout;
            timeout.tv_sec = TCP_CONNECT_TIMEOUT_MS / 1000;
            timeout.tv_usec = (TCP_CONNECT_TIMEOUT_MS % 1000) * 1000;
            setsockopt(tcp_sock, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
            setsockopt(tcp_sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

            int keepalive = 1;
            int keepidle = TCP_KEEPALIVE_MS / 1000;
            int keepintvl = 2;
            int keepcnt = 3;
            setsockopt(tcp_sock, SOL_SOCKET, SO_KEEPALIVE, &keepalive, sizeof(keepalive));
            setsockopt(tcp_sock, IPPROTO_TCP, TCP_KEEPIDLE, &keepidle, sizeof(keepidle));
            setsockopt(tcp_sock, IPPROTO_TCP, TCP_KEEPINTVL, &keepintvl, sizeof(keepintvl));
            setsockopt(tcp_sock, IPPROTO_TCP, TCP_KEEPCNT, &keepcnt, sizeof(keepcnt));

            int err = connect(tcp_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
            if (err != 0) {
                ESP_LOGW(TAG, "Socket unable to connect to %s:%d: errno %d", HOST_IP_ADDR, PORT, errno);
                close(tcp_sock);
                tcp_sock = -1;
                is_tcp_connected = false;
                vTaskDelay(TCP_RETRY_DELAY_MS / portTICK_PERIOD_MS);
                esp_task_wdt_reset();
                continue;
            }

            // 新增：设为非阻塞
            int flags = fcntl(tcp_sock, F_GETFL, 0);
            if (flags < 0) {
                ESP_LOGE(TAG, "fcntl F_GETFL failed: errno %d", errno);
                close(tcp_sock);
                tcp_sock = -1;
                is_tcp_connected = false;
                vTaskDelay(TCP_RETRY_DELAY_MS / portTICK_PERIOD_MS);
                esp_task_wdt_reset();
                continue;
            }
            if (fcntl(tcp_sock, F_SETFL, flags | O_NONBLOCK) < 0) {
                ESP_LOGE(TAG, "fcntl F_SETFL O_NONBLOCK failed: errno %d", errno);
                close(tcp_sock);
                tcp_sock = -1;
                is_tcp_connected = false;
                vTaskDelay(TCP_RETRY_DELAY_MS / portTICK_PERIOD_MS);
                esp_task_wdt_reset();
                continue;
            }

            // ESP_LOGI(TAG, "Successfully connected to %s:%d", HOST_IP_ADDR, PORT);
            is_tcp_connected = true;
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
        esp_task_wdt_reset();  // 每循环喂狗
    }

    if (tcp_sock != -1) {
        close(tcp_sock);
        tcp_sock = -1;
        is_tcp_connected = false;
    }
    free(uart_data);
    free(rx_buffer);
    vTaskDelete(NULL);
}

/* Timer callback to reset boot count */
static void boot_count_reset_timer_callback(void *arg)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err == ESP_OK) {
        err = nvs_set_u32(nvs_handle, BOOT_COUNT_KEY, 0);
        if (err == ESP_OK) {
            err = nvs_commit(nvs_handle);
            if (err == ESP_OK && debug_mode) {
                ESP_LOGI(TAG, "Boot count reset to 0 after %d ms uptime", BOOT_COUNT_RESET_DELAY_MS);
            } else if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to commit boot count reset: %s", esp_err_to_name(err));
            }
        } else {
            ESP_LOGE(TAG, "Failed to set boot count to 0: %s", esp_err_to_name(err));
        }
        nvs_close(nvs_handle);
    } else {
        ESP_LOGE(TAG, "Failed to open NVS for boot count reset: %s", esp_err_to_name(err));
    }
}

/* Handle boot count increment and check for factory reset */
static void handle_boot_count(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS for boot count: %s", esp_err_to_name(err));
        return;
    }

    uint32_t boot_count = 0;
    err = nvs_get_u32(nvs_handle, BOOT_COUNT_KEY, &boot_count);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        nvs_close(nvs_handle);
        return;
    }

    boot_count++;
    err = nvs_set_u32(nvs_handle, BOOT_COUNT_KEY, boot_count);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write boot count: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return;
    }
    err = nvs_commit(nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to commit boot count: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return;
    }

    if (boot_count > BOOT_COUNT_THRESHOLD) {
        ESP_LOGW(TAG, "Boot count %u exceeds threshold %d, resetting to factory defaults", 
                 boot_count, BOOT_COUNT_THRESHOLD);
        nvs_erase_all(nvs_handle);
        nvs_commit(nvs_handle);
        nvs_close(nvs_handle);
        esp_restart();
    }

    nvs_close(nvs_handle);

    // Start one-shot timer for boot count reset
    esp_timer_handle_t boot_count_timer;
    const esp_timer_create_args_t timer_args = {
        .callback = &boot_count_reset_timer_callback,
        .name = "boot_count_reset"
    };
    err = esp_timer_create(&timer_args, &boot_count_timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create boot count reset timer: %s", esp_err_to_name(err));
        return;
    }
    err = esp_timer_start_once(boot_count_timer, BOOT_COUNT_RESET_DELAY_MS * 1000);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start boot count reset timer: %s", esp_err_to_name(err));
        esp_timer_delete(boot_count_timer);
    }
}

static int s_retry_num = 0;

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch (event->event_id) {
        case SYSTEM_EVENT_STA_START:
            esp_wifi_connect();
            break;
        case SYSTEM_EVENT_STA_GOT_IP:
            // ESP_LOGI(TAG, "got ip:%s", ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
            s_retry_num = 0;
            xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
            xEventGroupClearBits(s_wifi_event_group, WIFI_FAIL_BIT);
            gpio_set_level(LED_PIN, 0); // LED on (active-low)
            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
            xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            gpio_set_level(LED_PIN, 1); // LED off (active-low)
            esp_wifi_connect();
            s_retry_num++;
            if (s_retry_num >= EXAMPLE_ESP_MAXIMUM_RETRY) {
                // ESP_LOGW(TAG, "Max WiFi retries reached, continuing to retry...");
            }
            break;
        default:
            break;
    }
    return ESP_OK;
}

void wifi_init_sta(const char *ssid, const char *password)
{
    s_wifi_event_group = xEventGroupCreate();
    tcpip_adapter_init();
    esp_err_t err = esp_event_loop_init(event_handler, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Event loop init failed: %s", esp_err_to_name(err));
        return;
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "WiFi init failed: %s", esp_err_to_name(err));
        return;
    }

    wifi_config_t wifi_config = {
        .sta = {
            .channel = 0,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK
        },
    };
    strlcpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));
    strlcpy((char *)wifi_config.sta.password, password, sizeof(wifi_config.sta.password));

    err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Set WiFi mode failed: %s", esp_err_to_name(err));
        return;
    }
    err = esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Set WiFi config failed: %s", esp_err_to_name(err));
        return;
    }
    err = esp_wifi_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "WiFi start failed: %s", esp_err_to_name(err));
        return;
    }

    err = esp_wifi_set_max_tx_power(82);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Set WiFi TX power failed: %s", esp_err_to_name(err));
    }
}

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err != ESP_OK) {
        nvs_flash_erase();
        err = nvs_flash_init();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "NVS flash init failed: %s", esp_err_to_name(err));
            return;
        }
    }

    // 初始化 TWDT：ESP8266 无参数版本
    esp_task_wdt_init();

    char ssid[32] = {0};
    char password[64] = {0};
    err = load_nvs_config(&current_baud_rate, &current_stop_bits, &current_parity, &current_data_bits, ssid, sizeof(ssid), password, sizeof(password), &debug_mode);
    if (err != ESP_OK) {
        strlcpy(ssid, DEFAULT_ESP_WIFI_SSID, sizeof(ssid));
        strlcpy(password, DEFAULT_ESP_WIFI_PASS, sizeof(password));
        current_baud_rate = 115200;
        current_stop_bits = UART_STOP_BITS_1;
        current_parity = UART_PARITY_DISABLE;
        current_data_bits = UART_DATA_8_BITS;
        debug_mode = 0;
        esp_log_level_set(TAG, ESP_LOG_NONE);
    }

    handle_boot_count();
    led_init();
    wifi_init_sta(ssid, password);
    uart_init(current_baud_rate, current_stop_bits, current_parity, current_data_bits);
    xTaskCreate(uart_tcp_bridge_task, "uart_tcp_bridge", 6144, NULL, 8, NULL);
    xTaskCreate(tcp_receive_task, "tcp_receive", 6144, NULL, 8, NULL);
}