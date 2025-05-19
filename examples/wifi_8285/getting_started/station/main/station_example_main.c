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

#define LED_PIN GPIO_NUM_4
#define LED_BLINK_PERIOD_MS 500
#define RSSI_LOG_INTERVAL_MS 5000

static uint32_t current_baud_rate = 115200;
static bool is_tcp_connected = false;
static int tcp_sock = -1;
static uint8_t debug_mode = 0;

/* Initialize UART0 */
static void uart_init(uint32_t baud_rate)
{
    uart_config_t uart_config = {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    esp_err_t err = uart_param_config(UART_NUM_0, &uart_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART param config failed: %s", esp_err_to_name(err));
    }
    err = uart_driver_install(UART_NUM_0, UART_BUF_SIZE * 2, 0, 0, NULL, 0);
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
    gpio_set_level(LED_PIN, 1); // LED off (active-low)
}

/* Get WiFi RSSI */
static int8_t get_wifi_rssi(void)
{
    wifi_ap_record_t ap_info;
    esp_err_t err = esp_wifi_sta_get_ap_info(&ap_info);
    if (err == ESP_OK) {
        return ap_info.rssi;
    } else {
        if (debug_mode) ESP_LOGE(TAG, "Failed to get RSSI: %s", esp_err_to_name(err));
        return INT8_MIN;
    }
}

/* LED control and RSSI logging task */
static void led_task(void *pvParameters)
{
    bool led_state = false;
    TickType_t last_rssi_time = 0;
    while (1) {
        if (!is_wifi_connected()) {
            if (debug_mode) ESP_LOGI(TAG, "LED off: WiFi disconnected");
            gpio_set_level(LED_PIN, 1); // LED off
        } else if (!is_tcp_connected) {
            led_state = !led_state;
            if (debug_mode) ESP_LOGI(TAG, "LED blink: WiFi connected, TCP disconnected");
            gpio_set_level(LED_PIN, led_state ? 0 : 1); // LED blinking
        } else {
            if (debug_mode) ESP_LOGI(TAG, "LED on: TCP connected");
            gpio_set_level(LED_PIN, 0); // LED on
        }

        if (debug_mode && is_wifi_connected()) {
            TickType_t current_time = xTaskGetTickCount();
            if ((current_time - last_rssi_time) * portTICK_PERIOD_MS >= RSSI_LOG_INTERVAL_MS) {
                int8_t rssi = get_wifi_rssi();
                if (rssi != INT8_MIN) {
                    ESP_LOGI(TAG, "WiFi RSSI: %d dBm", rssi);
                }
                last_rssi_time = current_time;
            }
        }

        vTaskDelay(LED_BLINK_PERIOD_MS / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

/* Load configurations from NVS */
static esp_err_t load_nvs_config(uint32_t *baud_rate, char *ssid, size_t ssid_len, char *password, size_t pass_len, uint8_t *debug)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        *baud_rate = 115200;
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
static void save_nvs_config(uint32_t baud_rate, const char *ssid, const char *password, uint8_t debug)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        return;
    }
    if (ssid) {
        nvs_set_str(nvs_handle, "wifi_ssid", ssid);
    }
    if (password) {
        nvs_set_str(nvs_handle, "wifi_pass", password);
    }
    nvs_set_u32(nvs_handle, "baud_rate", baud_rate);
    nvs_set_u8(nvs_handle, "debug_mode", debug);
    nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
}

/* Reset to factory defaults */
static void reset_to_defaults(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        return;
    }
    nvs_erase_all(nvs_handle);
    nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
    esp_restart();
}

/* Print raw log without formatting to UART0 */
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

static void print_raw_log(const char *data)
{
    if (!data) return;

    static bool initialized = false;
    if (!initialized) {
        raw_log_init();
        initialized = true;
    }

    size_t len = strlen(data);

    if (raw_log_mutex && xSemaphoreTake(raw_log_mutex, portMAX_DELAY) == pdTRUE) {
        uart_write_bytes(UART_NUM_0, data, len);
        uart_wait_tx_done(UART_NUM_0, 100 / portTICK_PERIOD_MS);
        xSemaphoreGive(raw_log_mutex);
    } else {
        uart_write_bytes(UART_NUM_0, data, len);
        uart_wait_tx_done(UART_NUM_0, 100 / portTICK_PERIOD_MS);
    }
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

    rx_buffer[len] = 0;
    print_raw_log(rx_buffer);
    return ESP_OK;
}

/* TCP receive task for server-initiated messages */
static void tcp_receive_task(void *pvParameters)
{
    char *rx_buffer = (char *) malloc(UART_BUF_SIZE);
    if (!rx_buffer) {
        ESP_LOGE(TAG, "Failed to allocate memory for receive buffer");
        vTaskDelete(NULL);
    }

    while (1) {
        if (!is_tcp_connected || tcp_sock == -1) {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }

        int len = recv(tcp_sock, rx_buffer, UART_BUF_SIZE - 1, 0);
        if (len < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                vTaskDelay(10 / portTICK_PERIOD_MS);
                continue;
            }
            ESP_LOGW(TAG, "Receive failed: errno %d, reconnecting...", errno);
            is_tcp_connected = false;
            close(tcp_sock);
            tcp_sock = -1;
            vTaskDelay(TCP_RETRY_DELAY_MS / portTICK_PERIOD_MS);
            continue;
        } else if (len == 0) {
            ESP_LOGI(TAG, "Connection closed by server");
            is_tcp_connected = false;
            close(tcp_sock);
            tcp_sock = -1;
            vTaskDelay(TCP_RETRY_DELAY_MS / portTICK_PERIOD_MS);
            continue;
        }

        rx_buffer[len] = 0;

        if (strncmp(rx_buffer, "BAUD=", 5) == 0) {
            uint32_t new_baud = atoi(rx_buffer + 5);
            if (new_baud >= 9600 && new_baud <= 921600) {
                ESP_LOGI(TAG, "Received server baud rate change request: %u", new_baud);
                uart_set_baudrate(UART_NUM_0, new_baud);
                current_baud_rate = new_baud;
                save_nvs_config(new_baud, NULL, NULL, debug_mode);
                const char *response = "OK\r\n";
                send(tcp_sock, response, strlen(response), 0);
                ESP_LOGI(TAG, "Baud rate changed to %u", new_baud);
            } else {
                ESP_LOGE(TAG, "Invalid baud rate from server: %s", rx_buffer + 5);
                const char *response = "Invalid baud rate\r\n";
                send(tcp_sock, response, strlen(response), 0);
            }
        } else if (strncmp(rx_buffer, "WIFI=", 5) == 0) {
            char *ssid = rx_buffer + 5;
            char *password = strchr(ssid, ',');
            if (password) {
                *password = 0;
                password++;
                char *end = strchr(password, '\r');
                if (end) *end = 0;
                if (strlen(ssid) == 0 || strlen(ssid) > 32 || !isprint((unsigned char)ssid[0])) {
                    ESP_LOGE(TAG, "Invalid SSID from server: %s", ssid);
                    const char *response = "Invalid SSID\r\n";
                    send(tcp_sock, response, strlen(response), 0);
                } else if (strlen(password) > 0 && (strlen(password) < 8 || strlen(password) > 64 || !isprint((unsigned char)password[0]))) {
                    ESP_LOGE(TAG, "Invalid password from server");
                    const char *response = "Invalid password\r\n";
                    send(tcp_sock, response, strlen(response), 0);
                } else {
                    save_nvs_config(current_baud_rate, ssid, password, debug_mode);
                    ESP_LOGI(TAG, "WiFi configuration from server successful: SSID=%s", ssid);
                    const char *response = "OK\r\n";
                    send(tcp_sock, response, strlen(response), 0);
                    esp_restart();
                }
            } else {
                ESP_LOGE(TAG, "Invalid WIFI command format from server");
                const char *response = "Invalid WIFI command format\r\n";
                send(tcp_sock, response, strlen(response), 0);
            }
        } else if (strncmp(rx_buffer, "DEBUG=", 6) == 0) {
            uint8_t new_debug = atoi(rx_buffer + 6);
            if (new_debug == 0 || new_debug == 1) {
                ESP_LOGI(TAG, "Received server debug mode change request: %u", new_debug);
                debug_mode = new_debug;
                esp_log_level_set(TAG, debug_mode ? ESP_LOG_INFO : ESP_LOG_NONE);
                save_nvs_config(current_baud_rate, NULL, NULL, debug_mode);
                const char *response = "OK\r\n";
                send(tcp_sock, response, strlen(response), 0);
                ESP_LOGI(TAG, "Debug mode set to %u", new_debug);
            } else {
                ESP_LOGE(TAG, "Invalid debug mode from server: %s", rx_buffer + 6);
                const char *response = "Invalid debug mode\r\n";
                send(tcp_sock, response, strlen(response), 0);
            }
        } else {
            print_raw_log(rx_buffer);
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
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

            bool expect_response = false;
            bool is_command = false;

            if (strncmp(cmd, "BAUD=", 5) == 0 || strncmp(cmd, "WIFI=", 5) == 0 ||
                strncmp(cmd, "DEBUG=", 6) == 0 || strcmp(cmd, "RSTDEFAULT\r\n") == 0) {
                is_command = true;
                if (len < 2 || cmd[len - 2] != '\r' || cmd[len - 1] != '\n') {
                    ESP_LOGE(TAG, "Command not properly terminated with \\r\\n");
                    print_raw_log("Command must end with \\r\\n\r\n");
                    continue;
                }
            }

            if (strncmp(cmd, "BAUD=", 5) == 0) {
                uint32_t new_baud = atoi(cmd + 5);
                if (new_baud < 9600 || new_baud > 921600) {
                    ESP_LOGE(TAG, "Invalid baud rate: %s", cmd + 5);
                    print_raw_log("Invalid baud rate\r\n");
                    continue;
                }
                if (tcp_sock == -1) {
                    ESP_LOGE(TAG, "TCP not connected, cannot change baud rate");
                    print_raw_log("TCP not connected\r\n");
                    continue;
                }
                ESP_LOGI(TAG, "Updating local baud rate to %u", new_baud);
                uart_set_baudrate(UART_NUM_0, new_baud);
                current_baud_rate = new_baud;
                save_nvs_config(new_baud, NULL, NULL, debug_mode);
                expect_response = true;
            } else if (strncmp(cmd, "WIFI=", 5) == 0) {
                char *ssid = cmd + 5;
                char *password = strchr(ssid, ',');
                if (!password) {
                    ESP_LOGE(TAG, "Invalid WIFI command format: missing comma");
                    print_raw_log("Invalid WIFI command format\r\n");
                    continue;
                }
                *password = 0;
                password++;
                char *end = strchr(password, '\r');
                if (end) *end = 0;
                if (strlen(ssid) == 0 || strlen(ssid) > 32 || !isprint((unsigned char)ssid[0])) {
                    ESP_LOGE(TAG, "Invalid SSID: %s", ssid);
                    print_raw_log("Invalid SSID\r\n");
                    continue;
                }
                size_t pass_length = strlen(password);
                if (pass_length > 0 && (pass_length < 8 || pass_length > 64 || !isprint((unsigned char)password[0]))) {
                    ESP_LOGE(TAG, "Invalid password length or characters");
                    print_raw_log("Invalid password\r\n");
                    continue;
                }
                if (tcp_sock == -1) {
                    ESP_LOGE(TAG, "TCP not connected, cannot change WiFi configuration");
                    print_raw_log("TCP not connected\r\n");
                    continue;
                }
                expect_response = true;
            } else if (strncmp(cmd, "DEBUG=", 6) == 0) {
                uint8_t new_debug = atoi(cmd + 6);
                if (new_debug != 0 && new_debug != 1) {
                    ESP_LOGE(TAG, "Invalid debug mode: %s", cmd + 6);
                    print_raw_log("Invalid debug mode\r\n");
                    continue;
                }
                if (tcp_sock == -1) {
                    ESP_LOGI(TAG, "TCP not connected, applying debug mode locally: %u", new_debug);
                    debug_mode = new_debug;
                    esp_log_level_set(TAG, debug_mode ? ESP_LOG_INFO : ESP_LOG_NONE);
                    save_nvs_config(current_baud_rate, NULL, NULL, debug_mode);
                    print_raw_log("OK\r\n");
                    continue;
                }
                expect_response = true;
            } else if (strcmp(cmd, "RSTDEFAULT\r\n") == 0) {
                ESP_LOGI(TAG, "Factory reset triggered");
                reset_to_defaults();
                continue;
            }

            if (tcp_sock != -1) {
                esp_err_t send_result = tcp_send_data(tcp_sock, cmd, len, rx_buffer, UART_BUF_SIZE, expect_response);
                if (send_result != ESP_OK) {
                    ESP_LOGW(TAG, "Failed to send UART data over TCP, reconnecting...");
                    close(tcp_sock);
                    tcp_sock = -1;
                    is_tcp_connected = false;
                    if (strncmp(cmd, "BAUD=", 5) == 0) {
                        ESP_LOGI(TAG, "Expecting server restart after BAUD change, retrying connection...");
                    }
                } else if (!is_command && debug_mode) {
                    ESP_LOGI(TAG, "Transparent message sent: %s", cmd);
                }
            } else {
                ESP_LOGE(TAG, "TCP not connected, cannot send data");
                print_raw_log("TCP not connected\r\n");
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
                continue;
            }
            ESP_LOGI(TAG, "Successfully connected to %s:%d", HOST_IP_ADDR, PORT);
            is_tcp_connected = true;
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
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

/* Task to reset boot count after 10 seconds uptime */
static void boot_count_reset_task(void *pvParameters)
{
    vTaskDelay(BOOT_COUNT_RESET_DELAY_MS / portTICK_PERIOD_MS);

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

    vTaskDelete(NULL);
}

/* Handle boot count increment and check for factory reset */
static void handle_boot_count(void)
{
    ESP_LOGI(TAG, "Set Boot count handle_boot_count");
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS for boot count: %s", esp_err_to_name(err));
        return;
    }

    uint32_t boot_count = 0;
    err = nvs_get_u32(nvs_handle, BOOT_COUNT_KEY, &boot_count);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGE(TAG, "Failed to read boot count: %s", esp_err_to_name(err));
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

    ESP_LOGI(TAG, "Read Boot count: %u", boot_count);
    err = nvs_commit(nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to commit boot count: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return;
    }

 
    ESP_LOGI(TAG, "Set Boot count: %u", boot_count);
    

    if (boot_count > BOOT_COUNT_THRESHOLD) {
        ESP_LOGW(TAG, "Boot count %u exceeds threshold %d, resetting to factory defaults", 
                 boot_count, BOOT_COUNT_THRESHOLD);
        nvs_erase_all(nvs_handle);
        nvs_commit(nvs_handle);
        nvs_close(nvs_handle);
        esp_restart();
    }

    nvs_close(nvs_handle);
}

static int s_retry_num = 0;

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch (event->event_id) {
        case SYSTEM_EVENT_STA_START:
            esp_wifi_connect();
            break;
        case SYSTEM_EVENT_STA_GOT_IP:
            ESP_LOGI(TAG, "got ip:%s", ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
            s_retry_num = 0;
            xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
            xEventGroupClearBits(s_wifi_event_group, WIFI_FAIL_BIT);
            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
            xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            ESP_LOGI(TAG, "Disconnected11111 from AP, retrying...");
            esp_wifi_connect();
            s_retry_num++;
            if (s_retry_num >= EXAMPLE_ESP_MAXIMUM_RETRY) {
                ESP_LOGW(TAG, "Max WiFi retries reached, continuing to retry...");
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
            .channel = 1,
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

    char ssid[32] = {0};
    char password[64] = {0};
    err = load_nvs_config(&current_baud_rate, ssid, sizeof(ssid), password, sizeof(password), &debug_mode);
    if (err != ESP_OK) {
        strlcpy(ssid, DEFAULT_ESP_WIFI_SSID, sizeof(ssid));
        strlcpy(password, DEFAULT_ESP_WIFI_PASS, sizeof(password));
        current_baud_rate = 115200;
        debug_mode = 0;
        esp_log_level_set(TAG, ESP_LOG_NONE);
    }

    handle_boot_count();
    xTaskCreate(boot_count_reset_task, "boot_count_reset", 2048, NULL, 4, NULL);

    led_init();
    wifi_init_sta(ssid, password);
    uart_init(current_baud_rate);
    xTaskCreate(uart_tcp_bridge_task, "uart_tcp_bridge", 4096, NULL, 5, NULL);
    xTaskCreate(tcp_receive_task, "tcp_receive", 4096, NULL, 5, NULL);
    xTaskCreate(led_task, "led_task", 2048, NULL, 4, NULL);
}