/* WiFi softAP Example with UART Configuration and Transparent LOG (Optimized for ESP8266)

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <time.h>
#include <ctype.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "tcpip_adapter.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include "driver/uart.h"
#include "driver/gpio.h"

#define DEFAULT_ESP_WIFI_SSID      "FUN_LIGHT"
#define DEFAULT_ESP_WIFI_PASS      "funlight"
#define EXAMPLE_MAX_STA_CONN       4
#define CONFIG_EXAMPLE_IPV4        1
#define NVS_NAMESPACE              "config"
#define DEFAULT_BAUD_RATE          115200

static const char *TAG = "master";
#define PORT 12345
#define BIND_RETRY_DELAY_MS 1000
#define RECV_TIMEOUT_MS 5000
#define UART_BUF_SIZE (1024)
#define MAX_CLIENTS 4
#define BAUD_RESPONSE_TIMEOUT_MS 2000
#define UART_TX_TIMEOUT_TICKS (100 / portTICK_PERIOD_MS)

#define LED_PIN GPIO_NUM_2
#define LED_BLINK_PERIOD_MS 500

typedef struct {
    int sock;
    char ip[16];
    bool active;
    bool responded;
} client_t;

static client_t clients[MAX_CLIENTS];
static uint32_t current_baud_rate = DEFAULT_BAUD_RATE;
static uint8_t debug_mode = 0;
static SemaphoreHandle_t client_count_mutex;
static int connected_clients = 0;

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
        return;
    }
    err = uart_driver_install(UART_NUM_0, UART_BUF_SIZE * 2, 0, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART driver install failed: %s", esp_err_to_name(err));
        return;
    }
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

/* LED control task */
static void led_task(void *pvParameters)
{
    bool led_state = false;
    while (1) {
        if (xSemaphoreTake(client_count_mutex, portMAX_DELAY) == pdTRUE) {
            if (connected_clients > 0) {
                if (debug_mode) ESP_LOGI(TAG, "LED on: Clients connected (%d)", connected_clients);
                gpio_set_level(LED_PIN, 0); // LED on
            } else {
                led_state = !led_state;
                if (debug_mode) ESP_LOGI(TAG, "LED blink: No clients connected");
                gpio_set_level(LED_PIN, led_state ? 0 : 1); // LED blinking
            }
            xSemaphoreGive(client_count_mutex);
        }
        vTaskDelay(LED_BLINK_PERIOD_MS / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

/* Load configurations from NVS */
static void load_nvs_config(uint32_t *baud_rate, char *ssid, size_t ssid_len, char *password, size_t pass_len, uint8_t *debug)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err == ESP_OK) {
        size_t len = ssid_len;
        err = nvs_get_str(nvs_handle, "wifi_ssid", ssid, &len);
        if (err != ESP_OK) {
            strlcpy(ssid, DEFAULT_ESP_WIFI_SSID, ssid_len);
        }
        len = pass_len;
        err = nvs_get_str(nvs_handle, "wifi_pass", password, &len);
        if (err != ESP_OK) {
            strlcpy(password, DEFAULT_ESP_WIFI_PASS, pass_len);
        }
        err = nvs_get_u32(nvs_handle, "baud_rate", baud_rate);
        if (err != ESP_OK) {
            *baud_rate = DEFAULT_BAUD_RATE;
        }
        err = nvs_get_u8(nvs_handle, "debug_mode", debug);
        if (err != ESP_OK) {
            *debug = 0;
        }
        nvs_close(nvs_handle);
    } else {
        *baud_rate = DEFAULT_BAUD_RATE;
        strlcpy(ssid, DEFAULT_ESP_WIFI_SSID, ssid_len);
        strlcpy(password, DEFAULT_ESP_WIFI_PASS, pass_len);
        *debug = 0;
    }

    size_t ssid_length = strlen(ssid);
    size_t pass_length = strlen(password);
    if (ssid_length == 0 || ssid_length > 32) {
        strlcpy(ssid, DEFAULT_ESP_WIFI_SSID, ssid_len);
    }
    if (pass_length > 0 && (pass_length < 8 || pass_length > 64)) {
        strlcpy(password, DEFAULT_ESP_WIFI_PASS, pass_len);
    }

    esp_log_level_set(TAG, *debug ? ESP_LOG_INFO : ESP_LOG_NONE);
}

/* Save configurations to NVS */
static void save_nvs_config(uint32_t baud_rate, const char *ssid, const char *password, uint8_t debug)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        return;
    }
    if (ssid) nvs_set_str(nvs_handle, "wifi_ssid", ssid);
    if (password) nvs_set_str(nvs_handle, "wifi_pass", password);
    nvs_set_u32(nvs_handle, "baud_rate", baud_rate);
    nvs_set_u8(nvs_handle, "debug_mode", debug);
    err = nvs_commit(nvs_handle);
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

/* Get log prefix */
static void get_log_prefix(char *prefix, size_t prefix_len, const char *client_ip)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    time_t seconds = tv.tv_sec;
    int milliseconds = tv.tv_usec / 1000;
    struct tm *tm_info = gmtime(&seconds);
    char *last_octet = strrchr(client_ip, '.');
    last_octet = last_octet ? last_octet + 1 : "0";
    snprintf(prefix, prefix_len, "[%02d h %02d m %02d s %03d ms][device %s]", 
             tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec, milliseconds, last_octet);
}

/* Check if any clients are connected */
static bool has_connected_clients(void)
{
    if (xSemaphoreTake(client_count_mutex, portMAX_DELAY) == pdTRUE) {
        bool result = connected_clients > 0;
        xSemaphoreGive(client_count_mutex);
        return result;
    }
    return false;
}

/* Broadcast data to all clients except the sender, wait for responses */
static bool broadcast_and_wait(const char *data, size_t len, int sender_sock)
{
    char log_prefix[64];
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (clients[i].active && clients[i].sock != sender_sock) {
            clients[i].responded = false;
            int err = send(clients[i].sock, data, len, 0);
            get_log_prefix(log_prefix, sizeof(log_prefix), clients[i].ip);
            if (err < 0) {
                ESP_LOGE(TAG, "%s Error sending to client: errno %d", log_prefix, errno);
                shutdown(clients[i].sock, SHUT_RDWR);
                close(clients[i].sock);
                clients[i].active = false;
                clients[i].responded = false;
                if (xSemaphoreTake(client_count_mutex, portMAX_DELAY) == pdTRUE) {
                    connected_clients--;
                    xSemaphoreGive(client_count_mutex);
                }
            }
        }
    }

    int timeout_ms = BAUD_RESPONSE_TIMEOUT_MS;
    bool all_responded = false;
    while (timeout_ms > 0) {
        all_responded = true;
        for (int i = 0; i < MAX_CLIENTS; i++) {
            if (clients[i].active && clients[i].sock != sender_sock && !clients[i].responded) {
                all_responded = false;
                break;
            }
        }
        if (all_responded) {
            break;
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
        timeout_ms -= 100;
    }
    return all_responded;
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
        uart_wait_tx_done(UART_NUM_0, UART_TX_TIMEOUT_TICKS);
        xSemaphoreGive(raw_log_mutex);
    } else {
        uart_write_bytes(UART_NUM_0, data, len);
        uart_wait_tx_done(UART_NUM_0, UART_TX_TIMEOUT_TICKS);
    }
}

/* UART command processing task */
static void uart_command_task(void *pvParameters)
{
    uint8_t *data = (uint8_t *)malloc(UART_BUF_SIZE);
    char response[128];
    char log_prefix[64];

    if (!data) {
        ESP_LOGE(TAG, "Failed to allocate UART buffer");
        vTaskDelete(NULL);
    }

    while (1) {
        int len = uart_read_bytes(UART_NUM_0, data, UART_BUF_SIZE, 20 / portTICK_PERIOD_MS);
        if (len > 0) {
            data[len] = 0;
            char *cmd = (char *)data;

            if (strncmp(cmd, "BAUD=", 5) == 0) {
                if (!has_connected_clients()) {
                    print_raw_log("No clients connected, modification not supported\r\n");
                    continue;
                }
                uint32_t new_baud = atoi(cmd + 5);
                if (new_baud >= 9600 && new_baud <= 921600) {
                    snprintf(response, sizeof(response), "BAUD=%u\r\n", new_baud);
                    if (broadcast_and_wait(response, strlen(response), -1)) {
                        get_log_prefix(log_prefix, sizeof(log_prefix), "0.0.0.0");
                        ESP_LOGI(TAG, "%s Changing baud rate to %u", log_prefix, new_baud);
                        uart_set_baudrate(UART_NUM_0, new_baud);
                        current_baud_rate = new_baud;
                        save_nvs_config(new_baud, NULL, NULL, debug_mode);
                        print_raw_log("OK\r\n");
                    } else {
                        get_log_prefix(log_prefix, sizeof(log_prefix), "0.0.0.0");
                        ESP_LOGE(TAG, "%s Not all clients responded, baud rate change aborted", log_prefix);
                        print_raw_log("Not all clients responded, baud rate change aborted\r\n");
                    }
                } else {
                    get_log_prefix(log_prefix, sizeof(log_prefix), "0.0.0.0");
                    ESP_LOGE(TAG, "%s Invalid baud rate: %s", log_prefix, cmd + 5);
                    print_raw_log("Invalid baud rate\r\n");
                }
            } else if (strncmp(cmd, "WIFI=", 5) == 0) {
                if (!has_connected_clients()) {
                    print_raw_log("No clients connected, modification not supported\r\n");
                    continue;
                }
                char *ssid = cmd + 5;
                char *password = strchr(ssid, ',');
                if (password) {
                    *password = 0;
                    password++;
                    char *end = strchr(password, '\r');
                    if (end) *end = 0;
                    if (strlen(ssid) == 0 || strlen(ssid) > 32 || !isprint((unsigned char)ssid[0])) {
                        get_log_prefix(log_prefix, sizeof(log_prefix), "0.0.0.0");
                        ESP_LOGE(TAG, "%s Invalid SSID: %s", log_prefix, ssid);
                        print_raw_log("Invalid SSID\r\n");
                        continue;
                    }
                    size_t pass_length = strlen(password);
                    if (pass_length > 0 && (pass_length < 8 || pass_length > 64 || !isprint((unsigned char)password[0]))) {
                        get_log_prefix(log_prefix, sizeof(log_prefix), "0.0.0.0");
                        ESP_LOGE(TAG, "%s Invalid password length or characters", log_prefix);
                        print_raw_log("Invalid password\r\n");
                        continue;
                    }
                    snprintf(response, sizeof(response), "WIFI=%s,%s\r\n", ssid, password);
                    if (broadcast_and_wait(response, strlen(response), -1)) {
                        get_log_prefix(log_prefix, sizeof(log_prefix), "0.0.0.0");
                        ESP_LOGI(TAG, "%s WiFi configured: SSID=%s, restarting...", log_prefix, ssid);
                        save_nvs_config(current_baud_rate, ssid, password, debug_mode);
                        print_raw_log("OK\r\n");
                        vTaskDelay(1000 / portTICK_PERIOD_MS);
                        esp_restart();
                    } else {
                        get_log_prefix(log_prefix, sizeof(log_prefix), "0.0.0.0");
                        ESP_LOGE(TAG, "%s Not all clients responded, WiFi configuration aborted", log_prefix);
                        print_raw_log("Not all clients responded, WiFi configuration aborted\r\n");
                    }
                } else {
                    get_log_prefix(log_prefix, sizeof(log_prefix), "0.0.0.0");
                    ESP_LOGE(TAG, "%s Invalid WIFI command format", log_prefix);
                    print_raw_log("Invalid WIFI command format\r\n");
                }
            } else if (strncmp(cmd, "DEBUG=", 6) == 0) {
                if (!has_connected_clients()) {
                    print_raw_log("No clients connected, modification not supported\r\n");
                    continue;
                }
                uint8_t new_debug = atoi(cmd + 6);
                if (new_debug == 0 || new_debug == 1) {
                    snprintf(response, sizeof(response), "DEBUG=%u\r\n", new_debug);
                    if (broadcast_and_wait(response, strlen(response), -1)) {
                        get_log_prefix(log_prefix, sizeof(log_prefix), "0.0.0.0");
                        ESP_LOGI(TAG, "%s Changing debug mode to %u", log_prefix, new_debug);
                        debug_mode = new_debug;
                        esp_log_level_set(TAG, debug_mode ? ESP_LOG_INFO : ESP_LOG_NONE);
                        save_nvs_config(current_baud_rate, NULL, NULL, debug_mode);
                        print_raw_log("OK\r\n");
                    } else {
                        get_log_prefix(log_prefix, sizeof(log_prefix), "0.0.0.0");
                        ESP_LOGE(TAG, "%s Not all clients responded, debug mode change aborted", log_prefix);
                        print_raw_log("Not all clients responded, debug mode change aborted\r\n");
                    }
                } else {
                    get_log_prefix(log_prefix, sizeof(log_prefix), "0.0.0.0");
                    ESP_LOGE(TAG, "%s Invalid debug mode: %s", log_prefix, cmd + 6);
                    print_raw_log("Invalid debug mode\r\n");
                }
            } else if (strcmp(cmd, "RSTDEFAULT\r\n") == 0) {
                get_log_prefix(log_prefix, sizeof(log_prefix), "0.0.0.0");
                ESP_LOGI(TAG, "%s Factory reset triggered", log_prefix);
                reset_to_defaults();
            } else {
                for (int i = 0; i < MAX_CLIENTS; i++) {
                    if (clients[i].active) {
                        int err = send(clients[i].sock, cmd, len, 0);
                        get_log_prefix(log_prefix, sizeof(log_prefix), clients[i].ip);
                        if (err < 0) {
                            ESP_LOGE(TAG, "%s Error sending to client: errno %d", log_prefix, errno);
                            shutdown(clients[i].sock, SHUT_RDWR);
                            close(clients[i].sock);
                            clients[i].active = false;
                            clients[i].responded = false;
                            if (xSemaphoreTake(client_count_mutex, portMAX_DELAY) == pdTRUE) {
                                connected_clients--;
                                xSemaphoreGive(client_count_mutex);
                            }
                        }
                    }
                }
            }
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    free(data);
    vTaskDelete(NULL);
}

/* TCP server task */
static void tcp_server_task(void *pvParameters)
{
    char rx_buffer[128];
    char addr_str[128];
    char log_prefix[64];
    int addr_family = AF_INET;
    int ip_protocol = IPPROTO_IP;

    for (int i = 0; i < MAX_CLIENTS; i++) {
        clients[i].sock = -1;
        clients[i].active = false;
        clients[i].responded = false;
    }

    while (1) {
        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(PORT);

        int listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
        if (listen_sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            vTaskDelay(BIND_RETRY_DELAY_MS / portTICK_PERIOD_MS);
            continue;
        }

        int opt = 1;
        setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        int err = bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err != 0) {
            ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
            close(listen_sock);
            vTaskDelay(BIND_RETRY_DELAY_MS / portTICK_PERIOD_MS);
            continue;
        }

        err = listen(listen_sock, 1);
        if (err != 0) {
            ESP_LOGE(TAG, "Error during listen: errno %d", errno);
            close(listen_sock);
            vTaskDelay(BIND_RETRY_DELAY_MS / portTICK_PERIOD_MS);
            continue;
        }

        struct sockaddr_in source_addr;
        socklen_t addr_len = sizeof(source_addr);
        int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
            close(listen_sock);
            vTaskDelay(BIND_RETRY_DELAY_MS / portTICK_PERIOD_MS);
            continue;
        }

        inet_ntoa_r(source_addr.sin_addr, addr_str, sizeof(addr_str) - 1);
        get_log_prefix(log_prefix, sizeof(log_prefix), addr_str);
        ESP_LOGI(TAG, "%s Client connected", log_prefix);

        int client_idx = -1;
        for (int i = 0; i < MAX_CLIENTS; i++) {
            if (!clients[i].active) {
                client_idx = i;
                clients[i].sock = sock;
                clients[i].active = true;
                clients[i].responded = false;
                strlcpy(clients[i].ip, addr_str, sizeof(clients[i].ip));
                break;
            }
        }
        if (client_idx == -1) {
            ESP_LOGE(TAG, "%s Max clients reached, rejecting connection", log_prefix);
            close(sock);
            close(listen_sock);
            continue;
        }

        if (xSemaphoreTake(client_count_mutex, portMAX_DELAY) == pdTRUE) {
            connected_clients++;
            xSemaphoreGive(client_count_mutex);
        }

        struct timeval timeout;
        timeout.tv_sec = RECV_TIMEOUT_MS / 1000;
        timeout.tv_usec = (RECV_TIMEOUT_MS % 1000) * 1000;
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

        // Add keepalive to client socket
        int keepalive = 1;
        int keepidle = 10; // 10 seconds
        int keepintvl = 2;
        int keepcnt = 3;
        setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &keepalive, sizeof(keepalive));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE, &keepidle, sizeof(keepidle));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPINTVL, &keepintvl, sizeof(keepintvl));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPCNT, &keepcnt, sizeof(keepcnt));

        while (1) {
            int len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
            if (len < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    vTaskDelay(10 / portTICK_PERIOD_MS);
                    continue;
                }
                ESP_LOGE(TAG, "%s Receive failed: errno %d", log_prefix, errno);
                break;
            } else if (len == 0) {
                ESP_LOGI(TAG, "%s Client disconnected", log_prefix);
                break;
            } else {
                rx_buffer[len] = 0;
                get_log_prefix(log_prefix, sizeof(log_prefix), addr_str);
                if (strcmp(rx_buffer, "OK\r\n") == 0) {
                    clients[client_idx].responded = true;
                } else if (strncmp(rx_buffer, "BAUD=", 5) == 0) {
                    uint32_t new_baud = atoi(rx_buffer + 5);
                    if (new_baud >= 9600 && new_baud <= 921600) {
                        ESP_LOGI(TAG, "%s Received client baud rate change request: %u", log_prefix, new_baud);
                        if (broadcast_and_wait(rx_buffer, len, sock)) {
                            ESP_LOGI(TAG, "%s Changing baud rate to %u", log_prefix, new_baud);
                            uart_set_baudrate(UART_NUM_0, new_baud);
                            current_baud_rate = new_baud;
                            save_nvs_config(new_baud, NULL, NULL, debug_mode);
                            send(sock, "OK\r\n", 4, 0);
                        } else {
                            ESP_LOGE(TAG, "%s Not all clients responded, baud rate change aborted", log_prefix);
                            send(sock, "Not all clients responded, baud rate change aborted\r\n", 52, 0);
                        }
                    } else {
                        ESP_LOGE(TAG, "%s Invalid baud rate: %s", log_prefix, rx_buffer + 5);
                        send(sock, "Invalid baud rate\r\n", 19, 0);
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
                            ESP_LOGE(TAG, "%s Invalid SSID: %s", log_prefix, ssid);
                            send(sock, "Invalid SSID\r\n", 14, 0);
                            continue;
                        }
                        size_t pass_length = strlen(password);
                        if (pass_length > 0 && (pass_length < 8 || pass_length > 64 || !isprint((unsigned char)password[0]))) {
                            ESP_LOGE(TAG, "%s Invalid password length or characters", log_prefix);
                            send(sock, "Invalid password\r\n", 18, 0);
                            continue;
                        }
                        if (broadcast_and_wait(rx_buffer, len, sock)) {
                            ESP_LOGI(TAG, "%s WiFi configured: SSID=%s, restarting...", log_prefix, ssid);
                            save_nvs_config(current_baud_rate, ssid, password, debug_mode);
                            send(sock, "OK\r\n", 4, 0);
                            vTaskDelay(1000 / portTICK_PERIOD_MS);
                            esp_restart();
                        } else {
                            ESP_LOGE(TAG, "%s Not all clients responded, WiFi configuration aborted", log_prefix);
                            send(sock, "Not all clients responded, WiFi configuration aborted\r\n", 52, 0);
                        }
                    } else {
                        ESP_LOGE(TAG, "%s Invalid WIFI command format", log_prefix);
                        send(sock, "Invalid WIFI command format\r\n", 29, 0);
                    }
                } else if (strncmp(rx_buffer, "DEBUG=", 6) == 0) {
                    uint8_t new_debug = atoi(rx_buffer + 6);
                    if (new_debug == 0 || new_debug == 1) {
                        ESP_LOGI(TAG, "%s Received Warsawclient debug mode change request: %u", log_prefix, new_debug);
                        if (broadcast_and_wait(rx_buffer, len, sock)) {
                            ESP_LOGI(TAG, "%s Changing debug mode to %u", log_prefix, new_debug);
                            debug_mode = new_debug;
                            esp_log_level_set(TAG, debug_mode ? ESP_LOG_INFO : ESP_LOG_NONE);
                            save_nvs_config(current_baud_rate, NULL, NULL, debug_mode);
                            send(sock, "OK\r\n", 4, 0);
                        } else {
                            ESP_LOGE(TAG, "%s Not all clients responded, debug mode change aborted", log_prefix);
                            send(sock, "Not all clients responded, debug mode change aborted\r\n", 52, 0);
                        }
                    } else {
                        ESP_LOGE(TAG, "%s Invalid debug mode: %s", log_prefix, rx_buffer + 6);
                        send(sock, "Invalid debug mode\r\n", 20, 0);
                    }
                } else if (strcmp(rx_buffer, "RSTDEFAULT\r\n") == 0) {
                    ESP_LOGI(TAG, "%s Factory reset triggered", log_prefix);
                    reset_to_defaults();
                } else {
                    print_raw_log(rx_buffer); // Transparent data
                    for (int i = 0; i < MAX_CLIENTS; i++) {
                        if (clients[i].active && clients[i].sock != sock) {
                            int err = send(clients[i].sock, rx_buffer, len, 0);
                            get_log_prefix(log_prefix, sizeof(log_prefix), clients[i].ip);
                            if (err < 0) {
                                ESP_LOGE(TAG, "%s Error sending to client: errno %d", log_prefix, errno);
                                shutdown(clients[i].sock, SHUT_RDWR);
                                close(clients[i].sock);
                                clients[i].active = false;
                                clients[i].responded = false;
                                if (xSemaphoreTake(client_count_mutex, portMAX_DELAY) == pdTRUE) {
                                    connected_clients--;
                                    xSemaphoreGive(client_count_mutex);
                                }
                            }
                        }
                    }
                }
            }
        }

        if (sock != -1) {
            shutdown(sock, SHUT_RDWR);
            close(sock);
            clients[client_idx].active = false;
            clients[client_idx].responded = false;
            if (xSemaphoreTake(client_count_mutex, portMAX_DELAY) == pdTRUE) {
                connected_clients--;
                xSemaphoreGive(client_count_mutex);
            }
            ESP_LOGI(TAG, "%s Client disconnected, remaining clients: %d", log_prefix, connected_clients);
        }
        close(listen_sock);
        vTaskDelay(BIND_RETRY_DELAY_MS / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

/* WiFi event handler */
static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch (event->event_id) {
        case SYSTEM_EVENT_AP_START:
            ESP_LOGI(TAG, "SoftAP started");
            break;
        case SYSTEM_EVENT_AP_STACONNECTED:
            ESP_LOGI(TAG, "Station connected to SoftAP");
            break;
        case SYSTEM_EVENT_AP_STADISCONNECTED:
            ESP_LOGI(TAG, "Station disconnected from SoftAP");
            break;
        default:
            break;
    }
    return ESP_OK;
}

/* Initialize WiFi as softAP */
static void wifi_init_softap(const char *ssid, const char *password)
{
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
        .ap = {
            .ssid_len = strlen(ssid),
            .channel = 1,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .max_connection = EXAMPLE_MAX_STA_CONN
        },
    };
    strlcpy((char *)wifi_config.ap.ssid, ssid, sizeof(wifi_config.ap.ssid));
    strlcpy((char *)wifi_config.ap.password, password, sizeof(wifi_config.ap.password));

    err = esp_wifi_set_mode(WIFI_MODE_AP);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Set WiFi mode failed: %s", esp_err_to_name(err));
        return;
    }
    err = esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Set WiFi config failed: %s", esp_err_to_name(err));
        return;
    }
    err = esp_wifi_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "WiFi start failed: %s", esp_err_to_name(err));
        return;
    }

    err = esp_wifi_set_max_tx_power(82); // 20.5 dBm (max for ESP8266)
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
            return;
        }
    }

    char ssid[32] = {0};
    char password[64] = {0};
    load_nvs_config(&current_baud_rate, ssid, sizeof(ssid), password, sizeof(password), &debug_mode);

    client_count_mutex = xSemaphoreCreateMutex();
    if (!client_count_mutex) {
        ESP_LOGE(TAG, "Failed to create client count mutex");
        return;
    }

    led_init();
    wifi_init_softap(ssid, password);
    uart_init(current_baud_rate);
    xTaskCreate(tcp_server_task, "tcp_server", 4096, NULL, 5, NULL);
    xTaskCreate(uart_command_task, "uart_command", 4096, NULL, 5, NULL);
    xTaskCreate(led_task, "led_task", 2048, NULL, 4, NULL);
}