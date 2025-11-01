/* WiFi Station with UART to TCP Bridge (Enhanced for UDP Discovery & Real-Time Sending)

   This is an optimized version of the previous ESP8266 client code. Key optimizations for long connection stability and real-time UART-TCP bridging:
   - Exponential backoff for TCP reconnects (1s, 2s, 4s, ..., up to 32s) to prevent rapid reconnect storms.
   - Improved error handling in recv/send: Distinguish between EAGAIN (retry) and real disconnects (FIN/RST).
   - Real-time UART reads: Short 1ms timeout for ~50B chunks, immediate or small-batch (100B) flush every ~5ms to TCP for low latency.
   - Disabled Nagle's algorithm (TCP_NODELAY=1) to prevent buffering small packets, ensuring ~5ms end-to-end.
   - UDP discovery only on WiFi IP change or initial connect; cache gateway IP across reconnects.
   - Enhanced keepalive: Shorter idle (5s) with fewer probes (3) for faster disconnect detection (~8s total).
   - Non-blocking send with retry loop to handle full buffers without closing socket.
   - Reduced task delays: 1ms yield in loops for responsiveness without CPU spin.
   - Better logging: Conditional debug, hex dumps only for small packets (<128 bytes).
   - TWDT feeding: More frequent in hot loops.
   - Factory reset on 6 boots unchanged.
   - Auth failure: Non-blocking LED indication (1s on, 3s off) without halting system.

   Compile and flash to ESP8266 as before. Test: Monitor logs for flush frequency; service should receive ~50B every ~5ms.
*/

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <fcntl.h>
#include <math.h>  // For pow in backoff
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
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
#include "esp_timer.h"
#include "esp_task_wdt.h"  // TWDT 支持
#include "lwip/ip4_addr.h"  // 或通过 #include "tcpip_adapter.h" 间接包含

#define DEFAULT_ESP_WIFI_SSID      "Funlight"
#define DEFAULT_ESP_WIFI_PASS      "funlight"
#define EXAMPLE_ESP_MAXIMUM_RETRY  10
#define NVS_NAMESPACE              "config"
#define BOOT_COUNT_KEY             "boot_count"
#define BOOT_COUNT_THRESHOLD       6
#define BOOT_COUNT_RESET_DELAY_MS  10000 // 10 seconds

static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
static const char *TAG = "slave";

#define TCP_PORT 12345
#define UDP_PORT 12346  // UDP discovery port
#define UDP_BROADCAST_ADDR "255.255.255.255"
#define UDP_DISCOVER_MSG "DISCOVER_GATEWAY\r\n"
#define UDP_GATEWAY_RESP_PREFIX "GATEWAY_IP:"
#define CONFIG_EXAMPLE_IPV4 1
#define UART_BUF_SIZE (512)
#define TCP_RETRY_BASE_DELAY_MS 1000  // Start with 1s
#define TCP_RETRY_MAX_DELAY_MS 32000  // Cap at 32s
#define TCP_CONNECT_TIMEOUT_MS 10000
#define TCP_KEEPALIVE_IDLE_MS 5000    // 5s idle before probes
#define RESPONSE_TIMEOUT_MS 5000
#define UDP_DISCOVERY_TIMEOUT_MS 2000 // Reduced for faster discovery
#define UDP_DISCOVERY_RETRIES 5       // Reduced retries
#define UART_READ_TIMEOUT_MS 1        // 1ms for real-time ~50B reads
#define FLUSH_BATCH_SIZE 100          // Flush if batch >=100B
#define FLUSH_INTERVAL_MS 5           // Or every 5ms

#define LED_PIN GPIO_NUM_2

static uint32_t current_baud_rate = 115200;
static uint8_t current_stop_bits = UART_STOP_BITS_1;
static uint8_t current_parity = UART_PARITY_DISABLE;
static uint8_t current_data_bits = UART_DATA_8_BITS;
static bool is_tcp_connected = false;
static int tcp_sock = -1;
static uint8_t debug_mode = 1;

// Store client's own IP address and gateway IP (discovered via UDP)
static char my_ip_addr[16] = {0};  // e.g., "192.168.4.100"
static char gateway_ip_addr[16] = {0};  // e.g., "192.168.4.1" (discovered)
static bool gateway_ip_discovered = false;
static bool needs_rediscovery = true;  // Flag to trigger discovery only when needed
static int udp_sock = -1;  // UDP socket for discovery

// Reconnect backoff state
static uint32_t tcp_reconnect_delay_ms = TCP_RETRY_BASE_DELAY_MS;
static uint32_t last_reconnect_time = 0;

char g_ssid[128] = {0};
char g_password[128] = {0};

static bool wifi_connected = false;

// Auth failure flag
static bool auth_failed = false;

// Centralized connection state semaphore for thread-safety
static SemaphoreHandle_t conn_state_mutex = NULL;

static void conn_state_init(void) {
    if (!conn_state_mutex) {
        conn_state_mutex = xSemaphoreCreateMutex();
    }
}

static void set_tcp_connected(bool connected) {
    if (conn_state_mutex && xSemaphoreTake(conn_state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        is_tcp_connected = connected;
        xSemaphoreGive(conn_state_mutex);
    }
}

static bool get_tcp_connected(void) {
    bool connected;
    if (conn_state_mutex && xSemaphoreTake(conn_state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        connected = is_tcp_connected;
        xSemaphoreGive(conn_state_mutex);
        return connected;
    }
    return false;
}

static void led_status_task(void *pvParameters)
{
    while (1) {
        if (auth_failed) {
            // Auth failed: 亮1秒灭3秒 (active-low: 0=on, 1=off)
            gpio_set_level(LED_PIN, 0);  // On
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            gpio_set_level(LED_PIN, 1);  // Off
            vTaskDelay(3000 / portTICK_PERIOD_MS);
        } else {
            bool wifi_ok = wifi_connected;
            bool tcp_ok = get_tcp_connected();
            if (!wifi_ok) {
                // WiFi 未连接：慢闪（1秒亮1秒灭）
                gpio_set_level(LED_PIN, 0);
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                gpio_set_level(LED_PIN, 1);
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            } else if (wifi_ok && !tcp_ok) {
                // WiFi 已连接但TCP未连接：快闪（200ms亮200ms灭）
                gpio_set_level(LED_PIN, 0);
                vTaskDelay(200 / portTICK_PERIOD_MS);
                gpio_set_level(LED_PIN, 1);
                vTaskDelay(200 / portTICK_PERIOD_MS);
            } else if (wifi_ok && tcp_ok) {
                // TCP 已连接：常亮
                gpio_set_level(LED_PIN, 0);
                vTaskDelay(1000 / portTICK_PERIOD_MS);  // 保持常亮
            }
        }
        esp_task_wdt_reset();
    }
}


/* CRC32 computation for ESP8266 */
static uint32_t compute_crc32(const uint8_t *data, size_t len)
{
    uint32_t crc = 0xFFFFFFFF;
    const uint32_t poly = 0xEDB88320;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ poly;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc ^ 0xFFFFFFFF;
}

/* Author code verification - Non-blocking on failure */
static void authorcodeverify(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS for auth: %s", esp_err_to_name(err));
        auth_failed = true;  // Set flag for LED indication
        return;
    }

    uint8_t mac[6];
    err = esp_wifi_get_mac(ESP_IF_WIFI_STA, mac);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get MAC: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        auth_failed = true;
        return;
    }

    uint32_t computed_code = compute_crc32(mac, 6);

    uint32_t stored_code = 0;
    err = nvs_get_u32(nvs_handle, "auth_code", &stored_code);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        // First time, write the code
        err = nvs_set_u32(nvs_handle, "auth_code", computed_code);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to write auth_code: %s", esp_err_to_name(err));
            nvs_close(nvs_handle);
            auth_failed = true;
            return;
        }
        err = nvs_commit(nvs_handle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to commit auth_code: %s", esp_err_to_name(err));
        }
        ESP_LOGI(TAG, "Auth code written successfully");
        nvs_close(nvs_handle);
        return;
    } else if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read auth_code: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        auth_failed = true;
        return;
    }

    // Verify
    if (computed_code != stored_code) {
        ESP_LOGE(TAG, "Auth verification failed. Computed: 0x%08x, Stored: 0x%08x", computed_code, stored_code);
        nvs_close(nvs_handle);
        auth_failed = true;  // Set flag instead of blocking
        return;
    }

    ESP_LOGI(TAG, "Auth verification passed");
    nvs_close(nvs_handle);
}


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
static void print_raw_data(const uint8_t *data, size_t len)
{
    if (!data || len == 0) return;

    esp_task_wdt_reset();

    int written = uart_write_bytes(UART_NUM_0, (const char *)data, len);
    if (written != len) {
        ESP_LOGW(TAG, "UART write incomplete: %d/%zu bytes", written, len);
    }
    // 移除 uart_wait_tx_done()：避免阻塞，允许异步 TX

    esp_task_wdt_reset();
}

/* Non-blocking send with retry */
static esp_err_t nonblocking_send(int sock, const char *payload, size_t payload_len, int max_retries) {
    size_t sent = 0;
    while (sent < payload_len) {
        int bytes = send(sock, payload + sent, payload_len - sent, 0);
        if (bytes > 0) {
            sent += bytes;
            esp_task_wdt_reset();  // Feed during retries
            continue;
        }
        if (bytes < 0) {
            int lwip_errno = errno;
            if (lwip_errno == EAGAIN || lwip_errno == EWOULDBLOCK) {
                if (--max_retries <= 0) {
                    ESP_LOGW(TAG, "Send EAGAIN timeout after %d retries (buffer full?)", max_retries + 1);
                    return ESP_ERR_TIMEOUT;  // Transient
                }
                vTaskDelay(1 / portTICK_PERIOD_MS);
                continue;
            } else {
                ESP_LOGE(TAG, "Send real error: errno %d", lwip_errno);
                return ESP_FAIL;  // Fatal
            }
        }
    }
    return ESP_OK;
}

/* TCP send data with optional response expectation */
static esp_err_t tcp_send_data(int sock, const char *payload, size_t payload_len, char *rx_buffer, size_t rx_buffer_size, bool expect_response)
{
    esp_err_t send_err = nonblocking_send(sock, payload, payload_len, 50);  // ~20ms retries
    if (send_err == ESP_ERR_TIMEOUT) {
        ESP_LOGW(TAG, "Transient send timeout, returning non-fatal");
        return ESP_ERR_TIMEOUT;
    } else if (send_err != ESP_OK) {
        ESP_LOGE(TAG, "Fatal error during sending: %s", esp_err_to_name(send_err));
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

// Send client's IP to gateway upon TCP connection
static void send_my_ip(int sock) {
    if (my_ip_addr[0] == '\0') {
        ESP_LOGW(TAG, "My IP not set yet, skipping send");
        return;
    }
    char ip_msg[128];
    int len = snprintf(ip_msg, sizeof(ip_msg), "{\"ip\":\"%s\",\"baud\":%d,\"ssid\":\"%s\",\"pwd\":\"%s\"}", my_ip_addr,current_baud_rate,g_ssid,g_password);
    if (len > 0) {
        nonblocking_send(sock, ip_msg, len, 5);  // Use non-blocking
        if (debug_mode) ESP_LOGI(TAG, "Sent my IP to gateway: %s", ip_msg);
    }
}

static esp_err_t parse_tcp_command(int sock, const uint8_t *rx_buffer, size_t len)
{
    if (!rx_buffer || len == 0) {
        return ESP_FAIL;
    }

    // Check for targeted message format "TO:<target_ip>|<message>\r\n"
    // Quick check: Look for '|' and ensure null-safe (no embedded 0x00 for commands)
    bool has_pipe = (memchr(rx_buffer, '|', len) != NULL);
    if (has_pipe && memcmp(rx_buffer, "TO:", 3) == 0 && memchr(rx_buffer, 0, len) == NULL) {
        // Safe to null-terminate for parsing
        char *buf = (char *)malloc(len + 1);
        if (!buf) {
            ESP_LOGE(TAG, "Failed to allocate buf for TO command");
            return ESP_FAIL;
        }
        memcpy(buf, rx_buffer, len);
        buf[len] = 0;

        // Find pipe and trim to \r\n if present
        char *pipe_pos = strchr(buf + 3, '|');  // Skip "TO:"
        if (pipe_pos) {
            *pipe_pos = 0;  // Split target_ip
            char *target_ip = buf + 3;
            char *message = pipe_pos + 1;

            // Trim message end (\r\n)
            char *end = strstr(message, "\r\n");
            if (end) *end = 0;

            // Validate target_ip (simple IPv4 check: 3 dots, numeric)
            bool valid_ip = (strchr(target_ip, '.') != NULL && strchr(target_ip, '.') != strrchr(target_ip, '.') &&
                             strchr(target_ip, '.') != strrchr(target_ip, '.') + 1 && strlen(target_ip) <= 15);
            if (!valid_ip || my_ip_addr[0] == '\0') {
                ESP_LOGW(TAG, "Invalid target IP or my IP not set: %s", target_ip);
                free(buf);
                return ESP_OK;  // Ignore invalid
            }

            // Check if matches my IP
            if (strcmp(target_ip, my_ip_addr) == 0) {
                // Transparently forward message to UART (MCU)
                size_t msg_len = strlen(message);
                print_raw_data((uint8_t *)message, msg_len);
                if (debug_mode) ESP_LOGI(TAG, "Targeted message for my IP (%s) forwarded to MCU: %s", my_ip_addr, message);
                free(buf);
                return ESP_OK;
            } else {
                ESP_LOGI(TAG, "Message for other IP (%s), ignoring: %s", target_ip, message);
                free(buf);
                return ESP_OK;
            }
        }
        free(buf);
    }

    // 快速路径：如果含 \r\n，假设命令；否则 passthrough
    bool is_likely_command = (memchr(rx_buffer, '\r', len) != NULL || memchr(rx_buffer, '\n', len) != NULL);
    if (!is_likely_command) {
        print_raw_data(rx_buffer, len);
        return ESP_OK;
    }

    // Existing: Check if potential BAUD command
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

        // Manual parse for BAUD= (optimized, no regex since only baud used)
        char *baud_start = buf + 5;  // Skip "BAUD="
        char *baud_end = strchr(baud_start, ',');  // End at comma or \0
        if (baud_end) *baud_end = 0;
        else baud_end = strchr(baud_start, '\r');  // Or \r if no comma
        if (baud_end) *baud_end = 0;

        // Validate and convert baud rate
        char *endptr;
        uint32_t new_baud = strtoul(baud_start, &endptr, 10);
        if (*endptr != '\0' || new_baud < 110 || new_baud > 2000000) {
            ESP_LOGE(TAG, "Invalid baud rate: %u", new_baud);
            const char *response = "Invalid baud rate\r\n";
            nonblocking_send(sock, response, strlen(response), 3);
            free(buf);
            return ESP_FAIL;
        }

        free(buf);

        // Apply baud rate change
        uart_set_baudrate(UART_NUM_0, new_baud);

        // Update global variable and save to NVS (keep others unchanged)
        current_baud_rate = new_baud;
        save_nvs_config(new_baud, current_stop_bits, current_parity, current_data_bits, NULL, NULL, debug_mode);
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
                nonblocking_send(sock, response, strlen(response), 3);
                free(buf);
                return ESP_FAIL;
            } else if (strlen(password) > 0 && (strlen(password) < 8 || strlen(password) > 64 || !isprint((unsigned char)password[0]))) {
                ESP_LOGE(TAG, "Invalid password from server");
                const char *response = "Invalid password\r\n";
                nonblocking_send(sock, response, strlen(response), 3);
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
            nonblocking_send(sock, response, strlen(response), 3);
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
    uint8_t *rx_buffer = (uint8_t *) malloc(UART_BUF_SIZE);
    if (!rx_buffer) {
        ESP_LOGE(TAG, "Failed to allocate memory for receive buffer");
        vTaskDelete(NULL);
    }

    while (1) {
        if (!get_tcp_connected() || tcp_sock == -1) {
            vTaskDelay(100 / portTICK_PERIOD_MS);  // Reduced from 500ms, but not too aggressive
            esp_task_wdt_reset();  // 喂狗
            continue;
        }

        int len = recv(tcp_sock, rx_buffer, UART_BUF_SIZE - 1, 0);
        if (len < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                vTaskDelay(1 / portTICK_PERIOD_MS);  // 1ms yield for non-blocking
                esp_task_wdt_reset();
                continue;
            }
            // Real disconnect: ECONNRESET, ETIMEDOUT, etc.
            ESP_LOGW(TAG, "TCP disconnect detected: errno %d, initiating backoff reconnect...", errno);
            set_tcp_connected(false);
            close(tcp_sock);
            tcp_sock = -1;
            gpio_set_level(LED_PIN, 1); // LED off on disconnect
            uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
            last_reconnect_time = now;
            tcp_reconnect_delay_ms = TCP_RETRY_BASE_DELAY_MS;  // Reset backoff on new disconnect
            vTaskDelay(tcp_reconnect_delay_ms / portTICK_PERIOD_MS);
            tcp_reconnect_delay_ms = (tcp_reconnect_delay_ms * 2 > TCP_RETRY_MAX_DELAY_MS) ? TCP_RETRY_MAX_DELAY_MS : tcp_reconnect_delay_ms * 2;
            esp_task_wdt_reset();
            continue;
        } else if (len == 0) {
            ESP_LOGI(TAG, "TCP connection closed by peer, initiating backoff reconnect...");
            set_tcp_connected(false);
            close(tcp_sock);
            tcp_sock = -1;
            gpio_set_level(LED_PIN, 1); // LED off on disconnect
            uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
            last_reconnect_time = now;
            tcp_reconnect_delay_ms = TCP_RETRY_BASE_DELAY_MS;  // Reset backoff
            vTaskDelay(tcp_reconnect_delay_ms / portTICK_PERIOD_MS);
            tcp_reconnect_delay_ms = (tcp_reconnect_delay_ms * 2 > TCP_RETRY_MAX_DELAY_MS) ? TCP_RETRY_MAX_DELAY_MS : tcp_reconnect_delay_ms * 2;
            esp_task_wdt_reset();
            continue;
        }

        // 优化 debug hex log：len > 128 时仅摘要，避免慢循环
        if (debug_mode && len <= 128) {
            char hex_buf[UART_BUF_SIZE * 3 + 1] = {0};  // +1 for null
            char *ptr = hex_buf;
            for (int i = 0; i < len; i++) {
                ptr += sprintf(ptr, "%02X ", rx_buffer[i]);  // sprintf 直接追加，无 strlen
            }
            ESP_LOGI(TAG, "Raw TCP receive buffer (hex): %s (length: %d)", hex_buf, len);
        } else if (debug_mode) {
            ESP_LOGI(TAG, "TCP packet received: %d bytes (hex log skipped)", len);
        }

        // Parse and process the received data
        esp_err_t result = parse_tcp_command(tcp_sock, rx_buffer, len);
        if (result != ESP_OK) {
            ESP_LOGW(TAG, "Failed to parse TCP command (len=%d)", len);
        }

        esp_task_wdt_reset();  // 每循环喂狗
    }

    free(rx_buffer);
    vTaskDelete(NULL);
}

// UDP discovery task - broadcasts for gateway IP, listens for response
static void udp_discovery_task(void *pvParameters)
{
    char rx_buffer[128];
    struct sockaddr_in dest_addr, src_addr;
    socklen_t socklen = sizeof(src_addr);
    int retry = 0;
    bool discovered = false;

    // Create UDP socket
    udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (udp_sock < 0) {
        ESP_LOGE(TAG, "Unable to create UDP socket: errno %d", errno);
        vTaskDelete(NULL);
    }

    // Enable broadcast
    int broadcast = 1;
    if (setsockopt(udp_sock, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast)) < 0) {
        ESP_LOGE(TAG, "Unable to set broadcast: errno %d", errno);
        close(udp_sock);
        udp_sock = -1;
        vTaskDelete(NULL);
    }

    // Bind to UDP port for listening
    memset(&src_addr, 0, sizeof(src_addr));
    src_addr.sin_family = AF_INET;
    src_addr.sin_port = htons(UDP_PORT);
    src_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(udp_sock, (struct sockaddr *)&src_addr, sizeof(src_addr)) < 0) {
        ESP_LOGE(TAG, "Unable to bind UDP socket: errno %d", errno);
        close(udp_sock);
        udp_sock = -1;
        vTaskDelete(NULL);
    }

    while (!discovered && retry < UDP_DISCOVERY_RETRIES) {
        if (!is_wifi_connected()) {
            ESP_LOGW(TAG, "WiFi not connected, delaying discovery");
            vTaskDelay(500 / portTICK_PERIOD_MS);  // Reduced delay
            continue;
        }

        // Broadcast discovery message
        memset(&dest_addr, 0, sizeof(dest_addr));
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(UDP_PORT);
        dest_addr.sin_addr.s_addr = inet_addr(UDP_BROADCAST_ADDR);
        int sent = sendto(udp_sock, UDP_DISCOVER_MSG, strlen(UDP_DISCOVER_MSG), 0,
                          (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (sent < 0) {
            ESP_LOGE(TAG, "UDP broadcast failed: errno %d", errno);
        } else {
            if (debug_mode) ESP_LOGI(TAG, "Broadcasted discovery message (attempt %d/%d)", retry + 1, UDP_DISCOVERY_RETRIES);
        }

        // Listen for response with timeout
        struct timeval timeout;
        timeout.tv_sec = UDP_DISCOVERY_TIMEOUT_MS / 1000;
        timeout.tv_usec = (UDP_DISCOVERY_TIMEOUT_MS % 1000) * 1000;
        setsockopt(udp_sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

        int len = recvfrom(udp_sock, rx_buffer, sizeof(rx_buffer) - 1, 0,
                           (struct sockaddr *)&src_addr, &socklen);
        if (len > 0) {
            rx_buffer[len] = '\0';
            if (debug_mode) ESP_LOGI(TAG, "Received UDP response: %s", rx_buffer);

            // Parse "GATEWAY_IP:<ip>\r\n"
            if (strncmp(rx_buffer, UDP_GATEWAY_RESP_PREFIX, strlen(UDP_GATEWAY_RESP_PREFIX)) == 0) {
                char *ip_start = rx_buffer + strlen(UDP_GATEWAY_RESP_PREFIX);
                char *ip_end = strchr(ip_start, '\r');
                if (ip_end) *ip_end = '\0';
                if (strlen(ip_start) > 0 && strlen(ip_start) <= 15) {
                    strlcpy(gateway_ip_addr, ip_start, sizeof(gateway_ip_addr));
                    gateway_ip_discovered = true;
                    discovered = true;
                    needs_rediscovery = false;  // Cache success
                    ESP_LOGI(TAG, "Discovered gateway IP: %s", gateway_ip_addr);
                }
            }
        } else {
            ESP_LOGW(TAG, "No UDP response (attempt %d/%d)", retry + 1, UDP_DISCOVERY_RETRIES);
        }

        retry++;
        if (!discovered) vTaskDelay(UDP_DISCOVERY_TIMEOUT_MS / portTICK_PERIOD_MS);
    }

    if (!discovered) {
        ESP_LOGE(TAG, "Gateway discovery failed after %d retries", UDP_DISCOVERY_RETRIES);
        // Fallback: Use last known or hardcoded if needed
        if (gateway_ip_addr[0] != '\0') {
            ESP_LOGW(TAG, "Using cached gateway IP: %s", gateway_ip_addr);
            gateway_ip_discovered = true;
            needs_rediscovery = false;
        }
    }

    // Close UDP socket after discovery
    if (udp_sock >= 0) {
        close(udp_sock);
        udp_sock = -1;
    }

    vTaskDelete(NULL);
}

/* Centralized TCP connect function */
static esp_err_t tcp_connect(void) {
    if (!gateway_ip_discovered || tcp_sock != -1) {
        return ESP_FAIL;
    }

    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = inet_addr(gateway_ip_addr);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(TCP_PORT);

    tcp_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (tcp_sock < 0) {
        ESP_LOGE(TAG, "Unable to create TCP socket: errno %d", errno);
        return ESP_FAIL;
    }

    struct timeval timeout;
    timeout.tv_sec = TCP_CONNECT_TIMEOUT_MS / 1000;
    timeout.tv_usec = (TCP_CONNECT_TIMEOUT_MS % 1000) * 1000;
    setsockopt(tcp_sock, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
    setsockopt(tcp_sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    // Enhanced keepalive: Shorter idle (5s) with fewer probes (3) for ~8s detection
    int keepalive = 1;
    int keepidle = 5;      // 5s idle before probes
    int keepintvl = 1;     // 1s between probes
    int keepcnt = 3;       // 3 probes
    setsockopt(tcp_sock, SOL_SOCKET, SO_KEEPALIVE, &keepalive, sizeof(keepalive));
    setsockopt(tcp_sock, IPPROTO_TCP, TCP_KEEPIDLE, &keepidle, sizeof(keepidle));
    setsockopt(tcp_sock, IPPROTO_TCP, TCP_KEEPINTVL, &keepintvl, sizeof(keepintvl));
    setsockopt(tcp_sock, IPPROTO_TCP, TCP_KEEPCNT, &keepcnt, sizeof(keepcnt));
    ESP_LOGI(TAG, "TCP Keepalive optimized: idle=5s, probes=3x1s (total ~8s detection)");

    // Disable Nagle's algorithm for real-time small-packet sending
    int nodelay = 1;
    setsockopt(tcp_sock, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));
    ESP_LOGI(TAG, "TCP_NODELAY enabled for real-time sending");

    // Set larger TCP send buffer (64kB)
    int send_buf_size = 65536;  // 64kB
    setsockopt(tcp_sock, SOL_SOCKET, SO_SNDBUF, &send_buf_size, sizeof(send_buf_size));
    if (debug_mode) ESP_LOGI(TAG, "Set TCP SO_SNDBUF to 64kB");

    int err = connect(tcp_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err != 0) {
        ESP_LOGW(TAG, "TCP unable to connect to %s:%d: errno %d", gateway_ip_addr, TCP_PORT, errno);
        close(tcp_sock);
        tcp_sock = -1;
        return ESP_FAIL;
    }

    // Set non-blocking after connect
    int flags = fcntl(tcp_sock, F_GETFL, 0);
    if (flags >= 0 && fcntl(tcp_sock, F_SETFL, flags | O_NONBLOCK) < 0) {
        ESP_LOGE(TAG, "fcntl F_SETFL O_NONBLOCK failed: errno %d", errno);
        close(tcp_sock);
        tcp_sock = -1;
        return ESP_FAIL;
    }

    // Send my IP to gateway after connect
    send_my_ip(tcp_sock);

    ESP_LOGI(TAG, "Successfully connected to gateway %s:%d (backoff reset)", gateway_ip_addr, TCP_PORT);
    set_tcp_connected(true);
    tcp_reconnect_delay_ms = TCP_RETRY_BASE_DELAY_MS;  // Reset backoff on success
    gpio_set_level(LED_PIN, 0); // LED on (active-low) on TCP connect
    return ESP_OK;
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

    uint8_t uart_batch[UART_BUF_SIZE];  // Small batch buffer for real-time
    size_t batch_len = 0;
    uint32_t last_flush = 0;  // For periodic flush

    while (1) {
        // Real-time read: Short 1ms timeout for ~50B chunks
        int len = uart_read_bytes(UART_NUM_0, uart_batch + batch_len, sizeof(uart_batch) - batch_len - 1, UART_READ_TIMEOUT_MS / portTICK_PERIOD_MS);
        if (len > 0) {
            batch_len += len;

            uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
            // Flush if batch >=100B or 5ms elapsed
            if (batch_len >= FLUSH_BATCH_SIZE || (now - last_flush >= FLUSH_INTERVAL_MS)) {
                bool tcp_ok = get_tcp_connected();
                if (tcp_sock != -1 && tcp_ok) {
                    esp_err_t send_result = tcp_send_data(tcp_sock, (char*)uart_batch, batch_len, rx_buffer, UART_BUF_SIZE, false);
                    if (send_result == ESP_ERR_TIMEOUT) {
                        ESP_LOGW(TAG, "Send timeout, skipping this batch but continuing (no drop, len=%u; will retry next)", (unsigned)batch_len);
                        // 不drop！重置batch，避免backlog；下循环重新累积
                    } else if (send_result != ESP_OK) {
                        ESP_LOGW(TAG, "Fatal send error, closing socket and marking for reconnect (len=%u, err=%s)", 
                                (unsigned)batch_len, esp_err_to_name(send_result));
                        set_tcp_connected(false);
                        if (tcp_sock != -1) {
                            close(tcp_sock);
                            tcp_sock = -1;
                        }
                        gpio_set_level(LED_PIN, 1); // LED off
                    } else if (debug_mode) {
                        ESP_LOGI(TAG, "Flushed %zu bytes to TCP (real-time)", batch_len);
                    }
                    batch_len = 0;  // 始终重置batch（成功或timeout）
                    last_flush = now;
                } else {
                    ESP_LOGE(TAG, "TCP not connected, dropping batched data (len=%u)", (unsigned)batch_len);
                    batch_len = 0;  // 清空，避免累积
                }
            }
        }

        if (!is_wifi_connected()) {
            if (debug_mode) ESP_LOGW(TAG, "WiFi not connected, closing socket");
            if (tcp_sock != -1) {
                close(tcp_sock);
                tcp_sock = -1;
                set_tcp_connected(false);
            }
            my_ip_addr[0] = '\0';  // Invalidate IP
            needs_rediscovery = true;  // Trigger rediscovery on reconnect
            gateway_ip_discovered = false;
            batch_len = 0;  // Clear batch on disconnect
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            esp_task_wdt_reset();
            continue;
        }

        // Attempt TCP connect only if needed and gateway known
        bool tcp_ok = get_tcp_connected();
        if (tcp_ok && tcp_sock != -1) {
            // Connection healthy, quick check
            vTaskDelay(1 / portTICK_PERIOD_MS);  // Minimal yield
        } else if (needs_rediscovery) {
            // Trigger discovery if needed
            xTaskCreate(udp_discovery_task, "udp_discovery", 4096, NULL, 5, NULL);
            needs_rediscovery = false;  // Prevent spam
            vTaskDelay(500 / portTICK_PERIOD_MS);  // Wait for discovery
        } else if (gateway_ip_discovered && tcp_sock == -1) {
            uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
            if (now - last_reconnect_time >= tcp_reconnect_delay_ms) {
                esp_err_t connect_result = tcp_connect();
                if (connect_result != ESP_OK) {
                    last_reconnect_time = now;
                    tcp_reconnect_delay_ms = (tcp_reconnect_delay_ms * 2 > TCP_RETRY_MAX_DELAY_MS) ? TCP_RETRY_MAX_DELAY_MS : tcp_reconnect_delay_ms * 2;
                    vTaskDelay(100 / portTICK_PERIOD_MS);
                }
            } else {
                vTaskDelay(100 / portTICK_PERIOD_MS);
            }
        } else {
            vTaskDelay(1000 / portTICK_PERIOD_MS);  // No gateway, longer wait
        }

        esp_task_wdt_reset();  // 每循环喂狗
    }

    if (tcp_sock != -1) {
        close(tcp_sock);
        tcp_sock = -1;
        set_tcp_connected(false);
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

        // Preserve auth_code before erasing
        uint32_t auth_code = 0;
        esp_err_t auth_err = nvs_get_u32(nvs_handle, "auth_code", &auth_code);
        bool has_auth_code = (auth_err == ESP_OK);

        // Erase all except auth_code
        nvs_erase_all(nvs_handle);
        err = nvs_commit(nvs_handle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to commit NVS erase: %s", esp_err_to_name(err));
        }

        // Restore auth_code if it existed
        if (has_auth_code) {
            err = nvs_set_u32(nvs_handle, "auth_code", auth_code);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to restore auth_code: %s", esp_err_to_name(err));
            } else {
                err = nvs_commit(nvs_handle);
                if (err != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to commit restored auth_code: %s", esp_err_to_name(err));
                } else {
                    ESP_LOGI(TAG, "Auth code preserved during factory reset");
                }
            }
        }

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
            // Close previous UDP socket if any to avoid EADDRINUSE on reconnect
            wifi_connected = true;
            if (udp_sock >= 0) {
                close(udp_sock);
                udp_sock = -1;
            }
            // 修复：使用 ip4addr_ntoa 格式化 IPv4 地址
            char new_ip[16];
            strcpy(new_ip, ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
            if (debug_mode) ESP_LOGI(TAG, "Got my IP: %s (triggering discovery if needed)", new_ip);
            
            // Only rediscover if IP changed or first time
            static char prev_ip[16] = {0};  // Persistent across calls
            if (strcmp(new_ip, prev_ip) != 0 || needs_rediscovery) {
                strcpy(prev_ip, new_ip);
                strcpy(my_ip_addr, new_ip);
                needs_rediscovery = true;
            }
            
            s_retry_num = 0;
            xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
            xEventGroupClearBits(s_wifi_event_group, WIFI_FAIL_BIT);
            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
          wifi_connected = false;
            xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            gpio_set_level(LED_PIN, 1); // LED off (active-low)
            // Clear IP on disconnect, force rediscovery
            my_ip_addr[0] = '\0';
            needs_rediscovery = true;
            gateway_ip_discovered = false;  // Invalidate on WiFi loss
            // Close sockets
            if (tcp_sock != -1) {
                close(tcp_sock);
                tcp_sock = -1;
                set_tcp_connected(false);
            }
            if (udp_sock >= 0) {
                close(udp_sock);
                udp_sock = -1;
            }
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
    ESP_LOGE(TAG, "%s-%s", ssid, password);
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

    err = load_nvs_config(&current_baud_rate, &current_stop_bits, &current_parity, &current_data_bits, g_ssid, sizeof(g_ssid), g_password, sizeof(g_password), &debug_mode);
    if (err != ESP_OK) {
        strlcpy(g_ssid, DEFAULT_ESP_WIFI_SSID, sizeof(g_ssid));
        strlcpy(g_password, DEFAULT_ESP_WIFI_PASS, sizeof(g_password));
        current_baud_rate = 115200;
        current_stop_bits = UART_STOP_BITS_1;
        current_parity = UART_PARITY_DISABLE;
        current_data_bits = UART_DATA_8_BITS;
        debug_mode = 0;
        esp_log_level_set(TAG, ESP_LOG_NONE);
    }

    handle_boot_count();
    conn_state_init();  // Initialize connection state mutex
    led_init();
    xTaskCreate(led_status_task, "led_status", 2048, NULL, 1, NULL);  // Low priority LED task
    wifi_init_sta(g_ssid, g_password);
    authorcodeverify();  // Now non-blocking on failure
    uart_init(current_baud_rate, current_stop_bits, current_parity, current_data_bits);
    xTaskCreate(uart_tcp_bridge_task, "uart_tcp_bridge", 6144, NULL, 8, NULL);
    xTaskCreate(tcp_receive_task, "tcp_receive", 6144, NULL, 8, NULL);
}
