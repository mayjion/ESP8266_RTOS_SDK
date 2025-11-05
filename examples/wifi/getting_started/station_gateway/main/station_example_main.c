/* WiFi Station with UART to TCP Bridge (Modified for TCP Data Transparent Protocol)
   This version implements the binary framing protocol with sequence numbers, ACK/NAK,
   byte escaping, and CRC-16-CCITT. Key changes:
   - Replaced length-prefix and \r\n parsing with protocol frames (Type, Seq, Len, Payload, CRC).
   - UART data sent as Type=0x00 frames with incrementing Seq.
   - Received frames parsed with unescaping and CRC verification.
   - Commands (Type=0x01) parsed as TLV, handled (e.g., set baud/WiFi), respond with Type=0x02.
   - Duplicate Seq detection (simple last Seq check).
   - Removed TO: and legacy \r\n command parsing (BAUD=, WIFI=).
   - Removed client IP/JSON send on connect (assumed handled by UDP discovery).
   - Added CRC-16-CCITT, escape/unescape functions.
   - Responses sent immediately for commands.
   - Reboot on WiFi config change after ACK.
   - Heartbeat (Type=0x03) ignored for now.
   Compile and flash to ESP8266 as before.
*/
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <fcntl.h>
#include <stdint.h>
#include <math.h> // For pow in backoff (unused but kept for completeness)
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
#include "esp_task_wdt.h"
#include "lwip/ip4_addr.h"
#define DEFAULT_ESP_WIFI_SSID "Funlight"
#define DEFAULT_ESP_WIFI_PASS "funlight"
#define EXAMPLE_ESP_MAXIMUM_RETRY 10
#define NVS_NAMESPACE "config"
#define BOOT_COUNT_KEY "boot_count"
#define BOOT_COUNT_THRESHOLD 6
#define BOOT_COUNT_RESET_DELAY_MS 10000 // 10 seconds
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1
static const char *TAG = "slave";
#define TCP_PORT 12345
#define UDP_PORT 12346 // UDP discovery port
#define UDP_BROADCAST_ADDR "255.255.255.255"
#define UDP_DISCOVER_MSG "DISCOVER_GATEWAY\r\n"
#define UDP_GATEWAY_RESP_PREFIX "GATEWAY_IP:"
#define CONFIG_EXAMPLE_IPV4 1
#define UART_BUF_SIZE (512)
#define TCP_RETRY_BASE_DELAY_MS 1000 // Start with 1s
#define TCP_RETRY_MAX_DELAY_MS 32000 // Cap at 32s
#define TCP_CONNECT_TIMEOUT_MS 10000
#define TCP_KEEPALIVE_IDLE_MS 5000 // 5s idle before probes
#define UDP_DISCOVERY_TIMEOUT_MS 2000 // Reduced for faster discovery
#define UDP_DISCOVERY_RETRIES 5 // Reduced retries
#define UART_READ_TIMEOUT_MS 1 // 1ms for real-time ~50B reads
#define FLUSH_BATCH_SIZE 100 // Flush if batch >=100B
#define FLUSH_INTERVAL_MS 5 // Or every 5ms
#define MAX_FRAME_UNESC 1024 // Max unescaped frame size (5 + 0xFFFF + 2, but cap)
#define MAX_RAW_ACCUM 2048 // Raw accumulation with escapes
#define LED_PIN GPIO_NUM_2
static uint32_t current_baud_rate = 115200;
static uint8_t current_stop_bits = UART_STOP_BITS_1;
static uint8_t current_parity = UART_PARITY_DISABLE;
static uint8_t current_data_bits = UART_DATA_8_BITS;
static bool is_tcp_connected = false;
static int tcp_sock = -1;
static uint8_t debug_mode = 1;
// Store gateway IP (discovered via UDP)
static char gateway_ip_addr[16] = {0}; // e.g., "192.168.4.1" (discovered)
static bool gateway_ip_discovered = false;
static bool needs_rediscovery = true; // Flag to trigger discovery only when needed
static int udp_sock = -1; // UDP socket for discovery
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
static uint16_t out_seq = 0; // Outgoing sequence number
static bool initial_info_sent = false; // Flag to ensure initial JSON sent once

// Forward declarations
static int nonblocking_send(int sock, const char *payload, size_t payload_len, int max_retries);
static void udp_discovery_task(void *pvParameters);
static uint32_t compute_crc32(const uint8_t *data, size_t len);
static void build_and_send_frame(int sock, uint8_t type, uint16_t seq, const uint8_t *payload, size_t pay_len);

// Send initial device info JSON as Type=0x00 frame
static void send_initial_device_info(int sock) {
  if (initial_info_sent) return;

  // Get actual MAC
  uint8_t mac[6];
  esp_wifi_get_mac(ESP_IF_WIFI_STA, mac);
  char mac_str[18];
  sprintf(mac_str, "%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  // Get actual IP
  tcpip_adapter_ip_info_t ip_info;
  tcpip_adapter_get_ip_info(ESP_IF_WIFI_STA, &ip_info);
  char ip_str[16];
  sprintf(ip_str, "%d.%d.%d.%d",
          IP2STR(&ip_info.ip));

  // Build JSON using sprintf
  char json_str[256];  // Sufficient for this JSON
int json_len = snprintf(json_str, sizeof(json_str),
                            "{\"mac\":\"%s\",\"ip\":\"%s\",\"baud\":%u,\"ssid\":\"%s\",\"pwd\":\"%s\"}",
                            mac_str, ip_str, current_baud_rate, g_ssid, g_password);


  if (json_len > 0 && json_len < (int)sizeof(json_str)) {
    build_and_send_frame(sock, 0x00 /* passthrough */, out_seq++, (const uint8_t *)json_str, (size_t)json_len);
    ESP_LOGI(TAG, "Sent initial device info JSON (%d bytes)", json_len);
    initial_info_sent = true;
  } else {
    ESP_LOGE(TAG, "Failed to build JSON string");
  }
}

static void conn_state_init(void) {
    if (!conn_state_mutex) {
        conn_state_mutex = xSemaphoreCreateMutex();
    }
}
static void set_tcp_connected(bool connected) {
    if (conn_state_mutex && xSemaphoreTake(conn_state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (!connected) {
            initial_info_sent = false;
        }
        is_tcp_connected = connected;
        if (connected && !initial_info_sent && tcp_sock >= 0) {
            send_initial_device_info(tcp_sock);
        }
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
            gpio_set_level(LED_PIN, 0); // On
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            gpio_set_level(LED_PIN, 1); // Off
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
                vTaskDelay(1000 / portTICK_PERIOD_MS); // 保持常亮
            }
        }
        esp_task_wdt_reset();
    }
}
/* CRC-16-CCITT implementation */
static uint16_t crc16_ccitt(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
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
/* Escape data */
static size_t escape_data(const uint8_t *in, size_t in_len, uint8_t *out, size_t out_max) {
    size_t out_idx = 0;
    for (size_t i = 0; i < in_len; i++) {
        uint8_t b = in[i];
        if (out_idx + 2 > out_max) return 0; // Overflow
        if (b == 0xFF) {
            out[out_idx++] = 0xFE;
            out[out_idx++] = 0xDF;
        } else if (b == 0xFE) {
            out[out_idx++] = 0xFE;
            out[out_idx++] = 0xDE;
        } else {
            out[out_idx++] = b;
        }
    }
    return out_idx;
}
/* Build and send a frame */
static void build_and_send_frame(int sock, uint8_t type, uint16_t seq, const uint8_t *payload, size_t pay_len) {
    if (pay_len > 0xFFFF) {
        ESP_LOGE(TAG, "Payload too long: %zu", pay_len);
        return;
    }
    // Unescaped body without CRC
    size_t unesc_len = 1 + 2 + 2 + pay_len;
    uint8_t unesc[unesc_len + 2]; // +2 for CRC
    unesc[0] = type;
    unesc[1] = (seq >> 8) & 0xFF;
    unesc[2] = seq & 0xFF;
    unesc[3] = (pay_len >> 8) & 0xFF;
    unesc[4] = pay_len & 0xFF;
    if (pay_len > 0) {
        memcpy(unesc + 5, payload, pay_len);
    }
    // Compute CRC
    uint16_t crc = crc16_ccitt(unesc, unesc_len);
    unesc[unesc_len] = (crc >> 8) & 0xFF;
    unesc[unesc_len + 1] = crc & 0xFF;
    // Escape the full body (including CRC)
    uint8_t esc_body[(unesc_len + 2) * 2];
    size_t esc_len = escape_data(unesc, unesc_len + 2, esc_body, sizeof(esc_body));
    if (esc_len == 0) {
        ESP_LOGE(TAG, "Escape failed");
        return;
    }
    // Full frame
    uint8_t full_frame[2 + esc_len];
    full_frame[0] = 0xFF;
    full_frame[1] = 0xFF;
    memcpy(full_frame + 2, esc_body, esc_len);
    // Send
    int sent = nonblocking_send(sock, (const char *)full_frame, 2 + esc_len, 10);
    if (sent == (int)(2 + esc_len)) {
        if (debug_mode) ESP_LOGI(TAG, "Sent frame type=0x%02X seq=%04X len=%zu", type, seq, pay_len);
    } else {
        ESP_LOGW(TAG, "Incomplete frame send: %d/%zu", sent, 2 + esc_len);
    }
}
/* Author code verification - Non-blocking on failure */
static void authorcodeverify(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS for auth: %s", esp_err_to_name(err));
        auth_failed = true; // Set flag for LED indication
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
        auth_failed = true; // Set flag instead of blocking
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
// Auxiliary for big-endian uint32
static void write_be32(uint8_t *buf, uint32_t value) {
    buf[0] = (value >> 24) & 0xFF;
    buf[1] = (value >> 16) & 0xFF;
    buf[2] = (value >> 8) & 0xFF;
    buf[3] = value & 0xFF;
}
/* Non-blocking send with retry, returns bytes sent (>=0) or -1 (fatal) */
static int nonblocking_send(int sock, const char *payload, size_t payload_len, int max_retries) {
    if (payload_len == 0) return 0;
    size_t sent = 0;
    int retries = max_retries;
    while (sent < payload_len) {
        int bytes = send(sock, payload + sent, payload_len - sent, 0);
        if (bytes > 0) {
            sent += bytes;
            retries = max_retries; /* 进展时重置重试，防止无进展死锁 */
            esp_task_wdt_reset(); // Feed during retries
            continue;
        }
        if (bytes < 0) {
            int lwip_errno = errno;
            if (lwip_errno == EAGAIN || lwip_errno == EWOULDBLOCK) {
                if (--retries <= 0) {
                    ESP_LOGW(TAG, "Send partial after %d retries: %zu/%zu bytes (buffer full?)", max_retries, sent, payload_len);
                    return (int)sent; /* 返回部分发送字节，而非 TIMEOUT */
                }
                vTaskDelay(2 / portTICK_PERIOD_MS); /* 延迟增至 2ms，给排水时间 */
                continue;
            } else {
                ESP_LOGE(TAG, "Send real error: errno %d", lwip_errno);
                return -1; /* 致命错误 */
            }
        } else { /* bytes == 0 */
            ESP_LOGE(TAG, "Send returned 0, treating as error");
            return -1;
        }
    }
    return (int)sent; /* 全发成功 */
}
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
    int keepidle = 5; // 5s idle before probes
    int keepintvl = 1; // 1s between probes
    int keepcnt = 3; // 3 probes
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
    int send_buf_size = 65536; // 64kB
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
    ESP_LOGI(TAG, "Successfully connected to gateway %s:%d (backoff reset)", gateway_ip_addr, TCP_PORT);
    set_tcp_connected(true);
    tcp_reconnect_delay_ms = TCP_RETRY_BASE_DELAY_MS; // Reset backoff on success
    gpio_set_level(LED_PIN, 0); // LED on (active-low) on TCP connect
    return ESP_OK;
}
/* UART to TCP bridge task - Now uses protocol frames */
static void uart_tcp_bridge_task(void *pvParameters)
{
    uint8_t *uart_data = (uint8_t *) malloc(UART_BUF_SIZE);
    if (!uart_data) {
        ESP_LOGE(TAG, "Failed to allocate memory for buffers");
        vTaskDelete(NULL);
    }
    uint8_t uart_batch[UART_BUF_SIZE]; // Small batch buffer for real-time
    size_t batch_len = 0;
    uint32_t last_flush = 0; // For periodic flush
    while (1) {
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        /* 实时读取 UART (短超时 ~1ms) */
        int len = uart_read_bytes(UART_NUM_0, uart_batch + batch_len, sizeof(uart_batch) - batch_len - 1, UART_READ_TIMEOUT_MS / portTICK_PERIOD_MS);
        if (len > 0) {
            batch_len += len;
            last_flush = now; // 更新时间戳，避免误判空闲
        }
        /* 检查 flush 条件：批次满或 5ms 超时 */
        if (batch_len >= FLUSH_BATCH_SIZE || (now - last_flush >= FLUSH_INTERVAL_MS)) {
            bool tcp_ok = get_tcp_connected();
            if (tcp_sock != -1 && tcp_ok && batch_len > 0) {
                // Send as data frame
                build_and_send_frame(tcp_sock, 0x00, out_seq++, uart_batch, batch_len);
                batch_len = 0;
            } else if (!tcp_ok || tcp_sock == -1) {
                /* TCP 未连：丢弃批次 */
                ESP_LOGE(TAG, "TCP not connected, dropping batched data (len=%zu)", (unsigned)batch_len);
                batch_len = 0;
            }
            last_flush = now; // 更新时间戳
        }
        /* 原 WiFi/TCP 连接逻辑（无变化） */
        if (!is_wifi_connected()) {
            if (debug_mode) ESP_LOGW(TAG, "WiFi not connected, closing socket");
            if (tcp_sock != -1) {
                close(tcp_sock);
                tcp_sock = -1;
                set_tcp_connected(false);
            }
            needs_rediscovery = true; // Trigger rediscovery on reconnect
            gateway_ip_discovered = false;
            batch_len = 0; // Clear batch on disconnect
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            esp_task_wdt_reset();
            continue;
        }
        // Attempt TCP connect only if needed and gateway known
        bool tcp_ok = get_tcp_connected();
        if (tcp_ok && tcp_sock != -1) {
            // Connection healthy, quick check
            vTaskDelay(1 / portTICK_PERIOD_MS); // Minimal yield
        } else if (needs_rediscovery) {
            // Trigger discovery if needed
            xTaskCreate(udp_discovery_task, "udp_discovery", 4096, NULL, 5, NULL);
            needs_rediscovery = false; // Prevent spam
            vTaskDelay(500 / portTICK_PERIOD_MS); // Wait for discovery
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
            vTaskDelay(1000 / portTICK_PERIOD_MS); // No gateway, longer wait
        }
        esp_task_wdt_reset(); // 每循环喂狗
    }
    if (tcp_sock != -1) {
        close(tcp_sock);
        tcp_sock = -1;
        set_tcp_connected(false);
    }
    free(uart_data);
    vTaskDelete(NULL);
}

// Additions to station_example_main.c
// Place these at the top of the file, after includes, before any functions.
// These provide the missing enums and helper functions from the protocol.

// Enums from protocol (add near other defines)
enum frame_type {
    FRAME_TYPE_DATA = 0x00,    // Passthrough
    FRAME_TYPE_CMD = 0x01,     // Command
    FRAME_TYPE_RESP = 0x02,    // ACK/NAK
    FRAME_TYPE_HEARTBEAT = 0x03
};

enum result_code {
    RESULT_OK = 0x00,          // Success
    RESULT_UNKNOWN_CMD = 0x01, // Unknown command
    RESULT_PARAM_ERR = 0x02,   // Param error
    RESULT_EXEC_FAIL = 0x03,   // Exec fail
    RESULT_CRC_ERR = 0x04      // CRC error
};

// Helper: Manual find FF FF header (ESP8266 compatible, no memmem)
static const uint8_t *find_header(const uint8_t *buf, size_t len) {
    for (size_t i = 0; i + 1 < len; i++) {
        if (buf[i] == 0xFF && buf[i + 1] == 0xFF) {
            return buf + i;
        }
    }
    return NULL;
}

// Byte unescape (symmetric to server's escape_data)
static size_t unescape_data(const uint8_t *in, size_t in_len, uint8_t *out, size_t out_max) {
    if (!in || in_len == 0 || !out || out_max < in_len) return 0;
    size_t out_idx = 0;
    size_t i = 0;
    while (i < in_len) {
        uint8_t b = in[i++];
        if (out_idx >= out_max) return 0;  // Overflow
        if (b == 0xFE) {
            if (i >= in_len) return 0;  // Incomplete escape
            uint8_t next = in[i++];
            if (next == 0xDF) {
                b = 0xFF;
            } else if (next == 0xDE) {
                b = 0xFE;
            } else {
                // Invalid escape sequence; treat FE as literal
                out[out_idx++] = 0xFE;
                b = next;  // Process next as normal
            }
        }
        out[out_idx++] = b;
    }
    return out_idx;
}

// Forward declare send_response_frame (before tcp_receive_task)
static esp_err_t send_response_frame(uint16_t seq, uint8_t type, uint8_t tag, uint8_t result, const uint8_t *resp_data, size_t data_len);

// Updated tcp_receive_task with progressive unescaping for correct parsing, additional diagnostic logging for raw data and parsing results
// Fixed bug: Previously assumed fixed positions without unescaping shifts; now unescapes progressively to parse len, then full frame.
// Added raw hex dump on every recv (limited to 128B to avoid flood), and detailed parsing logs.
// Set debug_mode=1 by default in app_main for testing.

static void tcp_receive_task(void *pvParameters)
{
    const char *TAG = "tcp_receive";
    uint8_t recv_buf[MAX_RAW_ACCUM];  // Buffer for raw TCP data (with escapes)
    int recv_len = 0;
    uint16_t last_seq = 0x0000;  // Track last processed seq to detect duplicates (simple)

    // Diagnostic: Log task start
    ESP_LOGI(TAG, "tcp_receive_task started (debug_mode=%u)", debug_mode);

    while (1) {
        if (!get_tcp_connected() || tcp_sock < 0) {
            // Diagnostic: Log disconnect state
            if (debug_mode) ESP_LOGD(TAG, "Task idle: not connected (tcp_sock=%d)", tcp_sock);
            vTaskDelay(pdMS_TO_TICKS(100));  // Backoff if not connected
            continue;
        }

        // Non-blocking recv (timeout 1ms for real-time)
        struct timeval tv = {0, 1000};  // 1ms
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(tcp_sock, &readfds);
        int sel = select(tcp_sock + 1, &readfds, NULL, NULL, &tv);
        if (sel <= 0) {
            continue;  // No data or error (common in idle)
        }

        // Read available data
        recv_len = recv(tcp_sock, recv_buf, sizeof(recv_buf), 0);
        if (recv_len <= 0) {
            if (recv_len < 0) {
                ESP_LOGE(TAG, "TCP recv error: %d (%s)", errno, strerror(errno));
            } else {
                ESP_LOGW(TAG, "TCP recv returned 0 (peer closed?)");
            }
            // On error or EOF, close and flag disconnect
            close(tcp_sock);
            tcp_sock = -1;
            set_tcp_connected(false);
            needs_rediscovery = true;  // Trigger reconnect
            continue;
        }

        // Diagnostic: Log every recv with raw hex dump (limit to first 128B to avoid flood)
        if (debug_mode) {
            ESP_LOGI(TAG, "TCP recv: %d bytes (first: %02X %02X ...)", recv_len, recv_buf[0], (recv_len > 1 ? recv_buf[1] : 0));
            // Hex dump of raw data
            char hex_buf[3 * 128 + 1] = {0};
            size_t hex_idx = 0;
            size_t dump_len = (recv_len > 128) ? 128 : (size_t)recv_len;
            for (size_t i = 0; i < dump_len; ++i) {
                hex_idx += snprintf(hex_buf + hex_idx, sizeof(hex_buf) - hex_idx, "%02X ", recv_buf[i]);
                if ((i + 1) % 16 == 0 || i == dump_len - 1) {
                    ESP_LOGI(TAG, "Raw recv hex: %s", hex_buf);
                    hex_idx = 0;
                    memset(hex_buf, 0, sizeof(hex_buf));
                }
            }
            if (recv_len > 128) {
                ESP_LOGI(TAG, "Raw recv: ... (truncated at 128B, total %dB)", recv_len);
            }
        }

        // Process the received buffer for frames (multi-frame possible)
        size_t pos = 0;
        while (pos < (size_t)recv_len) {
            // Find next frame header (FF FF)
            const uint8_t *frame_start = find_header(recv_buf + pos, recv_len - pos);
            if (!frame_start) {
                // Diagnostic: No header found
                if (debug_mode) ESP_LOGD(TAG, "No FF FF header from pos=%zu (remaining=%zu bytes)", pos, recv_len - pos);
                // No more frames; shift remaining partial data to front if any
                if (pos > 0) {
                    memmove(recv_buf, recv_buf + pos, recv_len - pos);
                    recv_len -= pos;
                }
                break;  // Exit inner loop
            }

            size_t header_offset = frame_start - (recv_buf + pos);
            pos += header_offset;  // Advance to header position
            size_t header_pos = pos;  // Remember for potential shift

            // Diagnostic: Header found
            if (debug_mode) ESP_LOGI(TAG, "Found header FF FF at pos=%zu", pos);

            // Start progressive unescaping from body (after header)
            const uint8_t *raw_body_ptr = recv_buf + pos + 2;
            size_t avail_raw = recv_len - (pos + 2);
            uint8_t unesc_frame[MAX_FRAME_UNESC + 7];  // type(1)+seq(2)+len(2)+payload(max)+crc(2)
            size_t unesc_idx = 0;
            size_t raw_body_consumed = 0;

            // Phase 1: Unescape first 5 bytes (type + seq + len) to parse len
            bool incomplete = false;
            while (unesc_idx < 5 && raw_body_consumed < avail_raw) {
                uint8_t b = raw_body_ptr[raw_body_consumed];
                raw_body_consumed++;
                if (b == 0xFE) {
                    if (raw_body_consumed >= avail_raw) {
                        incomplete = true;
                        break;
                    }
                    uint8_t next = raw_body_ptr[raw_body_consumed];
                    raw_body_consumed++;
                    if (next == 0xDF) {
                        b = 0xFF;
                    } else if (next == 0xDE) {
                        b = 0xFE;
                    } else {
                        // Invalid: treat as literal FE + next
                        unesc_frame[unesc_idx++] = 0xFE;
                        b = next;
                        // Don't increment unesc_idx again
                    }
                }
                if (unesc_idx < sizeof(unesc_frame)) {
                    unesc_frame[unesc_idx++] = b;
                } else {
                    incomplete = true;
                    break;
                }
            }

            if (incomplete || unesc_idx < 5) {
                if (debug_mode) ESP_LOGD(TAG, "Incomplete fixed header (unesc=%zu/5, raw_consumed=%zu/%zu)", unesc_idx, raw_body_consumed, avail_raw);
                // Shift buffer to keep header + partial body
                memmove(recv_buf, recv_buf + header_pos, recv_len - header_pos);
                recv_len -= header_pos;
                break;
            }

            // Parse fixed fields from unescaped
            uint8_t type = unesc_frame[0];
            uint16_t seq = (unesc_frame[1] << 8) | unesc_frame[2];
            uint16_t pay_len = (unesc_frame[3] << 8) | unesc_frame[4];

            // Diagnostic: Log parsed fixed fields
            if (debug_mode) ESP_LOGI(TAG, "Parsed fixed: type=0x%02X, seq=0x%04X, pay_len=%u (raw_body_consumed for fixed=%zu)", type, seq, pay_len, raw_body_consumed);

            // Validate pay_len
            if (pay_len > MAX_FRAME_UNESC - 7) {
                ESP_LOGE(TAG, "Invalid pay_len %u for Seq=0x%04X - too large", pay_len, seq);
                // Skip this frame: advance past header + consumed raw
                pos += 2 + raw_body_consumed;
                continue;
            }

            // Phase 2: Unescape remaining pay_len + 2 (payload + CRC)
            size_t remaining_unesc_needed = pay_len + 2;
            while (unesc_idx < 5 + remaining_unesc_needed && raw_body_consumed < avail_raw) {
                uint8_t b = raw_body_ptr[raw_body_consumed];
                raw_body_consumed++;
                if (b == 0xFE) {
                    if (raw_body_consumed >= avail_raw) {
                        incomplete = true;
                        break;
                    }
                    uint8_t next = raw_body_ptr[raw_body_consumed];
                    raw_body_consumed++;
                    if (next == 0xDF) {
                        b = 0xFF;
                    } else if (next == 0xDE) {
                        b = 0xFE;
                    } else {
                        unesc_frame[unesc_idx++] = 0xFE;  // Already incremented? No, append FE
                        b = next;
                    }
                }
                if (unesc_idx < sizeof(unesc_frame)) {
                    unesc_frame[unesc_idx++] = b;
                } else {
                    incomplete = true;
                    break;
                }
            }

            if (incomplete || unesc_idx < 5 + remaining_unesc_needed) {
                if (debug_mode) ESP_LOGD(TAG, "Incomplete full frame (unesc=%zu/%zu, raw_consumed=%zu/%zu)", unesc_idx, 5 + remaining_unesc_needed, raw_body_consumed, avail_raw);
                // Shift to keep header + partial
                memmove(recv_buf, recv_buf + header_pos, recv_len - header_pos);
                recv_len -= header_pos;
                break;
            }

            // Now full unescaped frame in unesc_frame[0..5+pay_len+1]
            const uint8_t *unesc_payload = unesc_frame + 5;
            uint16_t computed_crc = crc16_ccitt(unesc_frame, 5 + pay_len);
            uint16_t recv_crc = (unesc_frame[5 + pay_len] << 8) | unesc_frame[5 + pay_len + 1];

            // Diagnostic: Log CRC details
            if (debug_mode) ESP_LOGI(TAG, "CRC check: computed=0x%04X, received=0x%04X (Seq=0x%04X, total raw body consumed=%zu)", computed_crc, recv_crc, seq, raw_body_consumed);

            if (computed_crc != recv_crc) {
                ESP_LOGE(TAG, "CRC mismatch for Seq=0x%04X: computed=0x%04X != recv=0x%04X - dropping frame", seq, computed_crc, recv_crc);
                // Skip bad frame
                pos += 2 + raw_body_consumed;
                continue;
            }

            // Duplicate seq check (simple: ignore if matches last)
            if (seq == last_seq && type != FRAME_TYPE_RESP) {  // Resp may repeat for ACK
                ESP_LOGW(TAG, "Duplicate seq 0x%04X ignored", seq);
                pos += 2 + raw_body_consumed;
                continue;
            }
            last_seq = seq;

            // Diagnostic: Log successful parse
            if (debug_mode) ESP_LOGI(TAG, "Frame parsed successfully: type=0x%02X, seq=0x%04X, pay_len=%u, CRC OK", (unsigned int)type, (unsigned int)seq, (unsigned int)pay_len);

            // Dispatch by type
            switch (type) {
                case FRAME_TYPE_DATA:  // 0x00: Passthrough data
                    ESP_LOGI(TAG, ">>> ENTERED FRAME_TYPE_DATA (Seq=0x%04X, pay_len=%u) <<<", seq, pay_len);  // Marker log
                    if (pay_len > 0) {
                        // Forward to UART
                        int uart_sent = uart_write_bytes(UART_NUM_0, (const char *)unesc_payload, pay_len);
                        if (uart_sent != (int)pay_len) {
                            ESP_LOGE(TAG, "UART write incomplete: %d / %u", uart_sent, pay_len);
                        } else {
                            ESP_LOGI(TAG, "TCP→UART: Forwarded %u bytes passthrough data (Seq=0x%04X)", pay_len, seq);
                        }

                        // Debug logging for payload content (hex + ASCII preview)
                        if (debug_mode && pay_len > 0 && pay_len <= 128) {  // Limit to 128B to avoid log flood
                            // Hex dump (buffered to one log line)
                            char hex_buf[3 * 128 + 1] = {0};
                            size_t hex_idx = 0;
                            for (size_t i = 0; i < pay_len; ++i) {
                                hex_idx += snprintf(hex_buf + hex_idx, sizeof(hex_buf) - hex_idx, "%02X ", unesc_payload[i]);
                                if ((i + 1) % 16 == 0 || i == pay_len - 1) {  // Line every 16B
                                    ESP_LOGI(TAG, "Parsed pay hex: %s", hex_buf);
                                    hex_idx = 0;
                                    memset(hex_buf, 0, sizeof(hex_buf));
                                }
                            }

                            // ASCII preview (replace non-printable with '.')
                            char ascii_buf[129] = {0};  // +1 for null
                            size_t asc_idx = 0;
                            for (size_t i = 0; i < pay_len && asc_idx < 128; ++i) {
                                uint8_t b = unesc_payload[i];
                                ascii_buf[asc_idx++] = (b >= 32 && b <= 126) ? (char)b : '.';
                            }
                            ESP_LOGI(TAG, "Parsed pay ASCII: %s", ascii_buf);
                        } else if (debug_mode && pay_len > 128) {
                            ESP_LOGI(TAG, "TCP→UART: Large payload %uB (skipped detailed log)", pay_len);
                        }
                    } else {
                        ESP_LOGI(TAG, "TCP→UART: Empty passthrough frame (Seq=0x%04X)", seq);
                    }
                    ESP_LOGI(TAG, ">>> EXITED FRAME_TYPE_DATA <<<");  // Marker log
                    break;

                case FRAME_TYPE_CMD:  // 0x01: Command
                    ESP_LOGI(TAG, "Received CMD frame (Seq=0x%04X, Payload len=%u)", seq, pay_len);
                    // TODO: Parse TLV, handle commands (e.g., set baud/WiFi), send response
                    // For now, send generic ACK
                    send_response_frame(seq, FRAME_TYPE_RESP, 0x00, RESULT_OK, NULL, 0);  // Example ACK
                    break;

                case FRAME_TYPE_RESP:  // 0x02: Response (e.g., ACK/NAK from device, but here we receive from server?)
                    ESP_LOGI(TAG, "Received RESP frame (Seq=0x%04X, len=%u)", seq, pay_len);
                    // TODO: Handle if needed (e.g., for outgoing ACKs, but typically we send our own)
                    break;

                case FRAME_TYPE_HEARTBEAT:  // 0x03
                    ESP_LOGD(TAG, "Received heartbeat (Seq=0x%04X)", seq);
                    break;

                default:
                    ESP_LOGW(TAG, "Unknown frame type 0x%02X (Seq=0x%04X)", type, seq);
                    break;
            }

            // Advance pos past this full frame (header + raw body consumed)
            pos += 2 + raw_body_consumed;
        }  // End while(pos < recv_len)
    }  // End outer while(1)
}

// Corrected send_response_frame (now returns esp_err_t, uses build_and_send_frame for consistency)
static esp_err_t send_response_frame(uint16_t seq, uint8_t type, uint8_t tag, uint8_t result, const uint8_t *resp_data, size_t data_len)
{
    // Build payload: Tag + Result + Data
    uint8_t resp_payload[1 + 1 + MAX_FRAME_UNESC];
    size_t resp_pl_len = 2;
    resp_payload[0] = tag;
    resp_payload[1] = result;
    if (data_len > 0 && data_len <= MAX_FRAME_UNESC - 2) {
        memcpy(resp_payload + 2, resp_data, data_len);
        resp_pl_len += data_len;
    }

    // Use build_and_send_frame (non-blocking)
    if (get_tcp_connected() && tcp_sock >= 0) {
        build_and_send_frame(tcp_sock, type, seq, resp_payload, resp_pl_len);
        return ESP_OK;
    }
    return ESP_FAIL;
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
            vTaskDelay(500 / portTICK_PERIOD_MS); // Reduced delay
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
                    needs_rediscovery = false; // Cache success
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
            if (debug_mode) ESP_LOGI(TAG, "Got IP, triggering discovery if needed");
            needs_rediscovery = true;
            s_retry_num = 0;
            xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
            xEventGroupClearBits(s_wifi_event_group, WIFI_FAIL_BIT);
            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
          wifi_connected = false;
            xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            gpio_set_level(LED_PIN, 1); // LED off (active-low)
            // Clear on disconnect, force rediscovery
            needs_rediscovery = true;
            gateway_ip_discovered = false; // Invalidate on WiFi loss
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
    out_seq = 0;
    // Force debug_mode=1 for debugging (override NVS for now)
    debug_mode = 1;
    esp_log_level_set(TAG, ESP_LOG_INFO);  // Ensure logs are visible
    ESP_LOGI(TAG, "Debug mode forced to 1 for troubleshooting");
    err = load_nvs_config(&current_baud_rate, &current_stop_bits, &current_parity, &current_data_bits, g_ssid, sizeof(g_ssid), g_password, sizeof(g_password), &debug_mode);
    if (err != ESP_OK) {
        strlcpy(g_ssid, DEFAULT_ESP_WIFI_SSID, sizeof(g_ssid));
        strlcpy(g_password, DEFAULT_ESP_WIFI_PASS, sizeof(g_password));
        current_baud_rate = 115200;
        current_stop_bits = UART_STOP_BITS_1;
        current_parity = UART_PARITY_DISABLE;
        current_data_bits = UART_DATA_8_BITS;
        debug_mode = 1;  // Force 1
        esp_log_level_set(TAG, ESP_LOG_INFO);
    }
    handle_boot_count();
    conn_state_init(); // Initialize connection state mutex
    led_init();
    xTaskCreate(led_status_task, "led_status", 2048, NULL, 1, NULL); // Low priority LED task
    wifi_init_sta(g_ssid, g_password);
    authorcodeverify(); // Now non-blocking on failure
    uart_init(current_baud_rate, current_stop_bits, current_parity, current_data_bits);
    xTaskCreate(uart_tcp_bridge_task, "uart_tcp_bridge", 6144, NULL, 8, NULL);
    xTaskCreate(tcp_receive_task, "tcp_receive", 6144, NULL, 8, NULL);
}
