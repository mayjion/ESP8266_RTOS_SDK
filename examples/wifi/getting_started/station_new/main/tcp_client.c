// tcp_client.c
#include "tcp_client.h"
#include <fcntl.h>  // fcntl for nonblock
#include <sys/select.h>  // select for polling
#include <sys/time.h>
#include <unistd.h>
#include <errno.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "lwip/sockets.h"
#include "lwip/err.h"
#include "driver/uart.h"
#include "esp_wifi.h"  // For WiFi status
#include <stdlib.h>  // malloc
#include <string.h>  // memset, memcpy
#include "uarthandle.h"  // For forward_data_to_uart

static const char *TAG = "tcp_client";

uint8_t debug_mode = 0;
bool is_tcp_connected = false;
static int tcp_sock = -1;
static int udp_sock = -1;
static struct sockaddr_in dest_addr;  // Shared dest for TCP (UDP recv only)

// WiFi ready semaphore (signaled on IP got)
static SemaphoreHandle_t wifi_ready_sem = NULL;

// 共享 RX 缓冲（TCP/UDP 统一，10KB 支持 10KB/s RX）
#define SHARED_RX_BUF_SIZE TCP_RX_BUF_SIZE
static uint8_t shared_rx_buf[SHARED_RX_BUF_SIZE];
static volatile size_t shared_rx_head = 0;  // 读头
static volatile size_t shared_rx_tail = 0;  // 写尾
static volatile size_t shared_rx_count = 0;  // 已用字节

// UART -> Network 缓冲（通用，增大到 8KB 支持 50KB/s TX）
#define UART_TO_NET_BUF_SIZE 8192

static uint8_t uart_to_net_buf[UART_TO_NET_BUF_SIZE];
static volatile size_t uart_to_net_write_pos = 0;
static volatile size_t uart_to_net_read_pos = 0;
static volatile size_t uart_to_net_count = 0;

// 共享 RX 互斥
static SemaphoreHandle_t shared_rx_mutex = NULL;
static void shared_rx_init_mutex(void) {
    if (!shared_rx_mutex) shared_rx_mutex = xSemaphoreCreateMutex();
}

// 初始化 WiFi ready semaphore
static void wifi_ready_init(void) {
    if (!wifi_ready_sem) wifi_ready_sem = xSemaphoreCreateBinary();
}

// --- Global static TX scratch buffers (avoid allocating large arrays on task stacks) ---
static uint8_t g_tx_buf[4096];   // reused by uart_tcp_bridge_task and for force_send
// ---------------------------------------------------------------------------

// WiFi event handler (call from init_system or main)
void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
        if (wifi_ready_sem) xSemaphoreGive(wifi_ready_sem);  // Reset for reconnect trigger
        ESP_LOGI(TAG, "WiFi disconnected, attempting reconnect");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "WiFi Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        if (wifi_ready_sem) xSemaphoreGive(wifi_ready_sem);  // Signal TCP connect start
    }
}

// 初始化网络地址 (TCP only for send)
static void network_dest_init(void) {
    memset(&dest_addr, 0, sizeof(dest_addr));
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(PORT);
    dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
}

// TCP connect function (separate for reuse on reconnect)
static esp_err_t tcp_connect(void) {
    if (tcp_sock >= 0) {
        close(tcp_sock);
        tcp_sock = -1;
    }
    tcp_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (tcp_sock < 0) {
        ESP_LOGE(TAG, "Unable to create TCP socket: errno %d (%s)", errno, strerror(errno));
        return ESP_FAIL;
    }

    // Non-blocking connect
    fcntl(tcp_sock, F_SETFL, O_NONBLOCK);
    int err = connect(tcp_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err < 0 && errno != EINPROGRESS) {
        ESP_LOGE(TAG, "TCP socket connect error: %d (%s)", errno, strerror(errno));
        close(tcp_sock);
        tcp_sock = -1;
        return ESP_FAIL;
    }

    // Poll for connect completion
    fd_set writefds;
    FD_ZERO(&writefds);
    FD_SET(tcp_sock, &writefds);
    struct timeval conn_tv = { .tv_sec = TCP_CONNECT_TIMEOUT_MS / 1000, .tv_usec = (TCP_CONNECT_TIMEOUT_MS % 1000) * 1000 };
    int sel_res = select(tcp_sock + 1, NULL, &writefds, NULL, &conn_tv);
    if (sel_res > 0 && FD_ISSET(tcp_sock, &writefds)) {
        int optval;
        socklen_t optlen = sizeof(optval);
        getsockopt(tcp_sock, SOL_SOCKET, SO_ERROR, &optval, &optlen);
        if (optval == 0) {
            // Back to blocking mode
            int flags = fcntl(tcp_sock, F_GETFL, 0);
            flags &= ~O_NONBLOCK;
            fcntl(tcp_sock, F_SETFL, flags);

            is_tcp_connected = true;
            gpio_set_level(LED_PIN, 0); // on
            update_led_state();  // Assume defined elsewhere
            ESP_LOGI(TAG, "TCP connected to %s:%d (sock=%d)", HOST_IP_ADDR, PORT, tcp_sock);
            return ESP_OK;
        } else {
            ESP_LOGE(TAG, "TCP connect error optval=%d", optval);
            close(tcp_sock);
            tcp_sock = -1;
            return ESP_FAIL;
        }
    } else {
        ESP_LOGW(TAG, "TCP connect timeout");
        close(tcp_sock);
        tcp_sock = -1;
        return ESP_FAIL;
    }
}

static int s_retry_num = 0;
esp_err_t event_handler(void *ctx, system_event_t *event) {
    switch (event->event_id) {
        case SYSTEM_EVENT_STA_START:
            esp_wifi_connect();
            break;
        case SYSTEM_EVENT_STA_GOT_IP:
            ESP_LOGI(TAG, "got ip:%s", ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
            s_retry_num = 0;
            xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
            xEventGroupClearBits(s_wifi_event_group, WIFI_FAIL_BIT);
            update_led_state();
            if (wifi_ready_sem) xSemaphoreGive(wifi_ready_sem);  // 通知任务 WiFi 就绪
            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
            xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            update_led_state();
            esp_wifi_connect();
            s_retry_num++;
            if (s_retry_num >= EXAMPLE_ESP_MAXIMUM_RETRY) {
                ESP_LOGW(TAG, "Max WiFi retries reached, continuing to retry...");
            }
            if (wifi_ready_sem) xSemaphoreGive(wifi_ready_sem);  // 重置信号，允许重连重试
            is_tcp_connected = false;  // 重置 TCP 状态，避免旧连接干扰
            break;
        default:
            break;
    }
    return ESP_OK;
}

// 共享 RX 写
static esp_err_t shared_rx_write(const uint8_t *data, size_t len) {
    if (shared_rx_mutex) xSemaphoreTake(shared_rx_mutex, pdMS_TO_TICKS(10));
    size_t space = SHARED_RX_BUF_SIZE - shared_rx_count;
    if (len > space) {
        ESP_LOGW(TAG, "Shared RX overflow: drop %u bytes", (unsigned)(len - space));
        len = space;
    }
    size_t written = 0;
    while (written < len) {
        size_t to_write = len - written;
        if (shared_rx_tail + to_write > SHARED_RX_BUF_SIZE) to_write = SHARED_RX_BUF_SIZE - shared_rx_tail;
        memcpy(&shared_rx_buf[shared_rx_tail], &data[written], to_write);
        shared_rx_tail = (shared_rx_tail + to_write) % SHARED_RX_BUF_SIZE;
        written += to_write;
    }
    shared_rx_count += len;
    if (shared_rx_mutex) xSemaphoreGive(shared_rx_mutex);
    return ESP_OK;
}

// 共享 RX 读
static esp_err_t shared_rx_read(uint8_t *data, size_t *len) {
    if (shared_rx_mutex) xSemaphoreTake(shared_rx_mutex, pdMS_TO_TICKS(10));
    size_t avail = shared_rx_count;
    if (avail == 0) {
        *len = 0;
        if (shared_rx_mutex) xSemaphoreGive(shared_rx_mutex);
        return ESP_OK;
    }
    size_t to_read = (*len < avail) ? *len : avail;
    size_t read = 0;
    while (read < to_read) {
        size_t chunk = to_read - read;
        if (shared_rx_head + chunk > SHARED_RX_BUF_SIZE) chunk = SHARED_RX_BUF_SIZE - shared_rx_head;
        memcpy(&data[read], &shared_rx_buf[shared_rx_head], chunk);
        shared_rx_head = (shared_rx_head + chunk) % SHARED_RX_BUF_SIZE;
        read += chunk;
    }
    shared_rx_count -= to_read;
    *len = to_read;
    if (shared_rx_mutex) xSemaphoreGive(shared_rx_mutex);
    return ESP_OK;
}

// 共享 RX 批量 forward + parse (优化批量 4KB 支持 10KB/s)
static void shared_rx_forward_batch(void) {
    size_t avail = shared_rx_count;
    if (avail == 0) return;
    size_t batch = (avail > 4096) ? 4096 : avail;  // 增大批量到 4KB
    uint8_t batch_buf[4096];
    size_t actual = batch;
    if (shared_rx_read(batch_buf, &actual) == ESP_OK && actual > 0) {
        if (xSemaphoreTake(shared_rx_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            esp_err_t res = parse_command(batch_buf, actual);
            if (res != ESP_OK) forward_data_to_uart(batch_buf, actual);
            xSemaphoreGive(shared_rx_mutex);
            if (debug_mode) ESP_LOGD(TAG, "Forwarded %u bytes from shared RX to UART", (unsigned)actual);
        } else {
            forward_data_to_uart(batch_buf, actual);
        }
    }
}

// UART to Net 环形缓冲辅助函数：计算可用空间
static inline size_t uart_to_net_available_space(void) {
    return UART_TO_NET_BUF_SIZE - uart_to_net_count;
}

// 写入数据到环形缓冲
static esp_err_t uart_to_net_write(const uint8_t *data, size_t len) {
    if (len == 0 || !data) return ESP_OK;
    size_t space = uart_to_net_available_space();
    if (len > space) {
        ESP_LOGW(TAG, "UART to Net buffer overflow: %u > %u, dropping %u bytes", (unsigned)len, (unsigned)space, (unsigned)(len - space));
        len = space;  // 截断，避免覆盖
    }
    size_t written = 0;
    while (written < len) {
        size_t to_write = len - written;
        if (uart_to_net_write_pos + to_write > UART_TO_NET_BUF_SIZE) {
            to_write = UART_TO_NET_BUF_SIZE - uart_to_net_write_pos;  // 跨尾部
        }
        memcpy(&uart_to_net_buf[uart_to_net_write_pos], &data[written], to_write);
        uart_to_net_write_pos = (uart_to_net_write_pos + to_write) % UART_TO_NET_BUF_SIZE;
        written += to_write;
    }
    uart_to_net_count += len;
    return ESP_OK;
}

// 从环形缓冲读取数据 (用于发送)
static esp_err_t uart_to_net_read(uint8_t *data, size_t *len) {
    size_t avail = uart_to_net_count;
    if (avail == 0) {
        *len = 0;
        return ESP_OK;
    }
    size_t to_read = (*len < avail) ? *len : avail;
    size_t read = 0;
    while (read < to_read) {
        size_t chunk = to_read - read;
        if (uart_to_net_read_pos + chunk > UART_TO_NET_BUF_SIZE) {
            chunk = UART_TO_NET_BUF_SIZE - uart_to_net_read_pos;
        }
        memcpy(&data[read], &uart_to_net_buf[uart_to_net_read_pos], chunk);
        uart_to_net_read_pos = (uart_to_net_read_pos + chunk) % UART_TO_NET_BUF_SIZE;
        read += chunk;
    }
    uart_to_net_count -= to_read;
    *len = to_read;
    return ESP_OK;
}

// 清空缓冲
static void uart_to_net_clear(void) {
    uart_to_net_read_pos = 0;
    uart_to_net_write_pos = 0;
    uart_to_net_count = 0;
}

// ---- TCP send helper (robust, yields on EAGAIN, logs errors) ----
/* Optimized TCP send data with full payload transmission for non-blocking sockets */
esp_err_t tcp_send_data(int sock, const char *payload, size_t payload_len, char *rx_buffer, size_t rx_buffer_size, bool expect_response)
{
    if (payload_len == 0) {
        return ESP_OK;
    }

    // For sending UART data, ensure full transmission by looping in non-blocking mode
    size_t total_sent = 0;
    struct timeval send_timeout;
    send_timeout.tv_sec = 0;
    send_timeout.tv_usec = SEND_RETRY_TIMEOUT_MS * 1000;  // Short timeout per send attempt
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &send_timeout, sizeof(send_timeout));

    while (total_sent < payload_len) {
        int to_send = payload_len - total_sent;
        int sent = send(sock, payload + total_sent, to_send, 0);
        if (sent < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // Buffer full, yield briefly and retry
                vTaskDelay(1 / portTICK_PERIOD_MS);
                continue;
            }
            ESP_LOGE(TAG, "Error during sending: errno %d", errno);
            return ESP_FAIL;
        } else if (sent == 0) {
            // Connection closed
            ESP_LOGE(TAG, "Connection closed during send");
            return ESP_FAIL;
        }

        total_sent += sent;
        if (debug_mode && total_sent < payload_len) {
            ESP_LOGD(TAG, "Partial send: %d/%zu bytes sent, retrying...", total_sent, payload_len);
        }
    }

    if (debug_mode) {
        ESP_LOGD(TAG, "Fully sent %zu bytes over TCP", payload_len);
    }

    if (!expect_response) {
        return ESP_OK;
    }

    // For responses, use original logic
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

    forward_data_to_uart((uint8_t *)rx_buffer, len);
    return ESP_OK;
}

static uint8_t recv_buf[4096];  // 增大到 4KB 支持 10KB/s burst
void network_receive_task(void *pvParameters) {

    shared_rx_init_mutex();
    wifi_ready_init();
    network_dest_init();
    uart_to_net_clear();  // Ensure clean start

    ESP_LOGI(TAG, "network_receive_task started (TCP:%d UDP:%d, shared buf:%d bytes)", PORT, UDP_PORT, SHARED_RX_BUF_SIZE);

    // Wait for WiFi IP before TCP connect
    ESP_LOGI(TAG, "Waiting for WiFi connection...");
    while (xSemaphoreTake(wifi_ready_sem, pdMS_TO_TICKS(5000)) != pdTRUE) {
        ESP_LOGW(TAG, "WiFi not ready, retrying...");
        esp_task_wdt_reset();
    }

    // Initial TCP connect after WiFi ready
    uint32_t retry_delay = 100;  // Start with 100ms for fast initial reconnect
    while (1) {
        if (!is_tcp_connected) {
            esp_err_t connect_res = tcp_connect();
            if (connect_res == ESP_OK) {
                retry_delay = 100;  // Reset delay on success
            } else {
                ESP_LOGW(TAG, "TCP connect failed, retry in %u ms", retry_delay);
                vTaskDelay(pdMS_TO_TICKS(retry_delay));
                retry_delay = (retry_delay * 2 > 2000) ? 2000 : retry_delay * 2;  // Exponential backoff to 2s
                continue;
            }
        }

        // Init UDP sock (bind for recv only) - can be after WiFi too
        if (udp_sock < 0) {
            udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
            if (udp_sock < 0) {
                ESP_LOGE(TAG, "Unable to create UDP socket: errno %d (%s)", errno, strerror(errno));
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }
            struct sockaddr_in bind_addr;
            memset(&bind_addr, 0, sizeof(bind_addr));
            bind_addr.sin_family = AF_INET;
            bind_addr.sin_port = htons(UDP_PORT);
            bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
            if (bind(udp_sock, (struct sockaddr*)&bind_addr, sizeof(bind_addr)) < 0) {
                ESP_LOGE(TAG, "UDP bind failed: errno %d (%s)", errno, strerror(errno));
                close(udp_sock);
                udp_sock = -1;
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }
            struct timeval udp_timeout = { .tv_sec = 0, .tv_usec = 10000 };  // 10ms RX timeout
            setsockopt(udp_sock, SOL_SOCKET, SO_RCVTIMEO, &udp_timeout, sizeof(udp_timeout));
            ESP_LOGI(TAG, "UDP socket bound to port %d", UDP_PORT);
        }

        // Main poll loop
        int max_fd = -1;
        fd_set readfds;
        FD_ZERO(&readfds);
        if (tcp_sock >= 0 && is_tcp_connected) {
            FD_SET(tcp_sock, &readfds);
            max_fd = tcp_sock;
        }
        if (udp_sock >= 0) {
            FD_SET(udp_sock, &readfds);
            if (udp_sock > max_fd) max_fd = udp_sock;
        }
        if (max_fd < 0) {
            vTaskDelay(pdMS_TO_TICKS(1));
            esp_task_wdt_reset();
            continue;
        }

        struct timeval poll_tv = { .tv_sec = 0, .tv_usec = 5000 };  // 5ms poll 高响应
        int sel_res = select(max_fd + 1, &readfds, NULL, NULL, &poll_tv);
        if (sel_res < 0) {
            ESP_LOGE(TAG, "Select failed: errno %d (%s)", errno, strerror(errno));
            vTaskDelay(pdMS_TO_TICKS(10));
            esp_task_wdt_reset();
            continue;
        } else if (sel_res == 0) {
            // Timeout, check TCP keepalive
            if (is_tcp_connected && tcp_sock >= 0) {
                static TickType_t last_keepalive = 0;
                if (xTaskGetTickCount() - last_keepalive > pdMS_TO_TICKS(TCP_KEEPALIVE_MS)) {
                    // It's better to use OS-level keepalive, but send a small probe if desired
                    char probe = 0x00;
                    send(tcp_sock, &probe, 0, 0);  // 0-byte send for keepalive (best-effort)
                    last_keepalive = xTaskGetTickCount();
                }
            }
            esp_task_wdt_reset();
            continue;
        }

        // UDP ready? Handle and write to shared buf
        if (udp_sock >= 0 && FD_ISSET(udp_sock, &readfds)) {
            int len = recv(udp_sock, recv_buf, sizeof(recv_buf) - 1, 0);
            if (len > 0) {
                shared_rx_write(recv_buf, (size_t)len);
                if (debug_mode && len <= 200) {
                    char *hex_buf = (char *)malloc(len * 3 + 1);
                    if (hex_buf) {
                        char *ptr = hex_buf;
                        for (int i = 0; i < len; i++) {
                            ptr += sprintf(ptr, "%02X ", recv_buf[i]);
                        }
                        *ptr = 0;
                        ESP_LOGI(TAG, "Raw UDP RX (hex): %s (len: %d)", hex_buf, len);
                        free(hex_buf);
                    }
                } else if (debug_mode) {
                    ESP_LOGI(TAG, "UDP RX: %d bytes", len);
                }
            } else if (len < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
                ESP_LOGE(TAG, "UDP recv error: errno %d (%s)", errno, strerror(errno));
            }
        }

        // TCP ready? Handle and write to shared buf
        if (tcp_sock >= 0 && FD_ISSET(tcp_sock, &readfds)) {
            int len = recv(tcp_sock, recv_buf, sizeof(recv_buf) - 1, 0);
            if (len > 0) {
                shared_rx_write(recv_buf, (size_t)len);
                if (debug_mode && len <= 200) {
                    char *hex_buf = (char *)malloc(len * 3 + 1);
                    if (hex_buf) {
                        char *ptr = hex_buf;
                        for (int i = 0; i < len; i++) {
                            ptr += sprintf(ptr, "%02X ", recv_buf[i]);
                        }
                        *ptr = 0;
                        ESP_LOGI(TAG, "Raw TCP RX (hex): %s (len: %d)", hex_buf, len);
                        free(hex_buf);
                    }
                } else if (debug_mode) {
                    ESP_LOGI(TAG, "TCP RX: %d bytes", len);
                }
            } else if (len == 0) {
                ESP_LOGW(TAG, "TCP connection closed by server");
                is_tcp_connected = false;
                update_led_state();
                close(tcp_sock);
                tcp_sock = -1;
                // Continue to reconnect loop
            } else if (len < 0) {
                if (errno != EAGAIN && errno != EWOULDBLOCK) {
                    ESP_LOGE(TAG, "TCP recv error: errno %d (%s)", errno, strerror(errno));
                    is_tcp_connected = false;
                    update_led_state();
                    close(tcp_sock);
                    tcp_sock = -1;
                }
            }
        }

        // Unified forward from shared RX
        shared_rx_forward_batch();
        esp_task_wdt_reset();

        #if 0
        // Stack/Heap monitoring
        static TickType_t last_monitor = 0;
        if (xTaskGetTickCount() - last_monitor > pdMS_TO_TICKS(10000)) {
            UBaseType_t water_mark = uxTaskGetStackHighWaterMark(NULL);
            ESP_LOGI(TAG, "Network RX task stack high water mark: %u bytes free", (unsigned)water_mark);
            if (water_mark < 2048) {
                ESP_LOGW(TAG, "Low stack in Network RX");
            }
            ESP_LOGI(TAG, "Free heap in Network RX: %u bytes", (unsigned)esp_get_free_heap_size());
            last_monitor = xTaskGetTickCount();
        }
        #endif
        vTaskDelay(pdMS_TO_TICKS(20));  // Minimal delay for high throughput (1ms)
    }

    if (tcp_sock >= 0) close(tcp_sock);
    if (udp_sock >= 0) close(udp_sock);
    vTaskDelete(NULL);
}

static uint8_t uart_local_buf[1024];  // shrink local buffer to reduce stack usage (use g_tx_buf for large chunks)
void uart_tcp_bridge_task(void *pvParameters) {

    uart_to_net_clear();

    ESP_LOGI(TAG, "UART to TCP bridge task started (buf: %u bytes, target ~50KB/s)", (unsigned)UART_TO_NET_BUF_SIZE);

    while (1) {
        // 从 UART 读取数据 (短超时 5ms，高吞吐)
        int len = uart_read_bytes(UART_NUM_0, uart_local_buf, sizeof(uart_local_buf), pdMS_TO_TICKS(5));
        if (len > 0) {
            // 立即写入环形缓冲
            uart_to_net_write(uart_local_buf, (size_t)len);

            // 如果缓冲有数据，批量发送 via TCP only
            if (uart_to_net_count > 0 && is_tcp_connected) {
                size_t to_send = (uart_to_net_count > sizeof(g_tx_buf)) ? sizeof(g_tx_buf) : uart_to_net_count;

                size_t actual_sent = to_send;
                esp_err_t err = uart_to_net_read(g_tx_buf, &actual_sent);
                if (err == ESP_OK && actual_sent > 0) {
                    if (!is_tcp_connected || tcp_sock < 0) {
                        ESP_LOGW(TAG, "Skipping send because TCP not connected (sock=%d, connected=%d)", tcp_sock, is_tcp_connected ? 1 : 0);
                    } else {
                        esp_err_t send_err = tcp_send_data(tcp_sock, (const char *)g_tx_buf, actual_sent, NULL, 0, false);
                        if (send_err != ESP_OK) {
                            ESP_LOGW(TAG, "tcp_send_data returned error %d for %u bytes", (int)send_err, (unsigned)actual_sent);
                        } else if (debug_mode) {
                            ESP_LOGD(TAG, "Bridged %u bytes UART -> TCP", (unsigned)actual_sent);
                        }
                    }
                } else {
                    ESP_LOGW(TAG, "Failed to read %u bytes from UART buffer", (unsigned)to_send);
                }
            }

            // 如果缓冲接近满 (>90%)，强制清空 via TCP
            if (uart_to_net_count > (UART_TO_NET_BUF_SIZE * 9 / 10) && is_tcp_connected) {
                size_t force_send = uart_to_net_count;
                size_t actual = (force_send > sizeof(g_tx_buf)) ? sizeof(g_tx_buf) : force_send;
                while (actual > 0) {
                    uart_to_net_read(g_tx_buf, &actual);

                    ESP_LOGI(TAG, "Force sending %u bytes (sock=%d)", (unsigned)actual, tcp_sock);
                    tcp_send_data(tcp_sock, (const char *)g_tx_buf, actual, NULL, 0, false);

                    force_send = uart_to_net_count;  // Re-check
                    actual = (force_send > sizeof(g_tx_buf)) ? sizeof(g_tx_buf) : force_send;
                }
            }
        } else if (len < 0) {
            ESP_LOGW(TAG, "UART read error");
        }

        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(10));  // 1ms loop for high throughput

        #if 0
        // Stack/Heap monitoring
        static TickType_t last_monitor = 0;
        if (xTaskGetTickCount() - last_monitor > pdMS_TO_TICKS(10000)) {
            UBaseType_t water_mark = uxTaskGetStackHighWaterMark(NULL);
            ESP_LOGI(TAG, "UART bridge stack high water mark: %u bytes free", (unsigned)water_mark);
            if (water_mark < 2048) {
                ESP_LOGW(TAG, "Low stack in UART bridge");
            }
            last_monitor = xTaskGetTickCount();
        }
        #endif
    }

    uart_to_net_clear();
    vTaskDelete(NULL);
}

void app_main(void) {
    init_system();

    BaseType_t ret = xTaskCreate(led_task, "led_task", 3072, NULL, 7, NULL);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "FAILED to create led_task");
    } else {
        ESP_LOGI(TAG, "led_task created OK");
    }

    ret = xTaskCreate(uart_tcp_bridge_task, "uart_tcp_bridge", 8192, NULL, 8, NULL);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "FAILED uart_tcp_bridge");
    } else {
        ESP_LOGI(TAG, "uart_tcp_bridge created");
    }

    ret = xTaskCreate(network_receive_task, "network_receive", 8192, NULL, 8, NULL);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "FAILED network_receive");
    } else {
        ESP_LOGI(TAG, "network_receive created");
    }
    esp_task_wdt_reset();
}
