// tcp_client.h
#ifndef TCP_CLIENT_H
#define TCP_CLIENT_H

#include "init_sys.h"

#define PORT 12345
#define UDP_PORT PORT
#define CONFIG_EXAMPLE_IPV4 1
#define HOST_IP_ADDR "192.168.4.1"

#define TCP_RETRY_DELAY_MS 2000
#define TCP_CONNECT_TIMEOUT_MS 10000
#define TCP_KEEPALIVE_MS 10000
#define RESPONSE_TIMEOUT_MS 5000
#define SEND_RETRY_TIMEOUT_MS 50  // Short timeout for send retries in non-blocking mode

#define TCP_RX_BUF_SIZE 10240
#define UART_TO_TCP_BUF_SIZE 4096

extern uint8_t debug_mode;
extern bool is_tcp_connected;

esp_err_t tcp_send_data(int sock, const char *payload, size_t payload_len, char *rx_buffer, size_t rx_buffer_size, bool expect_response);
void tcp_receive_task(void *pvParameters);
void uart_tcp_bridge_task(void *pvParameters);
void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
esp_err_t event_handler(void *ctx, system_event_t *event);
#endif // TCP_CLIENT_H
