#ifndef INIT_SYS_H
#define INIT_SYS_H

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdarg.h>
#include <fcntl.h>
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
#include <regex.h>
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include "tcp_client.h"
#include "uarthandle.h"
#include "gpiohandle.h"

#define DEFAULT_ESP_WIFI_SSID      "FUNLIGHT"
#define DEFAULT_ESP_WIFI_PASS      "funlight"
#define EXAMPLE_ESP_MAXIMUM_RETRY  10
#define NVS_NAMESPACE              "config"
#define EXAMPLE_ESP_MAXIMUM_RETRY  10

EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1


void init_system(void);
bool is_wifi_connected(void);
void update_led_state(void);
void wifi_init_sta(const char *ssid, const char *password);
void handle_boot_count(void);
esp_err_t load_nvs_config(uint32_t *baud_rate, uint8_t *stop_bits, uint8_t *parity, uint8_t *data_bits, char *ssid, size_t ssid_len, char *password, size_t pass_len, uint8_t *debug);
void authorcodeverify(void);
void save_uart_config(uint32_t baud_rate, uint8_t stop_bits, uint8_t parity, uint8_t data_bits);
void save_wifi_config(const char *ssid, const char *password);
esp_err_t parse_command(const uint8_t *rx_buffer, size_t len);
#endif // INIT_SYS_H
