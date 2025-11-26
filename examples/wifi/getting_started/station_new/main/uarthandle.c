#include "uarthandle.h"

static const char *TAG = "uarthandle";

void uart_init(uint32_t baud_rate, uint8_t stop_bits, uint8_t parity, uint8_t data_bits) {
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
    // 增大缓冲：RX 4KB (高波特率读)，TX 8KB (批量写)
    err = uart_driver_install(UART_NUM_0, 4096, 8192, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART driver install failed: %s", esp_err_to_name(err));
    }
    ESP_LOGI(TAG, "UART init at %u baud, RX:4KB TX:8KB", baud_rate);

    // uart_enable_swap();
}

/* Print raw data (binary-safe) to UART0 */
static SemaphoreHandle_t raw_log_mutex = NULL;

static void raw_log_init(void) {
    if (!raw_log_mutex) {
        raw_log_mutex = xSemaphoreCreateMutex();
        if (!raw_log_mutex) {
            ESP_LOGE(TAG, "Failed to create raw log mutex");
        }
    }
}

void forward_data_to_uart(const uint8_t *data, size_t len) {
    if (!data || len == 0) return;

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
        xSemaphoreGive(raw_log_mutex);
    } else {
        uart_write_bytes(UART_NUM_0, (const char *)data, len);
    }

    esp_task_wdt_reset();
}