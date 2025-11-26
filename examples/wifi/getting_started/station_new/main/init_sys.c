#include "init_sys.h"

static const char *TAG = "init_sys";

EventGroupHandle_t s_wifi_event_group;

static uint32_t compute_crc32(const uint8_t *data, size_t len) {
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

void authorcodeverify(void) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS for auth: %s", esp_err_to_name(err));
        while (1) {
            gpio_set_level(LED_PIN, 0); // on (active-low)
            vTaskDelay(pdMS_TO_TICKS(1000));
            gpio_set_level(LED_PIN, 1); // off
            vTaskDelay(pdMS_TO_TICKS(3000));
        }
    }

    uint8_t mac[6];
    err = esp_wifi_get_mac(ESP_IF_WIFI_STA, mac);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get MAC: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        while (1) {
            gpio_set_level(LED_PIN, 0); // on
            vTaskDelay(pdMS_TO_TICKS(1000));
            gpio_set_level(LED_PIN, 1); // off
            vTaskDelay(pdMS_TO_TICKS(3000));
        }
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
            while (1) {
                gpio_set_level(LED_PIN, 0); // on
                vTaskDelay(pdMS_TO_TICKS(1000));
                gpio_set_level(LED_PIN, 1); // off
                vTaskDelay(pdMS_TO_TICKS(3000));
            }
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
        while (1) {
            gpio_set_level(LED_PIN, 0); // on
            vTaskDelay(pdMS_TO_TICKS(1000));
            gpio_set_level(LED_PIN, 1); // off
            vTaskDelay(pdMS_TO_TICKS(3000));
        }
    }

    // Verify
    if (computed_code != stored_code) {
        ESP_LOGE(TAG, "Auth verification failed. Computed: 0x%08x, Stored: 0x%08x", computed_code, stored_code);
        nvs_close(nvs_handle);
        while (1) {
            gpio_set_level(LED_PIN, 0); // on
            vTaskDelay(pdMS_TO_TICKS(1000));
            gpio_set_level(LED_PIN, 1); // off
            vTaskDelay(pdMS_TO_TICKS(3000));
        }
    }

    ESP_LOGI(TAG, "Auth verification passed");
    nvs_close(nvs_handle);
}

esp_err_t load_nvs_config(uint32_t *baud_rate, uint8_t *stop_bits, uint8_t *parity, uint8_t *data_bits, char *ssid, size_t ssid_len, char *password, size_t pass_len, uint8_t *debug) {
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

void save_uart_config(uint32_t baud_rate, uint8_t stop_bits, uint8_t parity, uint8_t data_bits) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS for saving config: %s", esp_err_to_name(err));
        return;
    }

    nvs_set_u32(nvs_handle, "baud_rate", baud_rate);
    nvs_set_u8(nvs_handle, "stop_bits", stop_bits);
    nvs_set_u8(nvs_handle, "parity", parity);
    nvs_set_u8(nvs_handle, "data_bits", data_bits);

    err = nvs_commit(nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to commit UART config: %s", esp_err_to_name(err));
    }
    nvs_close(nvs_handle);
}

void save_wifi_config(const char *ssid, const char *password) {
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

    err = nvs_commit(nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to commit WIFI config: %s", esp_err_to_name(err));
    }
    nvs_close(nvs_handle);
}

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

void handle_boot_count(void)
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
        err = nvs_erase_all(nvs_handle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to erase NVS: %s", esp_err_to_name(err));
            nvs_close(nvs_handle);
            return;
        }
        err = nvs_commit(nvs_handle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to commit NVS erase: %s", esp_err_to_name(err));
            nvs_close(nvs_handle);
            return;
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
        ESP_LOGI(TAG, "Factory reset completed, restarting...");
        vTaskDelay(pdMS_TO_TICKS(1000));
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

void update_led_state(void) {
    if (!is_wifi_connected()) {
        current_led_state = LED_SLOW_BLINK;
    } else if (!is_tcp_connected) {  // WiFi Á¬ + TCP Î´Á¬
        current_led_state = LED_FAST_BLINK;
    } else {
        current_led_state = LED_SOLID_ON;
    }
    if (debug_mode) {
        ESP_LOGI(TAG, "LED state updated to: %d (WiFi: %s, TCP: %s)", 
                 current_led_state, 
                 is_wifi_connected() ? "connected" : "disconnected",
                 is_tcp_connected ? "connected" : "disconnected");
    }
}



void wifi_init_sta(const char *ssid, const char *password) {
    // FIXED: 创建事件组现在会正确共享给其他文件。
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

    // OPTIONAL: 显式启动 DHCP（自动的，但此调用确保在边缘情况下生效；参考代码无此，但不冲突）。
    err = tcpip_adapter_dhcpc_start(ESP_IF_WIFI_STA);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "DHCP client start failed: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "DHCP client started (auto-redundant)");
    }

    err = esp_wifi_set_max_tx_power(82);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Set WiFi TX power failed: %s", esp_err_to_name(err));
    }
}

bool is_wifi_connected(void) {
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, 100 / portTICK_PERIOD_MS);
    return (bits & WIFI_CONNECTED_BIT) != 0;
}

void init_system(void) {
    uint32_t current_baud_rate;
    uint8_t current_stop_bits, current_parity, current_data_bits;
    char ssid[32] = {0};
    char password[64] = {0};

    esp_err_t err = nvs_flash_init();
    if (err != ESP_OK) {
        nvs_flash_erase();
        err = nvs_flash_init();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "NVS flash init failed: %s", esp_err_to_name(err));
            return;
        }
    }
    esp_task_wdt_init();

    err = load_nvs_config(&current_baud_rate, &current_stop_bits, &current_parity, &current_data_bits, ssid, sizeof(ssid), password, sizeof(password), &debug_mode);
    if (err != ESP_OK) {
        strlcpy(ssid, DEFAULT_ESP_WIFI_SSID, sizeof(ssid));
        strlcpy(password, DEFAULT_ESP_WIFI_PASS, sizeof(password));
        current_baud_rate = 115200;
        current_stop_bits = UART_STOP_BITS_1;
        current_parity = UART_PARITY_DISABLE;
        current_data_bits = UART_DATA_8_BITS;
    }
    handle_boot_count();
    led_init();

    wifi_init_sta(ssid, password);
    authorcodeverify();
    uart_init(current_baud_rate, current_stop_bits, current_parity, current_data_bits);
}


esp_err_t parse_command(const uint8_t *rx_buffer, size_t len)
{
    if (!rx_buffer || len == 0) {
        return ESP_FAIL;
    }

    bool is_likely_command = (memchr(rx_buffer, '\r', len) != NULL || memchr(rx_buffer, '\n', len) != NULL);
    if (!is_likely_command || len > 128) {
        forward_data_to_uart(rx_buffer, len);
        return ESP_OK;
    }

    // Check if potential BAUD command
    if (len >= 5 && memcmp(rx_buffer, "BAUD=", 5) == 0) {
        // Check for embedded 0x00; if present, treat as transparent binary data
        if (memchr(rx_buffer, 0, len) != NULL) {
            forward_data_to_uart(rx_buffer, len);
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
            free(buf);
            regfree(&regex);
            return ESP_FAIL;
        }

        ret = regexec(&regex, buf, 5, matches, 0);
        if (ret != 0) {
            ESP_LOGE(TAG, "Invalid BAUD command format: no match for '%s'", buf);
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
            free(buf);
            regfree(&regex);
            return ESP_FAIL;
        }
        strncpy(baud_str, buf + matches[1].rm_so, baud_len);
        uint32_t new_baud = strtoul(baud_str, &endptr, 10);
        if (*endptr != 0) {
            ESP_LOGE(TAG, "Invalid BAUD value: non-numeric '%s'", baud_str);
            free(buf);
            regfree(&regex);
            return ESP_FAIL;
        }

        // STOPBIT
        char stopbit_str[32] = {0};
        size_t stopbit_len = matches[2].rm_eo - matches[2].rm_so;
        if (stopbit_len >= sizeof(stopbit_str)) {
            ESP_LOGE(TAG, "STOPBIT value too long");
            free(buf);
            regfree(&regex);
            return ESP_FAIL;
        }
        strncpy(stopbit_str, buf + matches[2].rm_so, stopbit_len);
        uint8_t raw_stop_bits = strtoul(stopbit_str, &endptr, 10);
        if (*endptr != 0) {
            ESP_LOGE(TAG, "Invalid STOPBIT value: non-numeric '%s'", stopbit_str);
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
   
            free(buf);
            regfree(&regex);
            return ESP_FAIL;
        }
        strncpy(parity_str, buf + matches[3].rm_so, parity_len);
        uint8_t new_parity = strtoul(parity_str, &endptr, 10);
        if (*endptr != 0) {
            ESP_LOGE(TAG, "Invalid PARITY value: non-numeric '%s'", parity_str);
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
            free(buf);
            regfree(&regex);
            return ESP_FAIL;
        }
        strncpy(databit_str, buf + matches[4].rm_so, databit_len);
        uint8_t raw_data_bits = strtoul(databit_str, &endptr, 10);
        if (*endptr != 0) {
            ESP_LOGE(TAG, "Invalid DATABIT value: non-numeric '%s'", databit_str);
            free(buf);
            regfree(&regex);
            return ESP_FAIL;
        }
        regfree(&regex);
        free(buf);

        // Validate parameters
        if (new_baud < 110 || new_baud > 2000000) {
            ESP_LOGE(TAG, "Invalid baud rate: %u", new_baud);
            return ESP_FAIL;
        }

        if (new_stop_bits != UART_STOP_BITS_1 && new_stop_bits != UART_STOP_BITS_2) {
            ESP_LOGE(TAG, "Invalid stop bits: %u (raw value: %u)", new_stop_bits, raw_stop_bits);
            return ESP_FAIL;
        }
        if (new_parity != UART_PARITY_DISABLE && new_parity != UART_PARITY_ODD && new_parity != UART_PARITY_EVEN) {
            ESP_LOGE(TAG, "Invalid parity: %u", new_parity);
            return ESP_FAIL;
        }

        // raw_data_bits is from strtoul (unsigned), so check range before subtracting
        if (raw_data_bits < 5 || raw_data_bits > 8) {
            ESP_LOGE(TAG, "Invalid data bits: %u", raw_data_bits);
            return ESP_FAIL;
        }
        uint8_t new_data_bits = raw_data_bits - 5;

        // Apply UART configuration
        uart_set_baudrate(UART_NUM_0, new_baud);
        uart_set_stop_bits(UART_NUM_0, new_stop_bits);
        uart_set_parity(UART_NUM_0, new_parity);
        uart_set_word_length(UART_NUM_0, new_data_bits);

        save_uart_config(new_baud, new_stop_bits, new_parity, new_data_bits);
        return ESP_OK;
    } else if (len >= 5 && memcmp(rx_buffer, "WIFI=", 5) == 0) {
        // Similar fix for WIFI command: check for 0x00
        if (memchr(rx_buffer, 0, len) != NULL) {
            forward_data_to_uart(rx_buffer, len);
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

                free(buf);
                return ESP_FAIL;
            } else if (strlen(password) > 0 && (strlen(password) < 8 || strlen(password) > 64 || !isprint((unsigned char)password[0]))) {
                ESP_LOGE(TAG, "Invalid password from server");

                free(buf);
                return ESP_FAIL;
            } else {
                save_wifi_config(ssid, password);
                ESP_LOGI(TAG, "WiFi configuration from server successful: SSID=%s", ssid);
                free(buf);
                esp_restart();
                return ESP_OK;
            }
        } else {
            ESP_LOGE(TAG, "Invalid WIFI command format from server");

            free(buf);
            return ESP_FAIL;
        }
    } else {
        forward_data_to_uart(rx_buffer, len);
        return ESP_OK;
    }
}
