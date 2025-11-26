#include "gpiohandle.h"

static const char *TAG = "gpiohandle";
volatile led_state_t current_led_state = LED_SLOW_BLINK;

void led_init(void)
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

void led_task(void *pvParameters)
{
    // FIX: Add startup log to confirm task execution
    ESP_LOGI("led_task", "LED task STARTED - entering loop");

    // char led_status[20];
    while (1) {
        switch (current_led_state) {
            case LED_SLOW_BLINK:
                gpio_set_level(LED_PIN, 0); // on
                vTaskDelay(500 / portTICK_PERIOD_MS);
                gpio_set_level(LED_PIN, 1); // off
                vTaskDelay(1500 / portTICK_PERIOD_MS);
                break;
            case LED_FAST_BLINK:
                gpio_set_level(LED_PIN, 0); // on
                vTaskDelay(200 / portTICK_PERIOD_MS);
                gpio_set_level(LED_PIN, 1); // off
                vTaskDelay(300 / portTICK_PERIOD_MS);
                break;
            case LED_SOLID_ON:
                gpio_set_level(LED_PIN, 0); // on
                vTaskDelay(100 / portTICK_PERIOD_MS); // FIX: Reduced to 100ms for faster state checks without high CPU
                break;
        }
        // sprintf(led_status, "current_led_state=%d", current_led_state);
        // // FIX: Cast to uint8_t* (original &led_status was char** mismatch)
        // forward_data_to_uart((uint8_t *)led_status, strlen(led_status));
        esp_task_wdt_reset();
    }
    vTaskDelete(NULL);
}
