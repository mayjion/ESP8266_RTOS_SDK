#ifndef GPIOHANDLE_H
#define GPIOHANDLE_H

#include "init_sys.h"

typedef enum {
    LED_SLOW_BLINK,
    LED_FAST_BLINK,
    LED_SOLID_ON
} led_state_t;

#define BOOT_COUNT_KEY             "boot_count"
#define BOOT_COUNT_THRESHOLD       10
#define BOOT_COUNT_RESET_DELAY_MS  2000

#define LED_PIN GPIO_NUM_2

extern volatile led_state_t current_led_state;

void led_init(void);
void led_task(void *pvParameters);  // FIX: No change needed, but ensures non-static linkage

#endif // GPIOHANDLE_H