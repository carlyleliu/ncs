#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

#include <zephyr/drivers/gpio.h>
#include <app/drivers/blink.h>
#include <app_version.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

#define LED_THREAD_STACK_SIZE 500
#define LED_THREAD_PRIORITY 5
/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
int led_thread_entry_point(void *, void *, void *);

LOG_MODULE_REGISTER(led_thread, LOG_LEVEL_INF);

K_THREAD_DEFINE(led_tid, LED_THREAD_STACK_SIZE,
                led_thread_entry_point, NULL, NULL, NULL,
                LED_THREAD_PRIORITY, 0, 0);

int led_thread_entry_point(void* unused1, void* unused2, void* unused3)
{
	int ret;
	bool led_state = true;

	if (!gpio_is_ready_dt(&led)) {
		return 0;
	}

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return 0;
	}

	while (1) {
		ret = gpio_pin_toggle_dt(&led);
		if (ret < 0) {
			return 0;
		}

		led_state = !led_state;
		LOG_DBG("LED state: %s", led_state ? "ON" : "OFF");
		k_msleep(SLEEP_TIME_MS);
	}
	return 0;
}

