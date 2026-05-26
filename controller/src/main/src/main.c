/*
 * SkyWatch controller main.
 *
 * Stage: 2.1
 * Purpose: Bring up dual CDC-ACM USB (handled by usb_device.c at SYS_INIT)
 *           and confirm the data port via usb_serial_init(). After that the
 *           shell runs on ACM0 and main() simply heartbeats so we can see
 *           the board hasn't hung.
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include "usb_serial.h"
#include "usb_handler.h"
#include "ble_central.h"
#include "bridge.h"
#include "aircraft_db.h"
#include "db_publisher.h"
#include "collision.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/* Heartbeat LED — green LED on the Xiao-BLE (led1 = GPIO0_30, ACTIVE_LOW).
 * Pattern mirrors the sim mobile node (sim/mobile/src/main/src/codein.c):
 * GPIO_DT_SPEC_GET + configure_dt + set_dt. The red LED comes up briefly
 * during init then drops out; green stays solid once init has completed
 * and toggles slowly in the heartbeat loop so a frozen board (LED stuck
 * on or off) is obvious at a glance. */
#define LED_RED_NODE DT_ALIAS(led0)
#define LED_GREEN_NODE DT_ALIAS(led1)
static const struct gpio_dt_spec led_red   = GPIO_DT_SPEC_GET(LED_RED_NODE,   gpios);
static const struct gpio_dt_spec led_green = GPIO_DT_SPEC_GET(LED_GREEN_NODE, gpios);

int main(void)
{
	/* LEDs first so we can signal init progress visually. */
	if (gpio_is_ready_dt(&led_red))   gpio_pin_configure_dt(&led_red,   GPIO_OUTPUT_INACTIVE);
	if (gpio_is_ready_dt(&led_green)) gpio_pin_configure_dt(&led_green, GPIO_OUTPUT_INACTIVE);
	gpio_pin_set_dt(&led_red,   1);   /* red = initialising */
	gpio_pin_set_dt(&led_green, 0);

	aircraft_db_init();   /* zero-init pool + slist; reaper auto-starts via SYS_INIT */

	int err = usb_serial_init();
	if (err) {
		LOG_ERR("usb_serial_init failed: %d", err);
	}

	err = usb_handler_init();
	if (err) {
		LOG_ERR("usb_handler_init failed: %d", err);
	}

	err = bridge_init();
	if (err) {
		LOG_ERR("bridge_init failed: %d", err);
	}

	err = ble_central_init();
	if (err) {
		LOG_ERR("ble_central_init failed: %d", err);
	}

	err = db_publisher_init();
	if (err) {
		LOG_ERR("db_publisher_init failed: %d", err);
	}

	err = collision_init();
	if (err) {
		LOG_ERR("collision_init failed: %d", err);
	}

	/* Init complete — green steady = system healthy. */
	gpio_pin_set_dt(&led_red,   0);
	gpio_pin_set_dt(&led_green, 1);

	LOG_INF("SkyWatch controller up — shell on ACM0, data port on ACM1, BLE central scanning");

	while (1) {
		k_sleep(K_SECONDS(5));
		LOG_INF("heartbeat");
		gpio_pin_toggle_dt(&led_green);  /* slow blink → board still alive */
	}

	return 0;
}
