/*
 * SkyWatch controller main — Phase 2.1.
 *
 * Stage:    2.1
 * Purpose:  Bring up dual CDC-ACM USB (handled by usb_device.c at SYS_INIT)
 *           and confirm the data port via usb_serial_init(). After that the
 *           shell runs on ACM0 and main() simply heartbeats so we can see
 *           the board hasn't hung.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "usb_serial.h"
#include "usb_handler.h"
#include "ble_central.h"
#include "bridge.h"
#include "aircraft_db.h"
#include "db_publisher.h"
#include "collision.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

int main(void)
{
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

	LOG_INF("SkyWatch controller up — shell on ACM0, data port on ACM1, BLE central scanning");

	while (1) {
		k_sleep(K_SECONDS(5));
		LOG_INF("heartbeat");
	}

	return 0;
}
