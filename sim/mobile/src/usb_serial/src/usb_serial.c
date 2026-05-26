/*
 * SkyWatch sim mobile — USB-CDC TX helper.
 *
 * Wraps `uart_poll_out` over the single CDC ACM port (host-side
 * /dev/ttyACM0). Position JSON and WARN text lines from `codein.c`
 * go through here; the Zephyr shell + log backend share the same
 * port. Multiple producers can call usb_serial_println() — calls
 * are short and the underlying poll-out path is mutex-free on a
 * single core, so we don't add our own lock here.
 */
#include "usb_serial.h"

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(usb_serial, LOG_LEVEL_INF);

/* Single CDC ACM instance defined in app.overlay (cdc_acm_uart0).
 * Position JSON + WARN text lines are written here directly via
 * uart_poll_out so they don't fight the log backend for buffer
 * space. */
static const struct device *g_data_uart =
	DEVICE_DT_GET(DT_NODELABEL(cdc_acm_uart0));

// checks the data UART is ready and logs where things will come out
int usb_serial_init(void)
{
	if (!device_is_ready(g_data_uart)) {
		LOG_ERR("Data UART (cdc_acm_uart0 / ttyACM1) not ready");
		return -ENODEV;
	}
	LOG_INF("Data port ready — JSON on ttyACM1, shell on ttyACM0");
	return 0;
}

// sends a string + newline to the data UART (ttyACM1), only if host has DTR up
void usb_serial_println(const char *str)
{
	if (!str || !device_is_ready(g_data_uart)) {
		return;
	}

	for (const char *p = str; *p; p++) {
		uart_poll_out(g_data_uart, (uint8_t)*p);
	}
	uart_poll_out(g_data_uart, '\n');
}
