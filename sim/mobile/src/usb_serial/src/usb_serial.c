#include "usb_serial.h"

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(usb_serial, LOG_LEVEL_INF);

// ttyACM0 (board_cdc_acm_uart) — shell + logs, assigned via board DTSI
// zephyr,console = &board_cdc_acm_uart
// zephyr,shell-uart = &board_cdc_acm_uart
//
// ttyACM1 (cdc_acm_uart0) — data-only port defined in app.overlay.
// JSON position packets are written here directly via uart_poll_out,
// bypassing the shell/log backend entirely. ttyACM0 sees no JSON;
// ttyACM1 sees no shell noise or ANSI codes.
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
