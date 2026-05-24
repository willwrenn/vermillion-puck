/*
 * SkyWatch controller — USB CDC data-port Rx handler.
 *
 * Interrupt-driven Rx on `cdc_acm_uart1` (the data port,
 * host-side /dev/ttyACM1). Reassembles newline-delimited frames from
 * the ring buffer, hands each complete line to the JSON parser thread,
 * and routes the resulting `aircraft_t` to `aircraft_db_upsert()`.
 * Stats counters exposed below for shell inspection + soak verification.
 */

#ifndef USB_HANDLER_H
#define USB_HANDLER_H

#include <stdint.h>

/* Init the data-port Rx path. Returns 0 on success. */
int usb_handler_init(void);

/* Stats exposed for shell inspection / soak verification. */
struct usb_handler_stats {
	uint32_t bytes_rx;          /* bytes pushed through the ring buffer */
	uint32_t frames_rx;         /* complete newline-terminated frames that parsed OK */
	uint32_t ringbuf_drops;     /* bytes lost because ring buffer was full */
	uint32_t line_overruns;     /* frames lost because a single line exceeded the line buffer */
	uint32_t parse_errors;      /* frames that arrived intact but failed JSON parse */
};

void usb_handler_get_stats(struct usb_handler_stats *out);

#endif /* USB_HANDLER_H */
