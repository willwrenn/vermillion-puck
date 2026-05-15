/*
 * SkyWatch controller — USB CDC ACM1 data-port Rx handler.
 *
 * Stage:   2.2
 * Purpose: Interrupt-driven Rx on cdc_acm_uart0 (the data port, /dev/ttyACM1).
 *          Reassembles newline-delimited frames. Phase 2.2 logs each complete
 *          line via LOG_INF; Phase 2.3 will replace the log call with a hand-off
 *          to the JSON parser thread.
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
