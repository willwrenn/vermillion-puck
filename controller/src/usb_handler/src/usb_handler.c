/*
 * SkyWatch controller — USB CDC ACM1 data-port Rx handler (Phase 2.2).
 *
 * Architecture:
 *   - IRQ callback (interrupt_handler): drain the UART FIFO into a ring buffer,
 *     throttle Rx if the ring buffer fills.
 *   - Background thread (rx_thread): wake on semaphore, drain the ring buffer
 *     byte-by-byte, accumulate into a line buffer until '\n', then LOG_INF
 *     the line. Line-buffer overrun resets the buffer and counts a drop.
 *
 * Sizing rationale:
 *   - JSON frame for our AircraftFrame schema is ~140-180 B per record.
 *   - sdr_bridge.py emits one frame per aircraft per poll (2 Hz). Worst case
 *     ~20 aircraft -> ~3.6 kB/s. A 2 kB ring buffer gives >0.5 s of headroom,
 *     plenty for the thread to drain.
 *   - Line buffer is 512 B (3x worst-case frame) so a malformed unterminated
 *     line gets caught quickly and resets.
 */

#include "usb_handler.h"
#include "json_parser.h"
#include "aircraft_t.h"
#include "aircraft_db.h"

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/ring_buffer.h>

LOG_MODULE_REGISTER(usb_handler, LOG_LEVEL_INF);

#define DATA_UART_NODE      DT_NODELABEL(cdc_acm_uart0)
#define RINGBUF_BYTES       2048
#define LINE_BUF_BYTES      512

#define RX_THREAD_STACK     2048
#define RX_THREAD_PRIO      7

static const struct device *const data_uart = DEVICE_DT_GET(DATA_UART_NODE);

RING_BUF_DECLARE(rx_ringbuf, RINGBUF_BYTES);

static K_SEM_DEFINE(rx_sem, 0, 1);

static struct usb_handler_stats g_stats;
static struct k_spinlock stats_lock;

static bool rx_throttled;

/* ---------- IRQ handler: UART FIFO -> ring buffer --------------------- */

static void interrupt_handler(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);

	while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
		if (rx_throttled || !uart_irq_rx_ready(dev)) {
			continue;
		}

		uint8_t buf[64];
		size_t space = ring_buf_space_get(&rx_ringbuf);
		size_t want = MIN(sizeof(buf), space);

		if (want == 0) {
			/* No room — disable IRQ and let the thread catch up. */
			uart_irq_rx_disable(dev);
			rx_throttled = true;
			break;
		}

		int n = uart_fifo_read(dev, buf, want);
		if (n <= 0) {
			break;
		}

		uint32_t put = ring_buf_put(&rx_ringbuf, buf, n);
		if (put < (uint32_t)n) {
			/* Should not happen given the space check, but count. */
			K_SPINLOCK(&stats_lock) {
				g_stats.ringbuf_drops += n - put;
			}
		}
		K_SPINLOCK(&stats_lock) {
			g_stats.bytes_rx += put;
		}

		/* Wake the thread to drain. */
		k_sem_give(&rx_sem);
	}
}

/* ---------- Background thread: ring buffer -> line buffer -> log ------ */

static void rx_thread_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

	static char line_buf[LINE_BUF_BYTES];
	size_t line_len = 0;

	while (1) {
		k_sem_take(&rx_sem, K_FOREVER);

		uint8_t chunk[128];
		uint32_t n;
		while ((n = ring_buf_get(&rx_ringbuf, chunk, sizeof(chunk))) > 0) {
			for (uint32_t i = 0; i < n; i++) {
				char c = (char)chunk[i];

				if (c == '\r') {
					/* tolerate CR before LF */
					continue;
				}
				if (c == '\n') {
					if (line_len > 0) {
						line_buf[line_len] = '\0';
						struct aircraft_t ac;
						int err = json_parse_aircraft(line_buf,
									       line_len, &ac);
						if (err == 0) {
							/* Stage 4.2 — feed the DB. */
							int db_rc = aircraft_db_upsert(&ac);
							LOG_DBG("RX_JSON %s %s lat=%.6f lon=%.6f valid=0x%x db=%s",
								ac.icao,
								aircraft_source_str(ac.source),
								ac.lat, ac.lon, ac.valid_mask,
								(db_rc == 1) ? "INS" :
								(db_rc == 0) ? "UPD" : "ERR");
							K_SPINLOCK(&stats_lock) {
								g_stats.frames_rx++;
							}
						} else {
							LOG_WRN("parse failed (%d): %.80s",
								err, line_buf);
							K_SPINLOCK(&stats_lock) {
								g_stats.parse_errors++;
							}
						}
					}
					line_len = 0;
					continue;
				}
				if (line_len >= sizeof(line_buf) - 1) {
					/* line too long — drop and resync at next '\n'. */
					line_len = 0;
					K_SPINLOCK(&stats_lock) {
						g_stats.line_overruns++;
					}
					continue;
				}
				line_buf[line_len++] = c;
			}
		}

		/* If we throttled, drainage above just freed space. Re-enable. */
		if (rx_throttled && ring_buf_space_get(&rx_ringbuf) > 256) {
			rx_throttled = false;
			uart_irq_rx_enable(data_uart);
		}
	}
}

K_THREAD_DEFINE(rx_thread_id, RX_THREAD_STACK,
		rx_thread_fn, NULL, NULL, NULL,
		RX_THREAD_PRIO, 0, 0);

/* ---------- Public init ----------------------------------------------- */

int usb_handler_init(void)
{
	if (!device_is_ready(data_uart)) {
		LOG_ERR("data UART (cdc_acm_uart0) not ready");
		return -ENODEV;
	}

	int err = uart_irq_callback_user_data_set(data_uart,
						  interrupt_handler, NULL);
	if (err) {
		LOG_ERR("uart_irq_callback_user_data_set: %d", err);
		return err;
	}

	uart_irq_rx_enable(data_uart);
	LOG_INF("data-port Rx armed on cdc_acm_uart0 (ttyACM1)");
	return 0;
}

void usb_handler_get_stats(struct usb_handler_stats *out)
{
	K_SPINLOCK(&stats_lock) {
		*out = g_stats;
	}
}
