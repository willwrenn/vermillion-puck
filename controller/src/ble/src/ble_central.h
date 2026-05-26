/*
 * SkyWatch controller — BLE GATT central, both directions.
 *
 * Discovers the SkyWatch aircraft service on a peripheral advertising as
 * `skywatch-sim` (William's sim node). Two-stage GATT discovery:
 *   1. SkyWatch aircraft service + characteristic — NOTIFY, position
 *      frames in via bridge_submit_frame().
 *   2. Warning service (`ab340001-...`) + characteristic — WRITE,
 *      collision / diversion / CRASH frames out via
 *      ble_central_send_warning().
 *
 * Plus a watchdog k_work that auto-reconnects every 2 s when the link
 * drops, so the demo survives the sim being unplugged + replugged.
 */

#ifndef BLE_CENTRAL_H
#define BLE_CENTRAL_H

#include <stdint.h>
#include <stdbool.h>
#include <zephyr/shell/shell.h>

int  ble_central_init(void);
void ble_central_scan_start(void);
void ble_central_scan_stop(void);
bool ble_central_is_connected(void);
void ble_central_disconnect(void);
void ble_central_scan_list(const struct shell *sh);

/* write Will's legacy `warning_frame` to the sim node's WARN
 * characteristic via write-without-response. Caller fills the entire
 * `ble_warning_frame` including the XOR CRC (see ble_warning_crc()).
 * Returns 0 on accepted submit, -ENOTCONN if not connected, -EAGAIN if
 * the warning characteristic handle hasn't been discovered yet. */
struct ble_warning_frame;
int  ble_central_send_warning(const struct ble_warning_frame *f);

struct ble_central_stats {
	uint32_t notify_count;     /* total notifications received */
	uint32_t bad_length;       /* notifies with len != BLE_FRAME_SIZE */
	uint32_t submitted;        /* frames handed to bridge_submit_frame() */
	uint32_t connect_count;    /* successful connections since boot */
	uint32_t disconnect_count; /* disconnect callbacks since boot */
	bool     connected;
};

void ble_central_get_stats(struct ble_central_stats *out);

#endif /* BLE_CENTRAL_H */
