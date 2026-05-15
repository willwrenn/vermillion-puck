/*
 * SkyWatch controller — BLE GATT central, telemetry receive side.
 *
 * Stages 3.1 + 3.2 of the build plan.
 *
 * Discovers the SkyWatch aircraft service on a peripheral advertising as
 * `skywatch-sim` (William's sim node). Subscribes to the aircraft
 * characteristic (NOTIFY). On every notification, validates length and
 * hands the raw `struct ble_aircraft_frame` to `bridge/` via
 * `bridge_submit_frame()`. Connection-management plumbing
 * (scan/connect/MTU/discover/auto-reconnect) is intentionally kept thin
 * and *only* concerned with bytes-in.
 *
 * NB: the controller→sim warning service (Will's custom `ab340001-...`)
 * is intentionally deferred to Stage 8.3. We do not discover or write to
 * it here.
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
