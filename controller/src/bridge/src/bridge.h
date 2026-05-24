/*
 * SkyWatch controller — BLE binary frame -> aircraft_t bridge.
 *
 * Inputs:  `struct ble_aircraft_frame` (binary, common/src/ble_packet.h)
 *          via bridge_submit_frame() called from the BLE notify callback.
 * Outputs: `struct aircraft_t` (common/src/aircraft_t.h) routed to
 *          `aircraft_db_upsert()` so the sim's position fuses with any
 *          ADS-B traffic in the same per-aircraft KF.
 *
 * Owns the producer/consumer queue between the BLE stack's notify callback
 * (BT work-queue context) and the bridge worker thread that validates,
 * converts, and upserts.
 */

#ifndef BRIDGE_H
#define BRIDGE_H

#include <stdint.h>
#include "aircraft_t.h"
#include "ble_packet.h"

int  bridge_init(void);

/* Non-blocking: copies the frame into the internal msgq. Call from any
 * context (specifically: the BLE notify callback). Drops are counted in
 * stats.drops. */
void bridge_submit_frame(const struct ble_aircraft_frame *frame);

/* Synchronous converter — exposed for `skywatch ble inject` and for
 * possible future unit tests. Returns 0 and fills *out on success, or
 * negative errno on validation failure. */
int  bridge_convert(const struct ble_aircraft_frame *in, struct aircraft_t *out);

struct bridge_stats {
	uint32_t submitted;     /* bridge_submit_frame() calls */
	uint32_t drops;         /* msgq full at submit time */
	uint32_t converted;     /* bridge_convert() succeeded in the worker */
	uint32_t rej_source;    /* source not in {0, 1} */
	uint32_t rej_icao;      /* icao24 > 0xFFFFFF */
	uint32_t rej_latlon;    /* lat/lon out of range */
};

void bridge_get_stats(struct bridge_stats *out);

#endif /* BRIDGE_H */
