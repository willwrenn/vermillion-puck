/*
 * SkyWatch controller — BLE binary frame -> aircraft_t bridge (Stage 3.3).
 *
 * See ref/description.md for design context.
 *
 * Architecture:
 *   ble notify_cb  ──┐
 *   skywatch inject ─┴── bridge_submit_frame() ── k_msgq ── bridge_thread
 *                                                          │
 *                                                          ▼
 *                                       bridge_convert() ──▶  LOG_INF (3.3)
 *                                                          (Stage 4.2 → db_upsert)
 *
 * bridge_convert() is the only place that knows how to translate
 * `ble_aircraft_frame` into `aircraft_t`. It's exposed so that
 * `skywatch ble inject` can use it directly and not depend on the BT stack.
 */

#include "bridge.h"
#include "aircraft_db.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(bridge, LOG_LEVEL_INF);

#define BRIDGE_QUEUE_DEPTH   8
#define BRIDGE_STACK         2048
#define BRIDGE_PRIO          7

K_MSGQ_DEFINE(bridge_msgq, sizeof(struct ble_aircraft_frame),
	      BRIDGE_QUEUE_DEPTH, 4);

static struct bridge_stats g_stats;
static struct k_spinlock   stats_lock;

/* ---- Synchronous converter: BLE frame -> aircraft_t ------------------ */
//Purpose: This function converts a `ble_aircraft_frame` 
// (the raw data format received from the BLE characteristic)
int bridge_convert(const struct ble_aircraft_frame *in, struct aircraft_t *out)
{
	if (!in || !out) {
		return -EINVAL;
	}

	/* Validation per common/src/ble_packet.h "Receiver-side validation". */
	if (in->source != BLE_SRC_ADS_B && in->source != BLE_SRC_BLE_SIM) {
		K_SPINLOCK(&stats_lock) { g_stats.rej_source++; }
		return -EINVAL;
	}
	if (in->icao24 > 0xFFFFFFu) {
		K_SPINLOCK(&stats_lock) { g_stats.rej_icao++; }
		return -EINVAL;
	}
	if (in->lat < -90.0 || in->lat > 90.0 ||
	    in->lon < -180.0 || in->lon > 180.0) {
		K_SPINLOCK(&stats_lock) { g_stats.rej_latlon++; }
		return -EINVAL;
	}

	memset(out, 0, sizeof(*out));
	/* 24-bit ICAO -> 6 hex chars, lowercase, NUL-terminated. */
	snprintf(out->icao, AIRCRAFT_ICAO_BUF_LEN, "%06x", in->icao24 & 0xFFFFFFu);
	out->lat    = in->lat;
	out->lon    = in->lon;
	out->ts     = (double)in->ts_ms / 1000.0; /* sim-node uptime seconds */
	out->source = (in->source == BLE_SRC_BLE_SIM) ? SRC_BLE_SIM : SRC_ADS_B;

	if (in->flags & BLE_FLAG_ALT_VALID) {
		out->valid_mask |= AIRCRAFT_VALID_ALT;
		out->alt_ft = in->alt_ft;
	}
	if (in->flags & BLE_FLAG_VEL_VALID) {
		out->valid_mask |= AIRCRAFT_VALID_VEL;
		out->vel_kt = in->vel_kt;
	}
	if (in->flags & BLE_FLAG_HDG_VALID) {
		out->valid_mask |= AIRCRAFT_VALID_HDG;
		out->hdg_deg = in->hdg_deg;
	}
	return 0;
}

/* ---- Producer side: called from BLE notify callback ------------------ */
//Purpose: This function is called from the BLE notify callback
//  when a new frame is received.
void bridge_submit_frame(const struct ble_aircraft_frame *frame)
{
	if (!frame) {
		return;
	}
	K_SPINLOCK(&stats_lock) { g_stats.submitted++; }
	if (k_msgq_put(&bridge_msgq, frame, K_NO_WAIT) != 0) {
		K_SPINLOCK(&stats_lock) { g_stats.drops++; }
	}
}

/* ---- Consumer thread: drains msgq, converts, logs -------------------- */
//Purpose: This function is run by the bridge thread. 
// It waits for frames to be submitted to the message queue,
static void bridge_thread_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

	struct ble_aircraft_frame in;
	struct aircraft_t         out;

	while (1) {
		k_msgq_get(&bridge_msgq, &in, K_FOREVER);
		if (bridge_convert(&in, &out) != 0) {
			/* counter incremented inside bridge_convert(). */
			continue;
		}
		K_SPINLOCK(&stats_lock) { g_stats.converted++; }

		/* Stage 4.2 — feed the DB. */
		int db_rc = aircraft_db_upsert(&out);
		LOG_DBG("RX_BLE %s %s lat=%.6f lon=%.6f valid=0x%x db=%s",
			out.icao, aircraft_source_str(out.source),
			out.lat, out.lon, out.valid_mask,
			(db_rc == 1) ? "INS" :
			(db_rc == 0) ? "UPD" : "ERR");
	}
}

//Pur[pse: Spawn the bridge thread. This is called at SYS_INIT
//  after the kernel is up.]
K_THREAD_DEFINE(bridge_tid, BRIDGE_STACK, bridge_thread_fn,
		NULL, NULL, NULL, BRIDGE_PRIO, 0, 0);

/* ---- API ------------------------------------------------------------- */
//Purpose: This function initializes the BLE central.
int bridge_init(void)
{
	memset(&g_stats, 0, sizeof(g_stats));
	/* Thread auto-started via K_THREAD_DEFINE. */
	return 0;
}
//Purpose: Get the current stats counters for the bridge.
void bridge_get_stats(struct bridge_stats *out)
{
	K_SPINLOCK(&stats_lock) {
		*out = g_stats;
	}
}
