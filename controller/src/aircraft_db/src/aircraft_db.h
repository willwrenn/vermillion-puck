/*
 * SkyWatch controller — aircraft database (Stage 4.1).
 *
 * In-memory linked-list DB keyed by ICAO. Both producers (`usb_handler`
 * for ADS-B JSON, `bridge` for BLE binary) call `aircraft_db_upsert()`
 * with a populated `struct aircraft_t`; a stale-reaper thread evicts
 * entries that haven't been refreshed within AIRCRAFT_DB_STALE_MS.
 *
 * Ported from miniproject `node_db.c/h`: same `sys_slist_t` + `k_mutex`
 * + static pool allocator pattern, with aircraft-shaped fields.
 *
 * Thread-safety: every public function takes the internal mutex. Callers
 * may invoke from any context. The `for_each` callback runs WITH the
 * mutex held — keep callbacks short, no blocking, no further DB calls.
 */

#ifndef AIRCRAFT_DB_H
#define AIRCRAFT_DB_H

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/sys/slist.h>

#include "aircraft_t.h"
#include "kalman.h"

#define AIRCRAFT_DB_MAX_ENTRIES    32
#define AIRCRAFT_DB_STALE_MS       10000   /* per plan: 10 s */

/* Pred is published once the KF has seen at least this many measurements. */
#define AIRCRAFT_PRED_MIN_UPDATES  3
/* How many seconds ahead the published `pred_lat / pred_lon` projects.
 * 60 s @ ~250 kt = ~7.7 km — visible on a ±0.5° (≈55 km) map without being
 * silly. Bump higher for ARTCC-style centre views, lower for terminal. */
#define AIRCRAFT_PRED_HORIZON_S    60.0

struct aircraft_db_entry {
	sys_snode_t          node;
	struct aircraft_t    ac;
	int64_t              last_seen_ms;  /* k_uptime_get() at last upsert */
	struct kalman_state  kf;            /* per-aircraft CV Kalman state */
};

void aircraft_db_init(void);

/* Insert if absent, or update fields in place if present.
 * Returns 1 if inserted, 0 if updated, negative errno on failure. */
int  aircraft_db_upsert(const struct aircraft_t *src);

/* Copy current entry for `icao` into *out. Returns 0 found, -ENOENT missing. */
int  aircraft_db_lookup(const char *icao, struct aircraft_t *out);

/* Remove all entries. Counters reset. */
void aircraft_db_clear(void);

/* Total entry count (any age). */
int  aircraft_db_count(void);

/* Walk every entry. Callback returns false to stop early.
 * NB: mutex is held for the duration. */
typedef bool (*aircraft_db_cb_t)(const struct aircraft_db_entry *e, void *user);
void aircraft_db_for_each(aircraft_db_cb_t cb, void *user);

/* Lifetime counters for the shell. */
struct aircraft_db_stats {
	uint32_t inserts;
	uint32_t updates;
	uint32_t stale_evicts;
	uint32_t pool_full;     /* upserts that couldn't allocate a new slot */
};
void aircraft_db_get_stats(struct aircraft_db_stats *out);

#endif /* AIRCRAFT_DB_H */
