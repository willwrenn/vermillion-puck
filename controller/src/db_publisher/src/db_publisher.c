/*
 * SkyWatch controller — periodic DB serialiser onto ACM1 (Stage 5.1).
 *
 * Design:
 *   1. 2 Hz `k_work_delayable` schedules itself on the system workqueue.
 *   2. Handler walks `aircraft_db_for_each` and copies each entry into a
 *      local snapshot. The DB mutex is held only for this short copy.
 *   3. After releasing the mutex (by returning from the callback), the
 *      handler iterates the snapshot, builds JSON with snprintf and emits
 *      one line per aircraft via `usb_serial_println` (DTR-gated; if the
 *      laptop hasn't opened ACM1, output silently no-ops).
 *
 * Snapshot avoids holding the DB mutex while doing 32 × ~13 ms UART writes
 * at 115200 baud. Worst case (full DB of 32 entries × ~150 B): emit phase is
 * ~420 ms — within the 500 ms tick budget, and any overrun is just a
 * dropped beat, not a livelock.
 */

#include "db_publisher.h"
#include "aircraft_db.h"
#include "aircraft_t.h"
#include "usb_serial.h"

#include <stdio.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(db_publisher, LOG_LEVEL_INF);

#define PUBLISH_PERIOD_MS   500            /* 2 Hz */
#define MAX_SNAPSHOT        AIRCRAFT_DB_MAX_ENTRIES
#define LINE_CAP            256

static struct k_work_delayable s_work;

static struct db_publisher_stats s_stats;
static struct k_spinlock         s_stats_lock;

/* Snapshot collected under the DB mutex, drained after release. */
static struct aircraft_t s_snapshot[MAX_SNAPSHOT];
static int               s_snapshot_n;

static bool snapshot_cb(const struct aircraft_db_entry *e, void *user)
{
	ARG_UNUSED(user);
	if (s_snapshot_n >= MAX_SNAPSHOT) {
		return false;
	}
	s_snapshot[s_snapshot_n++] = e->ac;
	return true;
}

/* Build one AircraftFrame JSON object into `buf`. Returns negative on
 * truncation (i.e. snprintf would have exceeded `cap`). Returns the number
 * of bytes written (excluding the trailing NUL) on success. */
static int format_aircraft_json(char *buf, size_t cap, const struct aircraft_t *ac)
{
	int n = snprintf(buf, cap,
			 "{\"type\":\"aircraft\",\"icao\":\"%s\","
			 "\"lat\":%.6f,\"lon\":%.6f",
			 ac->icao, ac->lat, ac->lon);
	if (n < 0 || (size_t)n >= cap) return -1;

	if (ac->valid_mask & AIRCRAFT_VALID_ALT) {
		int k = snprintf(buf + n, cap - n, ",\"alt\":%d", ac->alt_ft);
		if (k < 0 || (size_t)(n + k) >= cap) return -1;
		n += k;
	}
	if (ac->valid_mask & AIRCRAFT_VALID_VEL) {
		int k = snprintf(buf + n, cap - n, ",\"vel\":%.1f", ac->vel_kt);
		if (k < 0 || (size_t)(n + k) >= cap) return -1;
		n += k;
	}
	if (ac->valid_mask & AIRCRAFT_VALID_HDG) {
		int k = snprintf(buf + n, cap - n, ",\"hdg\":%.1f", ac->hdg_deg);
		if (k < 0 || (size_t)(n + k) >= cap) return -1;
		n += k;
	}

	int k = snprintf(buf + n, cap - n,
			 ",\"ts\":%.3f,\"source\":\"%s\"}",
			 ac->ts, aircraft_source_str(ac->source));
	if (k < 0 || (size_t)(n + k) >= cap) return -1;
	return n + k;
}

static void work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	K_SPINLOCK(&s_stats_lock) { s_stats.ticks++; }

	/* Snapshot — mutex held only here. */
	s_snapshot_n = 0;
	aircraft_db_for_each(snapshot_cb, NULL);

	/* Emit phase — mutex released. */
	char line[LINE_CAP];
	for (int i = 0; i < s_snapshot_n; i++) {
		int n = format_aircraft_json(line, sizeof(line), &s_snapshot[i]);
		if (n < 0) {
			K_SPINLOCK(&s_stats_lock) { s_stats.format_errors++; }
			continue;
		}
		usb_serial_println(line);
		K_SPINLOCK(&s_stats_lock) { s_stats.frames_emitted++; }
	}

	/* Reschedule self for the next 2 Hz tick. */
	k_work_reschedule(&s_work, K_MSEC(PUBLISH_PERIOD_MS));
}

int db_publisher_init(void)
{
	memset(&s_stats, 0, sizeof(s_stats));
	k_work_init_delayable(&s_work, work_handler);
	k_work_reschedule(&s_work, K_MSEC(PUBLISH_PERIOD_MS));
	LOG_INF("publishing aircraft_db on ACM1 at %d ms", PUBLISH_PERIOD_MS);
	return 0;
}

void db_publisher_get_stats(struct db_publisher_stats *out)
{
	K_SPINLOCK(&s_stats_lock) {
		*out = s_stats;
	}
}
