/*
 * SkyWatch controller — pairwise collision detector (Stages 8.1 + 8.2).
 *
 * Implementation notes:
 *  - Snapshot pattern (same as db_publisher): take the DB mutex briefly,
 *    copy lat/lon and a velocity-in-m/s pair derived from the KF state,
 *    release the mutex, then iterate pairs and emit JSON without holding
 *    any locks except usb_serial's own.
 *  - Velocity source: prefer KF (smoothed) once update_count >= 2; fall
 *    back to ADS-B-reported hdg/vel if the KF hasn't seen enough updates
 *    yet but the source provided heading + speed.
 *  - State tracking: a small fixed-size array of `tracked_pair` records
 *    keyed by sorted ICAO pair. On level transitions we emit a JSON
 *    frame; on CLEAR transition we emit one final "level":"CLEAR" frame
 *    so the GUI can drop its red flash.
 */

#include "collision.h"
#include "aircraft_db.h"
#include "aircraft_t.h"
#include "kalman.h"
#include "usb_serial.h"
#include "ble_packet.h"
#include "ble_central.h"

#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(collision, LOG_LEVEL_INF);

#define KF_PI            3.14159265358979323846
#define DEG_PER_M_LAT    (1.0 / 111000.0)
#define KT_TO_MPS        0.514444
#define MAX_TRACKED      32   /* max pairs we keep transition state for */

static inline double deg_per_m_lon_at(double lat_deg)
{
	return 1.0 / (111000.0 * cos(lat_deg * (KF_PI / 180.0)));
}

/* ---- snapshot row used by the math loop ---------------------------- */

struct snap_row {
	char     icao[AIRCRAFT_ICAO_BUF_LEN];
	double   lat, lon;
	double   v_north_mps, v_east_mps;
	int32_t  alt_ft;        /* only valid if alt_valid */
	bool     v_valid;
	bool     alt_valid;
};

static struct snap_row s_snap[AIRCRAFT_DB_MAX_ENTRIES];
static int             s_snap_n;

/* ---- per-pair transition tracking ----------------------------------- */

struct tracked_pair {
	char     icao_a[AIRCRAFT_ICAO_BUF_LEN];
	char     icao_b[AIRCRAFT_ICAO_BUF_LEN];
	enum collision_level level;  /* last reported level (non-CLEAR) */
};
static struct tracked_pair s_tracked[MAX_TRACKED];
static int                  s_tracked_n;

static struct collision_stats s_stats;
static struct k_spinlock      s_stats_lock;
static struct k_work_delayable s_work;

/* Keep-alive for `collision_inject`: re-upserts the two synthetic aircraft
 * every 3 s so they don't stale-evict (10 s threshold) and stay on the GUI
 * indefinitely. Runs until `collision_inject_stop()` is called (also
 * surfaced as `skywatch collision stop`). Refiring `inject` is idempotent.
 */
static struct k_work_delayable s_inject_keepalive;
static bool                    s_inject_active;
static void                    inject_keepalive_handler(struct k_work *work);

/* Stage 8.6 — "ghost" aircraft on intercept with the live BLE_SIM target.
 * `start` snapshots the BLE aircraft, picks head-on heading + offset, and
 * arms the keep-alive. Each tick dead-reckons the ghost forward so it
 * actually moves on the GUI. Stops on `collision_ghost_stop()` (natural
 * stale-evict after 10 s). */
#define GHOST_ICAO            "gh05t1"
#define GHOST_SPEED_KT        250.0
#define GHOST_TCA_TARGET_S    23.0       /* default — overridable per-call */
#define GHOST_TICK_MS         3000

static struct k_work_delayable s_ghost_keepalive;
static bool                    s_ghost_active;
static double                  s_ghost_lat;
static double                  s_ghost_lon;
static double                  s_ghost_v_north_mps;
static double                  s_ghost_v_east_mps;
static double                  s_ghost_hdg_deg;
static int32_t                 s_ghost_alt_ft;       /* configurable per ghost-start */
static int64_t                 s_ghost_last_step_ms;
static void                    ghost_keepalive_handler(struct k_work *work);

const char *collision_level_str(enum collision_level lvl)
{
	switch (lvl) {
	case COLLISION_WARNING:  return "WARNING";
	case COLLISION_ADVISORY: return "ADVISORY";
	default:                 return "CLEAR";
	}
}

/* ---- velocity derivation -------------------------------------------- */

static bool derive_velocity_mps(const struct aircraft_db_entry *e,
				 double *v_north, double *v_east)
{
	/* Prefer the source-reported heading + speed when both are present.
	 * This is dump1090's measured ground speed/track (or the sim node's
	 * declared intent), which is canonical truth about *where this thing
	 * is going*. The KF estimates velocity from position deltas which
	 * goes to ~0 for any aircraft whose positions aren't actually moving
	 * sample-to-sample (notably our `collision inject` test aircraft —
	 * keep-alive re-upserts the same lat/lon, so KF velocity collapses
	 * and the collision detector wrongly reports CLEAR). */
	if ((e->ac.valid_mask & AIRCRAFT_VALID_VEL) &&
	    (e->ac.valid_mask & AIRCRAFT_VALID_HDG)) {
		double speed_mps = e->ac.vel_kt * KT_TO_MPS;
		double hdg = e->ac.hdg_deg * (KF_PI / 180.0);
		*v_north = speed_mps * cos(hdg);
		*v_east  = speed_mps * sin(hdg);
		return true;
	}
	/* Fall back to KF only when source didn't report hdg/vel. */
	if (e->kf.initialised && e->kf.update_count >= 2) {
		*v_north = e->kf.x[2] / DEG_PER_M_LAT;
		*v_east  = e->kf.x[3] / deg_per_m_lon_at(e->ac.lat);
		return true;
	}
	*v_north = 0.0; *v_east = 0.0;
	return false;
}

static bool snap_cb(const struct aircraft_db_entry *e, void *user)
{
	ARG_UNUSED(user);
	if (s_snap_n >= AIRCRAFT_DB_MAX_ENTRIES) {
		return false;
	}
	struct snap_row *r = &s_snap[s_snap_n++];
	memcpy(r->icao, e->ac.icao, AIRCRAFT_ICAO_BUF_LEN);
	r->lat = e->ac.lat;
	r->lon = e->ac.lon;
	r->v_valid = derive_velocity_mps(e, &r->v_north_mps, &r->v_east_mps);
	if (e->ac.valid_mask & AIRCRAFT_VALID_ALT) {
		r->alt_ft   = e->ac.alt_ft;
		r->alt_valid = true;
	} else {
		r->alt_ft   = 0;
		r->alt_valid = false;
	}
	return true;
}

/* ---- math: closest-approach for one pair ---------------------------- */

struct approach {
	double  tca_s;
	double  min_sep_m;
	bool    diverging;
	/* Vertical separation between the pair, in feet. Only meaningful when
	 * `alt_valid` is set (both aircraft reported altitude). */
	int32_t alt_diff_ft;
	bool    alt_valid;
};

static struct approach closest_approach(const struct snap_row *a,
					 const struct snap_row *b)
{
	double lat_mid = 0.5 * (a->lat + b->lat);
	double deg_lon = deg_per_m_lon_at(lat_mid);
	double drx = (b->lon - a->lon) / deg_lon;
	double dry = (b->lat - a->lat) / DEG_PER_M_LAT;
	double dvx = b->v_east_mps  - a->v_east_mps;
	double dvy = b->v_north_mps - a->v_north_mps;
	double dv_mag2 = dvx * dvx + dvy * dvy;
	double cur_sep = sqrt(drx * drx + dry * dry);

	struct approach r = { .tca_s = INFINITY, .min_sep_m = cur_sep,
			      .diverging = false,
			      .alt_diff_ft = 0, .alt_valid = false };

	/* Populate altitude pair-info if both snaps reported it. The vertical
	 * gate in work_handler reads from snap directly; this is for the wire
	 * frame + GUI display. */
	if (a->alt_valid && b->alt_valid) {
		int32_t dz = a->alt_ft - b->alt_ft;
		r.alt_diff_ft = (dz < 0) ? -dz : dz;
		r.alt_valid = true;
	}
	if (dv_mag2 == 0.0) {
		return r;
	}
	double t = -(drx * dvx + dry * dvy) / dv_mag2;
	if (t <= 0.0) {
		r.tca_s = 0.0;
		r.diverging = true;
		return r;
	}
	double rx = drx + dvx * t;
	double ry = dry + dvy * t;
	r.tca_s = t;
	r.min_sep_m = sqrt(rx * rx + ry * ry);
	return r;
}

static enum collision_level classify(const struct approach *a)
{
	if (a->diverging || !isfinite(a->tca_s)) {
		return COLLISION_CLEAR;
	}
	if (a->min_sep_m < COLLISION_WARNING_SEP_M &&
	    a->tca_s     < COLLISION_WARNING_TCA_S) {
		return COLLISION_WARNING;
	}
	if (a->min_sep_m < COLLISION_ADVISORY_SEP_M &&
	    a->tca_s     < COLLISION_ADVISORY_TCA_S) {
		return COLLISION_ADVISORY;
	}
	return COLLISION_CLEAR;
}

/* ---- per-pair tracking ---------------------------------------------- */

static void pair_key(const char *a, const char *b, const char **lo, const char **hi)
{
	if (strcmp(a, b) <= 0) { *lo = a; *hi = b; } else { *lo = b; *hi = a; }
}

static int find_tracked(const char *lo, const char *hi)
{
	for (int i = 0; i < s_tracked_n; i++) {
		if (strncmp(s_tracked[i].icao_a, lo, AIRCRAFT_ICAO_BUF_LEN) == 0 &&
		    strncmp(s_tracked[i].icao_b, hi, AIRCRAFT_ICAO_BUF_LEN) == 0) {
			return i;
		}
	}
	return -1;
}

static void tracked_remove(int idx)
{
	if (idx < 0 || idx >= s_tracked_n) return;
	for (int i = idx; i < s_tracked_n - 1; i++) {
		s_tracked[i] = s_tracked[i + 1];
	}
	s_tracked_n--;
}

/* ---- JSON emit ------------------------------------------------------ */

/* Pack the "other" icao (the one that ISN'T the BLE sim's, or just `lo` as
 * a fallback) into Will's 3-byte big-endian-ish ICAO field. Will's frame
 * struct uses 3 raw bytes; we parse from the 6-char hex string. */
static void pack_icao24_bytes(const char *icao_hex, uint8_t out[3])
{
	char buf[3] = {0};
	for (int i = 0; i < 3; i++) {
		buf[0] = icao_hex[i * 2];
		buf[1] = icao_hex[i * 2 + 1];
		out[i] = (uint8_t)strtoul(buf, NULL, 16);
	}
}

/* Stage 8.3 — fire Will's legacy warning_frame for one WARNING-level pair.
 * Called only when lvl==COLLISION_WARNING and the BLE central is connected.
 * We pick the "other" ICAO as the one that's NOT a BLE_SIM aircraft (so
 * Will sees the threat ID, not his own). If both sides are non-BLE we just
 * pick `lo` — Will isn't in the pair anyway in that case. */
static void send_ble_warning(const char *lo, const char *hi)
{
	if (!ble_central_is_connected()) {
		return;
	}
	struct aircraft_t a_lo, a_hi;
	const char *other = lo;
	if (aircraft_db_lookup(lo, &a_lo) == 0 && a_lo.source == SRC_BLE_SIM) {
		other = hi;
	} else if (aircraft_db_lookup(hi, &a_hi) == 0 && a_hi.source == SRC_BLE_SIM) {
		other = lo;
	}

	struct ble_warning_frame w = {0};
	w.msg_type  = BLE_WARN_MSG_COLLISION;
	pack_icao24_bytes(other, w.icao);
	/* lat / lon / alt / hdg_delta fields aren't used by Will's collision
	 * handler — leave zero. */
	w.severity  = BLE_WARN_SEV_WARNING;
	w.timestamp = (uint8_t)(k_uptime_get() & 0xFF);
	w.crc       = ble_warning_crc(&w);

	int rc = ble_central_send_warning(&w);
	if (rc) {
		LOG_DBG("BLE warning write rc=%d", rc);
	}
}

static void emit_collision(const char *lo, const char *hi,
			   enum collision_level lvl,
			   const struct approach *a)
{
	char buf[224];
	int n;
	if (a->alt_valid) {
		n = snprintf(buf, sizeof(buf),
			"{\"type\":\"collision\",\"icao_a\":\"%s\",\"icao_b\":\"%s\","
			"\"level\":\"%s\",\"tca_s\":%.1f,\"min_sep_m\":%.0f,"
			"\"alt_diff_ft\":%d,\"ts\":%.3f}",
			lo, hi,
			collision_level_str(lvl),
			isfinite(a->tca_s) ? a->tca_s : 0.0,
			a->min_sep_m,
			a->alt_diff_ft,
			(double)k_uptime_get() / 1000.0);
	} else {
		n = snprintf(buf, sizeof(buf),
			"{\"type\":\"collision\",\"icao_a\":\"%s\",\"icao_b\":\"%s\","
			"\"level\":\"%s\",\"tca_s\":%.1f,\"min_sep_m\":%.0f,"
			"\"ts\":%.3f}",
			lo, hi,
			collision_level_str(lvl),
			isfinite(a->tca_s) ? a->tca_s : 0.0,
			a->min_sep_m,
			(double)k_uptime_get() / 1000.0);
	}
	if (n < 0 || (size_t)n >= sizeof(buf)) return;
	usb_serial_println(buf);
	K_SPINLOCK(&s_stats_lock) { s_stats.frames_emitted++; }

	/* BLE warning is fired by the caller on transition INTO WARNING
	 * (see work_handler). We deliberately do NOT re-fire here on every
	 * tick: Will's mobile board buzzes for 10 s per frame and re-arms
	 * on each one, which kept the speaker chirping long after the
	 * banner cleared. One frame per transition is enough. */
}

/* ---- per-tick handler ----------------------------------------------- */

static void work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	K_SPINLOCK(&s_stats_lock) { s_stats.ticks++; }

	/* 1. snapshot */
	s_snap_n = 0;
	aircraft_db_for_each(snap_cb, NULL);

	uint32_t pairs = 0;
	uint32_t adv = 0, warn = 0;

	/* Mark all currently-tracked pairs as "not seen this tick" by
	 * stashing flags in a local array; entries still flagged at the end
	 * have gone CLEAR and need a final CLEAR emission. */
	bool seen[MAX_TRACKED] = {0};

	/* 2. all unique pairs */
	for (int i = 0; i < s_snap_n; i++) {
		if (!s_snap[i].v_valid) continue;
		for (int j = i + 1; j < s_snap_n; j++) {
			if (!s_snap[j].v_valid) continue;
			pairs++;
			struct approach app = closest_approach(&s_snap[i], &s_snap[j]);
			enum collision_level lvl = classify(&app);

			/* Vertical-separation gate: if BOTH aircraft report
			 * altitude AND the gap exceeds COLLISION_VERTICAL_SEP_FT,
			 * force CLEAR regardless of horizontal geometry. This is
			 * what lets Will's mobile divert altitude to escape a
			 * head-on horizontal collision. */
			if (lvl != COLLISION_CLEAR &&
			    s_snap[i].alt_valid && s_snap[j].alt_valid) {
				int32_t dz = s_snap[i].alt_ft - s_snap[j].alt_ft;
				if (dz < 0) dz = -dz;
				if (dz > COLLISION_VERTICAL_SEP_FT) {
					lvl = COLLISION_CLEAR;
				}
			}

			const char *lo, *hi;
			pair_key(s_snap[i].icao, s_snap[j].icao, &lo, &hi);
			int tk = find_tracked(lo, hi);

			if (lvl == COLLISION_CLEAR) {
				/* No-op unless previously tracked (handled in
				 * second pass below). */
				continue;
			}

			if (lvl == COLLISION_WARNING) warn++;
			else                          adv++;

			enum collision_level prev_level = COLLISION_CLEAR;
			if (tk >= 0) {
				seen[tk] = true;
				prev_level = s_tracked[tk].level;
				/* Emit every tick while non-CLEAR (both WARNING and
				 * ADVISORY). The GUI's alert dict expires entries after
				 * ~3 s of silence, so emitting only on transition would
				 * make a one-frame alert vanish before the operator
				 * could read it. CLEAR is handled in the post-loop pass. */
				s_tracked[tk].level = lvl;
				emit_collision(lo, hi, lvl, &app);
			} else if (s_tracked_n < MAX_TRACKED) {
				int idx = s_tracked_n++;
				strncpy(s_tracked[idx].icao_a, lo, AIRCRAFT_ICAO_BUF_LEN);
				strncpy(s_tracked[idx].icao_b, hi, AIRCRAFT_ICAO_BUF_LEN);
				s_tracked[idx].level = lvl;
				seen[idx] = true;
				emit_collision(lo, hi, lvl, &app);
			}

			/* Fire Will's buzzer ONLY on a fresh transition into WARNING
			 * (CLEAR/ADVISORY -> WARNING). His board re-arms a 10 s
			 * auto-clearing buzzer on each frame; re-firing every tick
			 * made the speaker keep chirping after the GUI banner had
			 * already cleared. One frame per encounter is enough. */
			if (lvl == COLLISION_WARNING && prev_level != COLLISION_WARNING) {
				send_ble_warning(lo, hi);
			}
		}
	}

	/* 3. tracked pairs that didn't reappear → emit CLEAR + drop. */
	for (int i = s_tracked_n - 1; i >= 0; i--) {
		if (!seen[i]) {
			struct approach a = { .tca_s = INFINITY,
					      .min_sep_m = 0.0, .diverging = true };
			emit_collision(s_tracked[i].icao_a, s_tracked[i].icao_b,
				       COLLISION_CLEAR, &a);
			tracked_remove(i);
		}
	}

	K_SPINLOCK(&s_stats_lock) {
		s_stats.pairs_checked += pairs;
		s_stats.advisory_count += adv;
		s_stats.warning_count  += warn;
	}

	k_work_reschedule(&s_work, K_MSEC(COLLISION_TICK_MS));
}

/* ---- API ------------------------------------------------------------ */

int collision_init(void)
{
	memset(&s_stats, 0, sizeof(s_stats));
	s_tracked_n = 0;
	s_inject_active = false;
	s_ghost_active  = false;
	k_work_init_delayable(&s_work, work_handler);
	k_work_init_delayable(&s_inject_keepalive, inject_keepalive_handler);
	k_work_init_delayable(&s_ghost_keepalive,  ghost_keepalive_handler);
	k_work_reschedule(&s_work, K_MSEC(COLLISION_TICK_MS));
	LOG_INF("collision detector armed @ %d ms", COLLISION_TICK_MS);
	return 0;
}

void collision_get_stats(struct collision_stats *out)
{
	K_SPINLOCK(&s_stats_lock) { *out = s_stats; }
}

/* Build the two canned converging aircraft. Same coords each call so the
 * keep-alive worker can re-upsert them without state. SRC_FICT marks them
 * so the GUI table clearly distinguishes them from real ADS-B / BLE_SIM.
 *
 * Geometry: head-on east/west, 6 km apart (off-CBD but both visible),
 * 250 kt each → closing speed ~257 m/s → TCA ≈ 23 s, min_sep ≈ 0 →
 * classifies as WARNING (<500 m, <30 s) rather than ADVISORY. */
static void build_inject_pair(struct aircraft_t *a, struct aircraft_t *b)
{
	memset(a, 0, sizeof(*a));
	strncpy(a->icao, "c011a1", AIRCRAFT_ICAO_BUF_LEN - 1);
	a->lat = -27.55;            /* same lat, head-on */
	a->lon = 152.97;            /* ~3 km west of -27.55,153.00 */
	a->source = SRC_FICT;
	a->valid_mask = AIRCRAFT_VALID_VEL | AIRCRAFT_VALID_HDG;
	a->vel_kt = 250.0;
	a->hdg_deg = 90.0;          /* east */
	a->ts = (double)k_uptime_get() / 1000.0;

	memset(b, 0, sizeof(*b));
	strncpy(b->icao, "c011b2", AIRCRAFT_ICAO_BUF_LEN - 1);
	b->lat = -27.55;
	b->lon = 153.03;            /* ~3 km east of -27.55,153.00 */
	b->source = SRC_FICT;
	b->valid_mask = AIRCRAFT_VALID_VEL | AIRCRAFT_VALID_HDG;
	b->vel_kt = 250.0;
	b->hdg_deg = 270.0;         /* west */
	b->ts = a->ts;
}

static void inject_keepalive_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	if (!s_inject_active) {
		LOG_INF("inject keep-alive stopped; c011a1/c011b2 will stale-evict in ~10 s");
		return;
	}
	struct aircraft_t a, b;
	build_inject_pair(&a, &b);
	aircraft_db_upsert(&a);
	aircraft_db_upsert(&b);
	LOG_DBG("inject keep-alive re-upserted c011a1 + c011b2");
	k_work_reschedule(&s_inject_keepalive, K_MSEC(3000));
}

int collision_inject_scenario(void)
{
	struct aircraft_t a, b;
	build_inject_pair(&a, &b);
	int ra = aircraft_db_upsert(&a);
	int rb = aircraft_db_upsert(&b);
	if (ra < 0 || rb < 0) return -EIO;

	/* Run the keep-alive indefinitely until `collision_inject_stop()`.
	 * Idempotent: re-firing `inject` just refreshes positions and
	 * (re)arms the worker — no double-schedule because k_work_reschedule
	 * cancels any pending instance first. */
	s_inject_active = true;
	k_work_reschedule(&s_inject_keepalive, K_MSEC(3000));

	LOG_INF("INJECT c011a1 (FICT, %.4f, %.4f, hdg %.0f) + c011b2 (FICT, %.4f, %.4f, hdg %.0f) — keep-alive until `collision stop`",
		a.lat, a.lon, a.hdg_deg, b.lat, b.lon, b.hdg_deg);
	return 0;
}

int collision_inject_stop(void)
{
	s_inject_active = false;
	/* Worker will see the flag on its next tick and exit. We do NOT remove
	 * c011a1/c011b2 explicitly — the natural 10 s aircraft_db stale reaper
	 * makes them fade out the same way a lost real aircraft would. */
	LOG_INF("INJECT stop requested; c011a1/c011b2 will stale-evict shortly");
	return 0;
}

/* ---- Stage 8.6 — ghost mode ---------------------------------------- */

/* Walk the DB and capture the first SRC_BLE_SIM aircraft into *out.
 * Returns true on found. Callback runs under the aircraft_db mutex; keep
 * it quick. */
struct find_ctx { struct aircraft_t *out; bool found; };

static bool find_ble_sim_cb(const struct aircraft_db_entry *e, void *user)
{
	struct find_ctx *c = user;
	if (e->ac.source == SRC_BLE_SIM) {
		*c->out = e->ac;
		c->found = true;
		return false;   /* stop iteration */
	}
	return true;
}

/* Write the current ghost state out to the aircraft DB. Source is FICT so
 * the GUI table and map clearly mark it as a test aircraft. */
static void ghost_upsert(void)
{
	struct aircraft_t g = {0};
	strncpy(g.icao, GHOST_ICAO, AIRCRAFT_ICAO_BUF_LEN - 1);
	g.source     = SRC_FICT;
	g.lat        = s_ghost_lat;
	g.lon        = s_ghost_lon;
	g.ts         = (double)k_uptime_get() / 1000.0;
	g.valid_mask = AIRCRAFT_VALID_ALT | AIRCRAFT_VALID_VEL | AIRCRAFT_VALID_HDG;
	g.alt_ft     = s_ghost_alt_ft;
	g.vel_kt     = GHOST_SPEED_KT;
	g.hdg_deg    = s_ghost_hdg_deg;
	aircraft_db_upsert(&g);
}

static void ghost_keepalive_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	if (!s_ghost_active) {
		LOG_INF("ghost keep-alive stopped; %s will stale-evict in ~10 s",
			GHOST_ICAO);
		return;
	}

	/* Dead-reckon forward by dt seconds since the last tick. v_north/v_east
	 * are in m/s; convert to deg/s via the per-axis scaling. */
	int64_t now = k_uptime_get();
	double dt = (now - s_ghost_last_step_ms) / 1000.0;
	if (dt < 0.0 || dt > 30.0) {
		dt = (double)GHOST_TICK_MS / 1000.0;   /* sanity fallback */
	}
	s_ghost_last_step_ms = now;

	s_ghost_lat += s_ghost_v_north_mps * dt * DEG_PER_M_LAT;
	s_ghost_lon += s_ghost_v_east_mps  * dt * deg_per_m_lon_at(s_ghost_lat);

	ghost_upsert();
	LOG_DBG("ghost step dt=%.2fs -> %.5f, %.5f", dt, s_ghost_lat, s_ghost_lon);
	k_work_reschedule(&s_ghost_keepalive, K_MSEC(GHOST_TICK_MS));
}

int collision_ghost_start(double tca_s, double miss_m, int alt_ft)
{
	/* If a ghost is already running, tear down its state so the new spawn
	 * registers as a clean transition (and re-fires Will's BLE warning if
	 * the new geometry is WARNING-level — we only emit BLE on transitions
	 * now, so without this the new ghost would be silent). The DB entry
	 * for GHOST_ICAO is harmlessly overwritten by the fresh upsert below;
	 * we wipe any tracked-pair state that names gh05t1 to reset the level
	 * transition machine. */
	if (s_ghost_active) {
		LOG_INF("ghost_start: re-entry — resetting previous ghost state");
		k_work_cancel_delayable(&s_ghost_keepalive);
		s_ghost_active = false;
		/* Drop any tracked pairs referencing gh05t1 so the next tick
		 * sees a "fresh" transition. Iterate backwards so removal-by-
		 * index doesn't skip entries. */
		for (int i = s_tracked_n - 1; i >= 0; i--) {
			if (strncmp(s_tracked[i].icao_a, GHOST_ICAO,
				    AIRCRAFT_ICAO_BUF_LEN) == 0 ||
			    strncmp(s_tracked[i].icao_b, GHOST_ICAO,
				    AIRCRAFT_ICAO_BUF_LEN) == 0) {
				tracked_remove(i);
			}
		}
	}

	/* 1. Snapshot Will's aircraft from the DB. */
	struct aircraft_t target = {0};
	struct find_ctx ctx = { .out = &target, .found = false };
	aircraft_db_for_each(find_ble_sim_cb, &ctx);
	if (!ctx.found) {
		LOG_WRN("ghost_start: no BLE_SIM aircraft in DB yet");
		return -ENOENT;
	}

	/* Defaults if caller passed sentinel values. */
	if (tca_s  <= 0.0) tca_s  = GHOST_TCA_TARGET_S;
	if (miss_m <  0.0) miss_m = 0.0;
	if (alt_ft <  0) {
		/* Match Will's current altitude so the test fires WARNING out of
		 * the box. Vertical-sep gate kicks in only if Will diverts. */
		alt_ft = (target.valid_mask & AIRCRAFT_VALID_ALT)
			 ? target.alt_ft : 5000;
	}
	s_ghost_alt_ft = alt_ft;

	/* 2. Head-on heading. */
	double target_hdg = (target.valid_mask & AIRCRAFT_VALID_HDG)
			    ? target.hdg_deg : 0.0;
	s_ghost_hdg_deg = fmod(target_hdg + 180.0, 360.0);

	/* 3. Velocity components from heading + speed (m/s). */
	double speed_mps = GHOST_SPEED_KT * KT_TO_MPS;
	double hdg_rad   = s_ghost_hdg_deg * (KF_PI / 180.0);
	s_ghost_v_north_mps = speed_mps * cos(hdg_rad);
	s_ghost_v_east_mps  = speed_mps * sin(hdg_rad);

	/* 4. Position the ghost: `range` along target's heading (so they close
	 * head-on) PLUS `miss_m` perpendicular (so they pass at min_sep = miss_m
	 * instead of through each other). Perpendicular = 90° right of heading.
	 * range = closing_speed × tca_s. */
	double closing_speed_mps = 2.0 * speed_mps;
	double range_m = closing_speed_mps * tca_s;
	double t_hdg_rad = target_hdg * (KF_PI / 180.0);
	/* along-heading unit vector (dn, de) */
	double along_dn  = cos(t_hdg_rad);
	double along_de  = sin(t_hdg_rad);
	/* perpendicular (90° clockwise — right) */
	double perp_dn   = -sin(t_hdg_rad);
	double perp_de   =  cos(t_hdg_rad);

	double dn = range_m * along_dn + miss_m * perp_dn;
	double de = range_m * along_de + miss_m * perp_de;
	s_ghost_lat = target.lat + dn * DEG_PER_M_LAT;
	s_ghost_lon = target.lon + de * deg_per_m_lon_at(target.lat);

	/* 5. Upsert + arm the dead-reckon worker. */
	s_ghost_last_step_ms = k_uptime_get();
	ghost_upsert();
	s_ghost_active = true;
	k_work_reschedule(&s_ghost_keepalive, K_MSEC(GHOST_TICK_MS));

	LOG_INF("GHOST start: target=%s hdg=%.0f alt=%d, ghost=%s @ (%.4f, %.4f) "
		"hdg=%.0f alt=%d tca=%.0fs miss=%.0fm range=%.0fm",
		target.icao, target_hdg,
		(target.valid_mask & AIRCRAFT_VALID_ALT) ? target.alt_ft : -1,
		GHOST_ICAO,
		s_ghost_lat, s_ghost_lon, s_ghost_hdg_deg,
		s_ghost_alt_ft, tca_s, miss_m, range_m);
	return 0;
}

int collision_ghost_stop(void)
{
	s_ghost_active = false;
	LOG_INF("GHOST stop requested; %s will stale-evict in ~10 s", GHOST_ICAO);
	return 0;
}
