/*
 * Guided missile pursuit worker. See missile.h for the design.
 *
 * Pure-pursuit kinematics (per tick, dt seconds):
 *   1. Snapshot target lat/lon from aircraft_db (first SRC_BLE_SIM).
 *   2. Desired heading = bearing(missile -> target). Bearing here is
 *      atan2(de, dn) where dn/de are local north/east deltas in metres.
 *   3. Angular delta = wrap(desired - current) clamped to ±max_turn*dt.
 *   4. Advance position: speed_mps*dt along the new heading.
 *   5. Upsert as FICT aircraft so the normal aircraft pipeline picks it up.
 *
 * Impact: handled implicitly by the existing collision pipeline. If the
 * missile pairs with the target at WARNING-level, collision.c emits the
 * usual JSON + fires Will's buzzer. We hook the same DB scan to spot the
 * close-range condition and cancel ourselves so a "destroyed" missile
 * doesn't keep tracking forever.
 */

#include "missile.h"
#include "aircraft_db.h"
#include "aircraft_t.h"

#include <errno.h>
#include <math.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(missile, LOG_LEVEL_INF);

#define M_PI_LOC          3.14159265358979323846
#define DEG_PER_M_LAT     (1.0 / 111000.0)
#define KT_TO_MPS         0.514444
#define MISSILE_ICAO      "m1s51l"

/* Stage 11 — the missile no longer self-cancels on proximity. The
 * collision detector's CRASH event (100 m horiz + 100 m vert — see
 * collision.h) is the single source of truth for what counts as a "hit",
 * and collision.c calls missile_cancel() on the transition so the
 * missile doesn't keep orbiting through the impact point. */

static inline double deg_per_m_lon_at(double lat_deg)
{
	return 1.0 / (111000.0 * cos(lat_deg * (M_PI_LOC / 180.0)));
}

/* Wrap an angle to (-180, +180]. */
static inline double wrap_180(double a)
{
	while (a >   180.0) a -= 360.0;
	while (a <= -180.0) a += 360.0;
	return a;
}

static struct k_work_delayable s_work;
static bool      s_active;
static double    s_lat, s_lon;
static double    s_hdg_deg;          /* missile current heading */
static double    s_speed_kt;
static double    s_turn_rate_dps;
static int32_t   s_alt_ft;           /* climbs to/holds target altitude */
static int64_t   s_launch_ms;
static int64_t   s_expire_ms;
static int64_t   s_last_step_ms;

/* DB scan helper: copy first SRC_BLE_SIM entry into ctx->out. */
struct find_ctx {
	struct aircraft_t *out;
	bool found;
};
static bool find_target_cb(const struct aircraft_db_entry *e, void *user)
{
	struct find_ctx *c = user;
	if (e->ac.source == SRC_BLE_SIM) {
		*c->out = e->ac;
		c->found = true;
		return false;   /* stop */
	}
	return true;
}

static void missile_upsert(void)
{
	struct aircraft_t a = {0};
	strncpy(a.icao, MISSILE_ICAO, AIRCRAFT_ICAO_BUF_LEN - 1);
	a.source     = SRC_FICT;
	a.lat        = s_lat;
	a.lon        = s_lon;
	a.ts         = (double)k_uptime_get() / 1000.0;
	a.valid_mask = AIRCRAFT_VALID_ALT | AIRCRAFT_VALID_VEL | AIRCRAFT_VALID_HDG;
	a.alt_ft     = s_alt_ft;
	a.vel_kt     = s_speed_kt;
	a.hdg_deg    = s_hdg_deg;
	aircraft_db_upsert(&a);
}

static void worker_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	if (!s_active) {
		return;
	}

	int64_t now = k_uptime_get();

	/* TTL expiry — self-destruct. */
	if (now >= s_expire_ms) {
		LOG_INF("missile %s self-destruct (ttl reached)", MISSILE_ICAO);
		s_active = false;
		return;
	}

	double dt = (now - s_last_step_ms) / 1000.0;
	if (dt < 0.0 || dt > 30.0) dt = (double)MISSILE_TICK_MS / 1000.0;
	s_last_step_ms = now;

	/* Snapshot target. */
	struct aircraft_t tgt = {0};
	struct find_ctx ctx = { .out = &tgt, .found = false };
	aircraft_db_for_each(find_target_cb, &ctx);
	if (!ctx.found) {
		/* Target gone — coast in current heading and wait; if target
		 * never returns the TTL expiry will eventually clean us up. */
		LOG_DBG("missile: target lost; coasting");
	} else {
		/* Bearing from missile to target, in degrees true (0..360). */
		double dn = (tgt.lat - s_lat) / DEG_PER_M_LAT;            /* metres north */
		double de = (tgt.lon - s_lon) / deg_per_m_lon_at(s_lat);  /* metres east  */
		double desired_hdg = atan2(de, dn) * (180.0 / M_PI_LOC);
		if (desired_hdg < 0.0) desired_hdg += 360.0;

		double delta = wrap_180(desired_hdg - s_hdg_deg);
		double max_step = s_turn_rate_dps * dt;
		if      (delta >  max_step) delta =  max_step;
		else if (delta < -max_step) delta = -max_step;
		s_hdg_deg = fmod(s_hdg_deg + delta + 360.0, 360.0);

		/* Climb (or descend) toward target altitude at a fixed vertical
		 * rate so altitude alone can't trivially defeat the missile. */
		if (tgt.valid_mask & AIRCRAFT_VALID_ALT) {
			int32_t target_alt = tgt.alt_ft;
			int32_t alt_step   = (int32_t)(2000.0 * dt);   /* 2000 ft/s — aggressive */
			if      (s_alt_ft < target_alt - alt_step) s_alt_ft += alt_step;
			else if (s_alt_ft > target_alt + alt_step) s_alt_ft -= alt_step;
			else                                       s_alt_ft  = target_alt;
		}

		/* No self-impact check — collision.c's CRASH transition is the
		 * authoritative hit detector (50 m / 50 m by default). When CRASH
		 * fires for a pair that includes m1s51l, collision.c calls
		 * missile_cancel() which stops the worker and lets the missile
		 * stale-evict from the DB ~10 s later. */
	}

	/* Dead-reckon forward. */
	double speed_mps = s_speed_kt * KT_TO_MPS;
	double hdg_rad   = s_hdg_deg * (M_PI_LOC / 180.0);
	s_lat += speed_mps * cos(hdg_rad) * dt * DEG_PER_M_LAT;
	s_lon += speed_mps * sin(hdg_rad) * dt * deg_per_m_lon_at(s_lat);

	missile_upsert();
	k_work_reschedule(&s_work, K_MSEC(MISSILE_TICK_MS));
}

int missile_launch(double ttl_s, double turn_rate_dps, double speed_kt)
{
	static bool s_work_inited;
	if (!s_work_inited) {
		k_work_init_delayable(&s_work, worker_handler);
		s_work_inited = true;
	}

	/* If a missile is already in flight, cancel before re-launching so
	 * the previous worker can't race against the new state. */
	if (s_active) {
		LOG_INF("missile: re-launch — cancelling previous in-flight missile");
		k_work_cancel_delayable(&s_work);
		s_active = false;
	}

	/* Apply defaults. */
	if (ttl_s          <= 0.0) ttl_s          = MISSILE_DEFAULT_TTL_S;
	if (turn_rate_dps  <= 0.0) turn_rate_dps  = MISSILE_DEFAULT_TURN_RATE;
	if (speed_kt       <= 0.0) speed_kt       = MISSILE_DEFAULT_SPEED_KT;

	/* Require a target to lock on at launch time so we can set initial
	 * heading sensibly. After launch, target-loss is tolerated (we
	 * coast). */
	struct aircraft_t tgt = {0};
	struct find_ctx ctx = { .out = &tgt, .found = false };
	aircraft_db_for_each(find_target_cb, &ctx);
	if (!ctx.found) {
		LOG_WRN("missile_launch: no BLE_SIM target in DB");
		return -ENOENT;
	}

	s_lat = MISSILE_LAUNCH_LAT;
	s_lon = MISSILE_LAUNCH_LON;
	s_alt_ft = MISSILE_LAUNCH_ALT_FT;
	s_speed_kt = speed_kt;
	s_turn_rate_dps = turn_rate_dps;

	/* Initial heading: straight at target's current position. */
	double dn = (tgt.lat - s_lat) / DEG_PER_M_LAT;
	double de = (tgt.lon - s_lon) / deg_per_m_lon_at(s_lat);
	double bearing = atan2(de, dn) * (180.0 / M_PI_LOC);
	if (bearing < 0.0) bearing += 360.0;
	s_hdg_deg = bearing;

	s_launch_ms     = k_uptime_get();
	s_last_step_ms  = s_launch_ms;
	s_expire_ms     = s_launch_ms + (int64_t)(ttl_s * 1000.0);
	s_active        = true;

	missile_upsert();
	k_work_reschedule(&s_work, K_MSEC(MISSILE_TICK_MS));

	LOG_INF("missile %s launched: target=%s bearing=%.0f° ttl=%.0fs turn=%.0f°/s speed=%.0fkt",
		MISSILE_ICAO, tgt.icao, bearing, ttl_s, turn_rate_dps, speed_kt);
	return 0;
}

int missile_cancel(void)
{
	if (!s_active) {
		return -EALREADY;
	}
	k_work_cancel_delayable(&s_work);
	s_active = false;
	LOG_INF("missile %s cancelled by operator", MISSILE_ICAO);
	return 0;
}
