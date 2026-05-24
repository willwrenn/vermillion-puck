/*
 * SkyWatch controller — pairwise collision detector (Stages 8.1 + 8.2).
 *
 * Algorithm is the direct C port of `scripts/stage8/collision_proto.py`
 * (10/10 pytest pass). Linear constant-velocity model:
 *   t* = -(Δr·Δv) / |Δv|²  →  closest-approach time
 *   min_sep = |Δr + Δv·t*|
 *
 * Runs at 1 Hz from the system work-queue:
 *   1. snapshot `aircraft_db` entries (under DB mutex, brief).
 *   2. release mutex, iterate every pair, compute, classify.
 *   3. emit `{"type":"collision",...}` JSON on ACM1 on level transitions
 *      and at least every cycle while non-CLEAR.
 *
 * Wire-format spec lives in `resources/standards/json_protocol.md`
 * (`CollisionFrame`). Threshold constants below match the plan's Stage 8.2
 * bands; keep them in lockstep with the Python prototype's constants.
 */

#ifndef COLLISION_H
#define COLLISION_H

#include <stdint.h>

#define COLLISION_WARNING_SEP_M    500.0
#define COLLISION_WARNING_TCA_S    30.0
#define COLLISION_ADVISORY_SEP_M   1000.0
#define COLLISION_ADVISORY_TCA_S   60.0

/* Stage 11 — CRASH thresholds. The single source of truth for what
 * counts as a "hit" anywhere in the system — the missile module no
 * longer self-cancels on proximity; it pursues all the way in and the
 * collision detector declares the CRASH event when these bounds are met.
 * If one aircraft has no altitude reported, only the horizontal
 * threshold applies. */
#define COLLISION_CRASH_SEP_M      100.0
#define COLLISION_CRASH_VERT_FT    328   /* ~100 m */

/* Vertical-separation gate. If both aircraft report altitude AND the
 * difference exceeds this, the pair is force-CLEAR regardless of
 * horizontal geometry. 1000 ft mirrors the standard ATC vertical
 * separation minimum below FL290. */
#define COLLISION_VERTICAL_SEP_FT  1000

/* 200 ms = 5 Hz. Fast enough that a 750 kt missile (~386 m/s) can't pass
 * through the 100 m CRASH zone between ticks (it covers ~77 m per tick at
 * that speed, vs the old 386 m per 1-Hz tick which missed the close
 * moment entirely). NOTE: `s_stats.advisory_count / warning_count /
 * pairs_checked / ticks` are now incremented 5x as often per real second —
 * raw values look bigger on `skywatch collision stats`, but rates per
 * second are unchanged. BLE re-fire stays gated by `last_ble_send_ms` in
 * wall-clock ms so the buzzer cadence doesn't change. */
#define COLLISION_TICK_MS          200    /* 5 Hz */

enum collision_level {
	COLLISION_CLEAR    = 0,
	COLLISION_ADVISORY = 1,
	COLLISION_WARNING  = 2,
	COLLISION_CRASH    = 3,   /* Stage 11 — terminal; fires once per pair */
};

/* Diversion suggestion sent to the BLE sim on every WARNING transition.
 * The picker chooses one based on relative geometry; Will's mobile
 * firmware already handles MSG_DIVERSION frames and prints them, and
 * his GUI shows them in the orange panel. */
enum collision_diversion {
	DIVERSION_LEFT    = 0,   /* turn -30°  */
	DIVERSION_RIGHT   = 1,   /* turn +30°  */
	DIVERSION_CLIMB   = 2,   /* +1000 ft   */
	DIVERSION_DESCEND = 3,   /* -1000 ft   */
	DIVERSION_RTB     = 4,   /* head toward UQ St Lucia */
	DIVERSION_HOLD    = 5,   /* enter holding circle    */
};

const char *collision_level_str(enum collision_level lvl);

int  collision_init(void);

struct collision_stats {
	uint32_t ticks;
	uint32_t pairs_checked;
	uint32_t advisory_count;
	uint32_t warning_count;
	uint32_t frames_emitted;
};
void collision_get_stats(struct collision_stats *out);

/* Test helper exposed for `skywatch collision inject` — synthesises two
 * converging aircraft directly into the DB and starts an indefinite
 * keep-alive (re-upsert every 3 s) so they don't stale-evict on the GUI. */
int  collision_inject_scenario(void);

/* Stop the keep-alive started by `collision_inject_scenario()`. The
 * injected aircraft remain in the DB until the natural 10 s stale window
 * elapses, then fade like a lost real aircraft. */
int  collision_inject_stop(void);

/* Stage 8.6 — "ghost" test scaffold targeted at Will's mobile.
 * Scans the DB for the first SRC_BLE_SIM aircraft, then spawns a single
 * FICT aircraft (`gh05t1`) on a head-on intercept trajectory:
 *   - heading = (target_hdg + 180°) mod 360
 *   - speed   = 250 kt (fixed)
 *
 * Configurable:
 *   - tca_s   : seconds to closest approach. Determines initial range
 *               (range = closing_speed × tca_s). Pass <=0 for default 23 s.
 *   - miss_m  : perpendicular offset from perfect head-on so they pass at
 *               this minimum separation. 0 = direct hit. Pass <0 for 0.
 *   - alt_ft  : ghost altitude in feet. Pass <0 to default to the target's
 *               current altitude (so the test fires WARNING out of the
 *               box; pass an explicit alt to test the vertical-separation
 *               gate at COLLISION_VERTICAL_SEP_FT).
 *
 * A background worker dead-reckons the ghost position every 3 s so it
 * actually moves on the map. Returns -ENOENT if no BLE_SIM aircraft is in
 * the DB yet. */
int  collision_ghost_start(double tca_s, double miss_m, int alt_ft);

/* Same as `collision_ghost_start` but targets an EXPLICIT ICAO from the
 * DB (BLE_SIM, ADS-B, or any FICT entry). Lets the operator put a ghost
 * on an intercept with any aircraft, not just Will's BLE sim. Returns
 * -ENOENT if `target_icao` isn't currently in the DB. Pass target_icao
 * == NULL to fall back to the BLE-SIM auto-find behaviour. */
int  collision_ghost_start_at(const char *target_icao,
			      double tca_s, double miss_m, int alt_ft);

/* Stop the ghost worker. Ghost stale-evicts naturally after 10 s. */
int  collision_ghost_stop(void);

/* Stage 11 — manually trigger a CRASH event between two ICAOs (test path
 * for the GUI overlay + sim freeze without needing a real impact).
 * Returns -ENOENT if either ICAO isn't currently in the DB. */
int  collision_crash_trigger(const char *icao_a, const char *icao_b);

/* Stage 11 — manually send a diversion suggestion to whichever BLE_SIM
 * aircraft is in the DB. Returns -ENOENT if no SIM target, or the
 * ble_central_send_warning() result code on attempted send. */
int  collision_diversion_send(enum collision_diversion d);

/* Human-readable name for a diversion (LEFT/RIGHT/CLIMB/...). */
const char *collision_diversion_str(enum collision_diversion d);

#endif /* COLLISION_H */
