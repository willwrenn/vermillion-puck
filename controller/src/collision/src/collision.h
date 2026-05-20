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

/* Vertical-separation gate. If both aircraft report altitude AND the
 * difference exceeds this, the pair is force-CLEAR regardless of
 * horizontal geometry. 1000 ft mirrors the standard ATC vertical
 * separation minimum below FL290. */
#define COLLISION_VERTICAL_SEP_FT  1000

#define COLLISION_TICK_MS          1000   /* 1 Hz */

enum collision_level {
	COLLISION_CLEAR    = 0,
	COLLISION_ADVISORY = 1,
	COLLISION_WARNING  = 2,
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

/* Stop the ghost worker. Ghost stale-evicts naturally after 10 s. */
int  collision_ghost_stop(void);

#endif /* COLLISION_H */
