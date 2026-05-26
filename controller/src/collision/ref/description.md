# collision

**Role:** Pairwise closest-approach collision detector. Runs over
every pair of aircraft in `aircraft_db` at **5 Hz**, classifies each
pair into CLEAR / ADVISORY / WARNING / CRASH, emits a
`CollisionFrame` JSON on level transitions and periodically while
non-CLEAR, and sends a 16-byte BLE warning frame back to the sim
mobile on every WARNING transition. Also owns the missile / ghost
synthetic-aircraft sandboxes and the geometry-based diversion picker.

**Files:** `src/collision.{c,h}`

**Algorithm:** linear constant-velocity model.

```
Δr  = r_a − r_b               (relative position, metres)
Δv  = v_a − v_b               (relative velocity, m/s)
t* = − (Δr · Δv) / |Δv|² (time to closest approach, s)
min_sep = | Δr + Δv · t* | (min separation at t*, m)
```

If `Δv ≈ 0` or `t* < 0` (diverging) the pair is reported with
`tca = 0` and `min_sep = current_distance`.

**Classification:**

| Level     | Trigger                                                       |
|-----------|---------------------------------------------------------------|
| CLEAR     | None of the below                                             |
| ADVISORY  | `tca ≤ 60 s AND min_sep < 1000 m`                             |
| WARNING   | `tca ≤ 30 s AND min_sep < 500 m`                              |
| CRASH     | `(current_horiz < 100 m) OR (predictive branch — see below)`, AND (no alt info OR `\|alt_a − alt_b\| < 100 m`) |

**Vertical-separation gate:** if both aircraft have a reported
altitude and `\|alt_diff_ft\|` exceeds `COLLISION_VERTICAL_SEP_FT`
(1000 ft — the ICAO sep minimum below FL290), the pair is **forced
to CLEAR** regardless of horizontal geometry.

**Predictive-CRASH branch:** ORs in `min_sep < 100 m AND \|tca\| <
0.5 s AND !diverging AND current_horiz < 300 m`. Catches fast
missiles passing through the 100 m CRASH zone between ticks — at
750 kt the missile covers ~77 m per 200 ms, fast enough that the
naive "current distance < 100 m" check would miss the closest
moment.

**CRASH lock:** on the transition into CRASH a per-pair
`crash_until_ms = now + 15000` is set; the pair stays CRASH for 15 s
regardless of subsequent geometry. Prevents oscillation as the
missile flies past a frozen sim.

**Diversion picker:** on every WARNING transition (only if the pair
involves the BLE sim) `pick_diversion()` selects one of:

| Choice    | Geometry trigger                              |
|-----------|-----------------------------------------------|
| `CLIMB`   | head-on (relative bearing within ±30° of own heading) OR vertical conflict where the other is below |
| `DESCEND` | vertical conflict where the other is above  |
| `LEFT`    | other aircraft to the right                  |
| `RIGHT`   | other aircraft to the left                   |
| `RTB`     | head-on, no altitude separation possible     |
| `HOLD`    | ambiguous fallback (rare)                    |

The choice is encoded into the BLE warning frame's `hdg_delta`
(±30 for LEFT/RIGHT, ±127 sentinel for RTB/HOLD) and `alt` field
(repurposed as a signed `alt_delta` in feet for CLIMB/DESCEND).

**Public API:**

- `int  collision_init(void);` — starts the 200 ms work-queue tick.
- `void collision_get_stats(struct collision_stats *out);` —
  `ticks / pairs_checked / advisory_count / warning_count /
  frames_emitted`.
- `int  collision_inject_scenario(void);` — synthetic head-on FICT
  pair (`c011a1` / `c011b2`); keep-alive worker re-upserts every 3 s
  so they don't stale-evict.
- `int  collision_inject_stop(void);`
- `int  collision_ghost_start(double tca, double miss, int alt);` —
  spawn a FICT aircraft on intercept with the first BLE sim in the
  DB.
- `int  collision_ghost_start_at(const char *icao, double tca, double miss, int alt);` —
  same but target any ICAO.
- `int  collision_ghost_stop(void);`
- `int  collision_crash_trigger(const char *a, const char *b);` —
  force-fire a CRASH event between two existing ICAOs (testing).
- `int  collision_diversion_send(enum collision_diversion d);` —
  manually fire a diversion suggestion to the sim.

**Wire format:** `CollisionFrame` schema in
`resources/protocol_reference.md` §3.2 and
`resources/standards/json_protocol.py` (`validate_collision`).
