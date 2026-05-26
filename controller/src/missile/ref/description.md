# missile

**Role:** Pure-pursuit guided-missile sandbox. Operator launches a
synthetic FICT aircraft (ICAO `m1s51l`) from UQ St Lucia at the BLE
sim; the missile dead-reckons in a local metric frame, recomputes
its heading every tick toward the current sim position, clamps the
turn to ±`turn_rate × dt`, and feeds its position back into
`aircraft_db` so the collision detector treats it like any other
aircraft and fires the WARNING → CRASH chain.

**Files:** `src/missile.{c,h}`

**Guidance math:**

```
bearing_to_target = atan2(lon_target − lon_self, lat_target − lat_self)
                    (in the local metric ENU frame, not great-circle)
desired_hdg       = bearing_to_target
hdg_step          = clamp(desired_hdg − cur_hdg, ±turn_rate × dt)
new_hdg           = cur_hdg + hdg_step
new_pos           = cur_pos + speed × dt × unit_vec(new_hdg)
```

Dead-reckoning lives in a local ENU frame centred on St Lucia so we
don't accumulate great-circle error over the missile's short
lifetime (≤60 s default).

**Public API:**

- `int  missile_launch(double ttl_s, double turn_rate_dps, double speed_kt);`
  — defaults `60 s / 15°/s / 500 kt`. Returns `-EALREADY` if a
  missile is already in flight (one at a time).
- `int  missile_cancel(void);` — abort the in-flight missile.
- `void missile_get_stats(struct missile_stats *out);` — ticks,
  current bearing, distance to target.

**Shell:**

```
skywatch missile launch [ttl_s] [turn_rate_dps] [speed_kt]
skywatch missile cancel
```

**Difficulty knobs:**

| Slow demo   | `skywatch missile launch 60 8 250`   |
|-------------|---------------------------------------|
| Default     | `skywatch missile launch 60 15 500`   |
| Hard mode   | `skywatch missile launch 30 25 750`   |

Hard mode is what justified the predictive-CRASH branch +
the 5 Hz collision tick — 750 kt closing covers ~77 m per 200 ms
and would otherwise pass through the 100 m CRASH zone between
ticks.

**Termination:**

- `ttl_s` elapsed without a CRASH-level encounter → missile
  removed from the DB silently (a "miss").
- CRASH classification fires in `collision.c` → that module's
  CRASH lock keeps the pair sticky for 15 s; the missile is
  cancelled.
