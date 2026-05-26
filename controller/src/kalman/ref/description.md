# kalman

**Role:** Per-aircraft 4-D constant-velocity Kalman filter
(`[lat, lon, v_lat, v_lon]` state). Smooths trajectory observations
and produces a 10 s predict-ahead position for the GUI's projection
arrow. One Kalman state lives inside each `aircraft_db` entry, not
in a parallel structure.

**Files:** `src/kalman.{c,h}`

**Public API:**

- `void kalman_init(kalman_t *k, double lat0, double lon0, double t0);`
  — seed with the first observation; state vector at rest.
- `void kalman_predict(kalman_t *k, double t_now);` — propagate the
  4-D state forward by `dt = t_now - k->t`.
- `void kalman_update(kalman_t *k, double lat_obs, double lon_obs);`
  — fuse the observation into the state via the Kalman gain. Uses
  a closed-form 2×2 matrix inversion (we don't link a generic
  linear-algebra library — the inversion is hand-written).
- `void kalman_predict_ahead(const kalman_t *k, double horizon_s, double *pred_lat, double *pred_lon);`
  — project the current state forward by `horizon_s` (10 s in the
  shipped GUI) without mutating the filter.
- `int  kalman_bench(int iters);` — shell hook
  (`skywatch kalman bench`); runs `iters` predict+update cycles and
  reports the elapsed µs. Used to verify the inversion stays under
  the 5 ms/1000-iter budget after any change.

**Process / measurement noise:**

- Process noise `Q` is constructed per-axis because
  metres-per-degree differs on lat vs lon at Brisbane latitude
  (`cos(lat × π/180)` factor on the longitude term).
- Measurement noise `R` is a scalar metres value, configurable from
  the shell (`skywatch kalman r <m>`). Default 10 m.
- Both Q and R can be re-tuned at runtime via the shell — useful
  for sweeping the GUI smoothing behaviour on noisy ADS-B feeds.

**Lifecycle:**

1. First `aircraft_db_upsert()` for a new ICAO calls
   `kalman_init()` with the observation's lat/lon/ts.
2. Every subsequent upsert calls `kalman_predict()` (advance to
   `ts_now`) then `kalman_update()` (fuse the new observation).
3. After ≥3 updates the entry's `valid_mask` gets
   `AIRCRAFT_VALID_PRED` set and `pred_lat / pred_lon` are written
   from `kalman_predict_ahead(k, 10.0, ...)`. The DB publisher
   serialises these fields as `pred_lat` / `pred_lon` on the wire.
