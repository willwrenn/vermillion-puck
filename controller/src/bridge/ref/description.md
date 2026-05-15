# bridge

**Role:** Convert binary `struct ble_aircraft_frame` packets coming off
`ble/` into the common `struct aircraft_t`. Validation lives here, not in
`ble/`. Mirror of the `json/` lib's role for the ADS-B path.

**Files:** `src/bridge.{c,h}`

**Architecture:**

```
ble notify_cb ──┐
shell inject ───┴── bridge_submit_frame() ── k_msgq (8 slots × 38 B) ── bridge_thread
                                                                       │
                                                                       ▼
                                                       bridge_convert() ──▶ LOG_INF
                                                       (Stage 4.2 → db_upsert)
```

**Public API:**
- `int  bridge_init(void);` — clears stats; thread is auto-started via
  `K_THREAD_DEFINE`.
- `void bridge_submit_frame(const struct ble_aircraft_frame *);` —
  non-blocking; `k_msgq_put(K_NO_WAIT)`. Safe from any context (the BLE
  stack's BT-rx workqueue calls it).
- `int  bridge_convert(in, out);` — synchronous validator + struct
  copier. Used by the worker thread and by `skywatch ble inject`.
- `void bridge_get_stats(struct bridge_stats *);` — counters.

**Conversion (`ble_aircraft_frame` → `aircraft_t`):**

| BLE field | `aircraft_t` field | Conversion |
|---|---|---|
| `source`            | `source`         | `BLE_SIM` → `SRC_BLE_SIM`, `ADS_B` → `SRC_ADS_B` |
| `icao24` (uint32)   | `icao` (string)  | `snprintf("%06x", icao24)` — 6 lowercase hex |
| `lat` (double)      | `lat` (double)   | direct, after range check `[-90, 90]` |
| `lon` (double)      | `lon` (double)   | direct, after range check `[-180, 180]` |
| `ts_ms` (uint32)    | `ts` (double)    | `ts_ms / 1000.0` (sim-node uptime seconds; wall-clock translation can happen later) |
| `alt_ft` (int16)    | `alt_ft` (int32) | only if `flags & BLE_FLAG_ALT_VALID`; sets `AIRCRAFT_VALID_ALT` |
| `vel_kt` (int16)    | `vel_kt` (double)| only if `flags & BLE_FLAG_VEL_VALID`; sets `AIRCRAFT_VALID_VEL` |
| `hdg_deg` (int16)   | `hdg_deg` (double)| only if `flags & BLE_FLAG_HDG_VALID`; sets `AIRCRAFT_VALID_HDG` |

**Validation drops (counted, never crash):**
`rej_source` (source ∉ {0,1}), `rej_icao` (high byte nonzero),
`rej_latlon` (lat/lon out of WGS-84 range).

**Counters** (via `skywatch ble stats`):
`submitted`, `drops` (msgq full), `converted`, `rej_source`, `rej_icao`,
`rej_latlon`.

**Sizing:** 8-slot msgq × 38 B = 304 B. At the sim's 5 Hz cadence this is
1.6 s of headroom; far more than the consumer thread needs.

**Plan stage mapped here:** 3.3 (and feeds 4.2 once `aircraft_db` exists).
