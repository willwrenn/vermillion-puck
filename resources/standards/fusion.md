# Data Fusion in the Controller Node

This document describes how the Zephyr controller (`vermillion-puck/controller/`)
merges two heterogeneous tracking streams into a single, query-able aircraft
database. It's the contract the controller's dual-source data fusion path implements.

## Inputs

| Source | Transport | Format | Rate | Defined in |
|---|---|---|---|---|
| **ADS-B** (real aircraft, via dump1090 on laptop) | USB-CDC ACM1 (Rx into controller) | `AircraftFrame` JSON, `\n`-terminated | ~2 Hz × N aircraft | `json_protocol.md` |
| **BLE-Sim** (simulated aircraft from William's node) | BLE GATT notify | `ble_aircraft_frame` binary, 38 B | 5 Hz × N aircraft | `ble_protocol.md` |

Both arrive **at the same node** (the Xiao-nRF52840 controller) and both must
end up in the same in-memory store: `aircraft_db` (ported from the
CSSE4011 miniproject's `node_db.c/h`).

## Output

| Sink | Format | Cadence |
|---|---|---|
| ATC GUI (laptop) | `AircraftFrame` JSON on ACM1 Tx | 2 Hz, one frame per DB entry per tick |
| Collision detector | direct in-memory read | event-driven, on each `db_upsert` |
| MQTT | `AircraftFrame` JSON on `skywatch/aircraft` | 1 Hz, batched |

## Fusion model

The controller treats both transports as **observations of the same world**,
keyed by 24-bit ICAO address.

### Steps per incoming observation

1. **Decode** into a transient `struct aircraft_t` (the common in-memory shape).
   - JSON path: `json_parser.c` fills the struct from a JSON line.
   - BLE path:  `ble_central.c` notify callback `memcpy`s the
     38-byte packed frame, then expands into `aircraft_t` with the same
     `source` enum but value `SRC_BLE_SIM`.
2. **Validate** per the source spec (lat/lon range, ICAO ≤ 24-bit, etc.).
   Invalid frames are dropped silently and counted in a shell-visible stat.
3. **`db_upsert(rec)`** — `aircraft_db.c` looks up by `icao24`:
   - If absent: insert. `source` recorded.
   - If present: update fields **in place**, refresh `last_seen_ms`. Record
     `source` of the most recent observation (so a stale BLE record being
     refreshed by a fresh ADS-B record flips the entry to `ADS_B`).

### Conflict policy: "freshest observation wins"

When the same ICAO is reported by both sources within the stale window, the
**most recent observation overwrites** the prior fields — we do **not** blend.

Rationale:
- Blending two independent estimates needs covariance, which we don't have
  for the sim node's "ground-truth" packets. Picking the freshest is a clean
  invariant the Kalman filter can sit on top of.
- The Kalman filter still smooths position over time, so source-flapping in
  the raw stream is absorbed before it reaches the GUI/collision logic.
- The `source` field on each DB entry tells operators which transport last
  saw the aircraft — useful when debugging "why did that plane jump?"

### Stale removal

`aircraft_db.c` runs the same `db_remove_stale()` reaper as `node_db.c` did:
entries whose `last_seen_ms` is older than 10 s are evicted. The reaper is
source-agnostic — once an aircraft stops being observed by **any** source,
it falls out of the DB.

### Concurrency

Both the JSON parser thread and the BLE notify dispatcher call `db_upsert()`.
The single `k_mutex` in `aircraft_db.c` (kept verbatim from `node_db.c`)
serialises them. The collision detector reads under the same mutex.

## Test expectations (success criteria)

- With `sdr_bridge.py` and the sim node both running, `db dump` shell
  command shows entries with both `source: ADS_B` and `source: BLE_SIM`
  coexisting (no source field is lost on upsert).
- A 5-minute soak run produces zero hard faults and zero `ENOMEM` logs.
- Killing the sim node causes all `BLE_SIM` entries to disappear within the
  10 s stale threshold while `ADS_B` entries continue to update.
- Killing `sdr_bridge.py` does the inverse.
