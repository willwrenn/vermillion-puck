# aircraft_db

**Role:** In-memory aircraft database. Single source of truth for the current
state of every tracked aircraft, regardless of which transport observed it
(ADS-B JSON over USB or BLE binary over GATT). Stage 5+ consumers
(GUI serialiser, Kalman, collision, MQTT) read from here.

**Files:** `src/aircraft_db.{c,h}`

**Layout:** static pool of `AIRCRAFT_DB_MAX_ENTRIES = 32`
`struct aircraft_db_entry { sys_snode_t; aircraft_t; int64_t last_seen_ms; }`
threaded into a `sys_slist_t`, serialised by one `k_mutex`. Lifted from the
miniproject `node_db.c` pattern.

**Public API:**
- `void aircraft_db_init(void)` — zero pool + slist + stats. Reaper auto-starts
  via `SYS_INIT(APPLICATION, 90)`.
- `int  aircraft_db_upsert(const aircraft_t *src)` — 1 = inserted,
  0 = updated in place, `-ENOMEM` when the pool is full. Holds the mutex.
- `int  aircraft_db_lookup(const char *icao, aircraft_t *out)` — `0` found,
  `-ENOENT` missing. Copies, does not return a pointer (callers don't share
  storage with the DB).
- `void aircraft_db_clear(void)` — wipes the list, resets stats.
- `int  aircraft_db_count(void)` — current entry count.
- `void aircraft_db_for_each(cb, user)` — walk under the mutex. **Callbacks
  run with the mutex held — keep them short, no blocking, no further DB
  calls.**
- `void aircraft_db_get_stats(struct aircraft_db_stats *)` — counters
  (`inserts / updates / stale_evicts / pool_full`).

**Fusion policy:** freshest-observation-wins. When ADS-B and BLE both report
the same ICAO, whichever upsert lands last overwrites all fields (including
`source`) and refreshes `last_seen_ms`. We do **not** blend (see
`resources/standards/fusion.md`). The Kalman filter (Stage 7) will smooth
over time on top, so source-flapping is absorbed before it reaches the GUI
or the collision detector.

**Stale removal:** dedicated reaper thread (`db_reaper`, prio 8) wakes every
2 s, scans the list under the mutex, and evicts entries whose
`last_seen_ms` is older than `AIRCRAFT_DB_STALE_MS = 10000` ms. Eviction is
source-agnostic — once nothing's been seeing an aircraft, it falls out.

**Producers (Stage 4.2):**
- `usb_handler/usb_handler.c` calls upsert after `json_parse_aircraft`.
- `bridge/bridge.c` calls upsert after `bridge_convert`.

**Consumers (planned):**
- `usb_serial` periodic worker (Stage 5.1) walks the DB at 2 Hz and emits one
  `AircraftFrame` JSON per entry on ACM1.
- Kalman lib (Stage 7.2) on each upsert; mutates `pred_lat / pred_lon`
  fields on the entry in place.
- Collision detector (Stage 8.1).
- MQTT publisher (Stage 9.2).

**Plan stages mapped here:** 4.1, 4.2, 4.3 (and consumed by every later
controller-side stage).
