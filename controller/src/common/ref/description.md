# common

**Role:** Shared domain types used by ≥2 libs. Header-only.

**Files:** `src/aircraft_t.h`

**What's here:**
- `struct aircraft_t` — the in-memory shape used by every observation source
  (ADS-B JSON, BLE binary) and every consumer (DB, Kalman, collision, GUI
  serialiser). Field naming mirrors `resources/standards/json_protocol.md`.
- `enum aircraft_source` — `SRC_ADS_B`, `SRC_BLE_SIM`, etc.
- `AIRCRAFT_VALID_*` flags — bitfield used in `aircraft_t.valid_mask` to track
  which optional fields the observation actually carried.

**Do not add:**
- Function implementations (this is a header-only lib).
- Types used by only one lib — keep those private to that lib's `src/`.
