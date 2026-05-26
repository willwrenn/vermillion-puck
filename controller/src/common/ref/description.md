# common

**Role:** Shared domain types used by ≥2 libs. Header-only.

**Files:** `src/aircraft_t.h`, `src/ble_packet.h`

**`aircraft_t.h`:**

- `struct aircraft_t` — the in-memory shape used by every observation
  source (ADS-B JSON, BLE binary) and every consumer (DB, Kalman,
  collision, GUI serialiser). Field naming mirrors
  `resources/standards/json_protocol.py`.
- `enum aircraft_source` — `SRC_UNKNOWN / SRC_ADS_B / SRC_BLE_SIM /
  SRC_FICT` (the FICT enum value is for synthetic test traffic from
  `skywatch collision inject / ghost`).
- `AIRCRAFT_VALID_*` flags — bitfield used in `aircraft_t.valid_mask`
  to track which optional fields the observation actually carried
  (`AIRCRAFT_VALID_ALT / _VEL / _HDG / _PRED`). `_PRED` is set once
  the Kalman filter has had ≥3 updates for this aircraft and
  `pred_lat / pred_lon` are meaningful.

**`ble_packet.h`:**

- Both BLE frame definitions: the 38-byte `ble_aircraft_frame`
  (sim → controller NOTIFY) and the 16-byte `ble_warning_frame`
  (controller → sim WRITE, with `msg_type` 0x02 collision / 0x03
  diversion / 0x04 free-text / 0x05 crash). Mirrored on the sim
  side in `sim/mobile/src/common/src/ble_packet.h` — change one,
  change both, bump `BLE_PROTOCOL_VERSION`.
- Inline XOR-CRC helper (`ble_warning_crc`) over the first 15 bytes
  of the warning frame.
- Custom 128-bit service UUIDs for both directions.

**Do not add:**

- Function implementations (this is a header-only lib).
- Types used by only one lib — keep those private to that lib's
  `src/`.
