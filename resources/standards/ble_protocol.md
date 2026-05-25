# BLE Internode Binary Protocol — Sim Node ↔ Controller

**Version:** 1 — confirmed live (sim node + controller). Lock until both sides bump in lockstep.
**Source of truth (Python):** `ble_protocol.py` in this folder
**Mirror (firmware C):** `vermillion-puck/controller/src/common/src/ble_packet.h` (single header included by both `ble/` and `bridge/`); also embedded in William's sim node (`resources/importedcode/Will_BLE/mobile/codein.c`).
**Transport:** BLE GATT characteristic (NOTIFY from sim → controller on the SkyWatch aircraft characteristic; WRITE from controller → sim on the legacy Warning service for Stage 8.3 — different protocol, see Will's `warning_frame`).
**Service UUID:** `4d8a4011-7f24-43c4-9c5b-a17c7af70001` (primary, advertised by sim node and in scan response).
**Aircraft char UUID:** `4d8a4011-7f24-43c4-9c5b-a17c7af70002` (NOTIFY).
**Sim node device name:** `skywatch-sim`.
**MTU requirement:** Default ATT MTU = 23 bytes (20 payload). Frame is **38 bytes** → MTU exchange to ≥ 41 is mandatory before any notify.
**Endianness:** Little-endian throughout (ARM Cortex-M native). No byte-swap on either side.
**Cadence:** 5 Hz (sim node sends every 200 ms).

## Byte map (38 bytes total)

```
offset  size  field        type         notes
 0      1     source       uint8        0=ADS_B, 1=BLE_SIM
 1      1     flags        uint8        bit0 on_ground, bit1 alt_valid,
                                        bit2 vel_valid, bit3 hdg_valid
 2      4     icao24       uint32 LE    24-bit ICAO in low 24 bits; top byte = 0
 6      4     -- pad --                 zero; aligns lat/lon to 8-byte boundary
10      8     lat          float64 LE   WGS-84 degrees
18      8     lon          float64 LE   WGS-84 degrees
26      2     alt_ft       int16  LE    feet MSL; ignore if !alt_valid
28      2     vel_kt       int16  LE    knots ground speed; ignore if !vel_valid
30      2     hdg_deg      int16  LE    degrees true 0..359; ignore if !hdg_valid
32      1     reserved     uint8        0
33      1     -- pad --                 alignment to next uint32
34      4     ts_ms        uint32 LE    ms since sim-node boot; controller
                                        converts to wall-clock on receipt
```

`struct.calcsize("<BBI4xddhhhBxI")` → **38**. A C `__packed` struct with the
fields in this exact order produces the same layout on Cortex-M.

## Equivalent C struct

```c
#include <stdint.h>

enum aircraft_source { SRC_ADS_B = 0, SRC_BLE_SIM = 1 };

#define FLAG_ON_GROUND  (1u << 0)
#define FLAG_ALT_VALID  (1u << 1)
#define FLAG_VEL_VALID  (1u << 2)
#define FLAG_HDG_VALID  (1u << 3)

struct __packed ble_aircraft_frame {
    uint8_t  source;
    uint8_t  flags;
    uint32_t icao24;      /* low 24 bits */
    uint8_t  _pad0[4];
    double   lat;
    double   lon;
    int16_t  alt_ft;
    int16_t  vel_kt;
    int16_t  hdg_deg;
    uint8_t  reserved;
    uint8_t  _pad1[1];
    uint32_t ts_ms;
};
_Static_assert(sizeof(struct ble_aircraft_frame) == 38, "BLE frame must be 38B");
```

## Validation rules (apply on receipt, both sides)

- `source` ∈ {0, 1}; reject otherwise.
- `icao24` ≤ `0xFFFFFF`; the top byte of the uint32 must be zero.
- `lat` ∈ [-90, 90]; `lon` ∈ [-180, 180]; reject otherwise.
- If a `*_valid` flag is clear, the corresponding numeric field is ignored
  (don't propagate stale data into `aircraft_db`).
- `hdg_deg` is reduced mod 360 by the sender; receiver may assert 0..359.

## Versioning

`PROTOCOL_VERSION = 1` in `ble_protocol.py`. Any change to the byte map,
field semantics, or flag definitions must bump the version and update **both**
files in the same commit. The firmware `__packed` struct lives in
`vermillion-puck/controller/src/ble_central.h` (Stage 3.1) and must match this
file exactly.
