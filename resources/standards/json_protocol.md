# JSON Protocol — Python ↔ Controller, Controller → ATC GUI

**Version:** 1
**Source of truth (Python):** `json_protocol.py` in this folder
**Mirror (firmware C):** `vermillion-puck/controller/src/json_parser.c` (Stage 2.3)
**Transport:** USB-CDC ACM (one direction per port; see "Ports" below)
**Framing:** one JSON object per line, terminated by `\n` (LF). No CR.
**Encoding:** UTF-8.

## Ports

| Direction | Transport | What flows |
|---|---|---|
| `sdr_bridge.py` (laptop) → controller | USB-CDC, controller's **ACM1** | `AircraftFrame` from ADS-B |
| Controller → `atc_gui.py` (laptop) | USB-CDC, controller's **ACM1** (bi-directional, same port) | `AircraftFrame` snapshots + `CollisionFrame` events |
| Controller shell | USB-CDC, controller's **ACM0** | Zephyr shell I/O (not protocol traffic) |

## `AircraftFrame` schema

```json
{
  "type":   "aircraft",
  "icao":   "a1b2c3",
  "lat":    -27.4975,
  "lon":    153.0137,
  "alt":    3500,
  "vel":    450.0,
  "hdg":    90.0,
  "ts":     1715683200.0,
  "source": "ADS_B"
}
```

| Field | Type | Required | Notes |
|---|---|---|---|
| `type` | string literal `"aircraft"` | yes | discriminator (see also `CollisionFrame`) |
| `icao` | string, 1–7 chars | yes | 24-bit ICAO address, lowercase hex; leading `~` stripped |
| `lat` | float, `-90..90` | yes | WGS-84 degrees |
| `lon` | float, `-180..180` | yes | WGS-84 degrees |
| `ts` | float | yes | POSIX epoch seconds; sub-second OK |
| `source` | enum `"ADS_B"` \| `"BLE_SIM"` | yes | provenance — fusion key in `aircraft_db` |
| `alt` | int | no | feet MSL (baro); **omit key entirely** when unknown |
| `vel` | float | no | knots ground speed; **omit key entirely** when unknown |
| `hdg` | float | no | degrees true, 0..360; **omit key entirely** when unknown |
| `pred_lat` | float | no | Kalman 10 s-ahead projected latitude; emitted by controller once the per-aircraft KF has ≥3 updates. Always paired with `pred_lon`. |
| `pred_lon` | float | no | Kalman 10 s-ahead projected longitude; always paired with `pred_lat`. |

Records without lat/lon are **never emitted** (dropped at the bridge).

> **`null` is not a valid wire value for any field.** The firmware parser
> (`zephyr/data/json.h`) rejects `null` for typed fields and aborts the whole
> parse with `-EINVAL`. When `alt`/`vel`/`hdg` are unknown the key is left
> out; the firmware's parser-return bitmask tells it which optionals were
> present.

## `CollisionFrame` schema (Stage 8.2)

```json
{
  "type":      "collision",
  "icao_a":    "a1b2c3",
  "icao_b":    "d4e5f6",
  "level":     "WARNING",
  "tca_s":     12.4,
  "min_sep_m": 380.0,
  "ts":        1715683210.5
}
```

| Field | Type | Required | Notes |
|---|---|---|---|
| `type`      | string literal `"collision"` | yes | discriminator |
| `icao_a`    | string, 1–7 chars | yes | lexicographically-sorted **first** ICAO of the pair |
| `icao_b`    | string, 1–7 chars | yes | lexicographically-sorted **second** ICAO of the pair |
| `level`     | enum `"CLEAR"` \| `"ADVISORY"` \| `"WARNING"` \| `"CRASH"` | yes | see thresholds below |
| `tca_s`     | float | yes | seconds to closest approach (0 if already past) |
| `min_sep_m` | float | yes | projected minimum separation in metres |
| `alt_diff_ft` | int | no | absolute altitude gap in feet. Present only when both aircraft reported altitude. Drives the vertical-separation gate (>1000 ft = forced CLEAR) and is what the GUI shows next to `min_sep_m` so the divert-to-clear scenario is observable in real time. |
| `ts`        | float | yes | controller uptime seconds at detection |

**Thresholds** (mirrored in `scripts/stage8/collision_proto.py` and
`vermillion-puck/controller/src/collision/src/collision.h` — bump in lockstep):

| level    | min_sep | TCA / extra |
|---|---|---|
| CRASH    | < 50 m  | both alts within ~164 ft (50 m). Treated as a terminal event — controller emits one frame, GUI hides icons for 20 s (real) / 10 s (sim). |
| WARNING  | < 500 m  | 0–30 s |
| ADVISORY | < 1000 m | 0–60 s |
| CLEAR    | otherwise (also when pair is diverging or has infinite TCA) |

**Emission cadence (controller side):** the collision tick runs every
1 s. A frame is published on transition into ADVISORY/WARNING and on
every tick while WARNING-active. When a previously-flagged pair drops to
CLEAR, a one-shot `"CLEAR"` frame is emitted so the GUI can drop its red
flash. `(icao_a, icao_b)` is always emitted in lexicographic order so the
GUI can key on a single tuple without sorting.

## Versioning

`PROTOCOL_VERSION = 1` in `json_protocol.py`. If a field is added, removed,
renamed, or has its type changed, bump the version and update both the Python
module and this document **in the same commit**.
