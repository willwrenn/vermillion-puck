"""
SkyWatch JSON wire-protocol — Python source of truth.

Used by:   sdr_bridge.py, atc_gui.py, tests. The controller firmware's
           json_parser.c mirrors this schema via zephyr/data/json.h
           descriptors.
Purpose:   Defines the AircraftFrame schema, dump1090 -> AircraftFrame
           normalisation, and a tiny runtime validator. One AircraftFrame
           per newline-terminated JSON object on the wire.
Notes:
    - Lat/lon are *floats* in JSON (not scaled integers). The 64-bit double
      representation is preserved end-to-end on the Python side; the firmware
      decodes to its own `double lat, lon;` fields.
    - Optional fields (alt, vel, hdg) may be null when dump1090 hasn't
      determined them yet. Consumers must handle null gracefully.
    - Bump PROTOCOL_VERSION whenever you change a field name, type, or
      add/remove a key. Firmware parser cares about field *names* only.
"""

from __future__ import annotations

from dataclasses import dataclass, asdict
from typing import Optional, Literal, TypedDict

PROTOCOL_VERSION = 1

Source = Literal["ADS_B", "BLE_SIM"]


class AircraftFrame(TypedDict, total=False):
    """One aircraft observation as it appears on the wire (JSON).

    Optional fields (alt, vel, hdg) are **omitted** from the JSON when not
    known — they are NEVER serialised as `null`. The firmware's Zephyr JSON
    parser rejects null-for-typed-field; absent keys are handled cleanly by
    the descriptor return bitmask.
    """
    # Required on every frame:
    type: Literal["aircraft"]
    icao: str          # 24-bit ICAO, lowercase 6 hex chars, no '~'
    lat: float         # WGS-84 degrees, -90..90
    lon: float         # WGS-84 degrees, -180..180
    ts: float          # POSIX epoch seconds, float (sub-second OK)
    source: Source     # provenance — fusion key on the controller
    # Optional (omit key entirely when unknown):
    alt: int           # feet, MSL (baro)
    vel: float         # ground speed in knots
    hdg: float         # track in degrees true, 0..360
    # Kalman 10 s-ahead projection — emitted once the controller's filter has
    # had >=3 updates for this aircraft. Always emitted together.
    pred_lat: float
    pred_lon: float


# --------------------------------------------------------------------------- #
# Optional helper: a frozen dataclass mirror for places that want attributes
# rather than dict access. Round-trips with asdict() / **mapping.
# --------------------------------------------------------------------------- #

@dataclass(frozen=True)
class AircraftRecord:
    icao: str
    lat: float
    lon: float
    alt: Optional[int]
    vel: Optional[float]
    hdg: Optional[float]
    ts: float
    source: Source
    type: str = "aircraft"

    def to_frame(self) -> AircraftFrame:
        return asdict(self)  # type: ignore[return-value]


# --------------------------------------------------------------------------- #
# dump1090 aircraft.json -> AircraftFrame
# --------------------------------------------------------------------------- #

def normalize_dump1090(ac: dict, ts: float) -> Optional[AircraftFrame]:
    """Map one dump1090 record into an AircraftFrame.

    Returns None if the record has no lat/lon (we can't place it on the map).
    Optional alt/vel/hdg keys are **omitted** from the result when their
    source fields are absent or None in the dump1090 record.
    """
    if "lat" not in ac or "lon" not in ac:
        return None
    icao_raw = ac.get("hex", "") or ""
    icao = icao_raw.lstrip("~").lower()
    out: AircraftFrame = {
        "type": "aircraft",
        "icao": icao,
        "lat": float(ac["lat"]),
        "lon": float(ac["lon"]),
        "ts": ts,
        "source": "ADS_B",
    }
    alt = ac.get("altitude", ac.get("alt_baro"))
    if alt is not None:
        out["alt"] = alt
    vel = ac.get("speed", ac.get("gs"))
    if vel is not None:
        out["vel"] = vel
    hdg = ac.get("track")
    if hdg is not None:
        out["hdg"] = hdg
    return out


# --------------------------------------------------------------------------- #
# Validator — used by tests and at integration boundaries.
# --------------------------------------------------------------------------- #

class ProtocolError(ValueError):
    pass


_REQUIRED_KEYS = {"type", "icao", "lat", "lon", "ts", "source"}
_OPTIONAL_KEYS = {"alt", "vel", "hdg", "pred_lat", "pred_lon"}
_ALLOWED_SOURCES = {"ADS_B", "BLE_SIM", "FICT"}

# Second frame type on the same ACM1 stream (alongside the `aircraft` frame).
_COLLISION_REQUIRED_KEYS = {"type", "icao_a", "icao_b", "level", "tca_s", "min_sep_m", "ts"}
_ALLOWED_COLLISION_LEVELS = {"CLEAR", "ADVISORY", "WARNING", "CRASH"}
# Controller adds an optional "diversion" field to every JSON frame for the
# encounter while a diversion has been suggested. The GUI banner shows it so
# the operator sees the controller's recommendation.
_ALLOWED_DIVERSIONS = {"LEFT", "RIGHT", "CLIMB", "DESCEND", "RTB", "HOLD"}


def validate(frame: dict) -> AircraftFrame:
    """Raise ProtocolError if frame is not a valid AircraftFrame; else return it.

    Optional alt/vel/hdg may be **absent**; if present they must be numeric.
    `null` is *not* a valid wire value for these fields — drop the key instead.

    Also dispatches `type=="collision"` to validate_collision() — that path
    returns the frame as-is so callers can branch on `frame["type"]` later.
    """
    if frame.get("type") == "collision":
        return validate_collision(frame)
    missing = _REQUIRED_KEYS - frame.keys()
    if missing:
        raise ProtocolError(f"missing required keys: {sorted(missing)}")
    if frame["type"] != "aircraft":
        raise ProtocolError(f"type must be 'aircraft' or 'collision', got {frame['type']!r}")
    icao = frame["icao"]
    if not isinstance(icao, str) or len(icao) == 0 or len(icao) > 7:
        raise ProtocolError(f"icao must be 1..7 char hex string, got {icao!r}")
    for k in ("lat", "lon", "ts"):
        if not isinstance(frame[k], (int, float)):
            raise ProtocolError(f"{k} must be numeric, got {type(frame[k]).__name__}")
    if not (-90.0 <= float(frame["lat"]) <= 90.0):
        raise ProtocolError(f"lat out of range: {frame['lat']}")
    if not (-180.0 <= float(frame["lon"]) <= 180.0):
        raise ProtocolError(f"lon out of range: {frame['lon']}")
    for k in _OPTIONAL_KEYS:
        if k in frame:
            v = frame[k]
            if not isinstance(v, (int, float)) or isinstance(v, bool):
                raise ProtocolError(f"{k} must be numeric if present, got {type(v).__name__}")
    if frame["source"] not in _ALLOWED_SOURCES:
        raise ProtocolError(f"source must be one of {_ALLOWED_SOURCES}, got {frame['source']!r}")
    return frame  # type: ignore[return-value]


def validate_collision(frame: dict) -> dict:
    """Validate a `CollisionFrame` (the `type=="collision"` discriminator)."""
    missing = _COLLISION_REQUIRED_KEYS - frame.keys()
    if missing:
        raise ProtocolError(f"collision: missing keys: {sorted(missing)}")
    for k in ("icao_a", "icao_b"):
        v = frame[k]
        if not isinstance(v, str) or not (1 <= len(v) <= 7):
            raise ProtocolError(f"collision: {k} must be 1..7 char hex, got {v!r}")
    if frame["level"] not in _ALLOWED_COLLISION_LEVELS:
        raise ProtocolError(
            f"collision: level must be one of {_ALLOWED_COLLISION_LEVELS}, got {frame['level']!r}"
        )
    for k in ("tca_s", "min_sep_m", "ts"):
        if not isinstance(frame[k], (int, float)):
            raise ProtocolError(
                f"collision: {k} must be numeric, got {type(frame[k]).__name__}"
            )
    # Optional `actual_sep_m`: CURRENT Euclidean separation (m) at emit
    # time. The history dashboard minimises over this to show how close
    # the aircraft actually got, not the projected min.
    if "actual_sep_m" in frame and not isinstance(frame["actual_sep_m"], (int, float)):
        raise ProtocolError(
            f"collision: actual_sep_m must be numeric, got {type(frame['actual_sep_m']).__name__}"
        )
    div = frame.get("diversion")
    if div is not None and div not in _ALLOWED_DIVERSIONS:
        raise ProtocolError(
            f"collision: diversion must be one of {_ALLOWED_DIVERSIONS}, got {div!r}"
        )
    return frame
