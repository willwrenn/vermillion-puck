"""
SkyWatch ATC GUI — InfluxDB Cloud writer (Phase 10.3).

Writes CollisionFrame events as time-series points to InfluxDB Cloud so
the built-in dashboard ("Active alerts" table, min-sep over time chart)
has data to render. Mirrors the miniproject's `_InfluxWriter` pattern
(stage2_map.py:20-50): queue + daemon thread so the UI never blocks on
a slow HTTPS round-trip to AWS.

Measurement schema:
    measurement: skywatch_events
    tags:
        pair       — "{icao_a}_{icao_b}" (already lexicographically sorted)
        level      — "CLEAR" | "ADVISORY" | "WARNING"
        icao_a     — first ICAO (lex-sorted)
        icao_b     — second ICAO
    fields:
        tca_s        : float  — seconds to closest approach
        min_sep_m    : float  — projected minimum separation in metres
        alt_diff_ft  : int    — |alt_a - alt_b|, present if both reported
        controller_ts: float  — controller uptime ts from the wire frame

The `pair` + `level` tags make the dashboard query trivial:
    `from(bucket: "skywatch") |> range(start: -10m)
        |> filter(fn: (r) => r._measurement == "skywatch_events"
                          and r.level == "WARNING")`

Credentials: the same token + org used by the miniproject works for the
final project — InfluxDB Cloud free tier supports multiple buckets per
org. Set the bucket via env var INFLUXDB_BUCKET (default "skywatch") so
miniproject data stays separate.
"""

from __future__ import annotations

import os
import queue
import sys
import threading
import time
from dataclasses import dataclass
from typing import Optional

try:
    from influxdb_client import InfluxDBClient, Point
    from influxdb_client.client.write_api import SYNCHRONOUS
except ImportError:                                  # pragma: no cover
    InfluxDBClient = None                             # noqa: N816
    Point          = None
    SYNCHRONOUS    = None


# Defaults match miniproject's run_dashboard.sh. Override via env vars
# so we don't bake the token into source.
DEFAULT_URL    = "https://us-east-1-1.aws.cloud2.influxdata.com"
DEFAULT_ORG    = "Blaise&Will.Incorporated"
DEFAULT_BUCKET = "skywatch"
DEFAULT_MEAS   = "skywatch_events"
QUEUE_MAX      = 256
HEARTBEAT_INTERVAL_S = 10.0    # how often write_heartbeat() fires on its own


@dataclass
class InfluxStats:
    written:    int = 0
    dropped:    int = 0
    write_errs: int = 0
    last_error: str = ""


class InfluxWriter:
    """Background-thread InfluxDB Cloud writer for CollisionFrame events.

    Use:
        wr = InfluxWriter()      # picks up env: INFLUXDB_URL/TOKEN/ORG/BUCKET
        if wr.start():
            wr.write_collision(frame_dict)   # non-blocking
        wr.stop()
    """

    def __init__(self,
                 url:    Optional[str] = None,
                 token:  Optional[str] = None,
                 org:    Optional[str] = None,
                 bucket: Optional[str] = None):
        self.url    = url    or os.environ.get("INFLUXDB_URL",    DEFAULT_URL)
        self.token  = token  or os.environ.get("INFLUXDB_TOKEN")
        self.org    = org    or os.environ.get("INFLUXDB_ORG",    DEFAULT_ORG)
        self.bucket = bucket or os.environ.get("INFLUXDB_BUCKET", DEFAULT_BUCKET)
        self.stats  = InfluxStats()
        self._q: "queue.Queue[Optional[Point]]" = queue.Queue(QUEUE_MAX)
        self._client: Optional["InfluxDBClient"] = None
        self._write_api = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()

    # ---- lifecycle --------------------------------------------------- #

    def start(self) -> bool:
        """Open the InfluxDB client + start the worker. Returns False
        if the lib isn't installed or no token is set (graceful degrade
        so the GUI keeps working without InfluxDB)."""
        if InfluxDBClient is None:
            self.stats.last_error = "influxdb-client not installed"
            return False
        if not self.token:
            self.stats.last_error = "INFLUXDB_TOKEN not set"
            return False
        try:
            self._client    = InfluxDBClient(url=self.url, token=self.token,
                                             org=self.org)
            self._write_api = self._client.write_api(write_options=SYNCHRONOUS)
        except Exception as e:                       # noqa: BLE001
            self.stats.last_error = f"client init: {e}"
            return False
        self._thread = threading.Thread(target=self._worker, name="influx-writer",
                                        daemon=True)
        self._thread.start()
        # Fire the first heartbeat NOW so the bucket has data the moment
        # the dashboard opens — no need to wait for the first collision.
        # Then a background ticker keeps it warm at HEARTBEAT_INTERVAL_S.
        self.write_heartbeat("startup")
        self._hb_thread = threading.Thread(target=self._heartbeat_loop,
                                           name="influx-heartbeat",
                                           daemon=True)
        self._hb_thread.start()
        return True

    def stop(self):
        self._stop.set()
        # Sentinel wakes the worker for clean shutdown.
        try:
            self._q.put_nowait(None)
        except queue.Full:
            pass
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        if self._client is not None:
            try:
                self._client.close()
            except Exception:
                pass

    # ---- caller-facing API ------------------------------------------ #

    def write_heartbeat(self, label: str = "alive") -> None:
        """Emit a status Point so the dashboard always has at least one
        fresh datapoint to render, even before any collision events fire.
        Called once at start() and periodically by the background timer
        so the bucket's 'most recent data' indicator stays current.

        Heartbeat lives in a separate measurement (`skywatch_status`)
        so it doesn't pollute the collision-events queries — the Active
        Alerts panel filters on `skywatch_events` and ignores these."""
        if Point is None:
            return
        p = (Point("skywatch_status")
             .tag("source", "gui")
             .tag("label",  label)
             .field("alive", 1)
             .time(time.time_ns()))
        try:
            self._q.put_nowait(p)
        except queue.Full:
            try:
                self._q.get_nowait()
            except queue.Empty:
                pass
            try:
                self._q.put_nowait(p)
            except queue.Full:
                self.stats.dropped += 1

    def write_collision(self, frame: dict) -> None:
        """Enqueue a CollisionFrame as a Point. Non-blocking; drops the
        oldest entry if the queue is full."""
        if Point is None:
            return
        a = frame.get("icao_a", "?")
        b = frame.get("icao_b", "?")
        pair = f"{a}_{b}"
        level = frame.get("level", "CLEAR")
        # InfluxDB Cloud charges by writes — sending the timestamp in ns
        # so the dashboard's time-aggregation buckets line up correctly
        # (the controller's `ts` is uptime-seconds; we use wall-clock for
        # ingest because that's what makes dashboards meaningful).
        p = (Point(DEFAULT_MEAS)
             .tag("pair",   pair)
             .tag("level",  level)
             .tag("icao_a", a)
             .tag("icao_b", b)
             .field("tca_s",         float(frame.get("tca_s", 0.0)))
             .field("min_sep_m",     float(frame.get("min_sep_m", 0.0)))
             .field("controller_ts", float(frame.get("ts", 0.0)))
             .time(time.time_ns()))
        alt_d = frame.get("alt_diff_ft")
        if alt_d is not None:
            p = p.field("alt_diff_ft", int(alt_d))
        try:
            self._q.put_nowait(p)
        except queue.Full:
            try:
                self._q.get_nowait()
            except queue.Empty:
                pass
            try:
                self._q.put_nowait(p)
            except queue.Full:
                self.stats.dropped += 1

    # ---- worker + heartbeat ----------------------------------------- #

    def _heartbeat_loop(self):
        """Tick a fresh heartbeat into the queue every HEARTBEAT_INTERVAL_S
        so the dashboard's 'time of last datapoint' stays current and the
        Active Alerts panel has a non-empty bucket to query from the
        moment the GUI launches."""
        while not self._stop.wait(HEARTBEAT_INTERVAL_S):
            self.write_heartbeat("tick")

    def _worker(self):
        while not self._stop.is_set():
            try:
                point = self._q.get(timeout=0.5)
            except queue.Empty:
                continue
            if point is None:
                return
            try:
                self._write_api.write(self.bucket, self.org, record=point)
                self.stats.written += 1
            except Exception as e:                   # noqa: BLE001
                self.stats.write_errs += 1
                self.stats.last_error = f"write: {e}"
                # Print only the first error of each kind to avoid
                # spamming stderr if the network is flaky.
                if self.stats.write_errs <= 3:
                    print(f"[influx] write error: {e}", file=sys.stderr)
