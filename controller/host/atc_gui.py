"""
SkyWatch ATC GUI — tabular + map live view of the controller's aircraft DB.

Stage:     6
Task IDs:  6.1 (table), 6.2 (map)
Inputs:
    Newline-delimited AircraftFrame JSON, one object per line, per
    resources/standards/json_protocol.md. Three input modes:
        --port /dev/ttyACM1         live from the controller's data port
        --stdin                     pipe from sdr_bridge --test or similar
        --file path                 replay a captured log
Outputs:
    PyQt6 window, dark theme, refreshed at 2 Hz.
    Default layout is split-pane: lat/lon-grid map on the left, table on the
    right. `--view` chooses one of {both, map, table}.
Run:
    cd /home/blaise/finalproject4011
    # live against firmware (5.1+):
    python3 scripts/stage6/atc_gui.py --port /dev/ttyACM1

    # hardware-free dev loop:
    python3 scripts/stage1/sdr_bridge.py --test 2>/dev/null \
      | python3 scripts/stage6/atc_gui.py --stdin

    # show just the map (good for debugging icon placement):
    python3 scripts/stage6/atc_gui.py --port /dev/ttyACM1 --view map
Notes:
    - Schema validation uses resources/standards/json_protocol.py — the same
      validator the test suite uses. Garbage lines are dropped silently and
      counted (status bar).
    - Stale rows: any aircraft not refreshed for >10 s is removed (matches
      firmware AIRCRAFT_DB_STALE_MS).
    - Reader runs in a daemon thread; rows update via a Qt signal/slot, so
      the main thread is the only one touching widgets.
    - Map bounds default to ±0.5° around Brisbane (-27.4975, 153.0137). Use
      `--lat-center / --lon-center / --span-deg` to retarget.
    - 6.3 will add a 50-point alpha-fade trail per aircraft on the map.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import pathlib
import queue
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Deque, Dict, List, Optional, Tuple

# resources/standards/ → single source of truth for the wire schema. Walks
# up from this script (following symlinks via .resolve()) until it finds the
# standards dir, so the GUI works whether invoked from scripts/stage6/
# (legacy path, now a symlink) or from vermillion-puck/controller/host/
# (in-repo canonical location).
def _find_standards() -> pathlib.Path:
    start = pathlib.Path(__file__).resolve().parent
    for cand in [start, *start.parents]:
        s = cand / "resources" / "standards"
        if (s / "json_protocol.py").is_file():
            return s
    raise RuntimeError("could not locate resources/standards/json_protocol.py")

_STANDARDS = _find_standards()
sys.path.insert(0, str(_STANDARDS))
from json_protocol import validate, ProtocolError  # noqa: E402

from PyQt6.QtCore    import Qt, QTimer, QPointF, QRectF, pyqtSignal, QObject
from PyQt6.QtGui     import (
    QFont, QColor, QBrush, QPen, QPolygonF, QPainter, QPixmap,
)
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QTableWidget, QTableWidgetItem,
    QStatusBar, QHeaderView, QAbstractItemView,
    QGraphicsScene, QGraphicsView, QGraphicsPolygonItem,
    QGraphicsSimpleTextItem, QGraphicsLineItem, QGraphicsPixmapItem,
    QSplitter, QWidget, QVBoxLayout, QLabel,
)

try:
    import serial
except ImportError:
    serial = None  # only required for --port


STALE_S = 10.0     # match firmware AIRCRAFT_DB_STALE_MS
REFRESH_HZ = 2.0


def aircraft_colour(icao: str) -> QColor:
    """Stable per-ICAO HSV colour. Hue is derived from MD5(icao) so the
    same aircraft always renders in the same colour across runs and across
    every widget (map icon, table ICAO cell, collision-history rows). The
    saturation + value are fixed at vivid-mid-bright so the colours pop
    against the dark theme without ever going white-on-white."""
    if not icao:
        return QColor("#cdd6f4")
    h = int(hashlib.md5(icao.encode("ascii")).hexdigest()[:6], 16)
    hue = h % 360
    c = QColor()
    c.setHsv(hue, 180, 230)
    return c


# ---------------------------------------------------------------------- #
# Reader: produces validated AircraftFrame dicts into a queue.
# ---------------------------------------------------------------------- #

@dataclass
class ReaderStats:
    lines_in: int = 0
    parsed_ok: int = 0
    parse_err: int = 0
    schema_err: int = 0


class JsonLineReader(threading.Thread):
    """Reads newline-delimited JSON from a file-like object and pushes
    validated dicts to `out_queue`. Daemon thread."""

    def __init__(self, src, out_queue: "queue.Queue[Tuple[float, dict]]",
                 stats: ReaderStats):
        super().__init__(daemon=True, name="json-reader")
        self.src = src
        self.q = out_queue
        self.stats = stats
        self._stop = False

    def stop(self):
        self._stop = True

    def run(self):
        # NB: do NOT use `iter(src.readline, b"")` — pyserial returns b"" on
        # every read-timeout, which would silently exit on the first quiet
        # gap between publisher ticks.
        while not self._stop:
            try:
                raw = self.src.readline()
            except Exception:
                time.sleep(0.2)
                continue
            if not raw:
                continue
            try:
                line = raw.decode("utf-8", errors="replace").strip()
            except Exception:
                self.stats.parse_err += 1
                continue
            if not line:
                continue
            self.stats.lines_in += 1
            try:
                obj = json.loads(line)
            except json.JSONDecodeError:
                self.stats.parse_err += 1
                continue
            try:
                validate(obj)
            except ProtocolError:
                self.stats.schema_err += 1
                continue
            # NB: we must let both `aircraft` AND `collision` frames through.
            # The previous filter (type=="aircraft") silently dropped every
            # CollisionFrame before it reached the dispatcher, so on_frame
            # never populated self.alerts and the banner could never fire.
            if obj.get("type") not in ("aircraft", "collision"):
                continue
            self.stats.parsed_ok += 1
            try:
                self.q.put_nowait((time.time(), obj))
            except queue.Full:
                pass


def open_source(args) -> Tuple[object, str]:
    if args.port:
        if serial is None:
            sys.exit("pyserial not installed; pip install pyserial")
        s = serial.Serial(args.port, args.baud, timeout=0.5)
        return s, f"serial {args.port}@{args.baud}"
    if args.stdin:
        return sys.stdin.buffer, "stdin"
    if args.file:
        return open(args.file, "rb"), f"file {args.file}"
    sys.exit("must supply one of --port / --stdin / --file")


# ---------------------------------------------------------------------- #
# Qt side: dispatcher pumps reader queue onto the main thread via signal.
# ---------------------------------------------------------------------- #

class FrameDispatcher(QObject):
    frame_received = pyqtSignal(dict, float)   # (frame, recv_wall_time)

    def __init__(self, q):
        super().__init__()
        self.q = q
        self.timer = QTimer(self)
        self.timer.setInterval(50)
        self.timer.timeout.connect(self._drain)
        self.timer.start()

    def _drain(self):
        for _ in range(200):
            try:
                recv_t, frame = self.q.get_nowait()
            except queue.Empty:
                break
            self.frame_received.emit(frame, recv_t)


# ---------------------------------------------------------------------- #
# Shared aircraft state, owned by the main window.
# entries: icao -> (frame_dict, recv_t)
# ---------------------------------------------------------------------- #

# ---------------------------------------------------------------------- #
# Table widget (6.1).
# ---------------------------------------------------------------------- #

COLUMNS = ["ICAO", "src", "lat", "lon", "alt(ft)", "vel(kt)", "hdg(°)", "age(s)"]


class AircraftTable(QTableWidget):
    # Pixel widths per column — sum ≈ 520 px, comfortably fits in a ~600 px pane.
    _COL_WIDTHS = [70, 60, 80, 85, 65, 55, 55, 55]

    def __init__(self):
        super().__init__(0, len(COLUMNS))
        self.setHorizontalHeaderLabels(COLUMNS)
        self.verticalHeader().setVisible(False)
        self.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        hh = self.horizontalHeader()
        hh.setSectionResizeMode(QHeaderView.ResizeMode.Interactive)
        for i, w in enumerate(self._COL_WIDTHS):
            self.setColumnWidth(i, w)
        hh.setStretchLastSection(True)
        self.setMinimumWidth(sum(self._COL_WIDTHS) + 16)

    def render_entries(self, entries: Dict[str, Tuple[dict, float]], now: float):
        rows = sorted(entries.items(), key=lambda kv: kv[0])
        self.setRowCount(len(rows))
        for r, (icao, (frame, recv_t)) in enumerate(rows):
            age = now - recv_t
            cells = [
                icao,
                frame.get("source", "?"),
                f"{frame.get('lat', 0.0):+.5f}",
                f"{frame.get('lon', 0.0):+.5f}",
                str(frame.get("alt", "—")) if "alt" in frame else "—",
                f"{frame.get('vel', 0.0):.0f}" if "vel" in frame else "—",
                f"{frame.get('hdg', 0.0):.0f}" if "hdg" in frame else "—",
                f"{age:.1f}",
            ]
            if age < 2.5:
                fg = QColor("#a6e3a1")
            elif age < 5.0:
                fg = QColor("#cdd6f4")
            else:
                fg = QColor("#f9e2af")
            src = frame.get("source", "")
            src_fg = QColor("#89b4fa") if src == "ADS_B" else QColor("#f5c2e7")
            for c, txt in enumerate(cells):
                item = QTableWidgetItem(str(txt))
                if c == 0:
                    # ICAO column carries the stable per-aircraft colour so
                    # the same plane is recognisable at a glance everywhere.
                    item.setForeground(QBrush(aircraft_colour(icao)))
                else:
                    item.setForeground(QBrush(src_fg if c == 1 else fg))
                if c >= 2:
                    item.setTextAlignment(Qt.AlignmentFlag.AlignRight |
                                          Qt.AlignmentFlag.AlignVCenter)
                self.setItem(r, c, item)


# ---------------------------------------------------------------------- #
# Collision history (Stage 8 polish).
#   - One row per (icao_a, icao_b) pair the controller has *ever* flagged
#     as ADVISORY or WARNING since GUI start.
#   - Persists across CLEAR — keeps level_max and the smallest-ever min_sep
#     and alt_diff so the operator can review the worst point of the event.
#   - Hard cap of 32 rows; oldest-by-first-seen evicted FIFO if hit.
# ---------------------------------------------------------------------- #

HISTORY_COLUMNS = ["ICAO A", "ICAO B", "Max", "Min sep (m)", "Δalt (ft)",
                   "Issued", "Updated"]
HISTORY_CAP = 32

_LEVEL_RANK = {"CLEAR": 0, "ADVISORY": 1, "WARNING": 2}
_LEVEL_BG = {
    "WARNING":  QColor("#a8132d"),
    "ADVISORY": QColor("#d9b400"),
    "CLEAR":    QColor("#45475a"),
}


class CollisionHistoryModel:
    """Pure-Python state for the dashboard. Owned by AtcMainWindow,
    mutated by _on_collision, read by CollisionHistoryTable.render_rows."""

    def __init__(self):
        # Ordered insertion (Python dict order = insertion order) so we
        # can FIFO-evict on overflow without a separate timestamp scan.
        self.rows: "Dict[Tuple[str, str], dict]" = {}

    def on_event(self, key, level, tca, sep, alt_diff_ft, recv_t):
        existing = self.rows.get(key)

        if existing is None:
            # User spec: don't create a row for a never-was-warned pair.
            if level == "CLEAR":
                return
            self.rows[key] = {
                "level_max":   level,
                "level_last":  level,
                "min_sep_m":   float(sep),
                "alt_diff_ft": int(alt_diff_ft) if alt_diff_ft is not None else None,
                "first_seen":  recv_t,
                "last_seen":   recv_t,
            }
        else:
            # Upgrade level_max only — CLEAR never downgrades the worst-ever.
            if _LEVEL_RANK.get(level, 0) > _LEVEL_RANK.get(existing["level_max"], 0):
                existing["level_max"] = level
            existing["level_last"] = level
            existing["min_sep_m"]  = min(existing["min_sep_m"], float(sep))
            if alt_diff_ft is not None:
                if existing["alt_diff_ft"] is None:
                    existing["alt_diff_ft"] = int(alt_diff_ft)
                else:
                    existing["alt_diff_ft"] = min(existing["alt_diff_ft"], int(alt_diff_ft))
            existing["last_seen"] = recv_t

        # Cap: FIFO-evict oldest if we just exceeded the limit.
        while len(self.rows) > HISTORY_CAP:
            self.rows.pop(next(iter(self.rows)))


class CollisionHistoryTable(QTableWidget):
    _COL_WIDTHS = [70, 70, 75, 80, 75, 75, 80]

    def __init__(self):
        super().__init__(0, len(HISTORY_COLUMNS))
        self.setHorizontalHeaderLabels(HISTORY_COLUMNS)
        self.verticalHeader().setVisible(False)
        self.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        hh = self.horizontalHeader()
        hh.setSectionResizeMode(QHeaderView.ResizeMode.Interactive)
        for i, w in enumerate(self._COL_WIDTHS):
            self.setColumnWidth(i, w)
        hh.setStretchLastSection(True)
        self.setMinimumWidth(sum(self._COL_WIDTHS) + 16)

    def render_rows(self, model: CollisionHistoryModel, now: float):
        # Newest-issued first so the most recent event is always on top.
        rows = sorted(model.rows.items(),
                      key=lambda kv: kv[1]["first_seen"], reverse=True)
        self.setRowCount(len(rows))
        for r, ((a, b), v) in enumerate(rows):
            issued_str  = time.strftime("%H:%M:%S", time.localtime(v["first_seen"]))
            updated_age = now - v["last_seen"]
            alt_d       = v["alt_diff_ft"]
            cells = [
                a,
                b,
                v["level_max"],
                f"{v['min_sep_m']:.0f}",
                "—" if alt_d is None else f"{alt_d:d}",
                issued_str,
                f"{updated_age:.1f}s ago",
            ]
            for c, txt in enumerate(cells):
                item = QTableWidgetItem(str(txt))
                if c == 0:
                    item.setForeground(QBrush(aircraft_colour(a)))
                elif c == 1:
                    item.setForeground(QBrush(aircraft_colour(b)))
                elif c == 2:
                    # Max-level cell: text contrasts on the level-coloured bg.
                    item.setBackground(QBrush(_LEVEL_BG.get(v["level_max"], _LEVEL_BG["CLEAR"])))
                    item.setForeground(QBrush(QColor("#1e1e2e")))
                else:
                    # Age-of-event subtle cue: dim if the pair has gone quiet.
                    fg = QColor("#a6adc8") if updated_age < 5.0 else QColor("#6c7086")
                    item.setForeground(QBrush(fg))
                if c >= 3:
                    item.setTextAlignment(Qt.AlignmentFlag.AlignRight |
                                          Qt.AlignmentFlag.AlignVCenter)
                self.setItem(r, c, item)


# ---------------------------------------------------------------------- #
# Map widget (6.2): lat/lon grid + per-aircraft triangle icon + label.
# Stage 6.3 will hang a 50-point alpha-fade trail off this.
# ---------------------------------------------------------------------- #

class MapView(QGraphicsView):
    SCENE_W = 800
    SCENE_H = 800
    GRID_STEP_DEG = 0.1   # 0.1° ≈ 11 km lat / ~10 km lon at Brisbane
    TRAIL_LEN = 50        # Stage 6.3: per-aircraft trail length (positions)

    def __init__(self, lat_center: float, lon_center: float, span_deg: float):
        super().__init__()
        self.lat_top    = lat_center + span_deg
        self.lat_bot    = lat_center - span_deg
        self.lon_left   = lon_center - span_deg
        self.lon_right  = lon_center + span_deg

        self.scene = QGraphicsScene()
        self.scene.setSceneRect(QRectF(0, 0, self.SCENE_W, self.SCENE_H))
        self.scene.setBackgroundBrush(QBrush(QColor("#11111b")))
        self.setScene(self.scene)
        self.setRenderHint(QPainter.RenderHint.Antialiasing)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setMinimumSize(400, 400)

        self._draw_basemap(lat_center, lon_center, span_deg)
        self._draw_grid()
        self._draw_center_marker(lat_center, lon_center)

        # Corner badges (child widgets overlaid on the view). The link badge
        # tracks any incoming JSON; the BLE badge tracks BLE_SIM frames
        # specifically so we can show sim-node connection state.
        self.status_badge = self._make_badge("● NO DATA", "#f38ba8")
        self.ble_badge    = self._make_badge("● BLE   scanning…", "#f38ba8")

        # icao -> (polygon_item, label_item)
        self.items: Dict[str, Tuple[QGraphicsPolygonItem, QGraphicsSimpleTextItem]] = {}
        # Stage 6.3: rolling trail of scene-coord points per ICAO.
        self.trails: Dict[str, Deque[Tuple[float, float]]] = {}
        # Stage 6.3: line-item segments per ICAO, recycled in place so we
        # don't churn the scene graph every tick.
        self.trail_segs: Dict[str, List[QGraphicsLineItem]] = {}
        # Stage 7.4: per-ICAO dashed line + arrowhead from current pos to
        # Kalman prediction. Only present when the firmware set
        # AIRCRAFT_VALID_PRED on the frame.
        self.pred_lines: Dict[str, QGraphicsLineItem] = {}
        self.pred_heads: Dict[str, QGraphicsPolygonItem] = {}

    def _badge_qss(self, fg: str) -> str:
        # Single f-string so the {{ / }} escapes resolve correctly. Splitting
        # into concatenated literals leaves the trailing }} unescaped because
        # only f-string pieces participate in brace-escaping.
        return (
            f"QLabel {{"
            f" background: rgba(17,17,27,210);"
            f" color: {fg};"
            f" padding: 4px 10px;"
            f" border: 1px solid #313244;"
            f" border-radius: 6px;"
            f" font-family: 'JetBrains Mono','Consolas',monospace;"
            f" font-size: 10pt;"
            f" }}"
        )

    def _make_badge(self, text: str, fg: str) -> QLabel:
        lbl = QLabel(self)
        lbl.setText(text)
        lbl.setStyleSheet(self._badge_qss(fg))
        lbl.adjustSize()
        return lbl

    def _layout_badges(self):
        """Stack badges in the top-right; called on resize + every update."""
        margin = 10
        gap = 6
        y = margin
        for lbl in (self.status_badge, self.ble_badge):
            lbl.adjustSize()
            x = max(margin, self.width() - lbl.width() - margin)
            lbl.move(x, y)
            y += lbl.height() + gap

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.fitInView(self.scene.sceneRect(), Qt.AspectRatioMode.KeepAspectRatio)
        self._layout_badges()

    def set_status(self, text: str, fg: str):
        self.status_badge.setText(text)
        self.status_badge.setStyleSheet(self._badge_qss(fg))
        self._layout_badges()

    def set_ble_status(self, text: str, fg: str):
        self.ble_badge.setText(text)
        self.ble_badge.setStyleSheet(self._badge_qss(fg))
        self._layout_badges()

    # ---- coordinate transform ----
    def to_xy(self, lat: float, lon: float) -> Tuple[float, float]:
        x = (lon - self.lon_left)  / (self.lon_right - self.lon_left)  * self.SCENE_W
        y = (self.lat_top - lat)   / (self.lat_top   - self.lat_bot)   * self.SCENE_H
        return x, y

    # ---- one-time scene setup ----
    def _draw_basemap(self, lat_center: float, lon_center: float, span_deg: float):
        """Load the pre-baked greyscale Brisbane basemap and stretch it to
        fill the scene rect. The PNG bbox is encoded in the filename so we
        only show it when it matches the current view. Rendered at low
        opacity over the dark backdrop so the per-aircraft colours pop.

        Generated by `python3 scripts/stage6/fetch_basemap.py` — if the
        file is absent the GUI just falls back to the plain dark grid."""
        name = (f"basemap_{lat_center:+.4f}_{lon_center:+.4f}"
                f"_{span_deg:.4f}.png")
        path = pathlib.Path(__file__).parent / "assets" / name
        if not path.is_file():
            print(f"[map] no basemap at {path} — using plain grid "
                  f"(bake one with `python3 scripts/stage6/fetch_basemap.py`)",
                  file=sys.stderr)
            return
        pix = QPixmap(str(path))
        if pix.isNull():
            print(f"[map] basemap {path} failed to load", file=sys.stderr)
            return
        item = QGraphicsPixmapItem(pix)
        # Scale the pixmap to exactly fill the scene rect (equirectangular
        # mapping matches what to_xy() does for points).
        sx = self.SCENE_W / pix.width()
        sy = self.SCENE_H / pix.height()
        item.setScale(sx)  # square scale — sy ≈ sx since we bake square
        if abs(sx - sy) > 1e-3:
            # If the user re-baked with a non-square aspect we'd need a
            # transform; warn so they re-bake with a matching aspect.
            print(f"[map] basemap aspect mismatch (sx={sx:.3f} sy={sy:.3f}); "
                  "consider re-baking", file=sys.stderr)
        item.setOpacity(0.45)        # dimmed for icon contrast
        item.setZValue(-100)         # sit under grid + icons + trails
        item.setTransformationMode(Qt.TransformationMode.SmoothTransformation)
        self.scene.addItem(item)

    def _draw_grid(self):
        grid_pen  = QPen(QColor("#2a3a4a"))
        label_pen = QColor("#546e7a")
        font = QFont("monospace", 8)

        lat = math.floor(self.lat_bot / self.GRID_STEP_DEG) * self.GRID_STEP_DEG
        while lat <= self.lat_top + 1e-9:
            _, y = self.to_xy(lat, self.lon_left)
            self.scene.addLine(0, y, self.SCENE_W, y, grid_pen)
            t = self.scene.addSimpleText(f"{lat:+.2f}°", font)
            t.setBrush(QBrush(label_pen))
            t.setPos(2, y - 11)
            lat += self.GRID_STEP_DEG

        lon = math.floor(self.lon_left / self.GRID_STEP_DEG) * self.GRID_STEP_DEG
        while lon <= self.lon_right + 1e-9:
            x, _ = self.to_xy(self.lat_top, lon)
            self.scene.addLine(x, 0, x, self.SCENE_H, grid_pen)
            t = self.scene.addSimpleText(f"{lon:+.2f}°", font)
            t.setBrush(QBrush(label_pen))
            t.setPos(x + 2, self.SCENE_H - 14)
            lon += self.GRID_STEP_DEG

        border = QPen(QColor("#4fc3f7"))
        border.setWidth(2)
        self.scene.addRect(0, 0, self.SCENE_W, self.SCENE_H, border)

    def _draw_center_marker(self, lat: float, lon: float):
        x, y = self.to_xy(lat, lon)
        pen = QPen(QColor("#f9e2af"))
        pen.setWidth(1)
        self.scene.addLine(x - 6, y, x + 6, y, pen)
        self.scene.addLine(x, y - 6, x, y + 6, pen)

    # ---- per-tick aircraft refresh ----
    @staticmethod
    def _aircraft_polygon() -> QPolygonF:
        # Triangle pointing up (north) when hdg_deg = 0. Center at origin.
        return QPolygonF([QPointF(0, -10), QPointF(-7, 8), QPointF(7, 8)])

    def _icon_colour(self, src: str, age: float) -> QColor:
        if age > 5.0:
            return QColor("#f9e2af")    # stale-ish
        if src == "BLE_SIM":
            return QColor("#f5c2e7")
        return QColor("#89b4fa")        # ADS_B (default)

    def _update_trail(self, icao: str, x: float, y: float, colour: QColor):
        """Append a point and redraw fading line segments for this aircraft."""
        trail = self.trails.get(icao)
        if trail is None:
            trail = deque(maxlen=self.TRAIL_LEN)
            self.trails[icao] = trail

        # Only append if the position actually moved (avoid stacking dupes
        # from the 2 Hz publisher when an aircraft is stationary or briefly
        # paused). Small epsilon in scene-pixel space.
        if not trail or (abs(trail[-1][0] - x) > 0.5 or abs(trail[-1][1] - y) > 0.5):
            trail.append((x, y))

        segs = self.trail_segs.setdefault(icao, [])
        needed = max(0, len(trail) - 1)

        # Grow / shrink segment list to match.
        while len(segs) < needed:
            line = QGraphicsLineItem()
            line.setZValue(-1)   # behind the icons
            self.scene.addItem(line)
            segs.append(line)
        while len(segs) > needed:
            line = segs.pop()
            self.scene.removeItem(line)

        # Paint each segment with alpha ramping older→newer.
        pts = list(trail)
        n = len(pts) - 1
        if n <= 0:
            return
        for i in range(n):
            x1, y1 = pts[i]
            x2, y2 = pts[i + 1]
            # i=0 is oldest, i=n-1 is newest. Alpha 0x18..0xff.
            alpha = int(0x18 + (0xff - 0x18) * (i / max(1, n - 1)))
            c = QColor(colour)
            c.setAlpha(alpha)
            pen = QPen(c)
            pen.setWidth(2)
            segs[i].setPen(pen)
            segs[i].setLine(x1, y1, x2, y2)

    def _drop_trail(self, icao: str):
        for line in self.trail_segs.pop(icao, []):
            self.scene.removeItem(line)
        self.trails.pop(icao, None)
        line = self.pred_lines.pop(icao, None)
        if line is not None:
            self.scene.removeItem(line)
        head = self.pred_heads.pop(icao, None)
        if head is not None:
            self.scene.removeItem(head)

    def _update_pred(self, icao: str, x0: float, y0: float,
                     pred_lat: Optional[float], pred_lon: Optional[float],
                     colour: QColor):
        """Draw / remove the dashed prediction line + arrowhead for one ICAO."""
        line = self.pred_lines.get(icao)
        head = self.pred_heads.get(icao)
        if pred_lat is None or pred_lon is None:
            if line is not None:
                self.scene.removeItem(line); del self.pred_lines[icao]
            if head is not None:
                self.scene.removeItem(head); del self.pred_heads[icao]
            return

        x1, y1 = self.to_xy(pred_lat, pred_lon)

        # Dashed line.
        if line is None:
            line = QGraphicsLineItem()
            line.setZValue(-0.5)   # above trail, below icon
            self.scene.addItem(line)
            self.pred_lines[icao] = line
        pen = QPen(colour)
        pen.setWidth(2)
        pen.setStyle(Qt.PenStyle.DashLine)
        line.setPen(pen)
        line.setLine(x0, y0, x1, y1)

        # Arrowhead at the predicted endpoint, rotated to match the segment.
        if head is None:
            head = QGraphicsPolygonItem(QPolygonF([
                QPointF(0, 0),      # tip
                QPointF(-9, -4),
                QPointF(-9,  4),
            ]))
            head.setPen(QPen(Qt.GlobalColor.black, 0.5))
            head.setZValue(-0.4)
            self.scene.addItem(head)
            self.pred_heads[icao] = head
        head.setBrush(QBrush(colour))
        head.setPos(x1, y1)
        head.setRotation(math.degrees(math.atan2(y1 - y0, x1 - x0)))

    def render_entries(self, entries: Dict[str, Tuple[dict, float]], now: float,
                       red_icaos: Optional[set] = None):
        red_icaos = red_icaos or set()
        # Update / add
        for icao, (frame, recv_t) in entries.items():
            lat = frame.get("lat"); lon = frame.get("lon")
            if lat is None or lon is None:
                continue
            x, y = self.to_xy(lat, lon)
            age = now - recv_t
            if icao in red_icaos:
                colour = QColor("#f38ba8")    # Catppuccin red — collision flag
            else:
                # Stable per-aircraft colour (hash of ICAO), with a darker
                # variant if the aircraft has gone quiet recently.
                colour = aircraft_colour(icao)
                if age > 5.0:
                    colour = colour.darker(160)

            # Trail BEFORE the icon so the icon paints on top.
            self._update_trail(icao, x, y, colour)

            poly_item, label_item = self.items.get(icao, (None, None))
            if poly_item is None:
                poly_item = QGraphicsPolygonItem(self._aircraft_polygon())
                poly_item.setPen(QPen(Qt.GlobalColor.black, 0.5))
                self.scene.addItem(poly_item)
                label_item = QGraphicsSimpleTextItem(icao)
                label_item.setFont(QFont("monospace", 8))
                self.scene.addItem(label_item)
                self.items[icao] = (poly_item, label_item)

            poly_item.setBrush(QBrush(colour))
            poly_item.setPos(x, y)
            hdg = frame.get("hdg")
            if isinstance(hdg, (int, float)):
                poly_item.setRotation(hdg)

            label_item.setBrush(QBrush(colour))
            label_item.setPos(x + 8, y - 14)

            # Stage 7.4 — dashed line to Kalman 10 s-ahead position.
            self._update_pred(icao, x, y,
                              frame.get("pred_lat"), frame.get("pred_lon"),
                              colour)

        # Remove icaos no longer in entries (covers stale-evictions).
        for icao in list(self.items.keys()):
            if icao not in entries:
                poly_item, label_item = self.items.pop(icao)
                self.scene.removeItem(poly_item)
                self.scene.removeItem(label_item)
                self._drop_trail(icao)


# ---------------------------------------------------------------------- #
# Main window: assembles the split-pane and owns the entries dict.
# ---------------------------------------------------------------------- #

DARK_QSS = """
QMainWindow, QWidget { background-color: #1e1e2e; color: #cdd6f4; }
QTableWidget {
    background-color: #181825; color: #cdd6f4;
    gridline-color: #313244; border: 1px solid #313244;
    selection-background-color: #45475a;
    font-family: "JetBrains Mono", "Consolas", monospace;
    font-size: 11pt;
}
QHeaderView::section {
    background-color: #313244; color: #f5e0dc;
    padding: 4px 6px; border: 0; font-weight: bold;
}
QSplitter::handle { background-color: #313244; }
QStatusBar { background-color: #11111b; color: #a6adc8; }
QGraphicsView { border: 1px solid #313244; }
"""


class AtcMainWindow(QMainWindow):
    def __init__(self, source_label: str, stats: ReaderStats,
                 dispatcher: FrameDispatcher, view_mode: str,
                 lat_center: float, lon_center: float, span_deg: float):
        super().__init__()
        self.setWindowTitle("SkyWatch ATC")
        self.resize(1400, 720)
        self.setStyleSheet(DARK_QSS)

        # Shared state: icao -> (frame, last_recv_t)
        self.entries: Dict[str, Tuple[dict, float]] = {}
        self.stats = stats
        self.source_label = source_label
        self.last_frame_t: float = 0.0   # wall-clock when most recent frame arrived
        self.last_ble_frame_t: float = 0.0
        self.last_ble_icao: str = ""
        # Stage 8.4: active collision events. Keyed by tuple (icao_a, icao_b)
        # — controller already emits them lexicographically sorted.
        # Value: (level_str, tca_s, min_sep_m, alt_diff_ft|None, last_seen_t)
        self.alerts: Dict[Tuple[str, str], Tuple] = {}

        # Persistent collision-history model + widget (always built so that
        # _on_collision can safely write to it even in map/table-only modes).
        self.history = CollisionHistoryModel()
        self.history_table = CollisionHistoryTable()

        self.table = AircraftTable() if view_mode != "map" else None
        self.map   = MapView(lat_center, lon_center, span_deg) \
                     if view_mode != "table" else None

        # Stage 8.4 — collision alert banner. Hidden until alerts.appears.
        self.alert_banner = QLabel()
        self.alert_banner.setVisible(False)
        self.alert_banner.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._alert_flash = False
        self._set_banner_style(flash=False)

        central = QWidget()
        outer = QVBoxLayout(central)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)
        outer.addWidget(self.alert_banner)
        if view_mode == "both":
            split = QSplitter(Qt.Orientation.Horizontal)
            split.addWidget(self.map)

            # Right pane: vertical split — aircraft table on top, collision
            # history below. Both share the same right column.
            right = QSplitter(Qt.Orientation.Vertical)
            right.addWidget(self.table)
            right.addWidget(self.history_table)
            right.setStretchFactor(0, 2)
            right.setStretchFactor(1, 1)
            split.addWidget(right)

            split.setStretchFactor(0, 7)
            split.setStretchFactor(1, 3)
            outer.addWidget(split, 1)
        elif view_mode == "map":
            outer.addWidget(self.map, 1)
        else:
            # Pure table mode also gets the history pane underneath.
            right = QSplitter(Qt.Orientation.Vertical)
            right.addWidget(self.table)
            right.addWidget(self.history_table)
            right.setStretchFactor(0, 2)
            right.setStretchFactor(1, 1)
            outer.addWidget(right, 1)
        self.setCentralWidget(central)

        self.status = QStatusBar(self)
        self.setStatusBar(self.status)

        dispatcher.frame_received.connect(self.on_frame)

        self.refresh_timer = QTimer(self)
        self.refresh_timer.setInterval(int(1000 / REFRESH_HZ))
        self.refresh_timer.timeout.connect(self.refresh_view)
        self.refresh_timer.start()

    def _set_banner_style(self, flash: bool, level: str = "WARNING"):
        # Two-tone pulse, palette depends on worst level currently shown.
        # WARNING  -> red    ADVISORY -> amber/yellow    CLEAR -> grey
        if level == "WARNING":
            bg = "#f38ba8" if flash else "#a8132d"
        elif level == "ADVISORY":
            bg = "#f9e2af" if flash else "#d9b400"
        else:
            bg = "#a6adc8" if flash else "#6c7086"
        self.alert_banner.setStyleSheet(
            f"QLabel {{ background: {bg}; color: #1e1e2e;"
            f" font-family: 'JetBrains Mono','Consolas',monospace;"
            f" font-size: 14pt; font-weight: bold;"
            f" padding: 8px 12px; border-bottom: 2px solid #11111b; }}"
        )

    def on_frame(self, frame: dict, recv_t: float):
        ftype = frame.get("type")
        if ftype == "collision":
            self._on_collision(frame, recv_t)
            self.last_frame_t = recv_t
            return
        icao = frame.get("icao")
        if icao:
            self.entries[icao] = (frame, recv_t)
        self.last_frame_t = recv_t
        if frame.get("source") == "BLE_SIM" and icao:
            self.last_ble_frame_t = recv_t
            self.last_ble_icao = icao

    def _on_collision(self, frame: dict, recv_t: float):
        key = (frame.get("icao_a", ""), frame.get("icao_b", ""))
        if not key[0] or not key[1]:
            return
        level = frame.get("level", "CLEAR")
        tca   = float(frame.get("tca_s", 0.0))
        sep   = float(frame.get("min_sep_m", 0.0))
        alt_d = frame.get("alt_diff_ft")  # may be None if firmware didn't report
        # Always feed the collision-history dashboard (it has its own logic
        # about WARNING/ADVISORY entries persisting after CLEAR).
        self.history.on_event(key, level, tca, sep, alt_d, recv_t)
        if level == "CLEAR":
            self.alerts.pop(key, None)
            return
        self.alerts[key] = (level, tca, sep, alt_d, recv_t)

    def refresh_view(self):
        now = time.time()
        for icao in list(self.entries.keys()):
            if now - self.entries[icao][1] > STALE_S:
                del self.entries[icao]

        # Stage 8.4 — expire alerts that haven't been refreshed in 3 s
        # (controller emits every 1 s when WARNING-active; 3 s is safe
        # against transient frame loss). Collect set of ICAOs to highlight.
        for k in list(self.alerts.keys()):
            if now - self.alerts[k][4] > 3.0:
                del self.alerts[k]
        red_icaos = set()
        for (a, b) in self.alerts.keys():
            red_icaos.add(a)
            red_icaos.add(b)

        if self.table is not None:
            self.table.render_entries(self.entries, now)
        self.history_table.render_rows(self.history, now)
        if self.map is not None:
            self.map.render_entries(self.entries, now, red_icaos=red_icaos)
            # Corner #1 — overall link freshness (any source).
            since = (now - self.last_frame_t) if self.last_frame_t else 1e9
            if since < 1.0:
                self.map.set_status(
                    f"● LINK   {self.source_label}   tracked={len(self.entries)}",
                    "#a6e3a1")
            elif since < 5.0:
                self.map.set_status(
                    f"● STALE  {since:.1f}s   tracked={len(self.entries)}",
                    "#f9e2af")
            else:
                self.map.set_status(
                    f"● NO DATA   {self.source_label}", "#f38ba8")

            # Corner #2 — BLE sim-node freshness (firmware watchdog auto-
            # reconnects in the background; we just report what we see).
            ble_since = (now - self.last_ble_frame_t) if self.last_ble_frame_t else 1e9
            if ble_since < 2.0:
                self.map.set_ble_status(
                    f"● BLE   live   {self.last_ble_icao}   last {ble_since:.1f}s",
                    "#a6e3a1")
            elif ble_since < 10.0:
                self.map.set_ble_status(
                    f"◐ BLE   stale  {self.last_ble_icao}   last {ble_since:.1f}s",
                    "#f9e2af")
            else:
                self.map.set_ble_status(
                    "↻ BLE   scanning — controller auto-reconnects every 2 s",
                    "#f38ba8")

        # Stage 8.4 — refresh the alert banner.
        if self.alerts:
            # Worst level first; flash background between two reds each tick.
            order = {"WARNING": 0, "ADVISORY": 1, "CLEAR": 2}
            items = sorted(self.alerts.items(),
                           key=lambda kv: (order.get(kv[1][0], 9), kv[0]))
            lines = []
            for (a, b), (lvl, tca, sep, alt_d, _) in items[:4]:   # cap to 4 visible
                if alt_d is None:
                    lines.append(f"{lvl}: {a} ↔ {b} · {sep:.0f} m · in {tca:.1f} s")
                else:
                    lines.append(
                        f"{lvl}: {a} ↔ {b} · {sep:.0f} m · Δalt {int(alt_d):+d} ft · in {tca:.1f} s"
                    )
            self.alert_banner.setText("    ".join(lines))
            self._alert_flash = not self._alert_flash
            # Worst-level drives the colour: any WARNING -> red, else amber.
            worst = "WARNING" if any(v[0] == "WARNING" for v in self.alerts.values()) else "ADVISORY"
            self._set_banner_style(flash=self._alert_flash, level=worst)
            self.alert_banner.setVisible(True)
        else:
            self.alert_banner.setVisible(False)

        s = self.stats
        self.status.showMessage(
            f"src={self.source_label}   tracked={len(self.entries)}   "
            f"alerts={len(self.alerts)}   "
            f"lines={s.lines_in}  ok={s.parsed_ok}  parse_err={s.parse_err}  "
            f"schema_err={s.schema_err}"
        )


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    src = p.add_mutually_exclusive_group(required=False)
    src.add_argument("--port", help="Serial port (e.g. /dev/ttyACM1).")
    src.add_argument("--stdin", action="store_true",
                     help="Read from stdin (pipe from sdr_bridge --test).")
    src.add_argument("--file", help="Replay a file of newline-delimited JSON.")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--view", choices=["both", "map", "table"], default="both",
                   help="Window layout (default both, map left + table right).")
    p.add_argument("--lat-center", type=float, default=-27.4975,
                   help="Map centre latitude (default Brisbane CBD).")
    p.add_argument("--lon-center", type=float, default=153.0137,
                   help="Map centre longitude (default Brisbane CBD).")
    p.add_argument("--span-deg", type=float, default=0.5,
                   help="Half-width of the map in degrees (default 0.5° ≈ 55 km).")
    return p


def main(argv: Optional[list] = None) -> int:
    args = build_parser().parse_args(argv)

    src, src_label = open_source(args)
    q: "queue.Queue[Tuple[float, dict]]" = queue.Queue(maxsize=4096)
    stats = ReaderStats()
    reader = JsonLineReader(src, q, stats)
    reader.start()

    app = QApplication.instance() or QApplication(sys.argv)
    dispatcher = FrameDispatcher(q)
    win = AtcMainWindow(src_label, stats, dispatcher,
                        view_mode=args.view,
                        lat_center=args.lat_center,
                        lon_center=args.lon_center,
                        span_deg=args.span_deg)
    win.show()
    rc = app.exec()

    reader.stop()
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
