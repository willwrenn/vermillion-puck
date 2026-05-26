#!/usr/bin/env python3
"""
Minimal lat/lon grid GUI — tkinter only, no PyQt6.

Serial mode (reads JSON from Zephyr firmware on ttyACM0):
    python3 simple_grid.py --port /dev/ttyACM0

TCP mode (test without hardware):
    python3 simple_grid.py --tcp 5005
    echo '{"lat":-27.4975,"lon":153.0137}' | nc localhost 5005

Map image overlay:
    python3 simple_grid.py --map brisbane_map.jpg
    (image must cover exactly LAT_BOT..LAT_TOP, LON_LEFT..LON_RIGHT)

JSON accepted as plain degrees OR degrees x1e7:
    {"lat":-27.4975,"lon":153.0137}
    {"lat":-274975000,"lon":1530137000}
"""

import tkinter as tk
from tkinter import scrolledtext
import json, socket, threading, argparse, sys, re, datetime, math, random
from collections import deque
import serial

try:
    from PIL import Image, ImageTk
    PIL_OK = True
except ImportError:
    PIL_OK = False

# Strips ANSI escape codes (colour, cursor movement, etc.)
_ANSI_RE = re.compile(r'\x1b\[[0-9;]*[A-Za-z]|\x1b[()][AB012]|\x1b[=>]')

def _strip_ansi(s):
    return _ANSI_RE.sub('', s)

def _extract_json(text):
    """Return first valid JSON object found anywhere in text, or None."""
    text = _strip_ansi(text)
    start = 0
    while True:
        idx = text.find('{', start)
        if idx == -1:
            return None
        end = text.find('}', idx)
        if end == -1:
            return None
        try:
            return json.loads(text[idx:end+1])
        except json.JSONDecodeError:
            start = idx + 1

LAT_TOP = -27.34
LAT_BOT = -27.535
LON_LEFT = 152.83
LON_RIGHT = 153.21
W, H = 900, 650


def to_canvas(lat, lon):
    x = (lon - LON_LEFT) / (LON_RIGHT - LON_LEFT) * W
    y = (LAT_TOP - lat) / (LAT_TOP - LAT_BOT) * H
    return x, y


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--port", default=None,
                   help="Serial port e.g. /dev/ttyACM0 (data port)")
    p.add_argument("--tcp", type=int, default=5005, metavar="PORT",
                   help="TCP port to listen on when not using serial (default 5005)")
    p.add_argument("--map", default=None, metavar="IMAGE",
                   help="Path to map image (PNG or JPEG) covering the exact bounds")
    return p.parse_args()


args = parse_args()
mode = "serial" if args.port else "tcp"

# ── Window & canvas ───────────────────────────────────────────────────────────
root = tk.Tk()
title = f"Lat/Lon Grid — {args.port}" if mode == "serial" else f"Lat/Lon Grid — TCP :{args.tcp}"
root.title(title)
root.configure(bg="#0d1b2a")

canvas = tk.Canvas(root, width=W, height=H, bg="#0d1b2a", highlightthickness=0)
canvas.pack(side="left")

# ── Map image overlay ─────────────────────────────────────────────────────────
_map_photo = None # keep reference so it isn't garbage collected

if args.map:
    if not PIL_OK:
        print("WARNING: Pillow not installed — run: pip install pillow")
        print(" Map image will not be shown.")
    else:
        try:
            img = Image.open(args.map).resize((W, H), Image.LANCZOS)
            _map_photo = ImageTk.PhotoImage(img)
            canvas.create_image(0, 0, anchor="nw", image=_map_photo)
            print(f"Map image loaded: {args.map}")
        except Exception as e:
            print(f"WARNING: Could not load map image '{args.map}': {e}")

# ── Grid lines ────────────────────────────────────────────────────────────────
# Use semi-transparent-style colours when map is loaded (brighter so visible)
_grid_col = "#4a6a7a" if args.map else "#2a3a4a"
_label_col = "#90b4c4" if args.map else "#546e7a"

_STEP = 0.05
# snap to nearest clean 0.05° multiple so labels exactly match line positions
_lat_start = math.ceil( round(LAT_BOT / _STEP, 8)) * _STEP
_lon_start = math.ceil(round(LON_LEFT / _STEP, 8)) * _STEP

lat = _lat_start
while lat <= LAT_TOP + 1e-9:
    lat = round(lat, 6)
    x1, y1 = to_canvas(lat, LON_LEFT)
    x2, _ = to_canvas(lat, LON_RIGHT)
    canvas.create_line(x1, y1, x2, y1, fill=_grid_col)
    canvas.create_text(4, y1, text=f"{lat:.2f}°", fill=_label_col,
                       anchor="w", font=("Courier", 8))
    lat = round(lat + _STEP, 6)

lon = _lon_start
while lon <= LON_RIGHT + 1e-9:
    lon = round(lon, 6)
    x1, y1 = to_canvas(LAT_TOP, lon)
    _, y2 = to_canvas(LAT_BOT, lon)
    canvas.create_line(x1, y1, x1, y2, fill=_grid_col)
    canvas.create_text(x1, H - 4, text=f"{lon:.2f}°", fill=_label_col,
                       anchor="s", font=("Courier", 8))
    lon = round(lon + _STEP, 6)

canvas.create_rectangle(0, 0, W, H, outline="#4fc3f7", width=2)

# ── Trail ─────────────────────────────────────────────────────────────────────
TRAIL_LEN = 50

_BG = (0x0d, 0x1b, 0x2a)
_TR = (0xff, 0x44, 0x44)
_trail_colors = [
    "#{:02x}{:02x}{:02x}".format(
        int(_BG[0] + (_TR[0] - _BG[0]) * i / (TRAIL_LEN - 1)),
        int(_BG[1] + (_TR[1] - _BG[1]) * i / (TRAIL_LEN - 1)),
        int(_BG[2] + (_TR[2] - _BG[2]) * i / (TRAIL_LEN - 1)),
    )
    for i in range(TRAIL_LEN)
]

trail_pts = deque(maxlen=TRAIL_LEN)
_TR = 3
trail_items = [
    canvas.create_oval(-_TR, -_TR, _TR, _TR,
                       fill=_trail_colors[i], outline="", state="hidden")
    for i in range(TRAIL_LEN)
]

# ── Red dot ───────────────────────────────────────────────────────────────────
cx, cy = to_canvas((LAT_TOP + LAT_BOT) / 2, (LON_LEFT + LON_RIGHT) / 2)
dot = canvas.create_oval(cx-8, cy-8, cx+8, cy+8,
                            fill="red", outline="white", width=2)
coord = canvas.create_text(cx + 14, cy, text="", fill="red",
                            anchor="w", font=("Courier", 9))

# ── Right panel ───────────────────────────────────────────────────────────────
panel = tk.Frame(root, bg="#0a0f1a", width=220)
panel.pack(side="right", fill="y", padx=6, pady=6)
panel.pack_propagate(False)

status_var = tk.StringVar(value="Connecting...")
tk.Label(panel, textvariable=status_var, bg="#0a0f1a", fg="#ffb300",
         font=("Courier", 9), wraplength=200, justify="left").pack(anchor="w", pady=(8, 4))

lat_var = tk.StringVar(value="LAT: --")
lon_var = tk.StringVar(value="LON: --")
alt_var = tk.StringVar(value="ALT: --")
pkt_var = tk.StringVar(value="Packets: 0")
raw_var = tk.StringVar(value="--")

for v in (lat_var, lon_var, alt_var, pkt_var):
    tk.Label(panel, textvariable=v, bg="#0a0f1a", fg="#4fc3f7",
             font=("Courier", 10), anchor="w").pack(fill="x", padx=4)

tk.Label(panel, text="\nLast packet:", bg="#0a0f1a", fg="#546e7a",
         font=("Courier", 8)).pack(anchor="w", padx=4)
tk.Label(panel, textvariable=raw_var, bg="#0a0f1a", fg="#546e7a",
         font=("Courier", 8), wraplength=200, justify="left").pack(anchor="w", padx=4)

pkt_count = [0]

# ── Collision warning panel ───────────────────────────────────────────────────
_WARN_OFF = "#0a0f1a"
_WARN_ON = "#cc0000"

warn_box = tk.Frame(panel, bg=_WARN_OFF)
warn_box.pack(fill="x", padx=4, pady=(14, 0))
warn_title = tk.Label(warn_box, text="⚠ COLLISION WARNING ⚠",
                      bg=_WARN_OFF, fg=_WARN_OFF,
                      font=("Courier", 9, "bold"), anchor="center")
warn_title.pack(fill="x")
warn_icao_var = tk.StringVar(value="")
tk.Label(warn_box, textvariable=warn_icao_var,
         bg=_WARN_OFF, fg="#ff9999",
         font=("Courier", 8), anchor="center").pack(fill="x")

_warn_flash_job = [None]
_warn_clear_job = [None]
_FLASH_MS = 450

def _do_flash():
    cur = warn_title.cget("bg")
    nxt = _WARN_ON if cur == _WARN_OFF else _WARN_OFF
    warn_title.config(bg=nxt, fg="white" if nxt == _WARN_ON else _WARN_OFF)
    warn_box.config(bg=nxt)
    _warn_flash_job[0] = root.after(_FLASH_MS, _do_flash)

def _clear_warning():
    if _warn_flash_job[0]:
        root.after_cancel(_warn_flash_job[0])
        _warn_flash_job[0] = None
    warn_title.config(bg=_WARN_OFF, fg=_WARN_OFF)
    warn_box.config(bg=_WARN_OFF)
    warn_icao_var.set("")

def trigger_warning(icao_text):
    warn_icao_var.set(icao_text)
    if _warn_clear_job[0]:
        root.after_cancel(_warn_clear_job[0])
    if _warn_flash_job[0] is None:
        _do_flash()
    _log_warn_event(icao_text)

# ── Diversion instructions panel ─────────────────────────────────────────────
_DIV_OFF = "#0a0f1a"
_DIV_ON = "#7a4a00"

div_box = tk.Frame(panel, bg=_DIV_OFF, relief="flat")
div_box.pack(fill="x", padx=4, pady=(10, 0))
tk.Label(div_box, text="── DIVERSION ──",
         bg=_DIV_OFF, fg=_DIV_OFF,
         font=("Courier", 8, "bold"), anchor="center").pack(fill="x")
div_icao_var = tk.StringVar(value="")
div_instr_var = tk.StringVar(value="")
tk.Label(div_box, textvariable=div_icao_var,
         bg=_DIV_OFF, fg="#ffd54f",
         font=("Courier", 8), anchor="center").pack(fill="x")
tk.Label(div_box, textvariable=div_instr_var,
         bg=_DIV_OFF, fg="#ffd54f",
         font=("Courier", 10, "bold"), anchor="center", wraplength=200).pack(fill="x")

def _clear_diversion():
    for w in div_box.winfo_children():
        w.config(bg=_DIV_OFF, fg=_DIV_OFF)
    div_box.config(bg=_DIV_OFF)
    div_icao_var.set("")
    div_instr_var.set("")
    # Also cancel any pending sticky-clear so we don't leak tkinter
    # after-handles after an explicit `WARN clear`.
    if _div_clear_job[0]:
        try: root.after_cancel(_div_clear_job[0])
        except Exception: pass
        _div_clear_job[0] = None

def trigger_diversion(message):
    div_icao_var.set("")
    div_instr_var.set(message)
    for w in div_box.winfo_children():
        w.config(bg=_DIV_ON,
                 fg="#ffd54f" if w.cget("font") != "Courier 8 bold" else "#ffffff")
    div_box.config(bg=_DIV_ON)

# Sticky-clear so the orange panel stays visible while frames arrive
# (controller re-fires every 3 s). On each call: paint the panel and
# (re-)arm a 4.5 s clear timer. 4.5 s comfortably exceeds the 3 s
# re-fire interval so the panel never blinks off between frames; clears
# 4.5 s after the LAST frame for the encounter.
_div_clear_job = [None]
def _trigger_diversion_sticky(msg):
    trigger_diversion(msg)
    if _div_clear_job[0]:
        try: root.after_cancel(_div_clear_job[0])
        except Exception: pass
    _div_clear_job[0] = root.after(4500, _clear_diversion)

# ── Collision warning log ─────────────────────────────────────────────────────
_warn_log = [] # [{icao, start, end|None}], newest first, max 20

tk.Label(panel, text="── WARN LOG ──", bg="#0a0f1a", fg="#546e7a",
         font=("Courier", 8, "bold")).pack(anchor="w", padx=4, pady=(12, 2))

warn_log_st = scrolledtext.ScrolledText(
    panel, height=7, width=26,
    bg="#060b12", fg="#546e7a",
    font=("Courier", 7),
    state="disabled", relief="flat", bd=0,
    wrap="none",
)
warn_log_st.pack(fill="x", padx=4)
warn_log_st.tag_config("ongoing", foreground="#ffb300")
warn_log_st.tag_config("cleared", foreground="#4caf50")

def _render_warn_log():
    warn_log_st.config(state="normal")
    warn_log_st.delete("1.0", "end")
    for e in _warn_log:
        if e["end"] is None:
            line = f"[{e['start']}] {e['icao']} → ongoing\n"
            tag = "ongoing"
        else:
            line = f"[{e['start']}] {e['icao']}\n → clr {e['end']}\n"
            tag = "cleared"
        warn_log_st.insert("end", line, tag)
    warn_log_st.config(state="disabled")

def _log_warn_event(icao):
    if any(e["icao"] == icao and e["end"] is None for e in _warn_log):
        return
    now = datetime.datetime.now().strftime("%H:%M:%S")
    _warn_log.insert(0, {"icao": icao, "start": now, "end": None})
    del _warn_log[20:]
    _render_warn_log()

def _log_clear_event():
    now = datetime.datetime.now().strftime("%H:%M:%S")
    for e in _warn_log:
        if e["end"] is None:
            e["end"] = now
            break
    _render_warn_log()

def _clear_all():
    _clear_warning()
    _clear_diversion()
    _log_clear_event()

# ── LOST CONNECTION overlay ──────────────────────────────────────────────────
# When the controller sends a CRASH BLE frame to the mobile (50 m horiz +
# 50 m vert), the mobile freezes physics for 10 s and prints "WARN crash"
# to ttyACM0. This overlay catches that line and renders a full-screen
# TV-static effect with a centred "LOST CONNECTION" box for the matching
# 10 s, then auto-clears when the mobile sends its post-reset
# "WARN clear" line.
#
# The static is a grid of _PIXEL-square rectangles whose fill toggles
# random black/white every 80 ms — much more "thematic crash" than the
# original plain-red overlay. Ported from Will's variant in
# resources/importedcode/Importedgui/willdeathscreen.py.
_PIXEL = 15 # block size for B&W static
_crash_items = [] # all canvas items in the overlay
_crash_px = [] # just the pixel rects (for animation updates)
_static_job = [None] # tkinter `after` handle for the animation loop

def _animate_static():
    """Re-roll the colour of every static pixel every 80 ms. Stops when
    `_crash_items` is empty (i.e. _clear_crash ran)."""
    if not _crash_items:
        return
    for item in _crash_px:
        canvas.itemconfig(item, fill="#ffffff" if random.random() > 0.5 else "#000000")
    _static_job[0] = root.after(80, _animate_static)

def _trigger_crash():
    global _crash_items, _crash_px
    if _crash_items:
        return
    # Tile the canvas with static pixels.
    px = []
    for row in range(0, H, _PIXEL):
        for col in range(0, W, _PIXEL):
            color = "#ffffff" if random.random() > 0.5 else "#000000"
            px.append(canvas.create_rectangle(
                col, row, col + _PIXEL, row + _PIXEL, fill=color, outline=""))
    # Centred LOST CONNECTION box on top.
    bg = canvas.create_rectangle(
        W // 2 - 340, H // 2 - 55, W // 2 + 340, H // 2 + 55,
        fill="#000000", outline="#ffffff", width=3)
    txt = canvas.create_text(W // 2, H // 2, text="LOST CONNECTION",
                             fill="white", font=("Courier", 50, "bold"))
    _crash_px = px
    _crash_items = px + [bg, txt]
    _animate_static()
    # Belt-and-suspenders: auto-clear at the same 10 s horizon even if
    # the mobile fails to send "WARN clear" after its reset.
    root.after(10000, _clear_crash)

def _clear_crash():
    global _crash_items, _crash_px
    if _static_job[0]:
        root.after_cancel(_static_job[0])
        _static_job[0] = None
    for item in _crash_items:
        try:
            canvas.delete(item)
        except Exception:
            pass
    _crash_items = []
    _crash_px = []

# ── Dot update ────────────────────────────────────────────────────────────────
def handle_line(text):
    text = text.strip()
    if not text:
        return
    clean = _strip_ansi(text)
    print(f"RX: {clean}")
    raw_var.set(clean[:38])
    pkt = _extract_json(clean)
    if pkt is None:
        if "WARN collision" in clean:
            icao = clean.split("ICAO=")[1].split()[0] if "ICAO=" in clean else ""
            root.after(0, lambda i=icao: trigger_warning(f"ICAO {i}"))
        elif clean.startswith("WARN diversion") or "WARN diversion" in clean:
            # Controller-suggested diversion. Format from codein.c:
            # "WARN diversion ICAO=xxxxxx hdg_delta=N alt_delta=M"
            # Encoding mirrors send_ble_diversion() in collision.c:
            # ±30 hdg_delta = LEFT / RIGHT
            # ±100 hdg_delta = (reserved, currently unused)
            # +127 hdg_delta = RTB sentinel
            # -127 hdg_delta = HOLD sentinel (we no longer send this)
            # alt_delta ≠ 0 = CLIMB / DESCEND (hdg_delta = 0)
            import re
            hm = re.search(r"hdg_delta=(-?\d+)", clean)
            am = re.search(r"alt_delta=(-?\d+)", clean)
            hdg_v = int(hm.group(1)) if hm else 0
            alt_v = int(am.group(1)) if am else 0
            if hdg_v == +127: label = "RETURN TO BASE"
            elif hdg_v == -127: label = "HOLD PATTERN"
            elif alt_v > 0: label = f"CLIMB {alt_v} ft"
            elif alt_v < 0: label = f"DESCEND {-alt_v} ft"
            elif hdg_v > 0: label = f"DIVERT RIGHT {hdg_v}°"
            elif hdg_v < 0: label = f"DIVERT LEFT {-hdg_v}°"
            else: label = "DIVERSION (no-op)"
            root.after(0, lambda m=label: _trigger_diversion_sticky(m))
        elif "WARN msg" in clean:
            msg = clean.split("WARN msg", 1)[1].strip()
            root.after(0, lambda m=msg: _trigger_diversion_sticky(m))
        elif "WARN crash" in clean:
            # Full-screen TV-static LOST CONNECTION overlay for 10 s.
            root.after(0, _trigger_crash)
        elif "WARN clear" in clean:
            root.after(0, _clear_all)
            # Also clear the CRASHED overlay if the mobile reset early.
            root.after(0, _clear_crash)
        return
    try:
        lat = float(pkt["lat"])
        lon = float(pkt["lon"])
        if abs(lat) > 1000:
            lat /= 1e7
            lon /= 1e7
    except (KeyError, ValueError, TypeError):
        return

    x, y = to_canvas(lat, lon)

    trail_pts.append((x, y))
    n = len(trail_pts)
    pts = list(trail_pts)
    for i, item in enumerate(trail_items):
        pt_idx = i - (TRAIL_LEN - n)
        if pt_idx >= 0:
            px, py = pts[pt_idx]
            r = 3
            canvas.coords(item, px - r, py - r, px + r, py + r)
            canvas.itemconfig(item, state="normal")
        else:
            canvas.itemconfig(item, state="hidden")

    canvas.coords(dot, x-8, y-8, x+8, y+8)
    canvas.coords(coord, x+14, y)
    canvas.itemconfig(coord, text=f"{lat:.4f}, {lon:.4f}")
    lat_var.set(f"LAT: {lat:+.5f}°")
    lon_var.set(f"LON: {lon:.5f}°")
    alt_var.set(f"ALT: {int(pkt.get('alt', 0) * 3.28084)} ft")
    pkt_count[0] += 1
    pkt_var.set(f"Packets: {pkt_count[0]}")
    status_var.set("● LIVE")
    canvas.update_idletasks()


# ── Serial reader thread ──────────────────────────────────────────────────────
def serial_reader(port):
    try:
        ser = serial.Serial(port, 115200, timeout=0.5)
        ser.dtr = True
        root.after(0, lambda: status_var.set(f"● {port}"))
        print(f"Opened {port}")
    except Exception as e:
        root.after(0, lambda msg=str(e): status_var.set(f"ERROR: {msg}"))
        print(f"Serial error: {e}")
        return

    buf = b""
    while True:
        try:
            chunk = ser.read(256)
        except Exception:
            break
        if chunk:
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                text = line.decode("utf-8", errors="replace").rstrip("\r")
                root.after(0, handle_line, text)


# ── TCP server thread ─────────────────────────────────────────────────────────
def tcp_server(port):
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", port))
    srv.listen(1)
    root.after(0, lambda: status_var.set(f"Listening TCP :{port}"))
    print(f"Listening on TCP 127.0.0.1:{port}")
    while True:
        conn, addr = srv.accept()
        print(f"Connection from {addr}")
        root.after(0, lambda: status_var.set(f"● TCP connected"))
        buf = b""
        with conn:
            while True:
                try:
                    chunk = conn.recv(256)
                except OSError:
                    break
                if not chunk:
                    break
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    text = line.decode("utf-8", errors="replace").rstrip("\r")
                    root.after(0, handle_line, text)


# ── Start background thread ───────────────────────────────────────────────────
if mode == "serial":
    threading.Thread(target=serial_reader, args=(args.port,), daemon=True).start()
else:
    threading.Thread(target=tcp_server, args=(args.tcp,), daemon=True).start()

root.mainloop()