# CSSE4011 Final Project — SkyWatch ATC

## Overview
- **Sim mobile node** — Xiao-nRF52840 BLE peripheral, broadcasts its own aircraft position (lat/lon/alt/spd/hdg) over a custom GATT NOTIFY characteristic. Driven by wasd-style shell commands from a PC terminal, or auto-circles via `aircraft circle <kt> <m>`.
- **Controller node** — Xiao-nRF52840 BLE central, connects to the sim, also ingests real ADS-B JSON from dump1090 over USB-CDC, fuses both sources into a per-aircraft Kalman filter, runs a pairwise closest-approach collision detector at 5 Hz (ADVISORY/WARNING/CRASH levels), fires BLE diversion suggestions back to the sim, plus a pure-pursuit guided-missile sandbox.
- **PC (controller side)** — PyQt6 ATC GUI (`controller/host/atc_gui.py`) reads the JSON stream, plots aircraft on a Brisbane basemap with per-ICAO colours, runs a live collision-history dashboard, mirror-publishes events to MQTT + InfluxDB Cloud.
- **PC (sim side)** — tkinter GUI (`sim/pc/simple_grid.py`) shows the sim aircraft on a Brisbane background, flashes a CRASH overlay when it dies.

```
   SDR (NESDR + dump1090)                Sim mobile (Xiao + Zephyr)
        │                                     │ BLE GATT
        ▼ HTTP-JSON          USB-CDC          ▼
   sdr_bridge.py ─────────► CONTROLLER (Xiao + Zephyr)
                                  ├─ aircraft_db (slist + 4D Kalman)
                                  ├─ collision detector (pairwise CA, 5 Hz)
                                  ├─ missile / ghost / diversion sandboxes
                                  └─ dual CDC-ACM:
                                       ACM0 — Zephyr shell  (`skywatch ...`)
                                       ACM1 — JSON data stream
                                              │
                                              ▼
                                  controller/host/atc_gui.py (PyQt6)
                                       ├─ map + table + collision dashboard
                                       ├─ MQTT publish → Mosquitto
                                       └─ InfluxDB Cloud writes → dashboard
   sim/pc/simple_grid.py (tkinter)
        ▲ JSON + WARN lines over USB-CDC ttyACM0 (sim — single port,
        │                                          shell + data multiplexed)
        └─ Brisbane map + position trail + DIVERSION panel +
           full-screen TV-static "LOST CONNECTION" overlay on CRASH
```

Map bounds default to ±0.5° around Brisbane (-27.4975, 153.0137). Origin "St Lucia" used for `aircraft reset` and as the missile launch site.

# Setup

## General Flash instructions
1. Build the relevant subsystem to its `/build` folder.

2. Plug board into USB.
3. From Admin PowerShell run `usbipd list` to find the busid `x-y`.
4. Attach to WSL: `usbipd attach --wsl --busid x-y`.

5. Double-tap reset on the board. XIAO-SENSE drive should mount under `/media/$USER/`. Then drag-drop the UF2 (or use `west flash`):

CONTROLLER:
```bash
west build -p -b xiao_ble/nrf52840/sense -d firmware/finalproject/vermillion-puck/controller/build firmware/finalproject/vermillion-puck/controller
west flash --runner uf2 -d firmware/finalproject/vermillion-puck/controller/build
```

SIM:
```bash
west build -p -b xiao_ble/nrf52840/sense -d firmware/finalproject/vermillion-puck/sim/mobile/build firmware/finalproject/vermillion-puck/sim/mobile
west flash --runner uf2 -d firmware/finalproject/vermillion-puck/sim/mobile/build
```

6. Wait for green LED on the controller — solid + slow blink = healthy. Sim has no heartbeat LED but its red LED flashes on incoming warnings.
7. **Flash one board at a time** — both boards mount as XIAO-SENSE so you can't tell them apart in bootloader mode.
8. After flash, board often falls off WSL — `usbipd attach` again. If you need PuTTY, find the COM port in Windows Device Manager and use serial mode at 115200.

## Controller Node Setup (Laptop A)
1. Follow general flash for controller, leave attached to Laptop A.
2. `ls /dev/ttyACM*` in WSL — you should see ACM0 (shell) + ACM1 (data). With only the controller plugged in there'll be exactly two ports.
3. `screen /dev/ttyACM0 115200` to get the `uart:~$` prompt. Press Enter once.
4. Head to PC GUI section to launch the visual side (`atc_gui.py` runs on Laptop A reading ACM1).

## Sim Mobile Node (Laptop B)
1. Follow general flash for sim. Can run off USB battery for portability — though for the demo we keep it plugged into Laptop B so the sim shell + tkinter GUI are reachable.
2. Sim auto-advertises as `skywatch-mobile` over BLE — the controller (Laptop A) auto-discovers and connects. **From Laptop A's controller shell (`screen /dev/ttyACM0`):**

```
skywatch ble stats
```

Should show `connected=yes` once the sim is in range. If not, give it ~2 s — the auto-reconnect watchdog scans every 2 s. Range is line-of-sight ~10 m with the on-PCB antennas; both laptops can be on the same desk.

3. **From Laptop B**, verify sim is sending position via `screen /dev/ttyACM0 115200` (sim's shell + data port — single port on the sim laptop). You should see:

```json
{"lat":-274975000,"lon":1530137000,"alt":100,"spd":0,"hdg":0}
```

## PC GUI

### Controller GUI (PyQt6 — our side)
From the repo root:

```bash
bash controller/host/run_skywatch.sh --no-sdr
```

`--no-sdr` skips the dump1090 reachability check. Drop the flag if you have the SDR antenna + dump1090 running. The script auto-detects the data port (override with `SKYWATCH_PORT=/dev/ttyACMn`), spawns the SDR bridge if `SDR_BRIDGE_PY` is exported, then launches `atc_gui.py`.

Mouse-wheel zoom, click an aircraft to track it, `0` to reset view. Bottom status bar shows lines-in / parse-err / schema-err counters.

### Sim GUI (tkinter — Will's side, on Laptop B)

```bash
cd sim/pc
python3 simple_grid.py --port /dev/ttyACM0 --map staticmap.jpeg
```

Aircraft shows as a red dot with a position trail. On CRASH the whole window fills with TV-static "LOST CONNECTION" for 10 s, then auto-clears once the sim respawns at St Lucia.

(If you're running both boards on a single laptop, the sim port is `ACM2` instead — see the two-laptop note below.)

### Two-laptop demo layout

For the marker demo we run **one laptop per board** — keeps the USB / WSL stuff dead simple and means each side only ever sees its own board's two CDC ports.

**Laptop A (controller node — Blaise's):**
| Port | What it does |
|------|--------------|
| `/dev/ttyACM0` | Controller Zephyr shell + log backend |
| `/dev/ttyACM1` | Controller JSON data port (aircraft + collision frames) — what `atc_gui.py` reads |

**Laptop B (sim node — Will's):**
| Port | What it does |
|------|--------------|
| `/dev/ttyACM0` | Sim Zephyr shell + position JSON output multiplexed on the same port — `simple_grid.py` reads it, `aircraft ...` commands go to it |

Note the ACM0/ACM1 numbering is local to each laptop. If you happen to run both on the same laptop (dev mode), the sim port gets pushed to ACM2 — see the bottom of this file for that case.

The two laptops talk over **BLE only** between the boards — no networking between laptops needed. The host PCs are independent: each runs its own GUI against its own board.

WSL renumbers ports after every flash. If unsure: `timeout 2 head -c 200 /dev/ttyACMn` and see what's coming out.

### Sim aircraft helper functions (Laptop B)
Paste once per terminal session **on Laptop B**:

```bash
function ac()     { echo -e "aircraft $*\r" > /dev/ttyACM0; }
function circle() { echo -e "aircraft circle $*\r" > /dev/ttyACM0; }
```

Then `ac wasd w`, `ac wasd d`, `circle 80 800`, `ac reset`, etc.

(If you run sim + controller on the same laptop, sim shell ends up on
`/dev/ttyACM2` instead of `ACM0` — adjust the helpers accordingly.)

# Operation

## 5. Useful Shell Commands (controller node)

| Command | What it does |
|---------|-------------|
| `skywatch ping` | Reply `pong` (liveness check) |
| `skywatch ble stats` | Connection state + scan/discovery counters |
| `skywatch db dump` | List all aircraft in the DB |
| `skywatch collision stats` | Tick + pair count + advisory/warning counters |
| `skywatch collision inject` | Spawn two synthetic FICT aircraft converging head-on (no BLE needed) |
| `skywatch collision crash <icao_a> <icao_b>` | Force-fire a CRASH event between two ICAOs already in the DB |
| `skywatch collision stop` | Halt the inject keep-alive |
| `skywatch collision ghost [tca_s] [miss_m] [alt_ft]` | Spawn FICT aircraft on intercept with the BLE sim (default 23 s TCA, 0 m miss, alt-match) |
| `skywatch collision ghost_at <icao> [tca_s] [miss_m] [alt_ft]` | Same but target any ICAO (BLE / ADS-B / FICT) |
| `skywatch collision ghost_stop` | Cancel current ghost |
| `skywatch missile launch [ttl_s] [turn_rate_dps] [speed_kt]` | Launch a pure-pursuit missile from UQ St Lucia at the BLE sim (default 60 s / 15°/s / 500 kt) |
| `skywatch missile cancel` | Abort the in-flight missile |
| `skywatch diversion <left\|right\|climb\|descend\|rtb\|hold>` | Manually fire a diversion suggestion to the sim |
| `skywatch kalman q <m/s²>` | Tune Kalman process noise (default 0.5) |
| `skywatch kalman r <m>` | Tune Kalman measurement noise (default 10) |
| `skywatch kalman bench` | 1000 predict+update iterations, <5 ms target |
| `skywatch usb stats` | Data-port Rx counters (bytes, frames, ringbuf drops) |

## 5b. Useful Shell Commands (sim node)

| Command | What it does |
|---------|-------------|
| `aircraft status` | Print current lat/lon/alt/heading/speed |
| `aircraft reset` | Snap back to St Lucia origin (also clears warnings) |
| `aircraft wasd w` | +10 kt |
| `aircraft wasd s` | -10 kt |
| `aircraft wasd a` | -10° (left turn) |
| `aircraft wasd d` | +10° (right turn) |
| `aircraft wasd q` | +50 m altitude |
| `aircraft wasd e` | -50 m altitude |
| `aircraft set <lat_e7> <lon_e7> <alt_m> <kt> <hdg>` | Set exact state (lat/lon × 1e7) |
| `aircraft circle <kt> <radius_m>` | Auto-fly circles (0 to stop) |

