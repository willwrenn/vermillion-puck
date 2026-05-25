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
function ac()     { echo -e "aircraft $*\r"        > /dev/ttyACM0; }
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

## BLE Frame Format

Sim mobile → controller, over GATT NOTIFY (`ble_aircraft_frame`, defined
in `sim/mobile/src/common/src/ble_packet.h`):

| Field | Type | Notes |
|---|---|---|
| `source` | uint8 | `1` = BLE_SRC_BLE_SIM |
| `flags` | uint8 | ALT_VALID / VEL_VALID / HDG_VALID bitfield |
| `icao24` | uint32 LE | Low 24 bits |
| `lat` | float64 LE | WGS-84 degrees |
| `lon` | float64 LE | WGS-84 degrees |
| `alt_ft` | int16 LE | Altitude in feet MSL |
| `vel_kt` | int16 LE | Ground speed in knots |
| `hdg_deg` | int16 LE | Heading degrees true 0–359 |
| `ts_ms` | uint32 LE | ms since boot |

Frame size: 38 bytes. Update rate: 5 Hz.

Note that `alt_ft` is in feet on the wire; the shell `aircraft set` /
`aircraft wasd q/e` commands take metres and the firmware converts on
send (× 3.28084).

Controller → sim mobile uses a separate `ble_warn_frame` with
`msg_type` bytes for collision (0x02), diversion (0x03), free-text
(0x04), and crash (0x05). See `controller/src/common/src/ble_packet.h`.

## Zephyr Files Referencing

WATCHDOG
- We use `task_wdt` like the miniproject —
- HW_STACK_PROTECTION enabled via MPU, which catches stack overflows the watchdog wouldn't.

USB/UART
- `zephyr/samples/subsys/usb/cdc_acm` — for the dual CDC ACM device tree pattern.
- `zephyr/samples/drivers/uart` — for the poll-out byte path.

BLUETOOTH
- `zephyr/samples/bluetooth/central_hr/src/main.c` — central role + GATT discovery shape.
- `zephyr/samples/bluetooth/peripheral/src/main.c` — sim side advertising loop.
- `zephyr/samples/bluetooth/mtu_update/central/src/central_mtu_update.c` — needed for ATT MTU bump (our position frames don't fit the default 23 B MTU).
- `zephyr/samples/bluetooth/mtu_update/peripheral/src/peripheral_mtu_update.c` — sim peripheral side of the MTU bump.

SHELL
- `zephyr/samples/subsys/shell/shell_module` — for the `SHELL_STATIC_SUBCMD_SET_CREATE` nested-tree pattern we use for `skywatch <subcmd> [args]`.

JSON
- `zephyr/samples/subsys/lib/json` — for the JSON_OBJ_DESCR + json_obj_parse pattern we use to parse ADS-B frames coming off ttyACM1.

# Libraries Used

### FIRMWARE (Standard)
#### math.h
- `sin()`, `cos()`, `atan2()` — bearing math for missile pursuit + diversion picker.
- `sqrt()`, `fabs()` — closest-approach geometry (`|Δr + Δv·t*|`).
- `cos(lat*π/180)` — metres-per-degree-longitude scaling for lat/lon ↔ metric.
- `fmod()` — angle wrap.
- `isfinite()` — guard against NaN/Inf TCA when relative velocity is zero.

#### string.h
- `memcpy()`, `memset()` — clearing snap rows + Kalman matrices each tick.
- `strncpy()`, `strncmp()` — ICAO copy + lookup (always BUF_LEN-1 + explicit NUL after Phase 12 strncpy fix).

#### stdlib.h
- `strtod()`, `strtol()` — shell arg parsing for `skywatch collision ghost [tca_s] [miss_m]` etc.
- `abs()` — signed altitude diff.

#### stdio.h
- `snprintf()` — JSON serialisation for the data port.

#### stdbool.h / stdint.h
- Standard integer + boolean types throughout.

### FIRMWARE (Zephyr)
- `zephyr/kernel.h` — `K_THREAD_DEFINE`, `k_work_delayable` for periodic collision/missile/ghost timers, `k_mutex` for the aircraft_db, `K_SPINLOCK` for stats counters across the ISR/work-queue boundary, `k_uptime_get()` for wall-clock ms.
- `zephyr/sys/slist.h` — `sys_slist_t` + `sys_slist_for_each_container()` for the aircraft_db linked list.
- `zephyr/logging/log.h` — `LOG_INF` / `LOG_WRN` / `LOG_DBG` per-module logging.
- `zephyr/bluetooth/bluetooth.h` — `bt_enable()`, scanning, address parsing.
- `zephyr/bluetooth/conn.h` — `bt_conn_le_create()` + connection callbacks for the central role.
- `zephyr/bluetooth/gatt.h` — `bt_gatt_discover()`, `bt_gatt_subscribe()`, `bt_gatt_write_without_response()` for the two-stage discovery (SkyWatch service for NOTIFY, Warning service for WRITE).
- `zephyr/bluetooth/hci.h` — disconnect reason constants.
- `zephyr/bluetooth/uuid.h` — `BT_UUID_INIT_128()` for both custom services.
- `zephyr/data/json.h` — `json_obj_parse` for the ADS-B ingest path.
- `zephyr/shell/shell.h` — `SHELL_CMD_REGISTER`, `SHELL_STATIC_SUBCMD_SET_CREATE`, `shell_print` for the `skywatch ...` tree.
- `zephyr/usb/usbd.h` — `USBD_DEVICE_DEFINE` + manual usbd_add_configuration for dual CDC ACM (we control enumeration order so ACM0 = shell + ACM1 = data deterministically).
- `zephyr/drivers/uart.h` — `uart_poll_out`, `uart_line_ctrl_get` for DTR detection on the data port.
- `zephyr/drivers/gpio.h` — `gpio_dt_spec`, `gpio_pin_configure_dt`, `gpio_pin_toggle_dt` for the heartbeat LED.
- `zephyr/device.h` — `DEVICE_DT_GET` + `device_is_ready` for the data UART.
- `zephyr/init.h` — `SYS_INIT` priority chain for USB device → app threads ordering.

### HOST (Python)
- PyQt6 — main ATC GUI (QGraphicsScene for the map, QTableWidget for the aircraft + collision-history dashboards).
- tkinter — sim PC GUI (`simple_grid.py`).
- pyserial — both GUIs read CDC-ACM JSON via this.
- paho-mqtt — collision-event publish to Mosquitto.
- influxdb-client — collision-event writes to InfluxDB Cloud (HTTPS REST).
- Pillow (PIL) — Brisbane basemap PNG decode for both GUIs.
- requests — sdr_bridge.py pulls dump1090 over HTTP.
- collections.deque — per-aircraft position trail buffer (50 points, alpha-fade).
- json, threading, queue — frame dispatcher + background-thread fan-out.
- math, hashlib, re — geometry math + per-ICAO hash colour assignment + log line parsing.

## Zephyr References

### BLE
- https://docs.zephyrproject.org/latest/connectivity/bluetooth/index.html
- https://docs.zephyrproject.org/latest/connectivity/bluetooth/api/gatt.html
- https://docs.zephyrproject.org/latest/samples/bluetooth/central_hr/README.html
- https://docs.zephyrproject.org/latest/samples/bluetooth/peripheral/README.html
- https://docs.zephyrproject.org/latest/samples/bluetooth/mtu_update/README.html

### SHELL
- https://docs.zephyrproject.org/latest/services/shell/index.html

### LOGGING
- https://docs.zephyrproject.org/latest/services/logging/index.html

### JSON
- https://docs.zephyrproject.org/latest/services/serialization/json.html

### USB
- https://docs.zephyrproject.org/latest/connectivity/usb/device/usb_device.html

### MESSAGE QUEUE / WORK QUEUES
- https://docs.zephyrproject.org/latest/kernel/services/data_passing/message_queues.html
- https://docs.zephyrproject.org/latest/kernel/services/threads/workqueue.html

### UART
- https://docs.zephyrproject.org/latest/hardware/peripherals/uart.html

### GPIO
- https://docs.zephyrproject.org/latest/hardware/peripherals/gpio.html

## Self References
Used the miniproject Kalman filter design + node_db slist pattern + dual-CDC ACM device setup pretty heavily — we ported all of those forward into this project. Shell command structure also lifted from miniproject's base node.

## External References

- ADS-B / Mode-S  for understanding what dump1090 actually decodes off the SDR before it gets to us.
- dump1090  JSON aircraft endpoint format — for the bridge script's parser.
- CartoDB Positron tile server — for the greyscale Brisbane basemap (bake script in `scripts/stage6/fetch_basemap.py` of the host workspace).


## AI Referencing

### controller/src/main/main.c
- Helped structure the boot-init priority chain (USB → DB → handlers → bridge → BLE → publisher → collision) and the heartbeat LED toggle in the main loop.

### controller/src/shell/shell.c
- Built the nested subcommand structure (`skywatch ...` → seven subcommand groups).
- Helped with `strtod`/`atoi` arg parsing for the ghost / missile / diversion commands.
- Wrote the `imm_dump_cb` style iterator pattern (later removed when IMM was reverted).
- Suggested splitting `inject` / `inject_stop` rather than reusing arg parsing.

### controller/src/ble/ble_central.c
- Designed the two-stage GATT discovery (SkyWatch service for NOTIFY then Warning service for WRITE), since the controller is BOTH a consumer and a producer over BLE.
- Wrote the MTU exchange + the watchdog auto-reconnect scan loop.
- Helped debug the connection_create error code handling.
- `ble_central_send_warning()` packing of the 16-byte legacy frame including the XOR CRC.

### controller/src/bridge/bridge.c
- Suggested the msgq + worker-thread pattern so the BLE notify callback (BT work-queue context) doesn't do struct conversion / `aircraft_db_upsert` inline.
- Helped with validation (icao24 mask, lat/lon range) before the upsert call.

### controller/src/aircraft_db/aircraft_db.c
- Ported the miniproject's slist + stale-reaper pattern.
- Helped structure the per-entry Kalman state field + the upsert vs insert branches.
- The stale-reaper (10 s timeout) was suggested by AI after we saw stuck entries during the first BLE disconnect test.

### controller/src/json/json_parser.c
- Helped write the `JSON_OBJ_DESCR_*` descriptors matching `resources/standards/json_protocol.md`.
- Suggested checking the parser-return bitmask to know which optional fields were present (alt/vel/hdg can each be absent on partial ADS-B fixes).

### controller/src/db_publisher/db_publisher.c
- Wrote the snapshot-under-mutex / emit-after-release pattern so the publisher doesn't hold the DB lock during USB-CDC writes.
- Helped with the optional-field snprintf (only emit `alt` / `vel` / `hdg` / `pred_lat,pred_lon` if their valid bits are set).

### controller/src/kalman/kalman.c
- Port of the miniproject 2-D Kalman to 4-D lat/lon state.
- Helped with the per-axis Q construction (different metres-per-degree on lat vs lon at Brisbane latitude).
- Wrote the closed-form 2×2 inversion for the K gain so we don't need a generic NxN solver.
- Predict-ahead helper for the 10 s projection arrow on the GUI.

### controller/src/collision/collision.c
- Designed the `closest_approach()` math (Δr · Δv / |Δv|² → TCA → min_sep at TCA).
- Threshold structure for ADVISORY (1 km / 60 s) / WARNING (500 m / 30 s) / CRASH (100 m / 100 m horizontal+vertical).
- Helped write the per-pair transition tracker + the 15-s CRASH lock that prevents oscillation as a missile passes through a frozen sim.
- Diversion picker (geometry → LEFT/RIGHT/CLIMB/DESCEND/RTB).
- Predictive CRASH branch for fast-moving missiles that would otherwise pass through the 100 m zone between ticks (added Phase 12 when missiles at 750 kt were missed on first pass).

### controller/src/missile/missile.c
- Pure-pursuit guidance math (desired heading = bearing-to-target, clamped to ±turn_rate × dt).
- Helped with the metric-frame dead-reckoning so the missile actually moves between ticks instead of getting reset.
- Configurable TTL / turn rate / speed args so we can demo slow vs hard difficulty.

### controller/src/usb_device/usb_device.c
- IAD class triple + USBD configuration for dual CDC ACM on a single USB device. We had no example for this anywhere — pure AI assist.

### controller/src/usb_serial/usb_serial.c
- DTR-gated `uart_poll_out` loop with a mutex around the entire write so concurrent producers (collision detector + db_publisher + missile worker) don't interleave at character granularity.

### controller/src/usb_handler/usb_handler.c
- Interrupt-driven Rx with a ring buffer and a line-reassembly state machine. AI helped with the ring-buffer-full / line-overrun stat counters.

### controller/prj.conf
- AI walked through the Kconfig flags needed for BLE central + GATT client + dual CDC + JSON parser + FPU sharing. Big help for the initial bring-up since we had no working baseline.

### controller/app.overlay
- Overlay for the second `cdc_acm_uart1` instance plus the chosen-node remap for the shell-uart staying on ACM0.

### controller/host/atc_gui.py
- Wrote the QGraphicsScene-based map widget with the basemap + lat/lon grid + per-aircraft polygon items.
- Suggested the per-ICAO hash-based colour scheme so the same aircraft is the same colour across runs.
- Built the collision-history dashboard model + table widget (sticky `level_max`, FIFO 32-cap, `actual_sep_m` minimum tracking).
- Mouse-wheel zoom + click-to-track + `Esc` keybinding code.
- Q signal/slot dispatcher so the serial reader thread can safely push frames to the GUI.
- LOST CONNECTION CRASH overlay (ported in from Will's sim GUI variant).

### controller/host/mqtt_publisher.py
- Background-thread + queue + last-will pattern. Helped with the paho-mqtt v1-vs-v2 callback API compatibility shim.

### controller/host/influx_writer.py
- Background-thread InfluxDB Cloud writer with a heartbeat tick so the dashboard always has fresh data even when no collisions are happening. AI helped with the Point / Tag / Field schema choice.

### controller/host/run_skywatch.sh
- Pre-flight check structure (dump1090 reachable / Python deps importable / mosquitto running / InfluxDB env set) before spawning the GUI.

### sim/mobile/src/main/codein.c
- Per-msg-type GATT write handler (0x02 collision / 0x03 diversion / 0x04 freetext / 0x05 crash). XOR CRC verify.
- CRASH freeze + `crash_reset_work` delayable that snaps the aircraft back to St Lucia after 10 s.

### sim/pc/simple_grid.py
- The diversion-panel + collision-warning-flash machinery.
- TV-static LOST CONNECTION overlay (`_animate_static` with 80 ms re-roll of pixel-grid colours).
- Sticky-clear timer pattern for the diversion panel so it doesn't blink between controller re-fires.

### General
- Code commenting + module-header style. AI consistently nagged us to add purpose blocks at the top of each `.c` file — we mostly went along with it.
