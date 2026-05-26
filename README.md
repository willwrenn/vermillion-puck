# SkyWatch ATC — CSSE4011 final project

Miniature air-traffic-control demo running on two Xiao-nRF52840 boards
(Zephyr RTOS) plus a PyQt6 host GUI. Real ADS-B aircraft from an
SDR (optional) are fused live with a BLE-driven simulated aircraft;
collision detection, guided-missile sandbox, MQTT/InfluxDB sidecars,
and a Tk "LOST CONNECTION" death screen on the sim side.

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
        ▲ JSON + WARN lines over USB-CDC ttyACM3 (sim data port)
        └─ Brisbane map + position trail + DIVERSION panel +
           full-screen TV-static "LOST CONNECTION" overlay on CRASH
   screen /dev/ttyACM2 → aircraft circle / set / wasd / reset (sim shell)
```

Repo layout (only what's tracked):

```
vermillion-puck/
├── controller/                Zephyr controller firmware + host GUI
│   ├── CMakeLists.txt
│   ├── prj.conf
│   ├── app.overlay            Dual cdc_acm_uart{0,1}
│   ├── src/
│   │   ├── main/              Entry + heartbeat LED
│   │   ├── shell/             `skywatch ...` subcommand tree on ACM0
│   │   ├── usb_device, usb_serial, usb_handler
│   │   ├── json/              Zephyr JSON parser
│   │   ├── ble/               BLE central + GATT discovery + warning Tx
│   │   ├── bridge/            BLE frame → aircraft_db
│   │   ├── aircraft_db/       slist DB + stale reaper + per-a/c KF
│   │   ├── db_publisher/      2 Hz snapshot → ACM1 JSON
│   │   ├── kalman/            4D constant-velocity Kalman filter
│   │   ├── collision/         pairwise CA detector + CRASH + diversion
│   │   ├── missile/           pure-pursuit guided FICT aircraft
│   │   └── common/            shared headers (aircraft_t, ble_packet)
│   └── host/
│       ├── atc_gui.py         PyQt6 ATC GUI (~1300 lines)
│       ├── mqtt_publisher.py  paho-mqtt background thread
│       ├── influx_writer.py   InfluxDB Cloud background thread
│       ├── run_skywatch.sh    One-shot launcher (this is what you run)
│       ├── assets/            Brisbane greyscale basemap PNG
│       └── scripts/
│           ├── test_pipeline.py  Synthetic CollisionFrame burst (Phase 10 verify)
│           └── SETUP.md          Mosquitto + InfluxDB dashboard recipe
├── sim/                       Will's sim mobile + tkinter PC GUI
│   ├── mobile/                Zephyr firmware (BLE peripheral)
│   └── pc/
│       ├── simple_grid.py     tkinter map GUI for the sim aircraft
│       ├── staticmap.jpeg     Brisbane background image
│       └── run_dashboard.sh
├── resources/standards/       Wire-format specs — single source of truth
│   ├── json_protocol.{md,py}    AircraftFrame + CollisionFrame schema
│   │                            (the .py is the runtime validator the GUI
│   │                            imports — DO NOT delete or the GUI won't start)
│   ├── ble_protocol.md          BLE GATT services + frame layouts
│   └── fusion.md                ADS-B + BLE fusion design notes
└── POSTER_FINAL_PROJECT.pptx  Presentation poster (final submission)
```

---

## Prerequisites (host)

```bash
sudo apt install -y mosquitto mosquitto-clients socat   # optional: MQTT
sudo systemctl enable --now mosquitto                   # ditto
pip install --break-system-packages \
    pyserial PyQt6 requests paho-mqtt influxdb-client Pillow
```

Zephyr toolchain (controller + sim firmware): standard `west` setup
with the Zephyr SDK. Boards: **`xiao_ble/nrf52840/sense`** for both.

---

## Build the firmware

```bash
# from your zephyr workspace, with zephyr-env.sh sourced:
west build -b xiao_ble/nrf52840/sense -d <build-dir>/controller \
    <path-to>/vermillion-puck/controller
west build -b xiao_ble/nrf52840/sense -d <build-dir>/sim \
    <path-to>/vermillion-puck/sim/mobile
```

Outputs: `<build-dir>/.../zephyr/zephyr.uf2` for each.

Sizes (current release): controller ~234 KB FLASH / 94 KB RAM; sim mobile
~195 KB FLASH / 76 KB RAM. Both well within the Xiao's 788 KB / 256 KB.

---

## Flash

Each Xiao enters the **Adafruit UF2 bootloader** on **double-tap reset**.
A `XIAO-SENSE` mass-storage volume mounts; drop the `.uf2` in and the
board reboots into the new firmware.

```bash
# Controller:
cp <build-dir>/controller/zephyr/zephyr.uf2 /media/<user>/XIAO-SENSE/ && sync
# Sim mobile (separately — both boards mount as XIAO-SENSE, so flash one at a time):
cp <build-dir>/sim/zephyr/zephyr.uf2 /media/<user>/XIAO-SENSE/ && sync
```

After flash the board re-enumerates with VID **`2fe3:0005`** (Zephyr CDC).
**Green LED** solid + slow-blinking = controller's main loop is healthy.

---

## Run the GUI

Two boards yields four `/dev/ttyACMn` ports. Standard mapping after a
clean attach:

| Port | Board | Role |
|---|---|---|
| `/dev/ttyACM0` | Controller | shell — `skywatch ...` commands |
| `/dev/ttyACM1` | Controller | data — JSON aircraft + collision frames |
| `/dev/ttyACM2` | Sim mobile | shell — `aircraft ...` commands |
| `/dev/ttyACM3` | Sim mobile | data — JSON position + WARN lines |

(Numbers shift each time a board re-enumerates — identify by sniffing output.)

### Launch options

**Full demo with SDR** (requires dump1090 running on the host + your own
copy of `sdr_bridge.py` — see `SDR_BRIDGE_PY` env var):

```bash
bash controller/host/run_skywatch.sh
```

**Controller + sim only, no SDR antenna**:

```bash
bash controller/host/run_skywatch.sh --no-sdr
```

**Manual launch** (no helper script):

```bash
python3 controller/host/atc_gui.py --port /dev/ttyACM1
```

**Sim GUI** (separate terminal):

```bash
cd sim/pc && python3 simple_grid.py --port /dev/ttyACM3 --map staticmap.jpeg
```

**Sim aircraft control** — open the sim shell:

```bash
screen /dev/ttyACM2 115200
# press Enter to get the uart:~$ prompt, then type aircraft commands directly
```

#### Position + flight parameters

```bash
ac set <lat_e7> <lon_e7> <alt_m> <speed_kt> <heading_deg>
```

| Parameter | Unit | Example |
|---|---|---|
| `lat_e7` | degrees × 1e7 | `-274975000` |
| `lon_e7` | degrees × 1e7 | `1530137000` |
| `alt_m` | metres | `500` |
| `speed_kt` | knots | `60` |
| `heading_deg` | degrees 0–359 | `90` |

```bash
ac set -274975000 1530137000 500 60 90   # St Lucia, 500 m, 60 kt heading east
```

#### WASD nudge

```bash
ac wasd w    # speed +10 kt
ac wasd s    # speed -10 kt
ac wasd a    # heading -10°
ac wasd d    # heading +10°
ac wasd q    # altitude +50 m
ac wasd e    # altitude -50 m
```

#### Circle mode

```bash
circle 60 1000     # 60 kt, 1000 m radius
circle 120 3000    # 120 kt, 3 km radius (wider turns)
circle 60 300      # 60 kt, 300 m radius (tight)
circle 0           # stop circling, hold current heading
```

#### Other

```bash
ac reset             # snap back to St Lucia, zero speed
ac status            # print current position as JSON
```

---

## Test (live demo scenarios)

All from the **controller shell** — open it with:

```bash
screen /dev/ttyACM0 115200
# press Enter to get the `uart:~$` prompt
```

### Collision tests (no SDR / no sim needed)

```text
skywatch collision inject               # spawn c011a1 + c011b2 converging @ 250 kt
                                        # → ~25 s later they collide
                                        # → yellow ADVISORY → red WARNING → dark-red CRASHED
skywatch collision stop                 # halt the keep-alive
skywatch collision crash c011a1 c011b2  # manual CRASH override on any pair in the DB
```

### Collision tests (with the BLE sim)

```text
skywatch collision ghost 30 800 -1      # ADVISORY @ 800 m, 30 s TCA
skywatch collision ghost 20 150 -1      # WARNING @ 150 m
skywatch collision ghost 15 0   -1      # direct CRASH
skywatch collision ghost_stop

# Or target any other aircraft (BLE / ADS-B / FICT) by ICAO:
skywatch db dump                        # list ICAOs in the DB
skywatch collision ghost_at <icao> [tca_s] [miss_m] [alt_ft]
```

### Missile sandbox (needs the BLE sim)

```text
skywatch missile launch 60 6 300        # SLOW — easy to dodge (60s TTL, 6°/s turn, 300 kt)
skywatch missile launch 60 15 500       # MED  — defaults
skywatch missile launch 30 25 750       # HARD — Will basically must climb to escape
skywatch missile cancel                 # abort
```

### Diversion suggestions (manual)

```text
skywatch diversion left | right | climb | descend | rtb | hold
# → Sim's Tk GUI shows orange DIVERSION panel + suggested manoeuvre.
# (Diversion also auto-fires on any ADVISORY-level encounter.)
```

### Filter + DB inspection

```text
skywatch ble stats          # BLE central state + scan/connect counters
skywatch db dump            # current aircraft DB (live)
skywatch kalman bench       # 1000 predict+update iterations (target <5 ms)
skywatch kalman show        # current process/measurement noise (q, r)
skywatch collision stats    # ticks / pairs checked / advisories / warnings
```

---

## BLE Frame Format

Sim mobile → controller over GATT NOTIFY (`ble_aircraft_frame`, defined in `sim/mobile/src/common/src/ble_packet.h`):

| Field | Type | Notes |
|---|---|---|
| `source` | uint8 | `1` = BLE_SRC_BLE_SIM |
| `flags` | uint8 | ALT_VALID, VEL_VALID, HDG_VALID |
| `icao24` | uint32 LE | Low 24 bits |
| `lat` | float64 LE | WGS-84 degrees |
| `lon` | float64 LE | WGS-84 degrees |
| `alt_ft` | int16 LE | Altitude in **feet** MSL |
| `vel_kt` | int16 LE | Ground speed in knots |
| `hdg_deg` | int16 LE | Heading degrees true 0–359 |
| `ts_ms` | uint32 LE | ms since boot |

Frame size: **38 bytes**. Update rate: **5 Hz**.

> `alt_ft` is feet. Shell commands (`ac set`, `ac wasd q/e`) use metres — firmware converts on send (metres × 3.28084).

---

## GUI Map Bounds

Both GUIs cover the greater Brisbane area:

| Corner | Latitude | Longitude |
|---|---|---|
| Top-left | -27.34° | 152.83° |
| Bottom-right | -27.535° | 153.21° |

---

## Optional — InfluxDB Cloud dashboard

`run_skywatch.sh` exports the InfluxDB credentials automatically. The
PyQt GUI's `influx_writer.py` writes every CollisionFrame to bucket
`skywatch` (measurement `skywatch_events`) with tags `pair / level /
icao_a / icao_b` and fields `tca_s / min_sep_m / actual_sep_m /
alt_diff_ft / controller_ts`. Build a dashboard in InfluxDB Cloud UI
(SQL on Serverless or Flux on TSM) — see `resources/handover.md` in
the host workspace for ready-to-paste queries.

---

## Optional — MQTT live monitor

```bash
mosquitto_sub -t 'skywatch/#' -v
# → live stream of every collision frame as the controller emits it.
```

Topics: `skywatch/collision` (every frame) + `skywatch/collision/{pair}`
(per-pair retained) + `skywatch/status` (online / offline last-will).

---

## Running on a second laptop from scratch (e.g. Will's machine)

Everything you need is in this repo. Assuming you have **both** Xiao
boards + a Linux/WSL host with Zephyr SDK + Python 3.10+:

```bash
# 1. Clone:
git clone https://github.com/willwrenn/vermillion-puck.git
cd vermillion-puck

# 2. Host deps (one-time):
sudo apt install -y mosquitto mosquitto-clients
sudo systemctl enable --now mosquitto
pip install --break-system-packages \
    pyserial PyQt6 requests paho-mqtt influxdb-client Pillow

# 3. Build both firmwares (with zephyr-env.sh sourced):
west build -b xiao_ble/nrf52840/sense -d /tmp/sw/controller controller
west build -b xiao_ble/nrf52840/sense -d /tmp/sw/sim        sim/mobile

# 4. Flash — ONE AT A TIME (both boards mount as XIAO-SENSE in
#    bootloader). Double-tap reset on the target board → drag-drop:
cp /tmp/sw/controller/zephyr/zephyr.uf2 /media/$USER/XIAO-SENSE/ && sync
# (wait for green LED, then double-tap reset on the OTHER board)
cp /tmp/sw/sim/zephyr/zephyr.uf2        /media/$USER/XIAO-SENSE/ && sync

# 5. Sanity-check the host pipeline WITHOUT firmware running:
source <(grep '^export INFLUXDB' controller/host/run_skywatch.sh)
python3 controller/host/scripts/test_pipeline.py
# → 6 synthetic CollisionFrames go to MQTT (broker) + InfluxDB.
# → Confirm by: mosquitto_sub -t 'skywatch/#' -v
# → And in InfluxDB Cloud Data Explorer:
#   SELECT * FROM "skywatch_events" WHERE time >= now() - interval '5 minutes'

# 6. Run the full stack in four terminals:
#    A — our PyQt ATC GUI (close window or Ctrl-C to quit):
bash controller/host/run_skywatch.sh --no-sdr
#    B — controller's Zephyr shell (for `skywatch ...` commands):
screen /dev/ttyACM0 115200
#    C — your tkinter sim GUI:
cd sim/pc && python3 simple_grid.py --port /dev/ttyACM3 --map staticmap.jpeg
#    D — sim aircraft control (Zephyr shell on ACM2):
screen /dev/ttyACM2 115200
```

That's the whole bring-up. From here, drive the scenarios in §**Test**
above. **No external scripts or out-of-repo dependencies required.**

WSL note: boards have to be `usbipd attach --wsl --busid X-Y`'d from
Windows-side Admin PowerShell before WSL sees them as /dev/ttyACM*.
If a board drops off WSL after a bootloader excursion, re-attach.

---

## Project status

Phases 1–12 complete, builds clean, all scenarios above demoed live.

The Phase 9 IMM Kalman experiment (CV + coordinated-turn filters
blended via Markov mixing, exposed as a `skywatch imm on|off` runtime
toggle) was prototyped, C-ported, and verified to beat pure-CV by
~16× RMSE on a synthetic 90° turn track. We removed it during live
testing — on real ADS-B traffic the IMM's curved predictions were
*less* accurate than pure-CV (commercial aircraft fly straighter than
the CT model assumes), and the extra ~14 KB of per-aircraft RAM
wasn't worth keeping for a dormant feature. The CV-only Kalman from
Phase 7 is the production predictor; see `git log` for the design +
reversion commits.

---

## Hardware

- 2× Seeed **Xiao nRF52840 Sense** (one controller, one sim).
- 1× NESDR / RTL-SDR USB dongle (optional, real ADS-B input).
- Host laptop running Linux / WSL with Zephyr SDK + Python 3.10+.

## References

### Zephyr RTOS examples used

The following Zephyr samples were used as structural references during development.
All are located under `zephyr/samples/bluetooth/` in the Zephyr SDK.

**[1] `zephyr/samples/bluetooth/peripheral/src/main.c`**
Used in `sim/mobile/src/main/src/codein.c`.
Source for the BLE peripheral pattern: `BT_CONN_CB_DEFINE`, `connected()` / `disconnected()` callback signatures, `bt_le_adv_start()` call structure, and the `BT_GATT_SERVICE_DEFINE` / `BT_GATT_CHARACTERISTIC` / `BT_GATT_CCC` macro idiom for registering a custom vendor GATT service.

**[2] `zephyr/samples/bluetooth/mtu_update/peripheral/src/peripheral_mtu_update.c`**
Used in `sim/mobile/src/main/src/codein.c`.
Source for the disconnected-then-re-advertise pattern: releasing the connection reference with `bt_conn_unref()` and immediately restarting advertising with `bt_le_adv_start()` so the central can reconnect without a board reset.

**[3] `zephyr/samples/bluetooth/peripheral_nus/src/main.c`**
Referenced in `sim/mobile/src/common/src/ble_packet.h` and `sim/mobile/src/main/src/codein.c`.
Source for the peripheral GAP advertising setup and the `BT_UUID_128_ENCODE()` / `BT_UUID_DECLARE_128()` idiom used to register 128-bit custom service UUIDs. NUS itself was not used — the UUID and frame format are entirely custom (see `ble_packet.h`).

**[4] `zephyr/samples/bluetooth/peripheral/src/main.c` — custom GATT service pattern**
Used in `controller/src/ble/src/ble_central.c`.
The Zephyr peripheral sample defines vendor characteristics with custom 128-bit UUIDs using `BT_UUID_INIT_128` + `BT_GATT_SERVICE_DEFINE`. The controller's GATT central mirrors this on the discovery side: the four-stage state machine (`DISC_SVC → DISC_CHR → DISC_WARN_SVC → DISC_WARN_CHR`) was written to discover the same two custom services that the sim node registers using this pattern.

### Mini-project reference

**[5] `firmware/CSSE4011-Mini-Project/base/src/ble_central.c`**
Used in `controller/src/ble/src/ble_central.c`.
The earlier mini-project BLE central (written by Will) provided the structural idea for scan → connect → MTU exchange → GATT discovery → watchdog thread. The controller's `ble_central.c` adapted this state-machine skeleton but replaced all UUIDs, frame formats, and the notification/write pathway with the new SkyWatch protocol defined in `common/src/ble_packet.h`.

### Where no example was available (original implementation)

The following components had no applicable Zephyr or external example and were implemented from scratch:

- **Dual bidirectional custom GATT services** — registering two `BT_GATT_SERVICE_DEFINE` blocks on the same peripheral (one NOTIFY up, one WRITE-WITHOUT-RESPONSE down) with a matching four-stage central discovery state machine. No Zephyr sample covers this pattern.
- **BLE warning frame protocol and XOR CRC** — the 16-byte `ble_warning_frame` layout, message-type enum (`0x02` collision / `0x03` diversion / `0x04` freetext / `0x05` crash), and CRC validation (`ble_warning_crc`) in `ble_packet.h` / `codein.c`.
- **Dead-reckoning physics loop** — the 5 Hz send thread in `codein.c` computing lat/lon deltas from heading and speed using trigonometry and the haversine approximation.
- **Circle / WASD aircraft control** — turn-rate calculation, heading accumulator, and bounded-coordinate clamping in `codein.c`.
- **Pairwise collision TCA algorithm** — the time-of-closest-approach geometry and ADVISORY / WARNING / CRASH classification in `controller/src/collision/src/collision.c`.
- **Kalman filter** — the 4D constant-velocity state estimator with white-noise-acceleration process noise in `controller/src/kalman/src/kalman.c`.

---

## License + attribution

Course project for CSSE4011 @ UQ, 2026. Mostly written from scratch
by Blaise (controller, host) + Will (sim mobile + sim PC GUI). BLE
GATT layout + the legacy 16-byte XOR-CRC warning frame are Will's
design; reused under the same license as the rest of the repo.
