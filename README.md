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
        ▲ JSON over USB-CDC ttyACM (sim's data port)
        └─ Brisbane map + position trail + DIVERSION panel +
           full-screen TV-static "LOST CONNECTION" overlay on CRASH
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
│       └── assets/            Brisbane greyscale basemap PNG
├── sim/                       Will's sim mobile + tkinter PC GUI
│   ├── mobile/                Zephyr firmware (BLE peripheral)
│   ├── pc/
│   │   ├── simple_grid.py     tkinter map GUI for the sim aircraft
│   │   ├── staticmap.jpeg     Brisbane background image
│   │   └── run_dashboard.sh
│   └── README.md
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
| `/dev/ttyACM2` | Sim mobile | raw single-byte wasd input (silent) |
| `/dev/ttyACM3` | Sim mobile | Zephyr shell + position JSON |

(Numbers shift each time a board re-enumerates — identify by sniffing
or use the helper script in `resources/handover.md`.)

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

**Sim aircraft control** (separate terminal — write into the sim's shell port):

```bash
function ac()     { echo -e "aircraft $*\r"        > /dev/ttyACM3; }
function circle() { echo -e "aircraft circle $*\r" > /dev/ttyACM3; }
ac reset             # snap to St Lucia origin
ac wasd w; ac wasd w # +20 kt
ac wasd a            # turn left 10°
circle 80 800        # auto-circle 80 kt, 800 m radius
circle 0             # stop circling
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

## License + attribution

Course project for CSSE4011 @ UQ, 2026. Mostly written from scratch
by Blaise (controller, host) + Will (sim mobile + sim PC GUI). BLE
GATT layout + the legacy 16-byte XOR-CRC warning frame are Will's
design; reused under the same license as the rest of the repo.
