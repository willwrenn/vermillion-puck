# CSSE4011 Final Project — SkyWatch ADS-B Simulator

## System Overview

Two nRF52840 (XIAO BLE) boards communicate over BLE:

- **Mobile node** — advertises as `skywatch-sim`, sends position frames, receives warnings
- **Base node** — connects to mobile, receives position frames, prints JSON to ttyACM0
- **GUI** (`pc/simple_grid.py`) — reads JSON from ttyACM0 and plots the aircraft on a lat/lon map

---

## Running the GUI

### Serial mode (with hardware — recommended)

Connect the **mobile node** USB to your PC. The mobile prints JSON directly to `ttyACM0`.

```bash
cd pc
python3 simple_grid.py --port /dev/ttyACM0
```

If using the **base node** instead, connect the base USB and point the GUI at its port:

```bash
cd csse4011/firmwaretest/CSSE4011-Final-Project/pc
python3 simple_grid.py --map staticmap.jpeg --port /dev/ttyACM1
```



### TCP mode (no hardware — testing only)

```bash
python3 simple_grid.py --tcp 5005
# Then send a position manually:
echo '{"lat":-27.4975,"lon":153.0137,"alt":500}' | nc localhost 5005
```

---

## Bash Helper Functions

Add these to `~/.bashrc` once, then `source ~/.bashrc`:

```bash
function ac()     { echo -e "aircraft $*\r"        > /dev/ttyACM1; }
function circle() { echo -e "aircraft circle $*\r" > /dev/ttyACM1; }
```

These send shell commands to the mobile node's command port (ttyACM1).

---

## Aircraft Control Commands

All commands are sent via the `ac` bash function (which calls the Zephyr `aircraft` shell on ttyACM1).

### Set position and flight parameters

```bash
ac set <lat_e7> <lon_e7> <alt_m> <speed_kt> <heading_deg>
```

| Parameter    | Unit              | Example        |
|--------------|-------------------|----------------|
| `lat_e7`     | degrees × 1e7     | `-274975000`   |
| `lon_e7`     | degrees × 1e7     | `1530137000`   |
| `alt_m`      | metres            | `500`          |
| `speed_kt`   | knots             | `60`           |
| `heading_deg`| degrees (0–359)   | `90`           |

Example — place aircraft over St Lucia at 500 m, 60 kt heading east:

```bash
ac set -274975000 1530137000 500 60 90
```

### Fly in circles

```bash
circle <speed_kt> <radius_m>    # start circling
circle 0                         # stop circling
```

Examples:

```bash
circle 60 1000     # 60 kt, 1000 m radius
circle 120 3000    # 120 kt, 3 km radius (wider, slower turns)
circle 60 300      # 60 kt, 300 m radius (tight)
circle 0           # stop — fly straight on current heading
```

### Manual nudge keys (wasd)

```bash
ac wasd w    # speed up (+10 kt)
ac wasd s    # slow down (-10 kt)
ac wasd a    # turn left (-10°)
ac wasd d    # turn right (+10°)
ac wasd q    # climb (+50 m)
ac wasd e    # descend (-50 m)
```

### Other

```bash
ac status    # print current position as JSON
ac reset     # return to St Lucia, stop circling, zero speed
```

---

## Warning Commands (Base Node)

Connect PuTTY or `screen` to the **base node** ttyACM0 shell, then:

```bash
warn collision [severity]              # 0=advisory 1=warning(default) 2=urgent
warn diversion <message words>         # free-text e.g: warn diversion lower altitude now
```

The mobile node flashes its red LED and buzzer for 10 seconds, then auto-clears.
The GUI shows a flashing red collision panel or an amber diversion panel.

---

## BLE Protocol

Frames sent over BLE use `ble_aircraft_frame` (defined in `mobile/ble_packet.h`):

| Field      | Type       | Notes                              |
|------------|------------|------------------------------------|
| `source`   | uint8      | `1` = BLE_SRC_BLE_SIM              |
| `flags`    | uint8      | ALT_VALID, VEL_VALID, HDG_VALID    |
| `icao24`   | uint32 LE  | Low 24 bits                        |
| `lat`      | float64 LE | WGS-84 degrees                     |
| `lon`      | float64 LE | WGS-84 degrees                     |
| `alt_ft`   | int16 LE   | Altitude in **feet** MSL           |
| `vel_kt`   | int16 LE   | Ground speed in knots              |
| `hdg_deg`  | int16 LE   | Heading degrees true 0–359         |
| `ts_ms`    | uint32 LE  | ms since boot                      |

Frame size: **38 bytes**. Update rate: **5 Hz**.

> Note: `alt_ft` is feet. Shell commands use metres — the firmware converts on send (metres × 3.28084).

---

## Map Bounds

The GUI map covers the greater Brisbane area:

| Corner       | Latitude   | Longitude  |
|--------------|------------|------------|
| Top-left     | -27.31°    | 152.75°    |
| Bottom-right | -27.53°    | 153.21°    |
