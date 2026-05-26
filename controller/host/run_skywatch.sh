#!/usr/bin/env bash
# SkyWatch ATC GUI — one-shot launcher (ships with the repo).
#
# Picks the controller's data port, exports the InfluxDB Cloud credentials
# for the live dashboard, optionally launches the SDR bridge for real
# ADS-B traffic, and runs the PyQt GUI in the foreground. Ctrl-C tears
# everything down cleanly.
#
# Run from anywhere — paths resolved relative to this script:
# bash controller/host/run_skywatch.sh # full SDR + GUI
# bash controller/host/run_skywatch.sh --no-sdr # GUI only (no antenna)
#
# Flags / env:
# --no-sdr Skip dump1090 check + don't spawn sdr_bridge.
# Use when the SDR antenna isn't connected.
# (Also via SKYWATCH_NO_SDR=1.)
# SKYWATCH_PORT=path Override auto-detected data port.
# SKYWATCH_DUMP1090_URL Override the default dump1090 JSON endpoint.
# SDR_BRIDGE_PY=path Path to a host-side sdr_bridge.py if you have
# one (the sample ships separately). Required
# only when you want SDR traffic.
# INFLUXDB_TOKEN=... Override the baked-in dashboard token.
#
# Pre-flight assumptions:
# 1. Both Xiao boards flashed + USB-attached (controller, optionally sim).
# 2. Will's mobile is paired via BLE GATT (if you want sim traffic).
# 3. (--no-sdr mode) you don't have / don't want the dump1090 path.

set -euo pipefail

# Resolve repo paths relative to THIS script regardless of cwd.
HOST_DIR="$(cd "$(dirname "$0")" && pwd)"
GUI="$HOST_DIR/atc_gui.py"

DUMP1090_URL="${SKYWATCH_DUMP1090_URL:-http://172.27.208.1:8080/data/aircraft.json}"
SDR_BRIDGE_PY="${SDR_BRIDGE_PY:-}"        # external; set if you have it

# --no-sdr / SKYWATCH_NO_SDR=1 → skip the SDR bringup entirely.
NO_SDR=0
for a in "$@"; do
    case "$a" in
        --no-sdr) NO_SDR=1 ;;
    esac
done
if [ "${SKYWATCH_NO_SDR:-0}" = "1" ]; then NO_SDR=1; fi

# InfluxDB Cloud creds — reused from the miniproject org. Override via env
# if you have your own bucket / token. Bucket is `skywatch` so we don't
# stomp on miniproject data in the same org.
export INFLUXDB_URL="${INFLUXDB_URL:-https://us-east-1-1.aws.cloud2.influxdata.com}"
export INFLUXDB_TOKEN="${INFLUXDB_TOKEN:-TEV5H3pmTZpo1rIjgbYopGmLwm7vSYe2AQ93Y8Zhb2S28He_xNNS5H29ThUc5cx0UpefoS3n5ikKbmaHAJyMiA==}"
export INFLUXDB_ORG="${INFLUXDB_ORG:-Blaise&Will.Incorporated}"
export INFLUXDB_BUCKET="${INFLUXDB_BUCKET:-skywatch}"

echo "── SkyWatch bring-up ─────────────────────────────────────────"

# 1. SDR (optional) — only nag if the operator asked for it.
if [ "$NO_SDR" = "1" ]; then
    echo "  ⊘ SDR disabled (--no-sdr) — skipping dump1090 + sdr_bridge"
else
    if curl -fsS --max-time 2 -o /dev/null "$DUMP1090_URL"; then
        echo "  ✓ dump1090 reachable at $DUMP1090_URL"
    else
        cat <<EOF
  ✗ dump1090 NOT reachable at $DUMP1090_URL

    Either start it on Windows:
        cd <Dump1090-main folder>
        .\\dump1090.exe --interactive --net

    Or skip the SDR step:
        bash $(basename "$0") --no-sdr
EOF
        exit 1
    fi
fi

# 2. Pick the controller data port — prefer ACM1, fall back to any non-ACM0.
PORT="${SKYWATCH_PORT:-}"
if [ -z "$PORT" ]; then
    if [ -e /dev/ttyACM1 ]; then
        PORT=/dev/ttyACM1
    else
        PORT="$(ls /dev/ttyACM* 2>/dev/null | grep -v 'ACM0$' | head -1 || true)"
    fi
fi
if [ -z "$PORT" ] || [ ! -e "$PORT" ]; then
    cat <<EOF
  ✗ no data port found (looked for /dev/ttyACM1, then any non-ACM0)

    The controller's data port lives at a different ttyACMn each time
    the board re-enumerates (especially after a bootloader excursion).
    Override with:
        SKYWATCH_PORT=/dev/ttyACMn bash $(basename "$0") [--no-sdr]

    Or under WSL, re-attach via Admin PowerShell:
        usbipd list
        usbipd attach --wsl --busid <x-y>
EOF
    exit 1
fi
echo "  ✓ data port: $PORT"

# 3. Python dep sanity. paho-mqtt + influxdb-client are optional; the GUI
# falls back cleanly if either is missing.
if ! python3 -c "import serial, PyQt6, requests" 2>/dev/null; then
    cat <<EOF
  ✗ missing Python deps. Install once:
        pip install --break-system-packages pyserial PyQt6 requests \\
                                            paho-mqtt influxdb-client Pillow
EOF
    exit 1
fi
echo "  ✓ python deps importable"
if ! python3 -c "import paho.mqtt.client, influxdb_client" 2>/dev/null; then
    echo "  ⚠ Phase-10 deps missing (MQTT/InfluxDB publishing disabled)"
fi

# 4. Mosquitto (optional). GUI auto-reconnects either way.
if command -v mosquitto >/dev/null 2>&1; then
    if pgrep -x mosquitto >/dev/null 2>&1; then
        echo "  ✓ mosquitto broker running (127.0.0.1:1883)"
    else
        echo "  ⚠ mosquitto installed but not running — sudo systemctl start mosquitto"
    fi
else
    echo "  ⚠ mosquitto not installed — sudo apt install mosquitto mosquitto-clients"
fi

# 5. InfluxDB creds sanity.
if [ -z "${INFLUXDB_TOKEN:-}" ]; then
    echo "  ⚠ INFLUXDB_TOKEN empty — dashboard writes will be disabled"
else
    echo "  ✓ InfluxDB Cloud configured (bucket=${INFLUXDB_BUCKET})"
fi
echo

# ── (Optional) SDR bridge in background ─────────────────────────────────────
LOGDIR="${SKYWATCH_LOGDIR:-/tmp}"
BRIDGE_LOG="$LOGDIR/sdr_bridge.log"
BRIDGE_PID=""

cleanup() {
    echo
    echo "── shutting down ────────────────────────────────────────────"
    if [ -n "$BRIDGE_PID" ] && kill -0 "$BRIDGE_PID" 2>/dev/null; then
        kill "$BRIDGE_PID" 2>/dev/null || true
        wait "$BRIDGE_PID" 2>/dev/null || true
        echo "  bridge log: $BRIDGE_LOG"
    fi
    echo "  done."
}
trap cleanup INT TERM EXIT

if [ "$NO_SDR" = "1" ]; then
    echo "[bridge] skipped (no-sdr mode)"
elif [ -n "$SDR_BRIDGE_PY" ] && [ -f "$SDR_BRIDGE_PY" ]; then
    echo "[bridge] starting → $PORT (log: $BRIDGE_LOG)"
    python3 "$SDR_BRIDGE_PY" --no-launch \
            --endpoint "$DUMP1090_URL" \
            --serial-port "$PORT" \
            > "$BRIDGE_LOG" 2>&1 &
    BRIDGE_PID=$!
    sleep 1
    if ! kill -0 "$BRIDGE_PID" 2>/dev/null; then
        echo "  ✗ sdr_bridge died on startup. Tail of its log:"
        tail -15 "$BRIDGE_LOG"
        exit 1
    fi
    echo "  ✓ bridge alive (pid $BRIDGE_PID)"
else
    echo "[bridge] no SDR_BRIDGE_PY exported — running GUI only (controller-direct mode)"
fi
echo

# ── GUI in foreground ─────────────────────────────────────────────────────
echo "[gui] launching atc_gui.py (close window or Ctrl-C to quit)"
python3 "$GUI" --port "$PORT"
