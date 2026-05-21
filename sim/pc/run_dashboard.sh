#!/bin/bash
export INFLUXDB_TOKEN="TEV5H3pmTZpo1rIjgbYopGmLwm7vSYe2AQ93Y8Zhb2S28He_xNNS5H29ThUc5cx0UpefoS3n5ikKbmaHAJyMiA=="
export INFLUXDB_ORG="Blaise&Will.Incorporated"
export INFLUXDB_BUCKET="miniproject"

INFLUX_URL="https://us-east-1-1.aws.cloud2.influxdata.com"
PYTHON="/home/blaise/csse4011/.venv/bin/python3"

cd "$(dirname "$0")"
"$PYTHON" stage2_map.py --influx "$INFLUX_URL" "$@"
