# Phase 10 — MQTT + InfluxDB Cloud setup

One-time setup so the GUI's MQTT publisher + InfluxDB writer have somewhere
to send data. After this, `bash scripts/run_skywatch.sh --no-sdr` brings
up the full pipeline and the InfluxDB Cloud dashboard renders live.

## 1. Local Mosquitto broker (one-time install)

```bash
sudo apt update && sudo apt install -y mosquitto mosquitto-clients
sudo systemctl enable --now mosquitto
# Verify:
mosquitto_sub -t 'skywatch/#' -v &     # subscribes in the background
mosquitto_pub -t skywatch/test -m hello
# You should see:  skywatch/test hello
kill %1
```

Mosquitto's default config listens on `127.0.0.1:1883` — fine for the
GUI + dashboard on the same laptop. For LAN access from another machine
(phone app, second laptop), allow external connections:

```bash
sudo tee /etc/mosquitto/conf.d/skywatch.conf <<'EOF'
listener 1883 0.0.0.0
allow_anonymous true
EOF
sudo systemctl restart mosquitto
```

## 2. InfluxDB Cloud bucket (one-time, in the web UI)

Go to <https://us-east-1-1.aws.cloud2.influxdata.com/> and log in with
the existing miniproject account.

1. **Load Data → Buckets → Create Bucket**
   - Name: `skywatch`
   - Retention: 30 days (or "Never" if you want to keep history)
   - Click *Create*

That's it for the bucket — the GUI already writes to it via the token +
org baked into `scripts/run_skywatch.sh` (env vars `INFLUXDB_*`).

## 3. Verify the pipeline (no firmware needed)

```bash
# Terminal A — watch MQTT live:
mosquitto_sub -t 'skywatch/#' -v
# Terminal B — bring up the env and fire synthetic events:
cd /home/blaise/finalproject4011
source <(grep '^export INFLUXDB' scripts/run_skywatch.sh)
python3 scripts/stage10/test_pipeline.py
```

Expected in terminal A:
```
skywatch/status online
skywatch/collision {"type":"collision","icao_a":"a1b2c3", ...}
skywatch/collision/a1b2c3_gh05t1 {...}
...
```

In InfluxDB Cloud, the `skywatch` bucket now has rows. Quick check via
Data Explorer:

```flux
from(bucket: "skywatch")
  |> range(start: -5m)
  |> filter(fn: (r) => r._measurement == "skywatch_events")
```

You should see 6 events with the various ICAOs.

## 4. Build the dashboard

In InfluxDB Cloud → **Dashboards → Create Dashboard → New Dashboard**.
Name it "SkyWatch ATC". Add cells:

### Panel 1 — Active alerts table

> "Latest state per (icao_a, icao_b) pair currently at ADVISORY or WARNING."

Click *Add Cell*, switch to **Script Editor** (top right), paste:

```flux
from(bucket: "skywatch")
  |> range(start: -10m)
  |> filter(fn: (r) => r._measurement == "skywatch_events")
  |> filter(fn: (r) => r._field == "min_sep_m" or r._field == "tca_s" or r._field == "alt_diff_ft")
  |> last()
  |> pivot(rowKey: ["pair"], columnKey: ["_field"], valueColumn: "_value")
  |> filter(fn: (r) => r.level == "ADVISORY" or r.level == "WARNING")
  |> keep(columns: ["_time", "pair", "level", "min_sep_m", "tca_s", "alt_diff_ft"])
  |> sort(columns: ["level", "min_sep_m"])
```

Visualization: **Table**.
Refresh: top-right → **5s**.

### Panel 2 (bonus) — Min separation per pair over time

```flux
from(bucket: "skywatch")
  |> range(start: -1h)
  |> filter(fn: (r) => r._measurement == "skywatch_events"
                    and r._field == "min_sep_m")
  |> aggregateWindow(every: 5s, fn: min, createEmpty: false)
  |> group(columns: ["pair"])
```

Visualization: **Graph** (Line). Y-axis label: "Min separation (m)".
This makes divert-to-clear visible — a pair's line drops then rises.

### Panel 3 (bonus) — Event count by level (last hour)

```flux
from(bucket: "skywatch")
  |> range(start: -1h)
  |> filter(fn: (r) => r._measurement == "skywatch_events"
                    and r._field == "min_sep_m")
  |> group(columns: ["level"])
  |> count()
  |> rename(columns: {_value: "count"})
```

Visualization: **Single Stat** per level, or **Bar Chart** grouped by `level`.

## 5. Run the live demo

```bash
cd /home/blaise/finalproject4011
bash scripts/run_skywatch.sh --no-sdr
```

Watch the InfluxDB dashboard tab. Trigger collisions via the controller
shell (`skywatch collision ghost`, `skywatch missile launch`, etc.) —
they'll appear in the Active Alerts panel within ~5 s (next refresh).

For an even-more-live demo experience, open `mosquitto_sub -t 'skywatch/#'`
in a side terminal and project both alongside the InfluxDB dashboard.
