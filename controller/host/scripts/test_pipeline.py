#!/usr/bin/env python3
"""
End-to-end smoke test — fires a few synthetic CollisionFrames
through the MQTT publisher AND InfluxDB writer so an InfluxDB dashboard
+ mosquitto_sub can be verified without the controller / sim hardware
running.

Run:
    # 1. Export the InfluxDB creds (run_skywatch.sh does this for you):
    source <(grep '^export INFLUXDB' controller/host/run_skywatch.sh)
    # 2. (optional) watch the MQTT side live:
    mosquitto_sub -t 'skywatch/#' -v &
    # 3. Fire the burst:
    python3 controller/host/scripts/test_pipeline.py
"""

from __future__ import annotations

import sys
import time
import pathlib

# Sibling .py files (mqtt_publisher, influx_writer) live one level up,
# in controller/host/. Walk to that dir so we can import them.
HOST_DIR = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HOST_DIR))

from mqtt_publisher import MqttPublisher    # noqa: E402
from influx_writer  import InfluxWriter      # noqa: E402


def main():
    mqtt   = MqttPublisher()
    influx = InfluxWriter()
    ok_m = mqtt.start()
    ok_i = influx.start()
    print(f"[test] mqtt   start={ok_m}  err={mqtt.stats.last_error!r}")
    print(f"[test] influx start={ok_i}  err={influx.stats.last_error!r}")
    time.sleep(0.5)        # give MQTT loop time to connect

    scenarios = [
        # (level,      icao_a,   icao_b,   tca,  sep,  alt_diff)
        ("ADVISORY",  "a1b2c3", "gh05t1", 25.0, 700.0,    0),
        ("WARNING",   "a1b2c3", "gh05t1", 12.0, 380.0,    0),
        ("WARNING",   "a1b2c3", "gh05t1",  8.0, 250.0,  300),
        ("CLEAR",     "a1b2c3", "gh05t1",  0.0, 1500.0, 1640),
        ("ADVISORY",  "d4e5f6", "m1s51l", 30.0, 850.0,  500),
        ("WARNING",   "d4e5f6", "m1s51l", 15.0, 420.0,  200),
    ]

    for i, (level, a, b, tca, sep, alt_d) in enumerate(scenarios):
        frame = {
            "type":        "collision",
            "icao_a":      a,
            "icao_b":      b,
            "level":       level,
            "tca_s":       tca,
            "min_sep_m":   sep,
            "alt_diff_ft": alt_d,
            "ts":          time.time(),
        }
        print(f"[test] #{i+1:02d}  {level:<8} {a} ↔ {b}  sep={sep:.0f}m  tca={tca}s")
        mqtt.publish_collision(frame)
        influx.write_collision(frame)
        time.sleep(1.0)

    # Flush + show stats.
    time.sleep(2.0)
    print()
    print(f"[test] mqtt   published={mqtt.stats.published}  dropped={mqtt.stats.dropped}  "
          f"reconnects={mqtt.stats.reconnects}  last_err={mqtt.stats.last_error!r}")
    print(f"[test] influx written={influx.stats.written}  dropped={influx.stats.dropped}  "
          f"write_errs={influx.stats.write_errs}  last_err={influx.stats.last_error!r}")

    mqtt.stop()
    influx.stop()
    print("[test] clean shutdown.")


if __name__ == "__main__":
    main()
