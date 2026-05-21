#!/usr/bin/env python3
"""
Aircraft position simulator — feeds the simple_grid.py GUI continuously.

Run the GUI in TCP mode first:
    python3 simple_grid.py --tcp 5005

Then in another terminal run this simulator:

  Circle (default, 2 km radius, 60 kt):
    python3 simulate.py

  Custom circle:
    python3 simulate.py --pattern circle --lat -27.4975 --lon 153.0137 \
        --alt 1500 --speed 60 --radius 2.0 --rate 10

  Straight line heading NE (wraps back to start after 50 km):
    python3 simulate.py --pattern straight --heading 45 --speed 120

  Multiple laps output to serial instead of TCP:
    python3 simulate.py --port /dev/ttyACM1

Arguments:
  --pattern  circle | straight        (default: circle)
  --lat      start latitude  degrees  (default: -27.4975)
  --lon      start longitude degrees  (default: 153.0137)
  --alt      altitude metres          (default: 1000)
  --speed    airspeed knots           (default: 60)
  --heading  initial heading degrees  (default: 0, used by straight)
  --radius   circle radius km         (default: 2.0)
  --rate     updates per second       (default: 10)
  --tcp      GUI TCP port             (default: 5005)
  --port     serial port (overrides TCP)
"""

import argparse, json, math, socket, time, sys
import serial

KTS_TO_MS  = 0.514444   # knots → m/s
DEG_PER_M_LAT = 1.0 / 111_320.0   # degrees latitude per metre (approx)


def deg_per_m_lon(lat_deg):
    return 1.0 / (111_320.0 * math.cos(math.radians(lat_deg)))


def parse_args():
    p = argparse.ArgumentParser(description="Aircraft position simulator for simple_grid.py")
    p.add_argument("--pattern",  default="circle",    choices=["circle", "straight"])
    p.add_argument("--lat",      type=float, default=-27.4975)
    p.add_argument("--lon",      type=float, default=153.0137)
    p.add_argument("--alt",      type=float, default=1000.0,  help="altitude in metres")
    p.add_argument("--speed",    type=float, default=60.0,    help="airspeed in knots")
    p.add_argument("--heading",  type=float, default=0.0,     help="initial heading degrees")
    p.add_argument("--radius",   type=float, default=2.0,     help="circle radius in km")
    p.add_argument("--rate",     type=float, default=10.0,    help="updates per second")
    p.add_argument("--tcp",      type=int,   default=5005,    help="GUI TCP port")
    p.add_argument("--port",     default=None,                help="serial port e.g. /dev/ttyACM1")
    return p.parse_args()


def open_output(args):
    if args.port:
        ser = serial.Serial(args.port, 115200, timeout=1)
        def send(line):
            ser.write((line + "\n").encode())
        print(f"Sending to serial {args.port}")
        return send
    else:
        # TCP — reconnect on drop so the GUI can be restarted independently
        state = {"sock": None}

        def connect():
            while True:
                try:
                    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    s.connect(("127.0.0.1", args.tcp))
                    state["sock"] = s
                    print(f"Connected to GUI TCP :{args.tcp}")
                    return
                except ConnectionRefusedError:
                    print(f"GUI not ready on TCP :{args.tcp} — retrying in 1 s...")
                    time.sleep(1)

        def send(line):
            if state["sock"] is None:
                connect()
            try:
                state["sock"].sendall((line + "\n").encode())
            except OSError:
                state["sock"] = None
                connect()
                state["sock"].sendall((line + "\n").encode())

        connect()
        return send


def simulate_circle(args, send):
    speed_ms = args.speed * KTS_TO_MS
    radius_m = args.radius * 1000.0
    # angular rate so that circumference / speed = period
    # omega (rad/s) = speed / radius
    omega = speed_ms / radius_m          # radians per second

    lat = args.lat
    lon = args.lon
    alt = args.alt
    dt  = 1.0 / args.rate

    # start at the northernmost point of the circle, heading east
    angle = 0.0   # angle around circle in radians (0 = north, increases clockwise)

    # centre of circle is south of start by radius
    centre_lat = lat - radius_m * DEG_PER_M_LAT
    centre_lon = lon

    while True:
        # position on circle
        pos_lat = centre_lat + radius_m * DEG_PER_M_LAT * math.cos(angle)
        pos_lon = centre_lon + radius_m * deg_per_m_lon(centre_lat) * math.sin(angle)

        # heading = tangent to circle (90° ahead of radius direction)
        hdg = math.degrees(angle) + 90.0
        hdg %= 360.0

        pkt = {
            "lat": round(pos_lat, 6),
            "lon": round(pos_lon, 6),
            "alt": int(alt),
            "speed": int(args.speed),
            "heading": int(hdg),
        }
        send(json.dumps(pkt))
        print(f"lat={pos_lat:.5f}  lon={pos_lon:.5f}  hdg={hdg:.1f}°  spd={args.speed}kt")

        angle += omega * dt   # advance around circle
        time.sleep(dt)


def simulate_straight(args, send):
    speed_ms = args.speed * KTS_TO_MS
    hdg_rad  = math.radians(args.heading)

    lat = args.lat
    lon = args.lon
    alt = args.alt
    dt  = 1.0 / args.rate

    # distance per tick in metres
    d_lat = speed_ms * dt * math.cos(hdg_rad) * DEG_PER_M_LAT
    # lon scaling updated each step

    start_lat = lat
    start_lon = lon
    MAX_DIST_M = 50_000.0   # wrap back after 50 km

    dist_m = 0.0

    while True:
        d_lon = speed_ms * dt * math.sin(hdg_rad) * deg_per_m_lon(lat)
        lat += d_lat
        lon += d_lon
        dist_m += speed_ms * dt

        if dist_m >= MAX_DIST_M:
            lat, lon, dist_m = start_lat, start_lon, 0.0

        pkt = {
            "lat": round(lat, 6),
            "lon": round(lon, 6),
            "alt": int(alt),
            "speed": int(args.speed),
            "heading": int(args.heading),
        }
        send(json.dumps(pkt))
        print(f"lat={lat:.5f}  lon={lon:.5f}  hdg={args.heading:.0f}°  spd={args.speed}kt")

        time.sleep(dt)


def main():
    args = parse_args()
    send = open_output(args)

    print(f"Pattern: {args.pattern}  speed: {args.speed} kt  "
          f"alt: {args.alt} m  rate: {args.rate} Hz")

    try:
        if args.pattern == "circle":
            simulate_circle(args, send)
        else:
            simulate_straight(args, send)
    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    main()
