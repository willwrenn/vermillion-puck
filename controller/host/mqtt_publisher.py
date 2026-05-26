"""
SkyWatch ATC GUI — MQTT publisher.

Publishes collision events to a Mosquitto broker so external subscribers
(future phone apps, command-line `mosquitto_sub` for the demo, etc.) can
see live ATC state without re-parsing the controller's USB-CDC stream.

Topics:
    skywatch/collision              — every CollisionFrame (level
                                      transitions: CLEAR→ADVISORY,
                                      ADVISORY→WARNING, WARNING→CLEAR).
                                      QoS 0, retained. Latest payload
                                      stays on the broker so a fresh
                                      subscriber sees the current state
                                      immediately.
    skywatch/collision/{pair}       — same payload, but topic-keyed by
                                      the lexicographically-sorted ICAO
                                      pair (icao_a_icao_b). Lets the
                                      dashboard / a phone app
                                      subscribe to a specific pair.
    skywatch/status                 — last-will heartbeat (so a stale
                                      GUI is detectable from the broker
                                      side).

Reliability: runs on a background thread with an internal queue so the
GUI's _on_collision() never blocks on broker latency. If the broker is
unreachable, paho's built-in auto-reconnect handles it — meanwhile the
queue drops messages with QoS 0 to avoid unbounded growth.

Design parallel to influx_writer.py: both consume the same
in-memory CollisionFrame dicts via .publish_collision(), letting the GUI
write to multiple sinks in one call.
"""

from __future__ import annotations

import json
import queue
import socket
import threading
import time
from dataclasses import dataclass, field
from typing import Optional

try:
    import paho.mqtt.client as mqtt
except ImportError:                                  # pragma: no cover
    mqtt = None                                       # noqa: N816


# Topic prefix — single point of truth so the dashboard, mosquitto_sub
# wildcards, and any future subscriber match what we publish.
TOPIC_BASE         = "skywatch"
TOPIC_COLLISION    = f"{TOPIC_BASE}/collision"
TOPIC_STATUS       = f"{TOPIC_BASE}/status"
DEFAULT_HOST       = "127.0.0.1"
DEFAULT_PORT       = 1883
DEFAULT_KEEPALIVE  = 30
QUEUE_MAX          = 256                              # drop oldest if exceeded


@dataclass
class MqttStats:
    published:    int = 0
    dropped:      int = 0    # queue full or client not connected
    reconnects:   int = 0
    last_error:   str = ""


@dataclass
class MqttPublisher:
    """Background-thread MQTT publisher. Create one per GUI session.

    Use:
        pub = MqttPublisher()
        pub.start()                 # spawns the loop thread
        pub.publish_collision(frame_dict)  # call from anywhere; non-blocking
        pub.stop()                  # at shutdown
    """

    host: str = DEFAULT_HOST
    port: int = DEFAULT_PORT
    keepalive_s: int = DEFAULT_KEEPALIVE
    client_id: Optional[str] = None

    def __post_init__(self):
        # Hostname as client ID by default — lets the broker log show
        # which laptop/process is connected (useful when subscribers
        # debug a missing-publisher mystery).
        self.client_id = self.client_id or f"skywatch-atc-{socket.gethostname()}"
        self.stats = MqttStats()
        self._q: "queue.Queue[tuple[str, str, int, bool]]" = queue.Queue(QUEUE_MAX)
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._client: Optional["mqtt.Client"] = None
        self._connected = False

    # ---- lifecycle --------------------------------------------------- #

    def start(self) -> bool:
        """Begin connecting + draining the publish queue. Returns False
        if paho-mqtt isn't installed (so the GUI can gracefully degrade
        rather than crashing on missing deps)."""
        if mqtt is None:
            self.stats.last_error = "paho-mqtt not installed"
            return False
        # CallbackAPIVersion.VERSION2 silences the v1 deprecation warning
        # in paho-mqtt >= 2.0 while staying compatible with both ages.
        try:
            self._client = mqtt.Client(
                callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
                client_id=self.client_id,
            )
        except AttributeError:
            # paho-mqtt < 2.0
            self._client = mqtt.Client(client_id=self.client_id)
        self._client.on_connect    = self._on_connect
        self._client.on_disconnect = self._on_disconnect
        # Last-will: if the GUI dies without a clean disconnect, the
        # broker auto-publishes 'offline' on skywatch/status so any
        # monitor knows the publisher is gone.
        self._client.will_set(TOPIC_STATUS, "offline", qos=0, retain=True)
        self._thread = threading.Thread(target=self._run, name="mqtt-publisher",
                                        daemon=True)
        self._thread.start()
        return True

    def stop(self):
        self._stop.set()
        # Push a synthetic 'offline' status so subscribers see the
        # clean shutdown immediately, before the broker's will fires.
        try:
            if self._client is not None and self._connected:
                self._client.publish(TOPIC_STATUS, "offline", qos=0, retain=True)
                self._client.loop_write()
        except Exception:
            pass
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        if self._client is not None:
            try:
                self._client.disconnect()
            except Exception:
                pass

    # ---- caller-facing publish API ----------------------------------- #

    def publish_collision(self, frame: dict) -> None:
        """Enqueue a CollisionFrame for publishing. Non-blocking.
        Drops oldest message if the queue is full so a stuck broker
        can't grow memory without bound."""
        payload = json.dumps(frame, separators=(",", ":"))
        a = frame.get("icao_a", "?")
        b = frame.get("icao_b", "?")
        per_pair = f"{TOPIC_COLLISION}/{a}_{b}"
        # qos=0 retain=True on both: latest payload sticks on the broker
        # so a fresh subscriber sees current state without waiting for
        # the next collision tick.
        self._enqueue(TOPIC_COLLISION, payload, qos=0, retain=True)
        self._enqueue(per_pair,        payload, qos=0, retain=True)

    # ---- internals --------------------------------------------------- #

    def _enqueue(self, topic: str, payload: str, qos: int, retain: bool):
        try:
            self._q.put_nowait((topic, payload, qos, retain))
        except queue.Full:
            # Drop oldest to make room, then enqueue. Keeps memory
            # bounded under a stuck-broker scenario.
            try:
                self._q.get_nowait()
            except queue.Empty:
                pass
            try:
                self._q.put_nowait((topic, payload, qos, retain))
            except queue.Full:
                self.stats.dropped += 1

    def _run(self):
        # connect_async + loop_start would be tempting, but a dedicated
        # thread gives us full control of error logging + reconnect
        # counting. paho's network thread is started inside loop_forever.
        backoff = 1.0
        while not self._stop.is_set():
            try:
                self._client.connect(self.host, self.port, self.keepalive_s)
                self._client.loop_start()
                # On successful connect, drain the queue forever until stop.
                self._drain_loop()
                self._client.loop_stop()
                return
            except (socket.error, OSError, ConnectionError) as e:
                self.stats.last_error = f"connect: {e}"
                self.stats.reconnects += 1
                # Exponential backoff, capped at 15 s. Lets the GUI keep
                # running fine if the broker isn't up yet at startup.
                self._stop.wait(min(15.0, backoff))
                backoff = min(15.0, backoff * 2)
            except Exception as e:                    # noqa: BLE001
                self.stats.last_error = f"mqtt loop: {e}"
                self._stop.wait(2.0)

    def _drain_loop(self):
        while not self._stop.is_set():
            try:
                topic, payload, qos, retain = self._q.get(timeout=0.5)
            except queue.Empty:
                continue
            if not self._connected:
                # Broker dropped us — paho's loop_start handles reconnect;
                # meanwhile drop the message (qos=0 semantics anyway).
                self.stats.dropped += 1
                continue
            try:
                self._client.publish(topic, payload, qos=qos, retain=retain)
                self.stats.published += 1
            except Exception as e:                    # noqa: BLE001
                self.stats.last_error = f"publish: {e}"
                self.stats.dropped += 1

    # paho callbacks --------------------------------------------------- #

    def _on_connect(self, client, userdata, flags, reason_code, properties=None):
        if reason_code == 0 or reason_code == "Success":
            self._connected = True
            # Announce ourselves so any monitor sees us come up.
            try:
                client.publish(TOPIC_STATUS, "online", qos=0, retain=True)
            except Exception:
                pass
        else:
            self._connected = False
            self.stats.last_error = f"connect refused: {reason_code}"

    def _on_disconnect(self, client, userdata, *args, **kwargs):
        self._connected = False
        # paho will reconnect via loop_start; just count it.
        self.stats.reconnects += 1
