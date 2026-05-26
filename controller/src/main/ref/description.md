# main

**Role:** Application entry point. Boots subsystems in order and
idles, with a heartbeat LED toggle and a periodic `LOG_INF`.

**Files:** `src/main.c`

**Hard rule:** stays under ~300 lines for the whole project. Anything
bigger gets factored into a new lib. This file is an orchestrator,
not a feature.

**Init order:**

1. `usb_device_init()` — bring up the dual CDC ACM USB stack
   (handled by `SYS_INIT(APPLICATION)` in `usb_device.c`, before
   `main()` runs).
2. `usb_serial_init()` — readies the ttyACM1 data port for TX.
3. `usb_handler_init()` — arms IRQ-driven Rx on ttyACM1 and starts
   the parsing thread that feeds `aircraft_db`.
4. `aircraft_db_init()` — clears the pool, starts the stale-reaper
   thread.
5. `bridge_init()` — clears stats; the bridge worker thread is
   auto-started via `K_THREAD_DEFINE`.
6. `ble_central_init()` — `bt_enable`, kicks off scan, starts the
   reconnect watchdog.
7. `db_publisher_init()` — starts the 2 Hz worker that walks the DB
   and emits one `AircraftFrame` JSON per entry on ttyACM1.
8. `collision_init()` — starts the 5 Hz collision-detector
   work-queue tick.

**Heartbeat:** toggles the on-board green LED and logs an
`LOG_INF("heartbeat")` every 5 s so we can tell at a glance over
`screen /dev/ttyACM0` that the scheduler is alive.

**No business logic here.** If you find yourself adding a feature to
`main.c`, make a new lib for it.
