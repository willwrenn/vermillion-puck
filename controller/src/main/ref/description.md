# main

**Role:** Application entry point. Boots subsystems in order and idles.

**Files:** `src/main.c`

**Hard rule:** stays under ~300 lines for the whole project. Anything bigger
gets factored into a new lib. This file is an orchestrator, not a feature.

**Init order:**
1. `usb_serial_init()` — readies the ttyACM1 data port for TX.
2. `usb_handler_init()` — arms IRQ-driven Rx on ttyACM1 and starts the
   parsing thread.
3. (Stage 3+) `ble_central_init()`, `aircraft_db_init()`, etc.

**Heartbeat:** prints `LOG_INF("heartbeat")` every 5 s so we can tell at a
glance over `screen /dev/ttyACM0` that the scheduler is alive.

**No business logic here.** If you find yourself adding a feature to main.c,
make a new lib for it.
