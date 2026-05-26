# usb_serial

**Role:** Output-side helper for the **ttyACM1 data port**
(`cdc_acm_uart0`). Sends one newline-terminated string per call,
only when the host has DTR up.

**Files:** `src/usb_serial.{c,h}`

**Public API:**

- `int  usb_serial_init(void)` — verify the data UART is ready.
- `void usb_serial_println(const char *str)` — DTR-gated TX of
  `str + "\n"`. Returns immediately if the device isn't ready or
  the host hasn't raised DTR (the host PyQt GUI raises DTR on
  open, so this gates "GUI is listening").

**Convention:** bytes go out via `uart_poll_out()` byte-by-byte.
Cheap and correct; we're sending small JSON frames at ≤5 Hz, not
bulk data.

**Concurrency:** multiple producers call `usb_serial_println()` —
the collision detector, the DB publisher, the missile / ghost
workers — so the write path is wrapped in a `K_MUTEX` to keep JSON
lines atomic on the wire (no character-level interleave).

**Callers:**

- `db_publisher/db_publisher.c` — every 500 ms, emits one
  `AircraftFrame` JSON per DB entry.
- `collision/collision.c` — emits one `CollisionFrame` JSON per
  pair per tick while non-CLEAR, plus on every level transition.

**Requires:** `CONFIG_UART_LINE_CTRL=y` (DTR query).
