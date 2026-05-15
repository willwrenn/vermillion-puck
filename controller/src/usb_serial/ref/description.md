# usb_serial

**Role:** Output-side helper for the **ttyACM1 data port** (`cdc_acm_uart0`).
Sends one newline-terminated string per call, only when the host has DTR up.

**Files:** `src/usb_serial.{c,h}`

**Public API:**
- `int  usb_serial_init(void)` — verify the data UART is ready.
- `void usb_serial_println(const char *str)` — DTR-gated TX of `str + "\n"`.
  Returns immediately if the device isn't ready or the host hasn't raised DTR.

**Convention:** bytes go out via `uart_poll_out()` byte-by-byte. Cheap and
correct; we're sending small JSON frames at 1–2 Hz, not bulk data.

**Stage 5.1 will add:** the periodic worker that walks `aircraft_db` and
emits one `AircraftFrame` JSON per entry via this lib. The library itself
stays a dumb sink — frame composition lives in the caller.

**Requires:** `CONFIG_UART_LINE_CTRL=y` (DTR query).
