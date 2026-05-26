# usb_handler

**Role:** Input-side handler for the **ttyACM1 data port**.
Reassembles newline-delimited JSON frames coming from the host's
`sdr_bridge.py`, parses each line into a `struct aircraft_t` via the
`json` lib, and upserts it into `aircraft_db`.

**Files:** `src/usb_handler.{c,h}`

**Architecture:**

```
UART IRQ ─┬→ uart_fifo_read → ring_buf_put → [k_sem_give]
          │                                       │
          │      ┌────────────────────────────────┘
          │      ▼
          │   rx_thread: ring_buf_get → assemble line on '\n'
          │              → json_parse_aircraft()
          │              → aircraft_db_upsert()
          └─ rx_throttled flag: if ring buffer fills, disable IRQ;
                                rx_thread re-enables on >256 B free.
```

**Sizing:**

- Ring buffer: **2 kB** (`RINGBUF_BYTES`). ~150 B/frame × 10 fps
  worst case ⇒ 1.5 kB/s ⇒ >1 s headroom before throttling kicks in.
- Line buffer: **512 B** (`LINE_BUF_BYTES`). 3× worst-case frame;
  unterminated garbage gets caught within one buffer and resets.
- Rx thread stack: **4 kB**. The original 2 kB was enough for the
  parse path on its own, but `aircraft_db_upsert()` runs Kalman
  matrix math inline which pushed the original stack over its guard
  during BLE-traffic bring-up.

**Public API:**

- `int  usb_handler_init(void)` — registers the IRQ callback,
  enables Rx.
- `void usb_handler_get_stats(struct usb_handler_stats *)` — read
  counters.

**Counters** (exposed via `skywatch usb stats`):
`bytes_rx`, `frames_rx`, `ringbuf_drops`, `line_overruns`,
`parse_errors`.

**Requires:** `CONFIG_UART_INTERRUPT_DRIVEN=y`,
`CONFIG_RING_BUFFER=y`.
