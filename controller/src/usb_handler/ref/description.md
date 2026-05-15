# usb_handler

**Role:** Input-side handler for the **ttyACM1 data port**. Reassembles
newline-delimited JSON frames from the laptop's `sdr_bridge.py` and pushes
each parsed `aircraft_t` into `aircraft_db` (Stage 4.2).

**Files:** `src/usb_handler.{c,h}`

**Architecture:**

```
UART IRQ ─┬→ uart_fifo_read → ring_buf_put → [k_sem_give]
          │                                       │
          │      ┌────────────────────────────────┘
          │      ▼
          │   rx_thread: ring_buf_get → assemble line on '\n'
          │              → json_parse_aircraft() → LOG_INF (Stage 2.3)
          │              → (Stage 4.2) db_upsert()
          └─ rx_throttled flag: if ringbuf fills, disable IRQ;
                                rx_thread re-enables on >256 B free.
```

**Sizing:**
- Ring buffer: **2 kB** (`RINGBUF_BYTES`). ~150 B/frame × 10 fps worst case
  ⇒ 1.5 kB/s ⇒ >1 s headroom.
- Line buffer: **512 B** (`LINE_BUF_BYTES`). 3× worst-case frame; unterminated
  garbage gets caught within one buffer and resets.

**Public API:**
- `int  usb_handler_init(void)` — registers the IRQ callback, enables Rx.
- `void usb_handler_get_stats(struct usb_handler_stats *)` — read counters.

**Counters** (exposed via `skywatch usb stats`):
`bytes_rx`, `frames_rx`, `ringbuf_drops`, `line_overruns`, `parse_errors`.

**Requires:** `CONFIG_UART_INTERRUPT_DRIVEN=y`, `CONFIG_RING_BUFFER=y`.
