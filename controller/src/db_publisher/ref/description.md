# db_publisher

**Role:** Periodic worker that walks `aircraft_db` under the mutex,
takes a snapshot, releases the mutex, and emits one `AircraftFrame`
JSON line per entry on the ttyACM1 data port via `usb_serial`. This
is what the host PyQt GUI consumes to draw the map.

**Files:** `src/db_publisher.{c,h}`

**Architecture:**

```
work_handler (every 500 ms via k_work_delayable)
   ├─ snapshot aircraft_db under mutex (fast — just memcpy)
   ├─ release mutex
   ├─ for each snapshot row:
   │     compose AircraftFrame JSON via snprintf
   │     (optional fields conditionally included based on valid_mask)
   │     usb_serial_println(line)
   └─ re-schedule self at +500 ms
```

The snapshot-then-release pattern means the USB-CDC write loop
doesn't hold the DB lock — the collision detector and the BLE +
ADS-B ingestion paths can keep upserting while the publisher is
writing.

**Public API:**

- `int  db_publisher_init(void);` — starts the delayable work.
- `void db_publisher_get_stats(struct db_publisher_stats *out);` —
  `ticks / rows_emitted / suppressed` counters.

**Output cadence:** 2 Hz (every 500 ms). Each tick emits up to 32
JSON lines (the `aircraft_db` pool size).

**Optional-field serialisation:** the publisher inspects each row's
`valid_mask` and conditionally includes `alt` / `vel` / `hdg` /
`pred_lat,pred_lon` only when the corresponding `AIRCRAFT_VALID_*`
bit is set. Absent fields are **omitted** from the JSON entirely
(not serialised as `null`), per the wire-format contract in
`resources/standards/json_protocol.py`.

**Wire format:** `AircraftFrame` schema — see
`resources/protocol_reference.md` §3.1.
