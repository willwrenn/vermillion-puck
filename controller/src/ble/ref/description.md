# ble

**Role:** BLE GATT **central**. Scans for `skywatch-sim`, connects,
MTU exchanges, discovers the SkyWatch service (`4d8a4011-…0001`),
subscribes to the aircraft characteristic (`4d8a4011-…0002`), and
hands every notified 38-byte payload off to `bridge` via
`bridge_submit_frame()`. Also discovers the controller-side warning
service (`ab340001-…`) and sends 16-byte warning frames back to the
sim on every collision-detector transition.

**Files:** `src/ble_central.{c,h}`

**Wire contract:** `common/src/ble_packet.h` — single source of
truth. Mirrored on the sim node side (Will's `sim/mobile/src/main/src/codein.c`).

**Public API:**

- `int  ble_central_init(void);` — `bt_enable`, kicks off scan,
  starts the reconnect-watchdog thread.
- `void ble_central_scan_{start,stop,list}(...)` — shell support.
- `bool ble_central_is_connected(void);`
- `void ble_central_disconnect(void);`
- `int  ble_central_send_warning(const struct ble_warning_frame *);`
  — DTR-style: drops with `-ENOTCONN` if no sim is connected.
  Computes the XOR CRC over the first 15 bytes before sending.
- `void ble_central_get_stats(struct ble_central_stats *);` — counters.

**Architecture:**

```
BT stack ─ device_found ──┐
                          │ (exact-name match: SKYWATCH_BLE_DEVICE_NAME)
                          ▼
                       do_connect → bt_conn_le_create
                          │
                   ┌──────┴─────┐
                   ▼            ▼
              connected_cb   disconnected_cb ── scan_start
                   │
              MTU exchange
                   │
                   ▼
              discover(SkyWatch SVC) → discover(aircraft CHR) → subscribe(NOTIFY)
                   │                                              │
                   ▼                                              ▼
              discover(Warning SVC) → discover(warn CHR)    notify_cb (length check)
                   │                                              │
                   ▼                                              ▼
              (cache write handle for                  bridge_submit_frame()
               ble_central_send_warning)
```

The reconnect watchdog (`ble_wd` thread, 2 s tick) re-arms the scan
if all three of `connected / connecting / scanning` are false for
more than 2 s.

**Counters** (via `skywatch ble stats`):
`notify_count`, `bad_length` (frames not 38 B), `submitted`,
`connect_count`, `disconnect_count`, `connected`, `warn_sent`,
`warn_fail`.

**Kconfig:** `CONFIG_BT=y`, `CONFIG_BT_CENTRAL=y`,
`CONFIG_BT_GATT_CLIENT=y`, `CONFIG_BT_OBSERVER=y`,
`CONFIG_BT_SCAN_WITH_IDENTITY=y`, `CONFIG_BT_L2CAP_TX_MTU=247`,
`CONFIG_BT_BUF_ACL_{RX,TX}_SIZE=251`.
