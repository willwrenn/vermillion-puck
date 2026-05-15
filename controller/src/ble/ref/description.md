# ble

**Role:** BLE GATT **central**. Scans for `skywatch-sim`, connects, MTU
exchanges, discovers the SkyWatch service (`4d8a4011-…0001`), subscribes
to the aircraft characteristic (`4d8a4011-…0002`), and hands every notified
38-byte payload off to `bridge` via `bridge_submit_frame()`.

**Files:** `src/ble_central.{c,h}`

**Wire contract:** `common/src/ble_packet.h` — single source of truth.
Mirrored on the sim node side (Will's `mobile/codein.c`).

**Public API:**
- `int  ble_central_init(void);` — `bt_enable`, kicks off scan, starts the
  reconnect-watchdog thread.
- `void ble_central_scan_{start,stop,list}(...)` — shell support.
- `bool ble_central_is_connected(void);`
- `void ble_central_disconnect(void);`
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
              MTU exchange → discover(SVC) → discover(CHR) → subscribe(NOTIFY)
                                                              │
                                                              ▼
                                                  notify_cb (length check)
                                                              │
                                                              ▼
                                                  bridge_submit_frame()
```

The reconnect watchdog (`ble_wd` thread, 2 s tick) re-arms the scan if all
three of `connected / connecting / scanning` are false for >2 s.

**Counters** (via `skywatch ble stats`):
`notify_count`, `bad_length` (frames not 38 B), `submitted`, `connect_count`,
`disconnect_count`, `connected`.

**Kconfig:** `CONFIG_BT=y`, `CONFIG_BT_CENTRAL=y`, `CONFIG_BT_GATT_CLIENT=y`,
`CONFIG_BT_OBSERVER=y`, `CONFIG_BT_SCAN_WITH_IDENTITY=y`,
`CONFIG_BT_L2CAP_TX_MTU=247`, `CONFIG_BT_BUF_ACL_{RX,TX}_SIZE=251`.

**Deferred:** The controller → mobile warning service (Will's
`ab340001-…` `warning_frame` + `custom_msg_frame`) is **not** discovered
here. It lands in Stage 8.3, in a separate sub-lib or as a sibling file in
this lib.

**Plan stages mapped here:** 3.1, 3.2.
