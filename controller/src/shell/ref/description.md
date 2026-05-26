# shell

**Role:** Zephyr shell command tree under the `skywatch` root,
exposed over USB CDC ACM0 (board-default `zephyr,shell-uart`).

**Files:** `src/shell.c`

**Subcommand groups:**

| Group | Purpose |
|---|---|
| `skywatch ping` | Replies `pong`. Liveness check. |
| `skywatch ble {stats,scan,disconnect,inject}` | BLE central state + manual scan controls + frame injection for offline test. |
| `skywatch db {dump,count,clear}` | Aircraft DB walk + entry count + wipe. |
| `skywatch collision {stats,inject,stop,crash,ghost,ghost_at,ghost_stop}` | Collision detector counters + synthetic-scenario hooks (head-on pair, ghost targeting BLE sim or any ICAO, manual CRASH trigger). |
| `skywatch missile {launch,cancel}` | Pure-pursuit guided-missile sandbox at the BLE sim. |
| `skywatch diversion <left\|right\|climb\|descend\|rtb\|hold>` | Manually fire a diversion suggestion to the sim. |
| `skywatch kalman {q,r,bench}` | Tune process / measurement noise; benchmark 1000 predict+update iterations. |
| `skywatch usb stats` | Data-port Rx counters from `usb_handler`. |
| `skywatch test_json` | Parse two hardcoded ADS-B JSON samples through `json_parser` — in-tree smoke test. |

**Convention:** every command sits under the `skywatch` root using
`SHELL_STATIC_SUBCMD_SET_CREATE` + `SHELL_CMD`. Group related
commands into a sub-tree (e.g. `sub_collision`, `sub_ble`).

**Adding a new subcommand:** drop a `cmd_skywatch_*` function in
`shell.c`, register it under the appropriate sub-tree. If the new
feature has its own state, expose a tiny accessor in the owning lib
(e.g. `usb_handler_get_stats()`, `collision_get_stats()`) and keep
`shell.c` thin.
