# shell

**Role:** Zephyr shell command tree under the `skywatch` root, exposed over
USB CDC ACM0 (board-default `zephyr,shell-uart`).

**Files:** `src/shell.c`

**Current commands:**
| Command | Purpose |
|---|---|
| `skywatch ping` | Replies `pong`. Liveness check. |
| `skywatch usb stats` | Prints `bytes_rx / frames_rx / ringbuf_drops / line_overruns / parse_errors` counters from `usb_handler`. |
| `skywatch test_json` | Parses two hardcoded ADS-B JSON samples through `json_parser` and prints every field, exercising both the all-fields-present and optionals-absent paths. |

**Convention:** every command sits under the `skywatch` root command using
`SHELL_STATIC_SUBCMD_SET_CREATE` + `SHELL_CMD`. Group related commands into a
sub-tree (see `sub_usb`).

**Adding a new subcommand:** drop a `cmd_skywatch_*` function in `shell.c`,
register it under the `skywatch` tree. If the new feature has its own state,
expose a tiny accessor in the owning lib (e.g. `usb_handler_get_stats()`) and
keep `shell.c` thin.
