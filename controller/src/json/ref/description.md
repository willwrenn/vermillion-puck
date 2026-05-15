# json

**Role:** Parse one newline-stripped JSON line (per the
`AircraftFrame` schema in `resources/standards/json_protocol.md`) into a
`struct aircraft_t`.

**Files:** `src/json_parser.{c,h}`

**Public API:**
- `int json_parse_aircraft(char *buf, size_t len, struct aircraft_t *out);`
  - `buf` is **mutated in place** (Zephyr's parser writes NULs into the
    text). Caller must pass writable storage and accept the buffer is
    trashed on return.
  - Returns `0` on success, negative errno (most often `-EINVAL = -22`) on
    parse error or missing required key.
  - On success, `out->valid_mask` flags which of the optional fields
    (`alt`/`vel`/`hdg`) were present in this frame.

**Schema contract** (locked by `resources/standards/json_protocol.md`):
- Required: `type=="aircraft"`, `icao`, `lat`, `lon`, `ts`, `source`.
- Optional: `alt`, `vel`, `hdg` — **omitted** from the wire when unknown.
  `null` is NOT a valid wire value (Zephyr's parser rejects `null` for
  typed descriptors and aborts the whole parse with `-EINVAL`).

**Kconfig dependencies:**
- `CONFIG_JSON_LIBRARY=y` — base parser.
- `CONFIG_JSON_LIBRARY_FP_SUPPORT=y` — required! Without it the lexer
  refuses to step past the `.` in `-27.4975` and every frame errors -22.
- `CONFIG_FPU=y`, `CONFIG_FPU_SHARING=y` — `JSON_TOK_DOUBLE_FP` needs FP.

**Test path:** `skywatch test_json` runs two canned samples (all-fields and
optionals-absent) through this lib and prints every struct field — the
plan's Stage 2.3 verification mechanism.
