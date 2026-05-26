/*
 * SkyWatch controller — JSON -> struct aircraft_t deserialiser.
 *
 * Wire format spec: resources/standards/json_protocol.md
 * Source-of-truth Python module: resources/standards/json_protocol.py
 *
 * The Zephyr `zephyr/data/json.h` parser is in-place and mutates the input
 * buffer; the caller must pass writable storage.
 */

#ifndef JSON_PARSER_H
#define JSON_PARSER_H

#include <stddef.h>
#include "aircraft_t.h"

/*
 * Parse one newline-stripped JSON object into *out.
 *
 *   buf : writable buffer holding the JSON text. Will be mutated.
 *   len : length in bytes (no trailing newline).
 *   out : caller-allocated; fully overwritten on success.
 *
 * Returns 0 on success. Negative errno on parse error or missing required
 * keys (-EINVAL most commonly). out->valid_mask reflects which optional
 * fields (alt/vel/hdg) were present in this frame.
 */
int json_parse_aircraft(char *buf, size_t len, struct aircraft_t *out);

#endif /* JSON_PARSER_H */
