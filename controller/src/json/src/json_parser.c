/*
 * SkyWatch controller — JSON -> struct aircraft_t deserialiser (Stage 2.3).
 *
 * Wire spec: resources/standards/json_protocol.md
 *
 * Implementation notes:
 *   - The on-the-wire AircraftFrame includes string fields (`type`, `icao`,
 *     `source`). Zephyr's parser deposits these as `const char *` pointing
 *     into the input buffer (NUL-terminated after parse). We copy them into
 *     the caller's struct so the result outlives the input buffer.
 *   - Optional `alt`, `vel`, `hdg` are simply absent from the JSON when
 *     unknown (the spec forbids `null` for typed fields). The Zephyr parser
 *     returns a per-descriptor bitfield indicating which keys were seen,
 *     which we project into out->valid_mask.
 *   - JSON_TOK_DOUBLE_FP is used for lat/lon/ts/vel/hdg so we get IEEE 754
 *     doubles directly. Requires CONFIG_FPU=y.
 */

#include "json_parser.h"

#include <errno.h>
#include <string.h>

#include <zephyr/data/json.h>
#include <zephyr/sys/util.h>

/* The "wire shape" used only inside this file. */
struct ac_json {
	const char *type;
	const char *icao;
	double      lat;
	double      lon;
	int32_t     alt;
	double      vel;
	double      hdg;
	double      ts;
	const char *source;
};

/* Descriptor table — order defines the bitfield indices below. */
static const struct json_obj_descr ac_descr[] = {
	JSON_OBJ_DESCR_PRIM(struct ac_json, type,   JSON_TOK_STRING),     /* 0 */
	JSON_OBJ_DESCR_PRIM(struct ac_json, icao,   JSON_TOK_STRING),     /* 1 */
	JSON_OBJ_DESCR_PRIM(struct ac_json, lat,    JSON_TOK_DOUBLE_FP),  /* 2 */
	JSON_OBJ_DESCR_PRIM(struct ac_json, lon,    JSON_TOK_DOUBLE_FP),  /* 3 */
	JSON_OBJ_DESCR_PRIM(struct ac_json, alt,    JSON_TOK_NUMBER),     /* 4 */
	JSON_OBJ_DESCR_PRIM(struct ac_json, vel,    JSON_TOK_DOUBLE_FP),  /* 5 */
	JSON_OBJ_DESCR_PRIM(struct ac_json, hdg,    JSON_TOK_DOUBLE_FP),  /* 6 */
	JSON_OBJ_DESCR_PRIM(struct ac_json, ts,     JSON_TOK_DOUBLE_FP),  /* 7 */
	JSON_OBJ_DESCR_PRIM(struct ac_json, source, JSON_TOK_STRING),     /* 8 */
};

#define IDX_TYPE   0
#define IDX_ICAO   1
#define IDX_LAT    2
#define IDX_LON    3
#define IDX_ALT    4
#define IDX_VEL    5
#define IDX_HDG    6
#define IDX_TS     7
#define IDX_SOURCE 8

#define BIT(n)  ((uint64_t)1 << (n))
#define REQUIRED (BIT(IDX_TYPE) | BIT(IDX_ICAO) | BIT(IDX_LAT) | \
		  BIT(IDX_LON) | BIT(IDX_TS) | BIT(IDX_SOURCE))

int json_parse_aircraft(char *buf, size_t len, struct aircraft_t *out)
{
	if (!buf || !out || len == 0) {
		return -EINVAL;
	}

	struct ac_json src;
	memset(&src, 0, sizeof(src));

	int64_t r = json_obj_parse(buf, len, ac_descr, ARRAY_SIZE(ac_descr), &src);
	if (r < 0) {
		return (int)r;
	}

	uint64_t present = (uint64_t)r;
	if ((present & REQUIRED) != REQUIRED) {
		return -EINVAL;
	}
	if (!src.type || strcmp(src.type, "aircraft") != 0) {
		return -EINVAL;
	}
	if (!src.icao || !src.source) {
		return -EINVAL;
	}

	memset(out, 0, sizeof(*out));

	/* icao: bounded copy + NUL terminate */
	size_t icao_len = strnlen(src.icao, AIRCRAFT_ICAO_BUF_LEN - 1);
	memcpy(out->icao, src.icao, icao_len);
	out->icao[icao_len] = '\0';

	out->lat = src.lat;
	out->lon = src.lon;
	out->ts  = src.ts;

	if (strcmp(src.source, "ADS_B") == 0) {
		out->source = SRC_ADS_B;
	} else if (strcmp(src.source, "BLE_SIM") == 0) {
		out->source = SRC_BLE_SIM;
	} else {
		out->source = SRC_UNKNOWN;
	}

	if (present & BIT(IDX_ALT)) {
		out->valid_mask |= AIRCRAFT_VALID_ALT;
		out->alt_ft = src.alt;
	}
	if (present & BIT(IDX_VEL)) {
		out->valid_mask |= AIRCRAFT_VALID_VEL;
		out->vel_kt = src.vel;
	}
	if (present & BIT(IDX_HDG)) {
		out->valid_mask |= AIRCRAFT_VALID_HDG;
		out->hdg_deg = src.hdg;
	}

	return 0;
}
