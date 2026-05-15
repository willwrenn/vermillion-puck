/*
 * SkyWatch controller — common in-memory aircraft record.
 *
 * Used by:
 *   - json_parser.c (Stage 2.3) populates this struct from an ADS-B JSON line.
 *   - ble_central.c (Stage 3.3) populates this struct from a BLE binary frame.
 *   - aircraft_db.c (Stage 4.1) stores entries of this shape.
 *   - kalman.c     (Stage 7.2) reads/writes pred_lat/pred_lon (added later).
 *
 * Field naming intentionally mirrors resources/standards/json_protocol.md.
 */

#ifndef AIRCRAFT_T_H
#define AIRCRAFT_T_H

#include <stdint.h>

#define AIRCRAFT_ICAO_LEN          6  /* 6 hex chars; storage = LEN+1 with NUL */
#define AIRCRAFT_ICAO_BUF_LEN      (AIRCRAFT_ICAO_LEN + 1)

/* Which optional fields are present in this observation. */
#define AIRCRAFT_VALID_ALT         (1u << 0)
#define AIRCRAFT_VALID_VEL         (1u << 1)
#define AIRCRAFT_VALID_HDG         (1u << 2)

enum aircraft_source {
	SRC_UNKNOWN = 0,
	SRC_ADS_B   = 1,
	SRC_BLE_SIM = 2,
};

struct aircraft_t {
	char     icao[AIRCRAFT_ICAO_BUF_LEN]; /* lowercase hex, NUL-terminated */
	double   lat;          /* WGS-84 degrees */
	double   lon;          /* WGS-84 degrees */
	double   ts;           /* POSIX epoch seconds (float for sub-second) */
	enum aircraft_source source;
	uint8_t  valid_mask;   /* AIRCRAFT_VALID_* */
	int32_t  alt_ft;       /* valid only if AIRCRAFT_VALID_ALT */
	double   vel_kt;       /* valid only if AIRCRAFT_VALID_VEL */
	double   hdg_deg;      /* valid only if AIRCRAFT_VALID_HDG; 0..360 */
};

static inline const char *aircraft_source_str(enum aircraft_source s)
{
	switch (s) {
	case SRC_ADS_B:   return "ADS_B";
	case SRC_BLE_SIM: return "BLE_SIM";
	default:          return "UNKNOWN";
	}
}

#endif /* AIRCRAFT_T_H */
