/*
 * SkyWatch controller — common in-memory aircraft record.
 *
 * Used by:
 * - json_parser.c populates this struct from an ADS-B JSON line.
 * - ble_central.c populates this struct from a BLE binary frame.
 * - aircraft_db.c stores entries of this shape.
 * - kalman.c reads/writes pred_lat/pred_lon (added later).
 *
 * Field naming intentionally mirrors resources/standards/json_protocol.md.
 */

#ifndef AIRCRAFT_T_H
#define AIRCRAFT_T_H

#include <stdint.h>

#define AIRCRAFT_ICAO_LEN 6 /* 6 hex chars; storage = LEN+1 with NUL */
#define AIRCRAFT_ICAO_BUF_LEN (AIRCRAFT_ICAO_LEN + 1)

/* Which optional fields are present in this observation. */
#define AIRCRAFT_VALID_ALT (1u << 0)
#define AIRCRAFT_VALID_VEL (1u << 1)
#define AIRCRAFT_VALID_HDG (1u << 2)
/* : set once the Kalman filter for this aircraft has had >=3
 * updates and `pred_lat/pred_lon` are meaningful. */
#define AIRCRAFT_VALID_PRED (1u << 3)

//Purpose: This enum represents the source of the aircraft data.
// ADS_B = real flight from dump1090
// BLE_SIM = Will's mobile sim node
// FICT = fictitious / test scaffold (`skywatch collision inject`,
// future `ghost`, anything else synthetic — clearly marked
// so they're never confused with live traffic)
enum aircraft_source {
	SRC_UNKNOWN = 0,
	SRC_ADS_B   = 1,
	SRC_BLE_SIM = 2,
	SRC_FICT    = 3,
};

//Purpose: This struct represents an aircraft record in the controller.
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
	double   pred_lat;     /* Kalman 10s-ahead lat — valid if AIRCRAFT_VALID_PRED */
	double   pred_lon;     /* Kalman 10s-ahead lon — valid if AIRCRAFT_VALID_PRED */
};


//Purpose: This function returns a string representation of the aircraft source enum value.
static inline const char *aircraft_source_str(enum aircraft_source s)
{
	switch (s) {
	case SRC_ADS_B:   return "ADS_B";
	case SRC_BLE_SIM: return "BLE_SIM";
	case SRC_FICT:    return "FICT";
	default:          return "UNKNOWN";
	}
}

#endif /* AIRCRAFT_T_H */
