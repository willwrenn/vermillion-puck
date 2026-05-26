/*
 * SkyWatch controller — periodic DB serialiser onto ACM1.
 *
 * Every 500 ms (2 Hz), walks `aircraft_db` and emits one `AircraftFrame`
 * JSON per entry on the ACM1 data port (via `usb_serial_println`). This is
 * the integration handshake the laptop-side GUI reads.
 *
 * Wire format: resources/standards/json_protocol.md (`AircraftFrame`).
 * Optional fields (alt/vel/hdg) are *omitted* when their AIRCRAFT_VALID_*
 * bit is clear — never serialised as `null`, matching the contract we set
 * in .
 */

#ifndef DB_PUBLISHER_H
#define DB_PUBLISHER_H

#include <stdint.h>

int db_publisher_init(void);

struct db_publisher_stats {
	uint32_t ticks;          /* work-handler invocations */
	uint32_t frames_emitted; /* JSON lines written to ACM1 */
	uint32_t format_errors;  /* snprintf cap exhausted */
};

void db_publisher_get_stats(struct db_publisher_stats *out);

#endif /* DB_PUBLISHER_H */
