/*
 * SkyWatch controller shell — Phase 2.1 skeleton.
 *
 * Stage:    2.1
 * Purpose:  Single `skywatch` subcommand tree with one `ping` leaf so we can
 *           prove the shell is alive over ACM0 ("skywatch ping" -> "pong").
 *           Later phases hang db/kalman/ble/collision subcommands off the
 *           same root (Stages 4.3, 7.3, 8.x).
 */

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>

#include "usb_handler.h"
#include "json_parser.h"
#include "aircraft_t.h"

static int cmd_skywatch_ping(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	shell_print(sh, "pong");
	return 0;
}

static int cmd_skywatch_usb_stats(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	struct usb_handler_stats s;
	usb_handler_get_stats(&s);
	shell_print(sh,
		    "bytes_rx=%u frames_rx=%u ringbuf_drops=%u line_overruns=%u parse_errors=%u",
		    s.bytes_rx, s.frames_rx, s.ringbuf_drops, s.line_overruns,
		    s.parse_errors);
	return 0;
}

/* Two canned ADS-B frames used by `skywatch test_json` —
 *   - "full"  exercises every field including the optionals,
 *   - "minimal" omits alt/vel/hdg to exercise the optional-absent path. */
static const char SAMPLE_FULL[] =
	"{\"type\":\"aircraft\",\"icao\":\"a1b2c3\","
	"\"lat\":-27.4975,\"lon\":153.0137,"
	"\"alt\":3500,\"vel\":450.0,\"hdg\":90.0,"
	"\"ts\":1715683200.0,\"source\":\"ADS_B\"}";

static const char SAMPLE_MINIMAL[] =
	"{\"type\":\"aircraft\",\"icao\":\"d4e5f6\","
	"\"lat\":-27.5,\"lon\":153.0,"
	"\"ts\":1715683201.0,\"source\":\"ADS_B\"}";

static void print_parsed(const struct shell *sh, const struct aircraft_t *ac)
{
	shell_print(sh, "  icao        = %s",   ac->icao);
	shell_print(sh, "  source      = %s",   aircraft_source_str(ac->source));
	shell_print(sh, "  lat         = %.6f", ac->lat);
	shell_print(sh, "  lon         = %.6f", ac->lon);
	shell_print(sh, "  ts          = %.3f", ac->ts);
	shell_print(sh, "  valid_mask  = 0x%x", ac->valid_mask);
	if (ac->valid_mask & AIRCRAFT_VALID_ALT) {
		shell_print(sh, "  alt_ft      = %d", ac->alt_ft);
	} else {
		shell_print(sh, "  alt_ft      = (absent)");
	}
	if (ac->valid_mask & AIRCRAFT_VALID_VEL) {
		shell_print(sh, "  vel_kt      = %.1f", ac->vel_kt);
	} else {
		shell_print(sh, "  vel_kt      = (absent)");
	}
	if (ac->valid_mask & AIRCRAFT_VALID_HDG) {
		shell_print(sh, "  hdg_deg     = %.1f", ac->hdg_deg);
	} else {
		shell_print(sh, "  hdg_deg     = (absent)");
	}
}

static int parse_and_print(const struct shell *sh, const char *src, const char *label)
{
	char buf[256];
	size_t n = strnlen(src, sizeof(buf) - 1);
	memcpy(buf, src, n);
	buf[n] = '\0';

	shell_print(sh, "--- %s ---", label);
	shell_print(sh, "JSON: %s", src);

	struct aircraft_t ac;
	int err = json_parse_aircraft(buf, n, &ac);
	if (err < 0) {
		shell_error(sh, "parse failed: %d", err);
		return err;
	}
	print_parsed(sh, &ac);
	return 0;
}

static int cmd_skywatch_test_json(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	int rc1 = parse_and_print(sh, SAMPLE_FULL,    "full sample");
	int rc2 = parse_and_print(sh, SAMPLE_MINIMAL, "minimal sample (alt/vel/hdg absent)");
	return (rc1 < 0) ? rc1 : rc2;
}


SHELL_STATIC_SUBCMD_SET_CREATE(sub_usb,
	SHELL_CMD(stats, NULL, "Print ACM1 Rx counters.", cmd_skywatch_usb_stats),
	SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_skywatch,
	SHELL_CMD(ping, NULL, "Reply with 'pong' (shell liveness check).",
		  cmd_skywatch_ping),
	SHELL_CMD(usb, &sub_usb, "USB CDC ACM1 Rx subcommands.", NULL),
	SHELL_CMD(test_json, NULL,
		  "Parse hardcoded ADS-B JSON samples and print each struct field.",
		  cmd_skywatch_test_json),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(skywatch, &sub_skywatch,
		   "SkyWatch controller commands.", NULL);
