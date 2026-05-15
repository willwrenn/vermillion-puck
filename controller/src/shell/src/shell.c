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
#include "ble_central.h"
#include "bridge.h"
#include "ble_packet.h"
#include "aircraft_db.h"
#include "db_publisher.h"

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

/* ---------- skywatch ble ... ----------------------------------------- */

static int cmd_ble_scan_start(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc); ARG_UNUSED(argv);
	ble_central_scan_start();
	shell_print(sh, "scan started");
	return 0;
}

static int cmd_ble_scan_stop(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc); ARG_UNUSED(argv);
	ble_central_scan_stop();
	shell_print(sh, "scan stopped");
	return 0;
}

static int cmd_ble_scan_list(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc); ARG_UNUSED(argv);
	ble_central_scan_list(sh);
	return 0;
}

static int cmd_ble_disconnect(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc); ARG_UNUSED(argv);
	ble_central_disconnect();
	shell_print(sh, "disconnect requested");
	return 0;
}

static int cmd_ble_stats(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc); ARG_UNUSED(argv);
	struct ble_central_stats cs;
	struct bridge_stats      bs;
	ble_central_get_stats(&cs);
	bridge_get_stats(&bs);
	shell_print(sh,
		    "ble: connected=%s notify=%u bad_len=%u submitted=%u conn=%u disc=%u",
		    cs.connected ? "yes" : "no",
		    cs.notify_count, cs.bad_length, cs.submitted,
		    cs.connect_count, cs.disconnect_count);
	shell_print(sh,
		    "bridge: submitted=%u drops=%u converted=%u rej_src=%u rej_icao=%u rej_latlon=%u",
		    bs.submitted, bs.drops, bs.converted,
		    bs.rej_source, bs.rej_icao, bs.rej_latlon);
	return 0;
}

/* Hardcoded sample BLE frame for Stage 3.3 verification without a live sim. */
static int cmd_ble_inject(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc); ARG_UNUSED(argv);
	struct ble_aircraft_frame f = {0};
	f.source  = BLE_SRC_BLE_SIM;
	f.flags   = BLE_FLAG_ALT_VALID | BLE_FLAG_VEL_VALID | BLE_FLAG_HDG_VALID;
	f.icao24  = 0xABCDEFu;
	f.lat     = -27.4975;
	f.lon     = 153.0137;
	f.alt_ft  = 3500;
	f.vel_kt  = 250;
	f.hdg_deg = 90;
	f.ts_ms   = (uint32_t)k_uptime_get();

	struct aircraft_t out;
	int err = bridge_convert(&f, &out);
	if (err) {
		shell_error(sh, "bridge_convert failed: %d", err);
		return err;
	}
	shell_print(sh, "icao=%s src=%s lat=%.6f lon=%.6f alt=%d vel=%.1f hdg=%.1f valid=0x%x",
		    out.icao, aircraft_source_str(out.source),
		    out.lat, out.lon, out.alt_ft, out.vel_kt, out.hdg_deg,
		    out.valid_mask);
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_ble_scan,
	SHELL_CMD(start, NULL, "Start active scan.",  cmd_ble_scan_start),
	SHELL_CMD(stop,  NULL, "Stop active scan.",   cmd_ble_scan_stop),
	SHELL_CMD(list,  NULL, "List discovered peers.", cmd_ble_scan_list),
	SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_ble,
	SHELL_CMD(scan,       &sub_ble_scan, "Scan subcommands.", NULL),
	SHELL_CMD(disconnect, NULL,          "Disconnect current peer.", cmd_ble_disconnect),
	SHELL_CMD(stats,      NULL,          "Print BLE + bridge counters.", cmd_ble_stats),
	SHELL_CMD(inject,     NULL,          "Inject a hardcoded ble_aircraft_frame through the bridge (Stage 3.3 verifier).",
		  cmd_ble_inject),
	SHELL_SUBCMD_SET_END
);

/* ---------- skywatch db ... ------------------------------------------ */

struct dump_ctx { const struct shell *sh; int64_t now; int n; };

static bool dump_cb(const struct aircraft_db_entry *e, void *user)
{
	struct dump_ctx *c = user;
	int64_t age = c->now - e->last_seen_ms;

	char alt[12], vel[12], hdg[12];
	if (e->ac.valid_mask & AIRCRAFT_VALID_ALT)
		snprintf(alt, sizeof(alt), "%d", e->ac.alt_ft);
	else
		snprintf(alt, sizeof(alt), "-");
	if (e->ac.valid_mask & AIRCRAFT_VALID_VEL)
		snprintf(vel, sizeof(vel), "%.0f", e->ac.vel_kt);
	else
		snprintf(vel, sizeof(vel), "-");
	if (e->ac.valid_mask & AIRCRAFT_VALID_HDG)
		snprintf(hdg, sizeof(hdg), "%.0f", e->ac.hdg_deg);
	else
		snprintf(hdg, sizeof(hdg), "-");

	shell_print(c->sh, "  %-7s %-7s %10.5f %11.5f  alt=%-6s vel=%-5s hdg=%-4s  age=%lldms",
		    e->ac.icao,
		    aircraft_source_str(e->ac.source),
		    e->ac.lat, e->ac.lon,
		    alt, vel, hdg,
		    (long long)age);
	c->n++;
	return true;
}

static int cmd_db_dump(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc); ARG_UNUSED(argv);
	struct dump_ctx ctx = { .sh = sh, .now = k_uptime_get(), .n = 0 };
	shell_print(sh, "  icao    src                lat         lon  fields");
	aircraft_db_for_each(dump_cb, &ctx);
	shell_print(sh, "%d entries", ctx.n);
	return 0;
}

static int cmd_db_get(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_error(sh, "usage: skywatch db get <icao>");
		return -EINVAL;
	}
	struct aircraft_t out;
	int err = aircraft_db_lookup(argv[1], &out);
	if (err) {
		shell_error(sh, "not found: %s", argv[1]);
		return err;
	}
	shell_print(sh, "  icao=%s src=%s lat=%.6f lon=%.6f valid=0x%x",
		    out.icao, aircraft_source_str(out.source),
		    out.lat, out.lon, out.valid_mask);
	if (out.valid_mask & AIRCRAFT_VALID_ALT)
		shell_print(sh, "  alt_ft=%d", out.alt_ft);
	if (out.valid_mask & AIRCRAFT_VALID_VEL)
		shell_print(sh, "  vel_kt=%.1f", out.vel_kt);
	if (out.valid_mask & AIRCRAFT_VALID_HDG)
		shell_print(sh, "  hdg_deg=%.1f", out.hdg_deg);
	return 0;
}

static int cmd_db_clear(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc); ARG_UNUSED(argv);
	aircraft_db_clear();
	shell_print(sh, "db cleared");
	return 0;
}

static int cmd_db_stats(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc); ARG_UNUSED(argv);
	struct aircraft_db_stats s;
	struct db_publisher_stats p;
	aircraft_db_get_stats(&s);
	db_publisher_get_stats(&p);
	shell_print(sh,
		    "entries=%d inserts=%u updates=%u stale_evicts=%u pool_full=%u",
		    aircraft_db_count(),
		    s.inserts, s.updates, s.stale_evicts, s.pool_full);
	shell_print(sh,
		    "publisher: ticks=%u frames=%u fmt_err=%u",
		    p.ticks, p.frames_emitted, p.format_errors);
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_db,
	SHELL_CMD(dump,  NULL, "List all entries.",                cmd_db_dump),
	SHELL_CMD(get,   NULL, "Look up one entry: get <icao>.",   cmd_db_get),
	SHELL_CMD(clear, NULL, "Wipe the database.",                cmd_db_clear),
	SHELL_CMD(stats, NULL, "Print db counters.",                cmd_db_stats),
	SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_usb,
	SHELL_CMD(stats, NULL, "Print ACM1 Rx counters.", cmd_skywatch_usb_stats),
	SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_skywatch,
	SHELL_CMD(ping, NULL, "Reply with 'pong' (shell liveness check).",
		  cmd_skywatch_ping),
	SHELL_CMD(usb, &sub_usb, "USB CDC ACM1 Rx subcommands.", NULL),
	SHELL_CMD(ble, &sub_ble, "BLE central / bridge subcommands.", NULL),
	SHELL_CMD(db,  &sub_db,  "Aircraft database subcommands.", NULL),
	SHELL_CMD(test_json, NULL,
		  "Parse hardcoded ADS-B JSON samples and print each struct field.",
		  cmd_skywatch_test_json),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(skywatch, &sub_skywatch,
		   "SkyWatch controller commands.", NULL);
