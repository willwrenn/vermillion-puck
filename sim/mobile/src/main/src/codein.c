#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <stdio.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/shell/shell.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "usb_serial.h"
#include "ble_packet.h"
// AI for watchdog
#include <zephyr/task_wdt/task_wdt.h>

#define BUZZER_NODE DT_ALIAS(buzzer)
static const struct gpio_dt_spec buzzer = GPIO_DT_SPEC_GET(BUZZER_NODE, gpios);
//CONFIG
#define RED DT_ALIAS(led0)
#define GREEN DT_ALIAS(led1)

static const struct gpio_dt_spec red = GPIO_DT_SPEC_GET(RED, gpios);
static const struct gpio_dt_spec green = GPIO_DT_SPEC_GET(GREEN, gpios);

// cdc_acm_uart1 = ttyACM1 — command input from GUI
#define DATA_UART_NODE DT_NODELABEL(cdc_acm_uart1)
static const struct device *data_uart = DEVICE_DT_GET(DATA_UART_NODE);
#define MOBILE_ID     SKYWATCH_BLE_DEVICE_NAME
#define MOBILE_ID_LEN (sizeof(SKYWATCH_BLE_DEVICE_NAME) - 1)

#define SEND_STACK_SIZE 4096
#define SEND_PRIORITY 7
#define CMD_STACK_SIZE 2048
#define CMD_PRIORITY   6
#define CMD_BUF_SIZE   80

static int g_wdt_channel = -1;

// Aircraft state globals
static int32_t  g_lat      = -274975000;  // -27.4975 St Lucia
static int32_t  g_lon      =  1530137000; // 153.0137
static uint16_t g_alt      = 100;
static uint16_t g_speed    = 0;
static uint16_t g_heading  = 0;
static uint8_t  g_icao[3]  = {0xDA, 0x77, 0x05};
static volatile bool g_warning_active;   // set by warning GATT write, cleared on reset
static float    g_turn_rate = 0.0f;      // degrees per 200ms tick; 0 = straight
static float    g_hdg_acc   = 0.0f;      // fractional heading accumulator for smooth circles

/* Stage 11 — CRASH freeze. When the controller sends a CRASH frame
 * (msg_type=0x05), we set this to k_uptime_get()+10000 ms; the send
 * thread skips dead-reckoning while frozen, and crash_reset_fn snaps
 * the aircraft back to St Lucia origin after the freeze window. */
static volatile int64_t g_frozen_until_ms = 0;

// Brisbane bounds
#define LAT_MIN -278000000  // -27.8 (south)
#define LAT_MAX -270000000  // -27.0 (north)
#define LON_MIN  1525000000 // 152.5 (west)
#define LON_MAX  1535000000 // 153.5 (east)
#define ALT_MIN  0
#define ALT_MAX  5000       // metres

// Warning frame — base to mobile (collision warnings / diversion suggestions)
struct __packed warning_frame {
	uint8_t  msg_type;   // 0x02=collision warning 0x03=diversion suggestion
	uint8_t  icao[3];    // ICAO of conflicting aircraft
	uint8_t  lat[3];     // conflict position lat
	uint8_t  lon[3];     // conflict position lon
	uint16_t alt;        // conflict altitude (metres)
	uint8_t  severity;   // 0=advisory 1=warning 2=urgent
	int8_t   hdg_delta;  // suggested heading change degrees (+ve=right)
	uint8_t  timestamp;
	uint8_t  crc;        // XOR CRC
};

// Custom text message frame — base to mobile (free-text diversion instruction)
struct __packed custom_msg_frame {
	uint8_t msg_type;    // 0x04 = free-text instruction
	char    text[28];    // null-terminated message e.g. "lower altitude"
	uint8_t timestamp;
	uint8_t crc;         // XOR CRC
};

// SkyWatch aircraft telemetry service (mobile → base, NOTIFY)
// UUIDs from ble_packet.h — must match base's discovery
#define SKY_SVC_UUID  BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0x4d8a4011, 0x7f24, 0x43c4, 0x9c5b, 0xa17c7af70001))
#define SKY_CHR_UUID  BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0x4d8a4011, 0x7f24, 0x43c4, 0x9c5b, 0xa17c7af70002))

// Warning service (base → mobile, WRITE) — unchanged
#define WARN_SVC_UUID  BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0xab340001, 0x1234, 0x5678, 0xabcd, 0x1234567890ab))
#define WARN_CHR_UUID  BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0xab340002, 0x1234, 0x5678, 0xabcd, 0x1234567890ab))

// auto-clear work — fires 10 s after a warning; notifies GUI then resets state
static void warning_clear_fn(struct k_work *work)
{
	g_warning_active = false;
	printk("WARN clear\n"); // GUI detects this line and stops flashing
}
static K_WORK_DELAYABLE_DEFINE(warning_clear_work, warning_clear_fn);

/* Stage 11 — after the 10 s CRASH freeze, snap the aircraft back to St
 * Lucia origin and clear all warning state. Same effect as the operator
 * typing 'r' over ttyACM1 (process_gui_cmd case 'r'). */
static void crash_reset_fn(struct k_work *work)
{
	g_lat            = -274975000;
	g_lon            =  1530137000;
	g_alt            = 100;
	g_speed          = 0;
	g_heading        = 0;
	g_turn_rate      = 0.0f;
	g_hdg_acc        = 0.0f;
	g_warning_active = false;
	g_frozen_until_ms = 0;
	k_work_cancel_delayable(&warning_clear_work);
	printk("WARN clear\n");    // GUI clears the CRASHED overlay
	printk("RESET St Lucia\n"); // diagnostic line on ttyACM0
}
static K_WORK_DELAYABLE_DEFINE(crash_reset_work, crash_reset_fn);

// GATT write handler — base pushes collision/diversion/message frames here
static ssize_t warn_write(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			  const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	if (len < 2) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}
	uint8_t msg_type = ((const uint8_t *)buf)[0];

	if (msg_type == 0x04) {
		// free-text instruction frame
		if (len != sizeof(struct custom_msg_frame)) {
			return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
		}
		const struct custom_msg_frame *m = buf;
		// verify XOR CRC
		const uint8_t *b = (const uint8_t *)m;
		uint8_t crc = 0;
		for (int i = 0; i < (int)sizeof(*m) - 1; i++) crc ^= b[i];
		if (crc != m->crc) {
			return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
		}
		// print text so GUI reads it on ttyACM0
		printk("WARN msg %.*s\n", (int)(sizeof(m->text) - 1), m->text);
		g_warning_active = true;
		k_work_reschedule(&warning_clear_work, K_SECONDS(3));
		return len;
	}

	// collision / diversion frame
	if (len != sizeof(struct warning_frame)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}
	const struct warning_frame *w = buf;
	// verify XOR CRC
	const uint8_t *b = (const uint8_t *)w;
	uint8_t crc = 0;
	for (int i = 0; i < (int)sizeof(*w) - 1; i++) crc ^= b[i];
	if (crc != w->crc) {
		return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
	}
	switch (w->msg_type) {
	case 0x02: // collision warning
		printk("WARN collision ICAO=%02x%02x%02x sev=%u\n",
		       w->icao[0], w->icao[1], w->icao[2], w->severity);
		g_warning_active = true;
		/* 5 s clear (was 2 s) — paired with the controller's 3 s re-fire
		 * cadence so the buzzer + red LED stay continuously on through
		 * the encounter instead of beeping 2 s on / 1 s off. */
		k_work_reschedule(&warning_clear_work, K_SECONDS(5));
		break;
	case 0x03: // diversion suggestion (hdg_delta + alt_delta)
		/* Suggestion only — print to ttyACM0 so the PC GUI shows the
		 * orange DIVERSION panel, set the warning flag for the LED/
		 * buzzer alert, but do NOT change heading. The human pilot
		 * (Will using wasd) decides whether to act on the suggestion.
		 *
		 * Print BOTH hdg_delta AND alt (signed) so the GUI can
		 * distinguish CLIMB / DESCEND (encoded via the `alt` field,
		 * hdg_delta=0) from LEFT / RIGHT (hdg_delta=±30, alt=0). */
		printk("WARN diversion ICAO=%02x%02x%02x hdg_delta=%d alt_delta=%d\n",
		       w->icao[0], w->icao[1], w->icao[2],
		       w->hdg_delta, (int16_t)w->alt);
		g_warning_active = true;
		k_work_reschedule(&warning_clear_work, K_SECONDS(5));
		break;
	case 0x05: // Stage 11 — CRASH: freeze 10 s then reset to St Lucia.
		printk("WARN crash\n");   // Will's GUI shows full-screen overlay
		g_frozen_until_ms = k_uptime_get() + 10000;
		g_warning_active = true;
		k_work_cancel_delayable(&warning_clear_work);
		k_work_reschedule(&crash_reset_work, K_MSEC(10000));
		break;
	default:
		break;
	}
	return len;
}

static struct bt_conn *s_conn;
static volatile bool   s_notif_enabled;

// CCC callback — tracks when base enables/disables aircraft notifications
static void aircraft_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	s_notif_enabled = (value == BT_GATT_CCC_NOTIFY);
}

// SkyWatch aircraft service — mobile notifies base with ble_aircraft_frame at 5Hz
// attrs[0]=service  attrs[1]=char-decl  attrs[2]=char-value  attrs[3]=CCCD
BT_GATT_SERVICE_DEFINE(skywatch_svc,
	BT_GATT_PRIMARY_SERVICE(SKY_SVC_UUID),
	BT_GATT_CHARACTERISTIC(SKY_CHR_UUID,
			       BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_NONE,
			       NULL, NULL, NULL),
	BT_GATT_CCC(aircraft_ccc_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

// Warning service: base writes alerts here; new SkyWatch service carries position frames up
BT_GATT_SERVICE_DEFINE(warn_svc,
	BT_GATT_PRIMARY_SERVICE(WARN_SVC_UUID),
	BT_GATT_CHARACTERISTIC(WARN_CHR_UUID,
			       BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
			       BT_GATT_PERM_WRITE,
			       NULL, warn_write, NULL),
);

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
	BT_DATA(BT_DATA_NAME_COMPLETE, MOBILE_ID, MOBILE_ID_LEN),
};

// SkyWatch service UUID in scan response so base can discover us
static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL,
		      BT_UUID_128_ENCODE(0x4d8a4011, 0x7f24, 0x43c4, 0x9c5b, 0xa17c7af70001)),
};

// called by BLE stack when a connection is established zephyr/samples/bluetooth/peripheral/src/main.c.
static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		return;
	}
	gpio_pin_set_dt(&red, 0);
	gpio_pin_set_dt(&green, 1);

	s_conn = bt_conn_ref(conn);  // take a reference so conn stays valid
}

// called when connection drops, releases conn ref and restarts advertising so base can
// reconnect zephyr/samples/bluetooth/mtu_update/peripheral/src/peripheral_mtu_update.c
static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	if (s_conn) {
		bt_conn_unref(s_conn); // release our reference; conn will be freed by stack
		s_conn = NULL;
	}
	s_notif_enabled  = false;
	g_warning_active = false;
	k_work_cancel_delayable(&warning_clear_work); // don't fire after disconnect

	gpio_pin_set_dt(&red, 1);
	gpio_pin_set_dt(&green, 0);

	bt_le_adv_stop();
	// zephyr/samples/bluetooth/peripheral_nus/src/main.c sets up the Peripheral (GAP)
	bt_le_adv_start(BT_LE_ADV_CONN_FAST_2, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
}

// registers connected/disconnected callbacks with BLE stack
BT_CONN_CB_DEFINE(conn_callbacks) = {.connected = connected, .disconnected = disconnected,};

// Clamp position to Brisbane bounds
static void clamp_position(void)
{
	if (g_lat < LAT_MIN) g_lat = LAT_MIN;
	if (g_lat > LAT_MAX) g_lat = LAT_MAX;
	if (g_lon < LON_MIN) g_lon = LON_MIN;
	if (g_lon > LON_MAX) g_lon = LON_MAX;
	if (g_alt < ALT_MIN) g_alt = ALT_MIN;
	if (g_alt > ALT_MAX) g_alt = ALT_MAX;
}

static void process_gui_cmd(uint8_t c)
{
	switch (c) {
	case 'w': // speed up
		g_speed += 10;
		if (g_speed > 500) g_speed = 500;
		break;
	case 's': // slow down
		if (g_speed >= 10) g_speed -= 10;
		else g_speed = 0;
		break;
	case 'a': // turn left
		g_heading = (g_heading + 360 - 10) % 360;
		break;
	case 'd': // turn right
		g_heading = (g_heading + 10) % 360;
		break;
	case 'q': // climb
		g_alt += 50;
		if (g_alt > ALT_MAX) g_alt = ALT_MAX;
		break;
	case 'e': // descend
		if (g_alt >= 50) g_alt -= 50;
		else g_alt = 0;
		break;
	case 'r': // reset to St Lucia
		g_lat = -274975000;
		g_lon = 1530137000;
		g_alt = 100;
		g_speed          = 0;
		g_heading        = 0;
		g_turn_rate      = 0.0f;
		g_hdg_acc        = 0.0f;
		g_warning_active = false;
		k_work_cancel_delayable(&warning_clear_work);
		break;
	default:
		break;
	}
}

static void send_position_json(void)
{
	printk("{\"lat\":%ld,\"lon\":%ld,\"alt\":%u,\"spd\":%u,\"hdg\":%u}\n",
		(long)g_lat, (long)g_lon,
		(unsigned int)g_alt,
		(unsigned int)g_speed,
		(unsigned int)g_heading);
}

// Shell command: set <lat> <lon> <alt> <speed> <heading>
// e.g. "aircraft set -274698000 1530251000 100 50 270"
static int cmd_aircraft_set(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 6) {
		shell_print(sh, "Usage: aircraft set <lat> <lon> <alt> <speed> <heading>");
		return -EINVAL;
	}
	g_lat     = (int32_t)atoi(argv[1]);
	g_lon     = (int32_t)atoi(argv[2]);
	g_alt     = (uint16_t)atoi(argv[3]);
	g_speed   = (uint16_t)atoi(argv[4]);
	g_heading = (uint16_t)atoi(argv[5]);
	send_position_json();
	return 0;
}

// Shell command: wasd <key> — control aircraft heading, speed and altitude
static int cmd_aircraft_wasd(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 2 || strlen(argv[1]) != 1) {
		shell_print(sh, "Usage: aircraft wasd <w/a/s/d/q/e>");
		return -EINVAL;
	}
	process_gui_cmd((uint8_t)argv[1][0]);
	send_position_json();
	return 0;
}

// Shell command: status — print current position as JSON
static int cmd_aircraft_status(const struct shell *sh, size_t argc, char **argv)
{
	send_position_json();
	return 0;
}

// Shell command: reset — return to St Lucia
static int cmd_aircraft_reset(const struct shell *sh, size_t argc, char **argv)
{
	g_lat            = -274975000;
	g_lon            =  1530137000;
	g_alt            = 100;
	g_speed          = 0;
	g_heading        = 0;
	g_turn_rate      = 0.0f;
	g_hdg_acc        = 0.0f;
	g_warning_active = false;
	send_position_json();
	return 0;
}

// Shell command: circle <speed_kt> <radius_m>  — fly constant circles
//                circle 0                       — stop circling
static int cmd_aircraft_circle(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 2) {
		shell_print(sh, "Usage: aircraft circle <speed_kt> <radius_m>  (radius default 500)");
		shell_print(sh, "       aircraft circle 0  — stop circling");
		return -EINVAL;
	}
	int spd = atoi(argv[1]);
	if (spd == 0) {
		g_turn_rate = 0.0f;
		shell_print(sh, "Circling stopped");
		return 0;
	}
	int radius_m = (argc >= 3) ? atoi(argv[2]) : 500;
	if (radius_m <= 0) {
		shell_error(sh, "radius_m must be > 0");
		return -EINVAL;
	}
	g_speed     = (uint16_t)spd;
	float speed_ms  = spd * 0.514444f;
	g_turn_rate = (speed_ms / (float)radius_m) * (180.0f / 3.14159f) * 0.2f;
	g_hdg_acc   = (float)g_heading;
	shell_print(sh, "Circling: %d kt, %d m radius, %.2f deg/tick", spd, radius_m, (double)g_turn_rate);
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(aircraft_cmds,
	SHELL_CMD(set,    NULL, "Set aircraft state: <lat> <lon> <alt> <speed> <heading>", cmd_aircraft_set),
	SHELL_CMD(wasd,   NULL, "Control: w=faster s=slower a=left d=right q=up e=down", cmd_aircraft_wasd),
	SHELL_CMD(status, NULL, "Print current aircraft state", cmd_aircraft_status),
	SHELL_CMD(reset,  NULL, "Reset aircraft to St Lucia", cmd_aircraft_reset),
	SHELL_CMD(circle, NULL, "Fly circles: <speed_kt> [radius_m]  or  0 to stop", cmd_aircraft_circle),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(aircraft, &aircraft_cmds, "Aircraft control commands", NULL);

/* Parse a complete line received on ttyACM1.
 *
 * "ac <lat> <lon> [alt] [speed_kt] [heading_deg]"
 *     lat/lon in decimal degrees, e.g.  ac -27.4975 153.0137 500 60 90
 *     Sets position immediately; dead reckoning continues from there.
 *
 * Single-char w/a/s/d/q/e/r still work for manual nudges.
 */
static void parse_line_cmd(const char *line)
{
	char buf[CMD_BUF_SIZE];
	strncpy(buf, line, sizeof(buf) - 1);
	buf[sizeof(buf) - 1] = '\0';
	int len = (int)strlen(buf);
	while (len > 0 &&
	       (buf[len - 1] == '\r' || buf[len - 1] == '\n' || buf[len - 1] == ' ')) {
		buf[--len] = '\0';
	}
	if (len == 0) {
		return;
	}

	if (len == 1) {
		process_gui_cmd((uint8_t)buf[0]);
		return;
	}

	/* "circle <speed_kt> <radius_m>"  — fly constant-rate circles
	 * "circle 0"                      — stop circling (keep current heading) */
	if (strncmp(buf, "circle ", 7) == 0) {
		char *p = buf + 7;
		char *end;
		long spd = strtol(p, &end, 10);
		if (end == p) {
			return;
		}
		if (spd == 0) {
			g_turn_rate = 0.0f;
			return;
		}
		p = end;
		long radius_m = strtol(p, &end, 10);
		if (end == p) {
			radius_m = 500; // default 500 m radius
		}
		g_speed = (uint16_t)spd;
		// ω (deg/tick) = (speed_ms / radius_m) * (180/π) * 0.2 s/tick
		float speed_ms = spd * 0.514444f;
		g_turn_rate = (speed_ms / (float)radius_m) * (180.0f / 3.14159f) * 0.2f;
		g_hdg_acc   = (float)g_heading; // start from current heading
		return;
	}

	if (strncmp(buf, "ac ", 3) == 0) {
		char *p = buf + 3;
		char *end;

		float lat_f = strtof(p, &end);
		if (end == p) {
			return;
		}
		p = end;

		float lon_f = strtof(p, &end);
		if (end == p) {
			return;
		}
		p = end;

		// if magnitude > 1000 it's already e7 (e.g. -274975000), else degrees
		g_turn_rate = 0.0f; // ac always stops circling
		g_lat = (lat_f < -1000.0f || lat_f > 1000.0f)
		        ? (int32_t)lat_f : (int32_t)(lat_f * 1e7f);
		g_lon = (lon_f < -1000.0f || lon_f > 1000.0f)
		        ? (int32_t)lon_f : (int32_t)(lon_f * 1e7f);

		long alt = strtol(p, &end, 10);
		if (end != p) {
			g_alt = (uint16_t)alt;
			p = end;
			long spd = strtol(p, &end, 10);
			if (end != p) {
				g_speed = (uint16_t)spd;
				p = end;
				long hdg = strtol(p, &end, 10);
				if (end != p) {
					g_heading = (uint16_t)(((hdg % 360) + 360) % 360);
				}
			}
		}

		clamp_position();
	}
}

/* GUI command receive thread
 * Buffers lines from ttyACM1; parses on newline.
 * echo "ac -27.4975 153.0137 500 60 90" > /dev/ttyACM1
 * Single-char w/a/s/d/q/e/r still accepted for manual nudges. */
static void cmd_thread_fn(void *a, void *b, void *c)
{
	static char line_buf[CMD_BUF_SIZE];
	static int  line_len;
	uint8_t ch;

	while (1) {
		if (device_is_ready(data_uart) &&
		    uart_poll_in(data_uart, &ch) == 0) {
			if (ch == '\n' || ch == '\r') {
				if (line_len > 0) {
					line_buf[line_len] = '\0';
					parse_line_cmd(line_buf);
					line_len = 0;
				}
			} else if (line_len < CMD_BUF_SIZE - 1) {
				line_buf[line_len++] = (char)ch;
			}
		}
		k_msleep(10);
	}
}

K_THREAD_DEFINE(cmd_tid, CMD_STACK_SIZE, cmd_thread_fn,
		NULL, NULL, NULL, CMD_PRIORITY, 0, 0);

/* Send thread
 * Builds binary ADS-B frame and sends to base via BLE NUS at 5Hz.
 * Shell commands still work on ttyACM0 via PuTTY simultaneously. */
static void send_thread_fn(void *a, void *b, void *c)
{
	while (1) {
		k_msleep(200); // 5Hz = 200ms

		// (AI) skip feed if watchdog init failed (-1), Feed watchdog every loop iteration
		if (g_wdt_channel >= 0) {
			task_wdt_feed(g_wdt_channel); //board resets if this doesn't run
		}

		// Stage 11 — CRASH freeze. While the 10 s window is open, skip
		// heading update + dead-reckoning entirely so the aircraft stays
		// pinned at the crash spot. We still fall through to the BLE
		// notify below so the base sees the aircraft is still alive
		// (just not moving). After the window, crash_reset_fn snaps us
		// back to St Lucia.
		bool frozen = (g_frozen_until_ms != 0) &&
			      (k_uptime_get() < g_frozen_until_ms);

		// Circle mode — rotate heading each tick before dead reckoning
		if (!frozen && g_turn_rate != 0.0f) {
			g_hdg_acc += g_turn_rate;
			if (g_hdg_acc >= 360.0f) g_hdg_acc -= 360.0f;
			if (g_hdg_acc <   0.0f)  g_hdg_acc += 360.0f;
			g_heading = (uint16_t)g_hdg_acc;
		}

		// Dead reckoning — update position based on heading and speed every 200ms
		// speed in knots, 1 knot = 1.852 km/h = 0.000514 m/s
		// at 200ms intervals: distance = speed * 0.514 * 0.2 = speed * 0.1028 metres per tick
		// 1 degree lat ~ 111,000m so delta_lat = distance * cos(hdg) / 111000
		// stored as int32 * 1e7 so multiply by 1e7
		if (!frozen) {
			float hdg_rad = g_heading * 3.14159f / 180.0f;
			float dist_m  = g_speed * 0.1028f; // metres per 200ms tick at given knots
			float dlat    = (dist_m * cosf(hdg_rad)) / 111000.0f; // degrees
			float dlon    = (dist_m * sinf(hdg_rad)) / (111000.0f * cosf((float)g_lat / 1e7f * 3.14159f / 180.0f));
			g_lat += (int32_t)(dlat * 1e7f);
			g_lon += (int32_t)(dlon * 1e7f);
			clamp_position(); // keep within Brisbane bounds
		}

		// Flash red LED while base warning is active; keep red off when connected and clear
		if (s_conn) {
			if (g_warning_active) {
				gpio_pin_toggle_dt(&red);
				gpio_pin_toggle_dt(&buzzer);
			} else {
				gpio_pin_set_dt(&red, 0);
				gpio_pin_set_dt(&buzzer, 0); 
			}
		}

		// always print JSON so GUI can read ttyACM0 directly (no base needed)
		send_position_json();

		if (s_conn && s_notif_enabled) {
			struct ble_aircraft_frame frame = {0};
			frame.source  = BLE_SRC_BLE_SIM;
			frame.flags   = BLE_FLAG_ALT_VALID | BLE_FLAG_VEL_VALID | BLE_FLAG_HDG_VALID;
			frame.icao24  = ((uint32_t)g_icao[0] << 16) |
			                ((uint32_t)g_icao[1] << 8)  |
			                 (uint32_t)g_icao[2];
			frame.lat     = (double)g_lat / 1e7;
			frame.lon     = (double)g_lon / 1e7;
			frame.alt_ft  = (int16_t)((float)g_alt * 3.28084f); // metres → feet
			frame.vel_kt  = (int16_t)g_speed;
			frame.hdg_deg = (int16_t)g_heading;
			frame.ts_ms   = (uint32_t)k_uptime_get();

			bt_gatt_notify(s_conn, &skywatch_svc.attrs[2], &frame, sizeof(frame));
		}
	}
}

K_THREAD_DEFINE(send_tid, SEND_STACK_SIZE, send_thread_fn, NULL, NULL, NULL, SEND_PRIORITY, 0, 0);

int main(void)
{
	//GPIO LEDS
	if (!gpio_is_ready_dt(&red) || !gpio_is_ready_dt(&green)) {
		return -1;
	}
	gpio_pin_configure_dt(&red, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&green, GPIO_OUTPUT_INACTIVE);
	gpio_pin_set_dt(&red, 1);
	gpio_pin_set_dt(&green, 0);

	gpio_pin_configure_dt(&buzzer, GPIO_OUTPUT_INACTIVE);

	//initialise and start the BLE controller
	int ret = bt_enable(NULL);
	if (ret) {
		return ret;
	}
	//begin advertising so base can find and connect
	ret = bt_le_adv_start(BT_LE_ADV_CONN_FAST_2, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (ret) {
		return ret;
	}

	// 15s watchdog, send thread feeds every 200ms
	// AI - register a 15 s task watchdog; send_thread_fn feeds it every 200 ms via task_wdt_feed.
	// If the send thread hangs for 15 s the board resets.
	g_wdt_channel = task_wdt_add(15000, NULL, NULL); /* 15000 ms timeout, no callback, no user data */
	if (g_wdt_channel < 0) {
		g_wdt_channel = -1;
	}
	return 0;
}
