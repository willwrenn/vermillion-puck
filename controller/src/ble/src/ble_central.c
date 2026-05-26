// SkyWatch controller — BLE GATT central, telemetry receive side.
// Wire contract: common/src/ble_packet.h.
// Sim node device name: SKYWATCH_BLE_DEVICE_NAME ("skywatch-sim").
// Structural pattern adapted from William's earlier `ble_central.c` (the
// scan + connect + MTU + state-machine discovery + watchdog idea), but the
// UUIDs and the frame format are now the new protocol. Warning send
// (controller → mobile) is deferred to .

#include "ble_central.h"
#include "bridge.h"
#include "ble_packet.h"

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ble_central, LOG_LEVEL_INF);

#define SCAN_MAX_RESULTS 8
#define RECONNECT_RETRY_MS 2000
#define DEVICE_NAME_MAX 24

// --- UUIDs from common/ble_packet.h -----------------------------------
static struct bt_uuid_128 sky_svc_uuid = BT_UUID_INIT_128(SKYWATCH_BLE_SERVICE_UUID_ENC);
static struct bt_uuid_128 sky_chr_uuid = BT_UUID_INIT_128(SKYWATCH_BLE_AIRCRAFT_UUID_ENC);
static struct bt_uuid_128 warn_svc_uuid = BT_UUID_INIT_128(WARN_SVC_UUID_ENC);
static struct bt_uuid_128 warn_chr_uuid = BT_UUID_INIT_128(WARN_CHR_UUID_ENC);

// --- Connection / discovery state -------------------------------------
static struct bt_conn *g_conn;
static struct bt_gatt_discover_params g_disc_params;
static struct bt_gatt_subscribe_params g_sub_params;
static struct bt_gatt_exchange_params g_mtu_params;

static uint16_t g_aircraft_handle;
static uint16_t g_warn_handle;
static bool g_scanning;
static bool g_connected;
static bool g_connecting;

// --- Scan results list (shell visibility) -----------------------------
struct scan_result {
	char name[DEVICE_NAME_MAX];
	bt_addr_le_t addr;
	bool valid;
};
static struct scan_result g_scan_results[SCAN_MAX_RESULTS];
static int g_scan_count;

// --- Auto-reconnect ----------------------------------------------------
static bt_addr_le_t g_last_connected_addr;
static bool g_last_connected_valid;
static int64_t g_last_attempt_ms;

// --- Stats ------------------------------------------------------------
static struct ble_central_stats g_stats;
static struct k_spinlock stats_lock;

// --- Discovery state machine (four stages) ---------------------------
// DISC_SVC → SkyWatch aircraft service
// DISC_CHR → aircraft characteristic; subscribe to NOTIFY here
// DISC_WARN_SVC → Will's legacy warning service (ab340001-...)
// DISC_WARN_CHR → warning characteristic; store value handle here
// Each stage transitions in `discover_cb` by re-arming g_disc_params and
// calling bt_gatt_discover again.
enum disc_state { DISC_SVC, DISC_CHR, DISC_WARN_SVC, DISC_WARN_CHR };
static enum disc_state g_disc_state;

// --- NOTIFY callback: validate length, hand bytes to bridge -----------
//Purpose: This function is called when a notification is received
// on the subscribed characteristic. It checks that the data length matches
// the expected BLE_FRAME_SIZE, and if so, it casts the data to a ble_aircraft_frame and submits it to the bridge.
// It also updates stats counters for received notifications, bad lengths, and submitted frames.
static uint8_t notify_cb(struct bt_conn *conn,
			  struct bt_gatt_subscribe_params *params,
			  const void *data, uint16_t length)
{
	if (!data) {
		// Subscription torn down.
		return BT_GATT_ITER_STOP;
	}

	K_SPINLOCK(&stats_lock) {
		g_stats.notify_count++;
	}

	if (length != BLE_FRAME_SIZE) {
		K_SPINLOCK(&stats_lock) {
			g_stats.bad_length++;
		}
		return BT_GATT_ITER_CONTINUE;
	}

	bridge_submit_frame((const struct ble_aircraft_frame *)data);

	K_SPINLOCK(&stats_lock) {
		g_stats.submitted++;
	}
	return BT_GATT_ITER_CONTINUE;
}

// --- Two-stage discovery: SkyWatch service -> aircraft char -----------
//Purpose: This function is called during GATT discovery. 
// It first looks for the SkyWatch service, and when found, 
// it initiates a second discovery for the aircraft characteristic within that service. 
// When the characteristic is found, it subscribes to notifications on that 
// characteristic using the notify_cb defined above.
static uint8_t discover_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			   struct bt_gatt_discover_params *params)
{
	if (!attr) {
		LOG_WRN("discovery stage %d: not found", (int)g_disc_state);
		return BT_GATT_ITER_STOP;
	}

	switch (g_disc_state) {
	case DISC_SVC:
		g_disc_state = DISC_CHR;
		g_disc_params.uuid = &sky_chr_uuid.uuid;
		g_disc_params.start_handle = attr->handle + 1;
		g_disc_params.end_handle = 0xffff;
		g_disc_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;
		bt_gatt_discover(conn, &g_disc_params);
		return BT_GATT_ITER_STOP;

	case DISC_CHR: {
		struct bt_gatt_chrc *chrc = attr->user_data;
		g_aircraft_handle = chrc->value_handle;

		g_sub_params.notify = notify_cb;
		g_sub_params.value_handle = g_aircraft_handle;
		g_sub_params.ccc_handle = g_aircraft_handle + 1;
		g_sub_params.value = BT_GATT_CCC_NOTIFY;
		int err = bt_gatt_subscribe(conn, &g_sub_params);
		if (err && err != -EALREADY) {
			LOG_WRN("bt_gatt_subscribe: %d", err);
		} else {
			LOG_INF("Subscribed to SkyWatch aircraft char (handle 0x%04x)",
				g_aircraft_handle);
		}

		// Now go after Will's warning service so we can write back to him.
		g_disc_state = DISC_WARN_SVC;
		g_disc_params.uuid = &warn_svc_uuid.uuid;
		g_disc_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
		g_disc_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
		g_disc_params.type = BT_GATT_DISCOVER_PRIMARY;
		int derr = bt_gatt_discover(conn, &g_disc_params);
		if (derr) {
			LOG_WRN("warn-svc discover: %d", derr);
		}
		return BT_GATT_ITER_STOP;
	}

	case DISC_WARN_SVC:
		g_disc_state = DISC_WARN_CHR;
		g_disc_params.uuid = &warn_chr_uuid.uuid;
		g_disc_params.start_handle = attr->handle + 1;
		g_disc_params.end_handle = 0xffff;
		g_disc_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;
		bt_gatt_discover(conn, &g_disc_params);
		return BT_GATT_ITER_STOP;

	case DISC_WARN_CHR: {
		struct bt_gatt_chrc *chrc = attr->user_data;
		g_warn_handle = chrc->value_handle;
		LOG_INF("Warning characteristic ready (handle 0x%04x) — controller can now send warnings",
			g_warn_handle);
		return BT_GATT_ITER_STOP;
	}
	}
	return BT_GATT_ITER_CONTINUE;
}

//Purpose: This function initiates the GATT discovery process by first looking 
// for the SkyWatch service.
static void start_discovery(struct bt_conn *conn)
{
	g_disc_state = DISC_SVC;
	g_disc_params.uuid = &sky_svc_uuid.uuid;
	g_disc_params.func = discover_cb;
	g_disc_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
	g_disc_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
	g_disc_params.type = BT_GATT_DISCOVER_PRIMARY;
	int err = bt_gatt_discover(conn, &g_disc_params);
	if (err) {
		LOG_WRN("bt_gatt_discover: %d", err);
	}
}

// --- MTU exchange -----------------------------------------------------
//Purpose: This function is called when the MTU exchange process 
// completes.
static void mtu_exchange_cb(struct bt_conn *conn, uint8_t err,
			    struct bt_gatt_exchange_params *params)
{
	if (err) {
		LOG_WRN("MTU exchange err 0x%02x — continuing anyway", err);
	}
	start_discovery(conn);
}

// --- Connect helper ---------------------------------------------------
//Purpose: This function attempts to connect to a device with the given
// address.
static void do_connect(const bt_addr_le_t *addr)
{
	if (g_connecting || g_connected) {
		return;
	}
	if (g_scanning) {
		bt_le_scan_stop();
		g_scanning = false;
	}
	g_connecting = true;
	g_last_attempt_ms = k_uptime_get();

	struct bt_conn *conn = NULL;
	int err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN,
				    BT_LE_CONN_PARAM_DEFAULT, &conn);
	if (err) {
		LOG_WRN("bt_conn_le_create: %d", err);
		g_connecting = false;
		ble_central_scan_start();
	} else if (conn) {
		bt_conn_unref(conn);
	}
}

// --- Name parsing helper for scan -------------------------------------

struct name_ctx { char name[DEVICE_NAME_MAX]; bool found; };

static bool parse_name_cb(struct bt_data *data, void *user)
{
	struct name_ctx *ctx = user;
	if (data->type != BT_DATA_NAME_COMPLETE && data->type != BT_DATA_NAME_SHORTENED) {
		return true;
	}
	size_t cp = MIN(data->data_len, sizeof(ctx->name) - 1);
	memcpy(ctx->name, data->data, cp);
	ctx->name[cp] = '\0';
	ctx->found = true;
	return false;
}

// --- Scan callback: filter on exact device name -----------------------
//Purpose: This function is called for each advertising packet 
// received during scanning.
static void device_found(const bt_addr_le_t *addr, int8_t rssi,
			 uint8_t type, struct net_buf_simple *ad)
{
	if (g_connected || g_connecting) {
		return;
	}
	if (type != BT_GAP_ADV_TYPE_ADV_IND && type != BT_GAP_ADV_TYPE_ADV_DIRECT_IND) {
		return;
	}

	struct name_ctx ctx = {0};
	bt_data_parse(ad, parse_name_cb, &ctx);

	if (!ctx.found || strcmp(ctx.name, SKYWATCH_BLE_DEVICE_NAME) != 0) {
		return;
	}

	bool already = false;
	for (int i = 0; i < g_scan_count; i++) {
		if (g_scan_results[i].valid &&
		    bt_addr_le_cmp(&g_scan_results[i].addr, addr) == 0) {
			already = true;
			break;
		}
	}
	if (!already && g_scan_count < SCAN_MAX_RESULTS) {
		strncpy(g_scan_results[g_scan_count].name, ctx.name, DEVICE_NAME_MAX - 1);
		g_scan_results[g_scan_count].name[DEVICE_NAME_MAX - 1] = '\0';
		bt_addr_le_copy(&g_scan_results[g_scan_count].addr, addr);
		g_scan_results[g_scan_count].valid = true;
		g_scan_count++;
	}

	bt_addr_le_copy(&g_last_connected_addr, addr);
	g_last_connected_valid = true;
	do_connect(addr);
}

// --- Connection lifecycle ---------------------------------------------
//Purpose: These functions handle connection and disconnection events.
static void connected_cb(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		LOG_WRN("connect err 0x%02x", err);
		g_connecting = false;
		ble_central_scan_start();
		return;
	}
	g_conn = bt_conn_ref(conn);
	g_connected = true;
	g_connecting = false;
	g_aircraft_handle = 0;
	g_warn_handle = 0;
	K_SPINLOCK(&stats_lock) {
		g_stats.connect_count++;
		g_stats.connected = true;
	}
	LOG_INF("Connected to %s", SKYWATCH_BLE_DEVICE_NAME);

	g_mtu_params.func = mtu_exchange_cb;
	int ret = bt_gatt_exchange_mtu(conn, &g_mtu_params);
	if (ret) {
		LOG_WRN("MTU exchange request: %d — starting discovery anyway", ret);
		start_discovery(conn);
	}
}
//Purpose: This function is called when the connection is disconnected. 
// It cleans up the connection state, updates stats, and restarts scanning.
static void disconnected_cb(struct bt_conn *conn, uint8_t reason)
{
	if (g_conn) {
		bt_conn_unref(g_conn);
		g_conn = NULL;
	}
	g_connected = false;
	g_aircraft_handle = 0;
	g_warn_handle = 0;
	K_SPINLOCK(&stats_lock) {
		g_stats.disconnect_count++;
		g_stats.connected = false;
	}
	LOG_INF("Disconnected (reason 0x%02x)", reason);

	// Always re-scan; sim node will re-advertise.
	ble_central_scan_start();
}

//Purpose: This structure registers the connection callbacks defined
// above with the Bluetooth stack.
BT_CONN_CB_DEFINE(conn_cbs) = {
	.connected = connected_cb,
	.disconnected = disconnected_cb,
};

// --- Watchdog thread: nudge scan back up if everything stalls ---------

#define WD_STACK 1024
#define WD_PRIO 7
static K_THREAD_STACK_DEFINE(wd_stack, WD_STACK);
static struct k_thread wd_thread;

//Purpose: This function is run by the watchdog thread. 
// It periodically checks if the central is not connected, 
// not connecting, and not scanning, and if enough time has passed 
// since the last connection attempt, it restarts scanning.
static void wd_thread_fn(void *a, void *b, void *c)
{
	while (1) {
		k_sleep(K_SECONDS(2));
		int64_t now = k_uptime_get();
		if (!g_connected && !g_connecting && !g_scanning &&
		    (now - g_last_attempt_ms) > RECONNECT_RETRY_MS) {
			ble_central_scan_start();
		}
	}
}

// --- Public API -------------------------------------------------------
//Purpose: This function initializes the BLE central. 
// It enables Bluetooth,
int ble_central_init(void)
{
	memset(&g_stats, 0, sizeof(g_stats));
	memset(g_scan_results, 0, sizeof(g_scan_results));
	g_scan_count = 0;
	g_last_connected_valid = false;
	g_last_attempt_ms = 0;

	int err = bt_enable(NULL);
	if (err) {
		LOG_ERR("bt_enable: %d", err);
		return err;
	}
	LOG_INF("BT enabled; scanning for '%s'", SKYWATCH_BLE_DEVICE_NAME);

	ble_central_scan_start();

	k_thread_create(&wd_thread, wd_stack, K_THREAD_STACK_SIZEOF(wd_stack),
			wd_thread_fn, NULL, NULL, NULL,
			WD_PRIO, 0, K_NO_WAIT);
	k_thread_name_set(&wd_thread, "ble_wd");
	return 0;
}
//Purpose: This function starts scanning for BLE devices.
void ble_central_scan_start(void)
{
	if (g_scanning || g_connected) {
		return;
	}
	struct bt_le_scan_param p = {
		.type = BT_LE_SCAN_TYPE_ACTIVE,
		.options = BT_LE_SCAN_OPT_NONE,
		.interval = BT_GAP_SCAN_FAST_INTERVAL,
		.window = BT_GAP_SCAN_FAST_WINDOW,
	};
	int err = bt_le_scan_start(&p, device_found);
	if (err && err != -EALREADY) {
		LOG_WRN("bt_le_scan_start: %d", err);
		return;
	}
	g_scanning = true;
}


//Purpose: This function stops scanning for BLE devices.
void ble_central_scan_stop(void)
{
	if (!g_scanning) return;
	bt_le_scan_stop();
	g_scanning = false;
}
//Purpose: This function returns whether the central is currently
// connected to a peripheral.
bool ble_central_is_connected(void)
{
	return g_connected;
}
//Purpose:	 This function disconnects from the currently connected
// peripheral, if any.
void ble_central_disconnect(void)
{
	if (!g_connected || !g_conn) return;
	bt_conn_disconnect(g_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
}


//Purpose: This function returns the number of scan results 
// currently stored in the list.
void ble_central_scan_list(const struct shell *sh)
{
	if (g_scan_count == 0) {
		shell_print(sh, "no peers yet — wait or 'skywatch ble scan start'");
		return;
	}
	shell_print(sh, "discovered (%d):", g_scan_count);
	for (int i = 0; i < g_scan_count; i++) {
		char a[BT_ADDR_LE_STR_LEN];
		bt_addr_le_to_str(&g_scan_results[i].addr, a, sizeof(a));
		shell_print(sh, " [%d] %-16s %s", i + 1, g_scan_results[i].name, a);
	}
	shell_print(sh, "state: %s",
		    g_connected ? "CONNECTED" : (g_scanning ? "scanning" : "idle"));
}

//Purpose: Get the current stats counters for the BLE central. 
// This copies the g_stats struct to the provided output pointer 
// while holding the stats_lock spinlock.
void ble_central_get_stats(struct ble_central_stats *out)
{
	K_SPINLOCK(&stats_lock) {
		*out = g_stats;
	}
}

// push a fully-formed warning_frame (msg_type+icao+...+crc)
// to Will's warning characteristic. WRITE-WITHOUT-RESPONSE: low-latency,
// no ack roundtrip. Will's mobile validates the CRC and rejects on
// mismatch (so we must compute it before calling — see ble_warning_crc).
// Returns 0 on accepted submit, -ENOTCONN if no peer, -EAGAIN if the
// warning chr hasn't been discovered yet, or a Zephyr negative on
// write-layer failure.
int ble_central_send_warning(const struct ble_warning_frame *f)
{
	if (!f) {
		return -EINVAL;
	}
	if (!g_connected || !g_conn) {
		return -ENOTCONN;
	}
	if (g_warn_handle == 0) {
		return -EAGAIN;
	}
	int rc = bt_gatt_write_without_response(g_conn, g_warn_handle,
						f, BLE_WARNING_FRAME_SIZE,
						false);
	if (rc) {
		LOG_WRN("warning write failed: %d", rc);
	}
	return rc;
}
