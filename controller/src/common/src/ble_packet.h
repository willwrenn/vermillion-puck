/*
 * SkyWatch BLE internode binary protocol — C source of truth.
 *
 * Mirrors:
 *   - resources/standards/ble_protocol.md   (specification)
 *   - resources/standards/ble_protocol.py   (Python helpers)
 *
 * Both ends (William's sim node and our controller) read this header.
 * If you change a field, change all three files in the same commit and bump
 * BLE_PROTOCOL_VERSION below.
 *
 * Endianness: little-endian. ARM Cortex-M native, zero conversion either end.
 * Frame size: 38 bytes. Default ATT MTU=23 → MTU exchange to ≥41 mandatory.
 */

#ifndef BLE_PACKET_H
#define BLE_PACKET_H

#include <stdint.h>
#include <zephyr/toolchain.h>

#define BLE_PROTOCOL_VERSION   1
#define BLE_FRAME_SIZE         38

/* --- BLE service / characteristic UUIDs ---
 *
 * Confirmed live: William's `mobile/codein.c` advertises this service and
 * notifies on this characteristic; our `ble/src/ble_central.c` discovers and
 * subscribes here. Do not edit without updating both sides in lockstep.
 *
 * Use BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(...)) on the Zephyr side; see
 * zephyr/samples/bluetooth/peripheral_nus for the idiom.
 */
#define SKYWATCH_BLE_SERVICE_UUID_STR   "4d8a4011-7f24-43c4-9c5b-a17c7af70001"
#define SKYWATCH_BLE_AIRCRAFT_UUID_STR  "4d8a4011-7f24-43c4-9c5b-a17c7af70002"

/* Same UUIDs in BT_UUID_128_ENCODE() argument order — drop these into
 * BT_UUID_DECLARE_128(...) on either side. */
#define SKYWATCH_BLE_SERVICE_UUID_ENC \
	BT_UUID_128_ENCODE(0x4d8a4011, 0x7f24, 0x43c4, 0x9c5b, 0xa17c7af70001)
#define SKYWATCH_BLE_AIRCRAFT_UUID_ENC \
	BT_UUID_128_ENCODE(0x4d8a4011, 0x7f24, 0x43c4, 0x9c5b, 0xa17c7af70002)

/* Sim node device name advertised — controller uses this for scan filter. */
#define SKYWATCH_BLE_DEVICE_NAME    "skywatch-sim"

/* --- Source enum (byte 0 of every frame) --- */
enum ble_aircraft_source {
	BLE_SRC_ADS_B   = 0,   /* Only for completeness — sim normally sends BLE_SIM. */
	BLE_SRC_BLE_SIM = 1,
};

/* --- Flags (byte 1 of every frame) ---
 * If a *_VALID bit is clear, the corresponding field's value is undefined and
 * MUST be ignored by the receiver. */
#define BLE_FLAG_ON_GROUND  (1u << 0)
#define BLE_FLAG_ALT_VALID  (1u << 1)
#define BLE_FLAG_VEL_VALID  (1u << 2)
#define BLE_FLAG_HDG_VALID  (1u << 3)

/* --- The wire frame ---
 * 38 bytes, little-endian. Pads are zero on send, ignored on receive.
 *
 * offset  size  field        type        notes
 *  0      1     source       uint8       BLE_SRC_*
 *  1      1     flags        uint8       BLE_FLAG_*
 *  2      4     icao24       uint32 LE   24-bit ICAO in low 24 bits; top byte = 0
 *  6      4     -- pad --                zero; aligns lat/lon to 8-byte boundary
 * 10      8     lat          float64 LE  WGS-84 degrees [-90, 90]
 * 18      8     lon          float64 LE  WGS-84 degrees [-180, 180]
 * 26      2     alt_ft       int16  LE   feet MSL; ignore unless ALT_VALID
 * 28      2     vel_kt       int16  LE   knots ground speed; ignore unless VEL_VALID
 * 30      2     hdg_deg      int16  LE   degrees true 0..359; ignore unless HDG_VALID
 * 32      1     reserved     uint8       0
 * 33      1     -- pad --                alignment to next uint32
 * 34      4     ts_ms        uint32 LE   ms since sim-node boot
 */
struct __packed ble_aircraft_frame {
	uint8_t  source;          /* enum ble_aircraft_source */
	uint8_t  flags;           /* BLE_FLAG_* */
	uint32_t icao24;          /* low 24 bits; top byte = 0 */
	uint8_t  _pad0[4];        /* zero-fill */
	double   lat;             /* WGS-84 degrees */
	double   lon;             /* WGS-84 degrees */
	int16_t  alt_ft;          /* valid only if flags & BLE_FLAG_ALT_VALID */
	int16_t  vel_kt;          /* valid only if flags & BLE_FLAG_VEL_VALID */
	int16_t  hdg_deg;         /* valid only if flags & BLE_FLAG_HDG_VALID, 0..359 */
	uint8_t  reserved;        /* 0 */
	uint8_t  _pad1[1];        /* alignment */
	uint32_t ts_ms;           /* uint32 ms since sim-node boot */
};

_Static_assert(sizeof(struct ble_aircraft_frame) == BLE_FRAME_SIZE,
	       "BLE frame must be 38 bytes — check field order / padding");

/* --- Receiver-side validation (apply on every notify) ---
 * 1. notify length == BLE_FRAME_SIZE; else drop.
 * 2. source ∈ {BLE_SRC_ADS_B, BLE_SRC_BLE_SIM}; else drop.
 * 3. icao24 <= 0xFFFFFF (i.e. the high byte of the uint32 is zero); else drop.
 * 4. -90 <= lat <= 90; -180 <= lon <= 180; else drop.
 * 5. If a *_VALID flag is clear, treat the corresponding numeric field as
 *    absent (do not store it in aircraft_db).
 */

#endif /* BLE_PACKET_H */
