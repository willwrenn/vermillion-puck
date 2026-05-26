/*
 * SkyWatch controller — guided-missile demo (sandbox).
 *
 * Spawns a single FICT aircraft (`m1s51l`) launched from the UQ St Lucia
 * "base station" coordinate, running a pure-pursuit guidance loop that
 * turns toward the current BLE_SIM target each tick — clamped by a
 * configurable max turn rate so the operator can dodge. The missile
 * shows up on the map/table like any other aircraft (per-ICAO colour,
 * fades on stale, etc.); "impact" is detected by the existing
 * pairwise-collision pipeline — if the missile-vs-target pair trips
 * WARNING, the collision module already fires Will's BLE buzzer once,
 * and the missile then auto-cancels.
 *
 * Lifecycle:
 * missile_launch() — kicks off the work-queue pursuit handler.
 *                       Cancels any in-flight missile first.
 * missile_cancel() — stops the worker. The DB entry stale-evicts
 *                       on its own (~10 s).
 * internal expiry — when (k_uptime - launch) >= ttl_s the worker
 *                       cancels itself (LOG_INF "self-destruct").
 *
 * Wire-format: emitted as a normal `AircraftFrame` (source=FICT) via
 * the aircraft_db -> db_publisher pipeline; no new JSON schema.
 */

#ifndef MISSILE_H
#define MISSILE_H

/* UQ St Lucia launch site. */
#define MISSILE_LAUNCH_LAT -27.4975
#define MISSILE_LAUNCH_LON 153.0137
#define MISSILE_LAUNCH_ALT_FT 0 /* surface — climbs to target alt */

/* Sensible defaults (mirrored in the shell help text). */
#define MISSILE_DEFAULT_TTL_S 60.0
#define MISSILE_DEFAULT_TURN_RATE 15.0 /* deg/sec — generous; lower = easier to dodge */
#define MISSILE_DEFAULT_SPEED_KT 500.0 /* fast enough to chase, slow enough to dodge */

/* Pursuit tick. 2 Hz keeps the map motion smooth without flooding USB. */
#define MISSILE_TICK_MS 500

/* Pass <=0 for any arg to use that arg's default. */
int  missile_launch(double ttl_s, double turn_rate_dps, double speed_kt);

/* Stop the pursuit worker; missile DB entry stale-evicts after ~10 s. */
int  missile_cancel(void);

#endif /* MISSILE_H */
