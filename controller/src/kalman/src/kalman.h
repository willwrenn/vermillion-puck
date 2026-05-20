/*
 * SkyWatch controller — 4D constant-velocity Kalman filter
 *               (state = [lat, lon, v_lat, v_lon], M=2 measurements).
 *
 * Stage 7.2. Mirrors `scripts/stage7/kalman_proto.py` (already RMSE-tested
 * against simulated tracks: <50 m on straight-line + diagonal + slow + still).
 * Hand-rolled double-precision matrix math — no CMSIS-DSP dependency.
 *
 * One `struct kalman_state` per aircraft, stored inside `aircraft_db_entry`.
 * `aircraft_db_upsert()` runs predict(dt) + update(lat, lon) on every
 * observation and writes the 10 s-ahead extrapolation into the entry's
 * `aircraft_t.pred_lat / pred_lon` (setting AIRCRAFT_VALID_PRED).
 */

#ifndef KALMAN_H
#define KALMAN_H

#include <stdbool.h>
#include <stdint.h>

/* Tunable parameters — `skywatch kalman q|r` updates these at runtime. */
extern double g_kalman_q_accel_mps2;   /* process-noise std, default 0.5 */
extern double g_kalman_r_meas_m;       /* measurement-noise std, default 10.0 */

#define KF_N   4   /* state vector length */
#define KF_M   2   /* measurement length */

struct kalman_state {
	double x[KF_N];            /* [lat, lon, v_lat, v_lon] */
	double P[KF_N * KF_N];     /* row-major covariance */
	uint32_t update_count;     /* number of measurements applied */
	bool initialised;
};

void kalman_init(struct kalman_state *s, double lat0, double lon0);
void kalman_predict(struct kalman_state *s, double dt_s);
void kalman_update(struct kalman_state *s, double meas_lat, double meas_lon);

/* Read-only extrapolation `dt_s` seconds into the future using the current
 * velocity estimate. Does not mutate state. */
void kalman_predict_ahead(const struct kalman_state *s, double dt_s,
			  double *out_lat, double *out_lon);

#endif /* KALMAN_H */
