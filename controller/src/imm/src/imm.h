/*
 * SkyWatch controller — IMM (Interacting Multiple Models) Kalman filter
 *                       (Stage 9.2).
 *
 * Runs a 4-D constant-velocity (CV) filter from kalman.c alongside a 5-D
 * coordinated-turn (CT) EKF, mixed at each step via a Markov-transition
 * matrix. The fused output replaces the CV-only prediction during
 * sustained turns — `pred_lat / pred_lon` follow the arc instead of going
 * tangent at the apex.
 *
 * Direct port of scripts/stage9/imm_proto.py (3/3 tests pass: 7.1 m RMSE
 * on a 90° turn track vs CV-only's 117 m). State layouts, mixing math,
 * CT sign convention (ω > 0 = right turn = clockwise rotation of the
 * velocity vector when viewed from above) all mirror the Python.
 *
 * One struct imm_state per aircraft, stored alongside the existing
 * kalman_state inside aircraft_db_entry. A global g_imm_enabled flag
 * (set by `skywatch imm on|off`) selects which output reaches
 * pred_lat/pred_lon — both filters run regardless so live A/B is
 * comparing apples to apples on identical measurement streams.
 *
 * Memory: 5×1 state + 5×5 P = 30 doubles plus the contained kalman_state
 * (4×1 + 4×4 + ~8 B = 184 B). Total ≈ 184 + 240 + 32 (mode/c_norm/flag)
 * ≈ 460 B per aircraft × 32 entries ≈ 15 KB. Comfortable in our 256 KB
 * RAM budget (currently ~86 KB used).
 */

#ifndef IMM_H
#define IMM_H

#include <stdbool.h>
#include <stdint.h>

#include "kalman.h"

/* Tunable parameters — `skywatch imm` will surface these in 9.3.
 * q_omega is the random-walk std of ω itself (rad/s per second);
 * small means "turn rate stays roughly constant" which makes the
 * mode probabilities (rather than ω drift) absorb turn onsets. */
extern double g_imm_q_omega;       /* default 0.05 rad/s */

/* Runtime switch from `skywatch imm on|off`. When true, db_publisher
 * publishes the IMM fused state instead of the CV-only Kalman output. */
extern bool   g_imm_enabled;

/* ---- Coordinated-Turn filter (5-D EKF) ----------------------------- */

#define CT_N   5   /* state: [lat, lon, v_lat, v_lon, omega] */
#define CT_M   2   /* measurement: [lat, lon] */

struct ct_state {
	double   x[CT_N];                /* [lat, lon, v_lat, v_lon, omega] */
	double   P[CT_N * CT_N];         /* row-major covariance */
	uint32_t update_count;
	bool     initialised;
};

void ct_init(struct ct_state *s, double lat0, double lon0);
void ct_predict(struct ct_state *s, double dt_s);
/* Returns the per-step innovation residual + S so the IMM mode update
 * can compute the per-mode measurement likelihood Λ_j = N(y; 0, S). */
struct ct_step_io {
	double y[CT_M];        /* innovation */
	double S[CT_M * CT_M]; /* innovation covariance */
};
void ct_update(struct ct_state *s, double meas_lat, double meas_lon,
	       struct ct_step_io *out);

/* ---- IMM top-level -------------------------------------------------- */

struct imm_state {
	struct kalman_state cv;       /* internal CV filter (own copy, not shared) */
	struct ct_state     ct;
	double mode_probs[2];         /* [CV, CT] — sum to 1 */
	double c_norm[2];             /* mixing normalization stashed by predict */
	bool   initialised;
};

void imm_init(struct imm_state *s, double lat0, double lon0);
void imm_predict(struct imm_state *s, double dt_s);
void imm_update(struct imm_state *s, double meas_lat, double meas_lon);

/* Fused state accessors (mode-probability-weighted average). */
double imm_lat(const struct imm_state *s);
double imm_lon(const struct imm_state *s);
double imm_v_lat(const struct imm_state *s);
double imm_v_lon(const struct imm_state *s);

/* Mode-weighted projection `dt_s` seconds into the future. During sustained
 * turns (mode_probs[1] ≈ 1) this follows the CT arc rather than going
 * tangent at the apex like the CV-only filter does. */
void imm_predict_ahead(const struct imm_state *s, double dt_s,
		       double *out_lat, double *out_lon);

#endif /* IMM_H */
