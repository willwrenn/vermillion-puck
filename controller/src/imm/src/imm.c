/*
 * SkyWatch controller — IMM filter implementation (Stage 9.2).
 *
 * Algorithm + sign conventions identical to scripts/stage9/imm_proto.py.
 * Hand-rolled doubles, row-major matrices. All 5×5 helpers inline below
 * (the existing kalman.c 4×4 helpers are not exposed by kalman.h, so the
 * CT filter has its own minimal set sized for 5×5 / 5×2). The CV mode
 * reuses kalman.c verbatim via the contained `struct kalman_state cv`.
 *
 * The textbook IMM step:
 *   1. Mixing:  μ_ij = π_ij·μ_i / c_j,  c_j = Σ_i π_ij·μ_i.
 *      Each filter is reseeded with a mixture of all filters' states
 *      (over the common subspace — for us, the 4 CV dimensions).
 *   2. Predict: each filter does its own predict(dt) independently.
 *   3. Update:  each filter applies the measurement; record (y_j, S_j).
 *   4. Mode-prob update:  μ_j = Λ_j·c_j / Σ_k Λ_k·c_k where
 *      Λ_j = N(y_j; 0, S_j) is the measurement likelihood under mode j.
 *   5. Fused output: x̂ = Σ_j μ_j x̂_j  (in the 4-D common subspace).
 *
 * For CT we keep the textbook formulation but evaluate everything in a
 * local Cartesian (metres) frame so the rotation is physically sensible,
 * then project back to deg/s for storage. The metric-frame projection of
 * P is done via a similarity transform (state-reorder permutation +
 * per-axis metric scaling); see _ct_P_to_metric / _ct_P_from_metric.
 *
 * Sign convention: ω > 0 ⇒ heading increases (right/clockwise turn). The
 * velocity rotation matrix is therefore [[c, s], [-s, c]] rather than the
 * textbook CCW [[c, -s], [s, c]]; the position-increment integrals carry
 * matching signs. See the Python prototype's `_ct_xy_advance` for the
 * derivation.
 */

#include "imm.h"

#include <math.h>
#include <string.h>

#include <zephyr/kernel.h>

#define IMM_PI            3.14159265358979323846
#define DEG_PER_M_LAT     (1.0 / 111000.0)

double g_imm_q_omega = 0.05;          /* rad/s */
bool   g_imm_enabled = false;         /* default OFF — opt-in via shell */

/* Markov transition matrix π (row = current mode, col = next mode).
 *   row 0 = CV, row 1 = CT.
 *   0.95 stay / 0.05 switch matches imm_proto.py DEFAULT_PI. */
static const double IMM_PI_MAT[2][2] = {
	{0.95, 0.05},
	{0.05, 0.95},
};

static inline double deg_per_m_lon_at(double lat_deg)
{
	return 1.0 / (111000.0 * cos(lat_deg * (IMM_PI / 180.0)));
}

/* ---- 5×5 / 5×2 matrix helpers (row-major) -------------------------- */

static void mat5_mul(const double a[25], const double b[25], double out[25])
{
	for (int i = 0; i < 5; i++)
		for (int j = 0; j < 5; j++) {
			double s = 0.0;
			for (int k = 0; k < 5; k++)
				s += a[i * 5 + k] * b[k * 5 + j];
			out[i * 5 + j] = s;
		}
}

static void mat5_transpose(const double m[25], double out[25])
{
	for (int i = 0; i < 5; i++)
		for (int j = 0; j < 5; j++)
			out[j * 5 + i] = m[i * 5 + j];
}

static void mat5_add(const double a[25], const double b[25], double out[25])
{
	for (int i = 0; i < 25; i++) out[i] = a[i] + b[i];
}

/* ---- CT advance + Jacobian (sign: ω > 0 = right turn) -------------- */

/* Returns dx, dy (metres), vxp, vyp (m/s after rotation). Robust at
 * omega ≈ 0 via CV limit. */
static void ct_xy_advance(double vx, double vy, double omega, double dt,
			  double *dx, double *dy, double *vxp, double *vyp)
{
	if (fabs(omega) < 1e-6) {
		*dx  = vx * dt;
		*dy  = vy * dt;
		*vxp = vx;
		*vyp = vy;
		return;
	}
	const double s = sin(omega * dt);
	const double c = cos(omega * dt);
	*dx  = (s / omega) * vx + ((1.0 - c) / omega) * vy;
	*dy  = -((1.0 - c) / omega) * vx + (s / omega) * vy;
	*vxp =  c * vx + s * vy;
	*vyp = -s * vx + c * vy;
}

/* Builds the 5×5 transition Jacobian F about the current state (in metric
 * frame, state order (px, py, vx, vy, ω)). Used inside ct_predict to
 * propagate the covariance: Pm = F·Pm·Fᵀ + Q. */
static void ct_F_jacobian(double vx, double vy, double omega, double dt,
			  double F[25])
{
	memset(F, 0, 25 * sizeof(double));
	F[0 * 5 + 0] = 1.0;
	F[1 * 5 + 1] = 1.0;
	F[2 * 5 + 2] = 1.0;
	F[3 * 5 + 3] = 1.0;
	F[4 * 5 + 4] = 1.0;
	if (fabs(omega) < 1e-6) {
		F[0 * 5 + 2] = dt;
		F[1 * 5 + 3] = dt;
		F[0 * 5 + 4] =  0.5 * vy * dt * dt;
		F[1 * 5 + 4] = -0.5 * vx * dt * dt;
		F[2 * 5 + 4] =  vy * dt;
		F[3 * 5 + 4] = -vx * dt;
		return;
	}
	const double s  = sin(omega * dt);
	const double c  = cos(omega * dt);
	const double sw = s / omega;
	const double cw = (1.0 - c) / omega;
	/* Position rows */
	F[0 * 5 + 2] =  sw;
	F[0 * 5 + 3] =  cw;
	F[1 * 5 + 2] = -cw;
	F[1 * 5 + 3] =  sw;
	/* Velocity rows */
	F[2 * 5 + 2] =  c;
	F[2 * 5 + 3] =  s;
	F[3 * 5 + 2] = -s;
	F[3 * 5 + 3] =  c;
	/* ω partials */
	const double d_sw = (dt * c - sw) / omega;
	const double d_cw = (dt * s - cw) / omega;
	F[0 * 5 + 4] =  d_sw * vx + d_cw * vy;
	F[1 * 5 + 4] = -d_cw * vx + d_sw * vy;
	F[2 * 5 + 4] = -s * dt * vx + c * dt * vy;
	F[3 * 5 + 4] = -c * dt * vx - s * dt * vy;
}

/* Per-tick process noise Q in the metric frame, white-noise-acceleration
 * blocks for position/velocity + an independent random walk on ω. */
static void ct_Q_metric(double dt, double Q[25])
{
	const double q   = g_kalman_q_accel_mps2 * g_kalman_q_accel_mps2;
	const double qw  = g_imm_q_omega        * g_imm_q_omega;
	const double dt2 = dt * dt;
	const double dt3 = dt2 * dt;
	const double dt4 = dt3 * dt;
	memset(Q, 0, 25 * sizeof(double));
	Q[0 * 5 + 0] = (dt4 / 4.0) * q;
	Q[1 * 5 + 1] = (dt4 / 4.0) * q;
	Q[0 * 5 + 2] = Q[2 * 5 + 0] = (dt3 / 2.0) * q;
	Q[1 * 5 + 3] = Q[3 * 5 + 1] = (dt3 / 2.0) * q;
	Q[2 * 5 + 2] = dt2 * q;
	Q[3 * 5 + 3] = dt2 * q;
	Q[4 * 5 + 4] = qw * dt;
}

/* Reorder + scale a 5×5 covariance from (lat, lon, v_lat, v_lon, ω) in
 * lat/lon-per-axis units → (px, py, vx, vy, ω) in metre-per-axis units.
 * permutation: index 0(lat) ↔ 1(lon), 2(v_lat) ↔ 3(v_lon). */
static const int CT_PERM[5] = {1, 0, 3, 2, 4};

static void ct_P_to_metric(const double P[25], double lat, double Pm[25])
{
	const double s_lat = 1.0 / DEG_PER_M_LAT;
	const double s_lon = 1.0 / deg_per_m_lon_at(lat);
	const double D[5] = {s_lat, s_lon, s_lat, s_lon, 1.0};
	/* tmp = D·P·Dᵀ then reorder rows/cols by CT_PERM */
	double tmp[25];
	for (int i = 0; i < 5; i++)
		for (int j = 0; j < 5; j++)
			tmp[i * 5 + j] = D[i] * P[i * 5 + j] * D[j];
	for (int i = 0; i < 5; i++)
		for (int j = 0; j < 5; j++)
			Pm[i * 5 + j] = tmp[CT_PERM[i] * 5 + CT_PERM[j]];
}

static void ct_P_from_metric(const double Pm[25], double lat, double P[25])
{
	/* CT_PERM is an involution: applying it twice = identity. */
	double tmp[25];
	for (int i = 0; i < 5; i++)
		for (int j = 0; j < 5; j++)
			tmp[i * 5 + j] = Pm[CT_PERM[i] * 5 + CT_PERM[j]];
	const double s_lat = DEG_PER_M_LAT;
	const double s_lon = deg_per_m_lon_at(lat);
	const double D[5] = {s_lat, s_lon, s_lat, s_lon, 1.0};
	for (int i = 0; i < 5; i++)
		for (int j = 0; j < 5; j++)
			P[i * 5 + j] = D[i] * tmp[i * 5 + j] * D[j];
}

/* ---- CT public API -------------------------------------------------- */

void ct_init(struct ct_state *s, double lat0, double lon0)
{
	memset(s, 0, sizeof(*s));
	s->x[0] = lat0;
	s->x[1] = lon0;
	/* v_lat, v_lon, omega all 0 — same convention as kalman_init. */
	const double sigma_pos_deg = 100.0 * DEG_PER_M_LAT;
	const double sigma_vel_deg =  50.0 * DEG_PER_M_LAT;
	const double sigma_omega   = 0.3;
	s->P[0 * 5 + 0] = sigma_pos_deg * sigma_pos_deg;
	s->P[1 * 5 + 1] = sigma_pos_deg * sigma_pos_deg;
	s->P[2 * 5 + 2] = sigma_vel_deg * sigma_vel_deg;
	s->P[3 * 5 + 3] = sigma_vel_deg * sigma_vel_deg;
	s->P[4 * 5 + 4] = sigma_omega   * sigma_omega;
	s->initialised = true;
}

void ct_predict(struct ct_state *s, double dt_s)
{
	if (!s->initialised) return;

	const double lat = s->x[0];
	const double m_per_lat = 1.0 / DEG_PER_M_LAT;
	const double m_per_lon = 1.0 / deg_per_m_lon_at(lat);
	const double vx_mps = s->x[3] * m_per_lon;
	const double vy_mps = s->x[2] * m_per_lat;
	const double omega  = s->x[4];

	double dx_m, dy_m, vxp_mps, vyp_mps;
	ct_xy_advance(vx_mps, vy_mps, omega, dt_s,
		      &dx_m, &dy_m, &vxp_mps, &vyp_mps);

	const double new_lat = lat       + dy_m * DEG_PER_M_LAT;
	const double new_lon = s->x[1] + dx_m * deg_per_m_lon_at(lat);
	s->x[0] = new_lat;
	s->x[1] = new_lon;
	s->x[2] = vyp_mps * DEG_PER_M_LAT;
	s->x[3] = vxp_mps * deg_per_m_lon_at(new_lat);
	/* ω: identity (random-walk via Q). */

	/* Covariance: in the metric frame so the EKF Jacobian is physically
	 * sensible, then project back to lat/lon units. */
	double Pm[25], F[25], Q[25], FP[25], Ft[25], FPFt[25], Pm_new[25];
	ct_P_to_metric(s->P, lat, Pm);
	ct_F_jacobian(vx_mps, vy_mps, omega, dt_s, F);
	ct_Q_metric(dt_s, Q);
	mat5_mul(F, Pm, FP);
	mat5_transpose(F, Ft);
	mat5_mul(FP, Ft, FPFt);
	mat5_add(FPFt, Q, Pm_new);
	ct_P_from_metric(Pm_new, new_lat, s->P);
}

void ct_update(struct ct_state *s, double meas_lat, double meas_lon,
	       struct ct_step_io *out)
{
	if (!s->initialised) return;

	/* Same shape as kalman_update — H selects the first two state rows,
	 * R is diagonal (lat/lon noise scaled to deg). We can hand-roll the
	 * gain math because S is 2×2 and K is 5×2. */
	const double r_lat = g_kalman_r_meas_m * DEG_PER_M_LAT;
	const double r_lon = g_kalman_r_meas_m * deg_per_m_lon_at(s->x[0]);
	const double R00 = r_lat * r_lat;
	const double R11 = r_lon * r_lon;

	const double y0 = meas_lat - s->x[0];
	const double y1 = meas_lon - s->x[1];

	const double S00 = s->P[0 * 5 + 0] + R00;
	const double S01 = s->P[0 * 5 + 1];
	const double S10 = s->P[1 * 5 + 0];
	const double S11 = s->P[1 * 5 + 1] + R11;

	const double det = S00 * S11 - S01 * S10;
	if (det == 0.0 || !isfinite(det)) {
		if (out) {
			out->y[0] = y0; out->y[1] = y1;
			out->S[0] = S00; out->S[1] = S01;
			out->S[2] = S10; out->S[3] = S11;
		}
		return;
	}
	const double inv_det = 1.0 / det;
	const double Si00 =  S11 * inv_det;
	const double Si01 = -S01 * inv_det;
	const double Si10 = -S10 * inv_det;
	const double Si11 =  S00 * inv_det;

	/* K (5×2) = first two columns of P · Sinv. */
	double K[5 * 2];
	for (int i = 0; i < 5; i++) {
		const double p_i0 = s->P[i * 5 + 0];
		const double p_i1 = s->P[i * 5 + 1];
		K[i * 2 + 0] = p_i0 * Si00 + p_i1 * Si10;
		K[i * 2 + 1] = p_i0 * Si01 + p_i1 * Si11;
	}

	/* x = x + K y. */
	for (int i = 0; i < 5; i++)
		s->x[i] += K[i * 2 + 0] * y0 + K[i * 2 + 1] * y1;

	/* P = (I - K H) P. */
	double IKH[25];
	for (int i = 0; i < 25; i++) IKH[i] = (i % 6 == 0) ? 1.0 : 0.0;
	for (int i = 0; i < 5; i++) {
		IKH[i * 5 + 0] -= K[i * 2 + 0];
		IKH[i * 5 + 1] -= K[i * 2 + 1];
	}
	double Pnew[25];
	mat5_mul(IKH, s->P, Pnew);
	memcpy(s->P, Pnew, sizeof(Pnew));

	s->update_count++;

	if (out) {
		out->y[0] = y0; out->y[1] = y1;
		out->S[0] = S00; out->S[1] = S01;
		out->S[2] = S10; out->S[3] = S11;
	}
}

/* ---- IMM top-level -------------------------------------------------- */

/* Multivariate-Gaussian PDF in 2-D at residual y with covariance S, mean
 * zero. Used as the per-mode measurement likelihood Λ_j. */
static double gaussian_pdf_2d(const double y[2], const double S[4])
{
	const double det = S[0] * S[3] - S[1] * S[2];
	if (det <= 0.0 || !isfinite(det)) return 0.0;
	const double inv_det = 1.0 / det;
	const double Si00 =  S[3] * inv_det;
	const double Si01 = -S[1] * inv_det;
	const double Si10 = -S[2] * inv_det;
	const double Si11 =  S[0] * inv_det;
	const double quad = y[0] * (Si00 * y[0] + Si01 * y[1])
			  + y[1] * (Si10 * y[0] + Si11 * y[1]);
	return exp(-0.5 * quad) / (2.0 * IMM_PI * sqrt(det));
}

void imm_init(struct imm_state *s, double lat0, double lon0)
{
	memset(s, 0, sizeof(*s));
	kalman_init(&s->cv, lat0, lon0);
	ct_init(&s->ct, lat0, lon0);
	s->mode_probs[0] = 0.7;     /* match imm_proto.py defaults */
	s->mode_probs[1] = 0.3;
	s->c_norm[0]     = 0.7;     /* initialised so update() works if called
				     * before the first predict() */
	s->c_norm[1]     = 0.3;
	s->initialised   = true;
}

void imm_predict(struct imm_state *s, double dt_s)
{
	if (!s->initialised) return;

	/* IMM step 1: mixing. c_j = Σ_i π_ij μ_i. */
	double c_norm[2] = {0, 0};
	for (int j = 0; j < 2; j++)
		for (int i = 0; i < 2; i++)
			c_norm[j] += IMM_PI_MAT[i][j] * s->mode_probs[i];

	/* μ_ij = π_ij μ_i / c_j. */
	double mu_ij[2][2];
	for (int j = 0; j < 2; j++) {
		const double cj = (c_norm[j] > 1e-12) ? c_norm[j] : 1e-12;
		for (int i = 0; i < 2; i++)
			mu_ij[i][j] = IMM_PI_MAT[i][j] * s->mode_probs[i] / cj;
	}

	/* Mix only over the common 4-D subspace (lat, lon, v_lat, v_lon).
	 * CT's ω row/col is preserved unchanged. */
	double x4_cv[4], x4_ct[4];
	for (int k = 0; k < 4; k++) { x4_cv[k] = s->cv.x[k]; x4_ct[k] = s->ct.x[k]; }

	double P4_cv[16], P4_ct[16];
	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++) {
			P4_cv[i * 4 + j] = s->cv.P[i * 4 + j];
			P4_ct[i * 4 + j] = s->ct.P[i * 5 + j];
		}

	/* Mixed initial states for each filter j. */
	double x_mix[2][4];
	for (int j = 0; j < 2; j++)
		for (int k = 0; k < 4; k++)
			x_mix[j][k] = mu_ij[0][j] * x4_cv[k]
				    + mu_ij[1][j] * x4_ct[k];

	double P_mix[2][16];
	for (int j = 0; j < 2; j++) {
		double dx_cv[4], dx_ct[4];
		for (int k = 0; k < 4; k++) {
			dx_cv[k] = x4_cv[k] - x_mix[j][k];
			dx_ct[k] = x4_ct[k] - x_mix[j][k];
		}
		for (int r = 0; r < 4; r++)
			for (int c = 0; c < 4; c++)
				P_mix[j][r * 4 + c] =
				    mu_ij[0][j] * (P4_cv[r * 4 + c] + dx_cv[r] * dx_cv[c])
				  + mu_ij[1][j] * (P4_ct[r * 4 + c] + dx_ct[r] * dx_ct[c]);
	}

	/* Write mixed initial states back into each filter. CT keeps its ω
	 * row/col (we don't reseed ω from CV; q_omega + the EKF mixing on
	 * the next predict handles the coupling). */
	for (int k = 0; k < 4; k++) {
		s->cv.x[k] = x_mix[0][k];
		s->ct.x[k] = x_mix[1][k];
	}
	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++) {
			s->cv.P[i * 4 + j] = P_mix[0][i * 4 + j];
			s->ct.P[i * 5 + j] = P_mix[1][i * 4 + j];
		}

	/* IMM step 2: each filter does its own predict. */
	kalman_predict(&s->cv, dt_s);
	ct_predict(&s->ct, dt_s);

	/* Stash c_norm for update(). */
	s->c_norm[0] = c_norm[0];
	s->c_norm[1] = c_norm[1];
}

void imm_update(struct imm_state *s, double meas_lat, double meas_lon)
{
	if (!s->initialised) return;

	/* CV update — replicate the CV update math here so we can get y_cv,
	 * S_cv for the likelihood computation (kalman_update doesn't expose
	 * them). The math is identical to kalman.c's update path. */
	double y_cv[2], S_cv_arr[4];
	{
		const double r_lat = g_kalman_r_meas_m * DEG_PER_M_LAT;
		const double r_lon = g_kalman_r_meas_m * deg_per_m_lon_at(s->cv.x[0]);
		const double R00 = r_lat * r_lat;
		const double R11 = r_lon * r_lon;
		y_cv[0] = meas_lat - s->cv.x[0];
		y_cv[1] = meas_lon - s->cv.x[1];
		const double S00 = s->cv.P[0 * 4 + 0] + R00;
		const double S01 = s->cv.P[0 * 4 + 1];
		const double S10 = s->cv.P[1 * 4 + 0];
		const double S11 = s->cv.P[1 * 4 + 1] + R11;
		S_cv_arr[0] = S00; S_cv_arr[1] = S01;
		S_cv_arr[2] = S10; S_cv_arr[3] = S11;
	}
	kalman_update(&s->cv, meas_lat, meas_lon);

	/* CT update — ct_update returns y, S to us. */
	struct ct_step_io ct_io;
	ct_update(&s->ct, meas_lat, meas_lon, &ct_io);

	/* IMM step 4: mode-prob update. μ_j = Λ_j c_j / Σ_k Λ_k c_k. */
	const double Lam_cv = gaussian_pdf_2d(y_cv, S_cv_arr);
	const double Lam_ct = gaussian_pdf_2d(ct_io.y, ct_io.S);
	const double post_cv = Lam_cv * s->c_norm[0];
	const double post_ct = Lam_ct * s->c_norm[1];
	const double sum = post_cv + post_ct;
	if (sum > 0.0 && isfinite(sum)) {
		s->mode_probs[0] = post_cv / sum;
		s->mode_probs[1] = post_ct / sum;
	} else {
		/* Pathological likelihoods — flat prior keeps the filter alive
		 * so a single bad tick doesn't permanently lock either mode. */
		s->mode_probs[0] = 0.5;
		s->mode_probs[1] = 0.5;
	}
}

/* ---- fused accessors ------------------------------------------------ */

double imm_lat(const struct imm_state *s)
{
	return s->mode_probs[0] * s->cv.x[0]
	     + s->mode_probs[1] * s->ct.x[0];
}
double imm_lon(const struct imm_state *s)
{
	return s->mode_probs[0] * s->cv.x[1]
	     + s->mode_probs[1] * s->ct.x[1];
}
double imm_v_lat(const struct imm_state *s)
{
	return s->mode_probs[0] * s->cv.x[2]
	     + s->mode_probs[1] * s->ct.x[2];
}
double imm_v_lon(const struct imm_state *s)
{
	return s->mode_probs[0] * s->cv.x[3]
	     + s->mode_probs[1] * s->ct.x[3];
}

void imm_predict_ahead(const struct imm_state *s, double dt_s,
		       double *out_lat, double *out_lon)
{
	if (!s->initialised) {
		*out_lat = 0.0; *out_lon = 0.0;
		return;
	}
	/* CV linear projection. */
	const double lat_cv = s->cv.x[0] + s->cv.x[2] * dt_s;
	const double lon_cv = s->cv.x[1] + s->cv.x[3] * dt_s;
	/* CT arc projection via the same metric-frame advance as predict. */
	const double lat0 = s->ct.x[0];
	const double m_per_lat = 1.0 / DEG_PER_M_LAT;
	const double m_per_lon = 1.0 / deg_per_m_lon_at(lat0);
	const double vx_mps = s->ct.x[3] * m_per_lon;
	const double vy_mps = s->ct.x[2] * m_per_lat;
	double dx_m, dy_m, _vxp, _vyp;
	ct_xy_advance(vx_mps, vy_mps, s->ct.x[4], dt_s,
		      &dx_m, &dy_m, &_vxp, &_vyp);
	const double lat_ct = lat0       + dy_m * DEG_PER_M_LAT;
	const double lon_ct = s->ct.x[1] + dx_m * deg_per_m_lon_at(lat0);

	*out_lat = s->mode_probs[0] * lat_cv + s->mode_probs[1] * lat_ct;
	*out_lon = s->mode_probs[0] * lon_cv + s->mode_probs[1] * lon_ct;
}
