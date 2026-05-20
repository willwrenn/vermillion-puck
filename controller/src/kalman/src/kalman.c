/*
 * SkyWatch controller — Kalman filter implementation (Stage 7.2).
 *
 * Implements the algorithm specified by scripts/stage7/kalman_proto.py.
 * Hand-rolled doubles, row-major matrices. All inline.
 *
 * Sizing: 4-vector x + 16-element P + bookkeeping = 21 doubles + uint32 +
 * bool. ~176 B per aircraft. With AIRCRAFT_DB_MAX_ENTRIES=32, ~5.6 KB total —
 * well under our RAM budget.
 *
 * Tuning constants `g_kalman_q_accel_mps2` and `g_kalman_r_meas_m` are
 * file-scope mutable so `skywatch kalman q|r` can change them live without
 * recompiling. Default values come from the prototype's 10/10 test pass.
 */

#include "kalman.h"

#include <math.h>
#include <string.h>

#include <zephyr/kernel.h>

/* Geo conversion constants matching kalman_proto.py exactly. */
#define DEG_PER_M_LAT   (1.0 / 111000.0)
#define KF_PI           3.14159265358979323846
static inline double deg_per_m_lon_at(double lat_deg)
{
	return 1.0 / (111000.0 * cos(lat_deg * (KF_PI / 180.0)));
}

/* Defaults — proven on the Python prototype. */
double g_kalman_q_accel_mps2 = 0.5;
double g_kalman_r_meas_m     = 10.0;

/* ---- inline matrix helpers (row-major, fixed 4x4 / 4x2) ------------- */

static inline void mat4_mul(const double a[16], const double b[16],
			    double out[16])
{
	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++) {
			double s = 0.0;
			for (int k = 0; k < 4; k++)
				s += a[i * 4 + k] * b[k * 4 + j];
			out[i * 4 + j] = s;
		}
}

static inline void mat4_transpose(const double m[16], double out[16])
{
	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++)
			out[j * 4 + i] = m[i * 4 + j];
}

static inline void mat4_add(const double a[16], const double b[16],
			    double out[16])
{
	for (int i = 0; i < 16; i++) out[i] = a[i] + b[i];
}

static inline void mat_vec4(const double m[16], const double v[4], double out[4])
{
	for (int i = 0; i < 4; i++) {
		double s = 0.0;
		for (int j = 0; j < 4; j++) s += m[i * 4 + j] * v[j];
		out[i] = s;
	}
}

/* ---- KF ------------------------------------------------------------- */

void kalman_init(struct kalman_state *s, double lat0, double lon0)
{
	memset(s, 0, sizeof(*s));
	s->x[0] = lat0;
	s->x[1] = lon0;
	/* v_lat, v_lon left as 0.0 — first prediction will be slow but the
	 * filter converges within a few updates. */

	/* P0: high initial pos uncertainty (~100 m one-sigma), moderate vel
	 * uncertainty (~50 m/s). Convert to deg² so it matches the state units. */
	const double sigma_pos_deg = 100.0 * DEG_PER_M_LAT;
	const double sigma_vel_deg =  50.0 * DEG_PER_M_LAT;
	s->P[0 * 4 + 0] = sigma_pos_deg * sigma_pos_deg;
	s->P[1 * 4 + 1] = sigma_pos_deg * sigma_pos_deg;
	s->P[2 * 4 + 2] = sigma_vel_deg * sigma_vel_deg;
	s->P[3 * 4 + 3] = sigma_vel_deg * sigma_vel_deg;

	s->update_count = 0;
	s->initialised = true;
}

void kalman_predict(struct kalman_state *s, double dt_s)
{
	if (!s->initialised) return;

	/* F = I + dt on F[0,2] and F[1,3] (constant-velocity model). */
	double F[16] = {
		1, 0, dt_s, 0,
		0, 1, 0,    dt_s,
		0, 0, 1,    0,
		0, 0, 0,    1,
	};

	/* x = F x */
	double x_new[4];
	mat_vec4(F, s->x, x_new);
	memcpy(s->x, x_new, sizeof(x_new));

	/* P = F P Fᵀ + Q */
	double FP[16], Ft[16], FPFt[16];
	mat4_mul(F, s->P, FP);
	mat4_transpose(F, Ft);
	mat4_mul(FP, Ft, FPFt);

	/* Q = white-noise-acceleration, per-axis scaled to deg. */
	const double q_lat = g_kalman_q_accel_mps2 * DEG_PER_M_LAT;
	const double q_lon = g_kalman_q_accel_mps2 * deg_per_m_lon_at(s->x[0]);
	const double q_lat2 = q_lat * q_lat;
	const double q_lon2 = q_lon * q_lon;
	const double dt2 = dt_s * dt_s;
	const double dt3 = dt2 * dt_s;
	const double dt4 = dt3 * dt_s;
	double Q[16] = {0};
	Q[0 * 4 + 0] = (dt4 / 4.0) * q_lat2;
	Q[1 * 4 + 1] = (dt4 / 4.0) * q_lon2;
	Q[0 * 4 + 2] = Q[2 * 4 + 0] = (dt3 / 2.0) * q_lat2;
	Q[1 * 4 + 3] = Q[3 * 4 + 1] = (dt3 / 2.0) * q_lon2;
	Q[2 * 4 + 2] = dt2 * q_lat2;
	Q[3 * 4 + 3] = dt2 * q_lon2;

	mat4_add(FPFt, Q, s->P);
}

void kalman_update(struct kalman_state *s, double meas_lat, double meas_lon)
{
	if (!s->initialised) return;

	/* H = [[1,0,0,0],[0,1,0,0]] — measurement = position only. With this
	 * shape we can do all the gain math by hand on a 2x2 inverse and skip
	 * the general MxN linear-algebra routines.
	 *
	 *   y = z - H x   (Mx1)
	 *   S = H P Hᵀ + R   (MxM = 2x2; this is just the top-left 2x2 of P + R)
	 *   K = P Hᵀ S⁻¹   (NxM = 4x2; this is the first two columns of P scaled by S⁻¹)
	 *   x = x + K y
	 *   P = (I - K H) P
	 */

	/* Measurement noise R in deg² per axis. */
	const double r_lat = g_kalman_r_meas_m * DEG_PER_M_LAT;
	const double r_lon = g_kalman_r_meas_m * deg_per_m_lon_at(s->x[0]);
	const double R00 = r_lat * r_lat;
	const double R11 = r_lon * r_lon;

	/* y = z - Hx  (2-vector). */
	const double y0 = meas_lat - s->x[0];
	const double y1 = meas_lon - s->x[1];

	/* S = (top-left 2x2 of P) + R. */
	const double S00 = s->P[0 * 4 + 0] + R00;
	const double S01 = s->P[0 * 4 + 1];
	const double S10 = s->P[1 * 4 + 0];
	const double S11 = s->P[1 * 4 + 1] + R11;

	/* S⁻¹ via 2x2 closed form. det should never be zero in practice; guard. */
	const double det = S00 * S11 - S01 * S10;
	if (det == 0.0 || !isfinite(det)) return;
	const double inv_det = 1.0 / det;
	const double Si00 =  S11 * inv_det;
	const double Si01 = -S01 * inv_det;
	const double Si10 = -S10 * inv_det;
	const double Si11 =  S00 * inv_det;

	/* K (4x2) = first two columns of P * Sinv (2x2).
	 * (P Hᵀ) selects the first two columns of P. */
	double K[4 * 2];
	for (int i = 0; i < 4; i++) {
		const double p_i0 = s->P[i * 4 + 0];
		const double p_i1 = s->P[i * 4 + 1];
		K[i * 2 + 0] = p_i0 * Si00 + p_i1 * Si10;
		K[i * 2 + 1] = p_i0 * Si01 + p_i1 * Si11;
	}

	/* x = x + K y. */
	for (int i = 0; i < 4; i++)
		s->x[i] += K[i * 2 + 0] * y0 + K[i * 2 + 1] * y1;

	/* P = (I - K H) P. (I - K H) is identity minus K's columns into
	 * P's first two rows — easiest to compute the 4x4 update directly. */
	double IKH[16];
	for (int i = 0; i < 16; i++) IKH[i] = (i % 5 == 0) ? 1.0 : 0.0;
	for (int i = 0; i < 4; i++) {
		IKH[i * 4 + 0] -= K[i * 2 + 0];
		IKH[i * 4 + 1] -= K[i * 2 + 1];
	}
	double Pnew[16];
	mat4_mul(IKH, s->P, Pnew);
	memcpy(s->P, Pnew, sizeof(Pnew));

	s->update_count++;
}

void kalman_predict_ahead(const struct kalman_state *s, double dt_s,
			  double *out_lat, double *out_lon)
{
	if (!s->initialised) {
		*out_lat = 0.0; *out_lon = 0.0;
		return;
	}
	*out_lat = s->x[0] + s->x[2] * dt_s;
	*out_lon = s->x[1] + s->x[3] * dt_s;
}
