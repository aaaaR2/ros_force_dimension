/** Copyright 2026 Neuromechatronics Lab, Carnegie Mellon University
 *
 *  This Source Code Form is subject to the terms of the Mozilla Public
 *  License, v. 2.0. If a copy of the MPL was not distributed with this
 *  file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

// Pure ellipse radial spring-damper force helper for the ring guide constraint.
//
// This header is intentionally device-free (no SDK calls) so it can be tested
// independently of the Force Dimension hardware.  The function signature and
// math implement the implicit-ellipse gradient formula from 80-RESEARCH.md Q2
// and exactly match the Python reference in
//   src/force_dimension/test/test_ellipse_math.py
//
// Circle-reduction proof:
//   When sa == sb == R:
//     u0 = d0/R,  u1 = d1/R  =>  r_eff = r/R  (r = Euclidean radius)
//     gx = u0/R = d0/R^2,  gy = u1/R = d1/R^2
//     g_mag = r/R^2  =>  n0 = d0/r,  n1 = d1/r   (exact circle normal)
//     depth = (r/R - 1) * R = r - R               (exact circle depth)
//     f_r = clamp(-Kp*(r-R) - Kv*v_r)             (exact legacy circle formula)

#ifndef FORCE_DIMENSION_ELLIPSE_HELPERS_HPP_
#define FORCE_DIMENSION_ELLIPSE_HELPERS_HPP_

#include <cmath>
#include <algorithm>

namespace force_dimension {

/** Bilateral ellipse radial spring-damper force.
 *
 *  Computes the in-plane force components that hold the effector on the rim of
 *  an axis-aligned ellipse centered at the origin of the local frame (caller
 *  subtracts the center before calling).  The force is BILATERAL — it pulls
 *  from outside the rim and pushes from inside, so the effector is held ON
 *  radius a/b but is free to orbit.
 *
 *  Implements the implicit-ellipse gradient formula from 80-RESEARCH.md Q2.
 *  Matches src/force_dimension/test/test_ellipse_math.py::ellipse_radial_force
 *  numerically (within floating-point tolerance).
 *
 *  @param d0            Displacement from ellipse centre along first in-plane
 *                       axis (m).
 *  @param d1            Displacement from ellipse centre along second in-plane
 *                       axis (m).
 *  @param vel0          Velocity along the first in-plane axis (m/s).
 *  @param vel1          Velocity along the second in-plane axis (m/s).
 *  @param sa            Semi-axis along the first in-plane axis (m). Must be > 0.
 *  @param sb            Semi-axis along the second in-plane axis (m). Must be > 0.
 *  @param Kp            Radial spring stiffness (N/m).
 *  @param Kv            Radial damping coefficient (N·s/m).
 *  @param fmax          Force clamp magnitude (N). Outputs are in [-fmax, +fmax].
 *  @param center_deadzone  Absolute deadzone radius (m). When the normalised
 *                       radius r_eff <= center_deadzone / min(sa, sb) the
 *                       function writes zeros and returns immediately.
 *  @param force0        [out] Force along the first in-plane axis (N).
 *  @param force1        [out] Force along the second in-plane axis (N).
 */
inline void ellipse_radial_force(
    double d0, double d1,
    double vel0, double vel1,
    double sa, double sb,
    double Kp, double Kv, double fmax,
    double center_deadzone,
    double& force0, double& force1)
{
  force0 = 0.0;
  force1 = 0.0;

  // Degenerate-axis guard: skip the entire radial term if either semi-axis is
  // near zero (avoids division-by-zero in the normalised-coordinate step).
  if (sa < 1e-6 || sb < 1e-6)
    return;

  // Normalised coordinates — the unit circle in "ellipse space".
  const double u0 = d0 / sa;
  const double u1 = d1 / sb;
  const double r_eff = std::sqrt(u0 * u0 + u1 * u1);  // 1.0 exactly on the rim

  // Deadzone skip: treat the region near the centre as a singularity-free zone.
  // Threshold is expressed in normalised space so it scales with the axes.
  const double deadzone_norm = center_deadzone / std::min(sa, sb);
  if (r_eff <= deadzone_norm)
    return;

  // Outward unit normal via the implicit-ellipse gradient:
  //   grad f(d0,d1) = (2*d0/sa^2, 2*d1/sb^2)  ∝  (u0/sa, u1/sb)
  const double gx = u0 / sa;   // = d0 / sa^2
  const double gy = u1 / sb;   // = d1 / sb^2
  const double g_mag = std::sqrt(gx * gx + gy * gy);

  // Guard against degenerate gradient (should be unreachable given r_eff > deadzone_norm,
  // but adds robustness if the caller passes extreme parameters).
  if (g_mag < 1e-12)
    return;

  const double n0 = gx / g_mag;   // unit outward normal, first axis
  const double n1 = gy / g_mag;   // unit outward normal, second axis

  // Radial velocity (projection of velocity onto the outward normal).
  const double v_r = vel0 * n0 + vel1 * n1;

  // Signed depth in metres. Exact (r − R) when sa == sb == R.
  const double depth = (r_eff - 1.0) * std::sqrt(sa * sb);

  // Bilateral radial spring-damper, clamped to ±fmax.
  const double raw_fr = -Kp * depth - Kv * v_r;
  const double f_r = (raw_fr > fmax) ? fmax : ((raw_fr < -fmax) ? -fmax : raw_fr);

  force0 = f_r * n0;
  force1 = f_r * n1;
}

}  // namespace force_dimension

#endif  // FORCE_DIMENSION_ELLIPSE_HELPERS_HPP_
