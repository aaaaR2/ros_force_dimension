/** Copyright 2026 Neuromechatronics Lab, Carnegie Mellon University
 *
 *  This Source Code Form is subject to the terms of the Mozilla Public
 *  License, v. 2.0. If a copy of the MPL was not distributed with this
 *  file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

// Workspace constraint configuration for the Force Dimension haptic node.
// Fields are cached from ROS2 parameters and read at 2 kHz in ApplyHapticForce.
// Updated at runtime via the parameter callback — no get_parameter() in the hot path.

#ifndef FORCE_DIMENSION_CONSTRAINTS_H_
#define FORCE_DIMENSION_CONSTRAINTS_H_

namespace force_dimension {

/** Cached state for channel and circle workspace constraints.
 *
 *  channels: up to 3 independent bilateral dead-zone walls, one per Cartesian
 *            axis (index 0=X, 1=Y, 2=Z). Each can be enabled independently.
 *  circle:   radial dead-zone boundary in the XY task plane (axes 0 and 1).
 *
 *  All constraints share stiffness and damping. Damping is projected onto the
 *  constraint normal only (tangential motion free), matching SDK constraints.cpp.
 *  No explicit force clamping — SDK motor saturation handles limits.
 */
struct ConstraintState {

  // Per-axis channel constraints (index 0=X, 1=Y, 2=Z).
  bool   channel_enabled[3]    = {false, false, false};
  double channel_offset[3]     = {0.0, 0.0, 0.0};           // m, auto-captured on first read
  double channel_half_width[3] = {0.003, 0.003, 0.003};      // m, dead-zone half-width (±)

  // Circle constraint (radial boundary in the plane orthogonal to the primary channel axis).
  // Plane axes are always the two axes that are NOT the primary channel axis (0=X by default).
  // circle_plane_axes[0] and [1] are set at home time from the active channel config.
  bool   circle_enabled       = false;
  double circle_radius        = 0.11;        // m
  double circle_center[2]     = {0.0, 0.0};  // in-plane center, set at home = device pos + home_offset
  double home_offset[2]       = {0.0, 0.0};  // m, added to device position when homing circle center
  int    circle_plane_axes[2] = {1, 2};      // default: YZ plane (channel on X)

  // Shared spring-damper (used when per-axis override is -1)
  double stiffness = 2000.0; // N/m
  double damping   = 50.0;   // N/(m/s), projected onto constraint normal

  // True once offsets and circle_center have been captured from the first
  // valid SDK position read. Reset to false on re-activation.
  bool homed = false;

  // Orientation lock: spring-damper PD controller that holds two of the three
  // Cartesian rotation axes at their home orientation while leaving the third
  // free (typically the yaw/Z axis). Runs in the 2 kHz loop and sums its
  // torques with the external Wrench command torque (so Python-side viscous
  // damping on the free axis composes cleanly).
  //
  // Implementation reads dhdGetOrientationRad (wrist Euler angles) plus
  // dhdGetAngularVelocityRad (Cartesian angular velocity). For small-to-
  // moderate rotations, Euler components map to Cartesian axes well enough
  // for an experiment-grade lock.
  bool   wrist_lock_enabled           = false;
  int    wrist_lock_free_axis         = 2;     // 0=X(roll), 1=Y(pitch), 2=Z(yaw)
  double wrist_lock_stiffness         = 0.5;   // N*m/rad, per locked axis
  double wrist_lock_damping           = 0.05;  // N*m*s/rad, per locked axis
  double wrist_free_axis_damping      = 0.0;   // N*m*s/rad, viscous on free axis
  // Position-error deadband for the locked axes. If |angle_error| is below
  // this value, the PD outputs zero torque for that axis. Eliminates the
  // 2 kHz small-torque hum when the participant holds slightly off home,
  // at the cost of allowing the handle to drift within the deadband.
  double wrist_lock_error_deadband    = 0.01;  // rad (~0.57 deg)
  // First-order low-pass on the free-axis angular velocity before applying
  // viscous damping. Suppresses encoder-quantization noise that otherwise
  // tremors the device at higher damping coefficients. alpha in [0,1]:
  //   1.0 = no filtering (raw omega)
  //   ~0.2 = strong smoothing, minimal perceptible lag
  double wrist_free_axis_filter_alpha = 0.2;
  double wrist_free_axis_omega_filt   = 0.0;   // filter state, reset on activation
  // Home orientation stored as a 3x3 rotation matrix so the PD error is
  // computed as a rotation vector in world frame — no Euler-axis convention
  // dependence, no gimbal coupling. Initialized to identity; overwritten on
  // first tick after activation.
  double wrist_home_rotation[3][3]    = {{1,0,0},{0,1,0},{0,0,1}};
  bool   wrist_homed                  = false;
};

} // namespace force_dimension

#endif // FORCE_DIMENSION_CONSTRAINTS_H_
