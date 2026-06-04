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

  // Translation position hold (force-mode primary hold). Unlike the channel
  // walls above (dead-zone, outward-only damping — fine as occasional walls but
  // they limit-cycle when driven as a continuous hold), this is a SDK-hold-style
  // bilateral spring+damper to the captured home (channel_offset), with a force
  // cap. Used when DRD regulation is stopped (force mode) so translation is held
  // smoothly. No dead-zone: continuous damping on both directions prevents the
  // overshoot/buzz the channel walls produce when used as a hold.
  bool   translation_hold_enabled   = false;
  double translation_hold_stiffness = 150.0;  // N/m
  double translation_hold_damping   = 25.0;   // N/(m/s), bilateral
  double translation_hold_max_force = 5.0;    // N, per-axis clamp

  // Circular track ("ring") guide — 2D version of a spherical workspace guide.
  // Confines the effector to a circle of radius ring_radius in the plane
  // orthogonal to ring_height_axis, centered at the captured home
  // (channel_offset). A BILATERAL radial spring+damper holds it ON the rim
  // (tangential motion is free, so the effector orbits the ring); a separate
  // spring+damper on ring_height_axis locks the "height" to the home value.
  // Velocity damping uses the SDK's measured vel[] (rate-independent). Forces
  // are capped per component. Inside ring_center_deadzone the radial term is
  // skipped (the radial direction is undefined at r=0 and we must not shove the
  // hand at startup, when the device sits at the center). Force-mode only.
  bool   ring_enabled         = false;
  double ring_radius          = 0.04;   // m
  int    ring_height_axis     = 2;      // axis locked to home height (0=X,1=Y,2=Z)
  double ring_stiffness       = 300.0;  // N/m (radial + height)
  double ring_damping         = 30.0;   // N/(m/s), bilateral
  double ring_max_force       = 6.0;    // N, per-component clamp
  double ring_center_deadzone = 0.005;  // m, no radial force inside this radius
  // In-plane ring center (the "central cylinder" axis), in world coords on the
  // two non-height axes. Set at startup by on_activate from the autocenter base
  // plus the configured planar offset (constraints.ring.center_offset_0/1). When
  // ring_center_valid is false (e.g. ring enabled live without relaunch), the
  // loop falls back to the auto-homed channel_offset as the center.
  double ring_center[2]       = {0.0, 0.0};
  bool   ring_center_valid    = false;

  // Clean wrist UPRIGHT lock — same design principles as the ring constraint:
  // force-mode, bilateral spring+damper, damping on the SDK's MEASURED joint
  // velocity (dhdGetJointVelocities — rate-independent, so NO finite-difference,
  // NO 200 Hz decimation/ZOH, NO LPF hacks that the legacy wrist_lock_joint_space
  // path used; those assumed a 2 kHz loop that actually runs ~4 kHz and buzz).
  // Holds all three wrist joints at the captured upright pose (home joints).
  // Reference gains (SDK hold.cpp): low stiffness, generous damping, small torque
  // cap => overdamped, cannot buzz. Tune live. Mutually exclusive with the legacy
  // wrist_lock (enable one). Force-mode only.
  // Gains match SDK hold.cpp (proven smooth on this hardware, including WSL).
  // UNIT NOTE (important): dhdGetWristJointAngles -> rad, dhdGetJointVelocities
  // -> rad/s (NOT degrees). hold.cpp's stiffness 0.01 N*m/deg = 0.57 N*m/rad, and
  // its damping 0.04 is applied to a rad/s velocity => ~0.04 N*m*s/rad (LIGHT,
  // ~half-critical for the wrist). We earlier mis-"converted" damping to ~2.0
  // assuming deg/s; at ~50x hold's value the -Kv*omega term amplified velocity
  // noise into violent shake. Keep damping LOW (~0.04). DO NOT crank it up.
  bool   wrist_upright_enabled    = false;
  double wrist_upright_stiffness  = 0.6;    // N*m/rad, per wrist joint
  double wrist_upright_damping    = 0.04;   // N*m*s/rad (rad/s velocity!) — keep low
  double wrist_upright_max_torque = 0.02;   // N*m, per-joint clamp (hold.cpp value)
  double wrist_upright_home[3]    = {0.0, 0.0, 0.0};  // captured upright joints (rad)
  bool   wrist_upright_homed      = false;
  // First-order low-pass on the SDK joint velocity before the damping term.
  // Over WSL/usbipd the joint-velocity samples are noisy; with heavy Kv the
  // -Kv*omega term injects that noise as torque chatter (shake) — MORE damping
  // makes it worse. Filtering omega fixes that. alpha in [0,1]: 1.0 = raw (no
  // filter, default), lower = smoother (e.g. 0.1-0.2). Adds a little lag.
  double wrist_upright_vel_filter_alpha = 1.0;
  double wrist_upright_omega_filt[3]    = {0.0, 0.0, 0.0};  // filter state
  // Optional released ("free") wrist joint: -1 = lock all three (default);
  // 0/1/2 = free w0/w1/w2 — that joint gets only viscous damping
  // (-free_axis_damping * omega), no position lock. Lets the clean lock cover
  // the wrist-flexion case (lock roll+pitch w0,w1; free yaw w2 with ramped
  // damping) without the legacy decimated path. free_axis_damping is the same
  // knob damping_progression can ramp.
  int    wrist_upright_free_axis        = -1;
  double wrist_upright_free_axis_damping = 0.0;  // N*m*s/rad on the free joint

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
  // First-order low-pass on the locked-axis rotation-vector error before the
  // PD spring. Killing encoder dither at the source stops boundary-flutter
  // at the deadband edge, which the smoothstep ramp alone can't fully fix
  // when the error itself is jittering across the threshold every tick.
  // Same alpha convention as the omega filter: 1.0 = raw, lower = smoother.
  double wrist_lock_error_filter_alpha = 0.15;
  double wrist_lock_error_filt[3]      = {0.0, 0.0, 0.0};
  // First-order low-pass on the locked-axis angular velocity before the PD
  // damping term. Without this, encoder velocity quantization near zero
  // motion produces a buzzing damping torque that dominates whenever the
  // handle is held at gravity-bias equilibrium (typically the bottom of
  // the pitch deadband). Same alpha convention as the other filters.
  double wrist_lock_omega_filter_alpha = 0.15;
  double wrist_lock_omega_filt[3]      = {0.0, 0.0, 0.0};
  // Joint-space PD mode. When true, the wrist lock reads per-joint angles
  // (dhdGetWristJointAngles) and writes per-joint torques
  // (drdSetForceAndWristJointTorquesAndGripperForce) instead of Cartesian
  // torque. This avoids the Sigma.7 parallel-mechanism Jacobian projecting
  // small Cartesian wrist torques onto the translation motors, which
  // otherwise fight DRD position-hold and produce buzz when roll/pitch
  // is non-zero. Joint mapping (empirical for Sigma.7): j0=roll, j1=pitch,
  // j2=yaw. The free axis index follows the same convention as the
  // Cartesian mode (0=roll, 1=pitch, 2=yaw).
  bool   wrist_lock_joint_space        = false;
  double wrist_home_joint[3]           = {0.0, 0.0, 0.0};
  bool   wrist_joint_homed             = false;
  double wrist_joint_prev[3]           = {0.0, 0.0, 0.0};
  double wrist_joint_vel_filt[3]       = {0.0, 0.0, 0.0};
  // Zero-order-hold the joint-space PD output for kJointHoldTicks ticks
  // between recomputes. The wrist's mechanical bandwidth is well under
  // 200 Hz, so the PD doesn't need to update at 2 kHz. Holding the torque
  // command for ~5 ms breaks the encoder-quantization positive-feedback
  // limit cycle that sustained the audible buzz: the cycle needs 2 kHz
  // closed-loop iteration to ring at ~kHz, and ZOH at 200 Hz removes the
  // bandwidth required to sustain it. Cycle period is determined by
  // kJointHoldTicks at the 2 kHz haptic loop rate.
  double wrist_joint_tau_hold[3]       = {0.0, 0.0, 0.0};
  int    wrist_joint_hold_counter      = 0;
  // Home orientation stored as a 3x3 rotation matrix so the PD error is
  // computed as a rotation vector in world frame — no Euler-axis convention
  // dependence, no gimbal coupling. Initialized to identity; overwritten on
  // first tick after activation.
  double wrist_home_rotation[3][3]    = {{1,0,0},{0,1,0},{0,0,1}};
  bool   wrist_homed                  = false;

  // --- Roll-axis lock-mode offset (upright / left / right) -----------------
  // The wrist-flexion task locks roll+pitch and leaves yaw free. To support
  // "left"/"right" lock modes the locked ROLL setpoint is offset from the
  // captured neutral by a slewed amount. All fields below are touched ONLY by
  // the 2 kHz haptic thread (plain doubles, no locking). The cross-thread
  // *target* offset lives in Node::wrist_roll_offset_target_ (std::atomic).
  // wrist_neutral_roll is the roll angle captured at home (= wrist_home_joint[0]
  // on the first tick); the effective roll setpoint is
  //   wrist_home_joint[0] + wrist_roll_offset_current
  // clamped into the device's safe roll range. The current offset slews toward
  // the target at wrist_roll_slew_rate so a mode switch never steps the
  // setpoint (a step would slam the wrist with Kp*offset N*m instantly).
  double wrist_neutral_roll           = 0.0;   // roll captured at home (rad)
  double wrist_roll_offset_current    = 0.0;   // slewed offset applied to roll (rad)
  double wrist_roll_min               = 0.0;   // device roll lower limit (rad, slot 3)
  double wrist_roll_max               = 0.0;   // device roll upper limit (rad, slot 3)
  double wrist_roll_range_margin      = 0.05;  // safety margin inside the limits (rad)
  double wrist_roll_slew_rate         = 0.5;   // offset slew rate (rad/s)
  bool   wrist_roll_range_valid       = false; // true once the live range was read
};

} // namespace force_dimension

#endif // FORCE_DIMENSION_CONSTRAINTS_H_
