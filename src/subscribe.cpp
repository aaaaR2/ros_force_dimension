/** Copyright 2022 Neuromechatronics Lab, Carnegie Mellon University
 *
 *  Created by: a. whit. (nml@whit.contact)
 *
 *  This Source Code Form is subject to the terms of the Mozilla Public
 *  License, v. 2.0. If a copy of the MPL was not distributed with this
 *  file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

// Functionality related to receiving ROS2 messages and applying haptic forces.

#include <cmath>

// Import node header.
#include "node.hpp"

// Import the Force Dimension haptics header.
#include "dhdc.h"
#include "drdc.h"

// Import package headers.
#include "messages.hpp"
#include "qos.hpp"
#include "topics.hpp"

// Scope namespace elements.
using force_dimension::Node;

// Subscribes to ROS messages that indicate an instantaneous force to be
// applied by the robot.
void Node::SubscribeForce(void) {
  auto callback = [this](ForceMessage m) { this->force_callback(m); };
  auto topic = FORCE_COMMAND_TOPIC;
  auto qos = DefaultQoS();
  force_subscription_ =
      this->create_subscription<ForceMessage>(topic, qos, callback);
}

// Stores the latest external force+torque command (additive zero-order hold).
void Node::force_callback(const ForceMessage message) {
  std::lock_guard<std::mutex> lock(force_mutex_);
  last_force_command_ = message;
}

// Subscribes to wrist lock-mode commands. Unity (or the CLI) publishes a
// std_msgs/String with value "upright" | "left" | "right" to switch the locked
// roll orientation live. Resolves to /robot/wrist_lock/set_mode under the node
// namespace, mirroring command/force.
void Node::SubscribeWristMode(void) {
  auto callback = [this](WristModeMessage m) { this->wrist_mode_callback(m); };
  auto topic = WRIST_MODE_COMMAND_TOPIC;
  auto qos = DefaultQoS();
  wrist_mode_subscription_ =
      this->create_subscription<WristModeMessage>(topic, qos, callback);
}

// Maps a lock-mode string to a roll offset (rad) relative to the captured
// upright neutral. Unknown strings are treated as upright (offset 0) with a
// warning so a typo can never silently leave the wrist in a stale offset.
double Node::mode_to_offset(const std::string &mode) const {
  if (mode == "upright") return 0.0;
  if (mode == "left")    return wrist_roll_offset_left_;
  if (mode == "right")   return wrist_roll_offset_right_;
  return 0.0;
}

// Stores the latest wrist lock mode by retargeting the slewed roll offset.
// The haptic loop slews the applied offset toward this target; it never steps.
void Node::wrist_mode_callback(const WristModeMessage message) {
  const std::string mode = message.data;
  if (mode != "upright" && mode != "left" && mode != "right") {
    Log("Wrist lock: ignoring unknown mode '" + mode +
        "' (expected upright|left|right)");
    return;
  }
  const double offset = mode_to_offset(mode);
  wrist_roll_offset_target_.store(offset);
  Log("Wrist lock mode -> " + mode + " (roll offset target " +
      std::to_string(offset) + " rad)");
  if (wrist_mode_publisher_) {
    WristModeMessage feedback;
    feedback.data = mode;
    wrist_mode_publisher_->publish(feedback);
  }
}

// Applies workspace constraint forces + external commands to the device at 2 kHz.
// Reads position and velocity directly from the SDK each iteration — no ROS transport.
// External force commands (last_force_command_) are added on top of constraint forces.
// Gravity compensation is applied automatically by the SDK on top of all forces.
void Node::ApplyHapticForce(void) {
  if (hardware_disabled_) return;

  // DRD watchdog: the regulation thread sometimes exits silently after a
  // few seconds when we drive it with drdSetForce* at high rate (the SDK
  // interprets the override as "caller is regulating now"). Detect and
  // restart, logging once per drop so we know if restarts start failing.
  // In force mode the regulation thread is intentionally stopped (drdStop in
  // on_activate), so drdIsRunning() is always false — skip the watchdog entirely,
  // otherwise it would spin trying to restart regulation every tick.
  static int drd_drop_count = 0;
  if (!force_mode_ && !drdIsRunning()) {
    ++drd_drop_count;
    const char *err_str = dhdErrorGetLastStr();
    std::string message = "DRD regulation thread stopped (drop #";
    message += std::to_string(drd_drop_count);
    message += "): ";
    message += err_str ? err_str : "(null)";
    Log(message);
    // Try to restart. Re-apply the regulate flags first so the SDK knows
    // which axes to hold; these calls are idempotent.
    drdRegulatePos(true);
    drdRegulateRot(false);
    drdRegulateGrip(true);
    if (drdStart() < 0) {
      std::string fail = "drdStart() after drop failed: ";
      fail += dhdErrorGetLastStr();
      Log(fail);
      return;  // skip this tick; try again next tick
    }
    Log("DRD regulation restarted");
  }

  // Read position/orientation and velocities directly from the SDK.
  // When wrist lock is enabled, use the consolidated drd* getters so we pack
  // position + Euler + linear + angular velocity into two USB transactions
  // instead of four (matters on WSL/usbipd). When wrist lock is off, stick
  // with the lighter dhd position+linear-velocity pair.
  double pos[3] = {};
  double vel[3] = {};
  double ang[3] = {};    // wrist Euler angles (rad) — unused for PD, kept for logging
  double omega[3] = {};  // Cartesian angular velocity (rad/s, world frame)
  double R[3][3] = {};   // end-effector rotation matrix (world frame)
  bool   got_orientation = false;
  if (constraints_.wrist_lock_enabled) {
    double pg = 0.0;       // gripper gap (unused here)
    double vg = 0.0;       // gripper velocity (unused here)
    drdGetPositionAndOrientation(&pos[0], &pos[1], &pos[2],
                                 &ang[0], &ang[1], &ang[2],
                                 &pg, R, device_id_);
    drdGetVelocity(&vel[0], &vel[1], &vel[2],
                   &omega[0], &omega[1], &omega[2],
                   &vg, device_id_);
    got_orientation = true;
  } else {
    dhdGetPosition(&pos[0], &pos[1], &pos[2], device_id_);
    dhdGetLinearVelocity(&vel[0], &vel[1], &vel[2], device_id_);
  }

  // Compute workspace constraint forces (channel + circle dead-zone).
  double cf[3] = {};
  ComputeConstraintForce(pos, vel, cf);

  // Force-mode translation hold: a bilateral spring+damper to the captured home
  // (channel_offset, set by the auto-home in ComputeConstraintForce above), with
  // a per-axis force cap. This is the SDK hold-example pattern. Unlike the
  // channel walls — which damp outward motion only and therefore limit-cycle
  // when used as a continuous hold — the continuous (both-directions) damping
  // here dissipates overshoot, so translation is held smoothly without buzz.
  if (constraints_.translation_hold_enabled && constraints_.homed) {
    const double Kp = constraints_.translation_hold_stiffness;
    const double Kv = constraints_.translation_hold_damping;
    const double fmax = constraints_.translation_hold_max_force;
    for (int i = 0; i < 3; ++i) {
      double f = -Kp * (pos[i] - constraints_.channel_offset[i]) - Kv * vel[i];
      if (f >  fmax) f =  fmax;
      if (f < -fmax) f = -fmax;
      cf[i] += f;
    }
  }

  // Sum constraint forces with any external force command (zero-order hold).
  // Copy under lock to avoid data race with force_callback on the ROS thread.
  ForceMessage cmd;
  {
    std::lock_guard<std::mutex> lock(force_mutex_);
    cmd = last_force_command_;
  }
  const double fx = cf[0] + cmd.force.x;
  const double fy = cf[1] + cmd.force.y;
  const double fz = cf[2] + cmd.force.z;

  // Wrist orientation lock: PD on two of three Cartesian rotation axes.
  // Leaves the free axis (typically Z/yaw) untouched so the Python
  // viscous_damping_wrist node can drive it via cmd.torque.*.
  // Orientation + angular velocity were already read above in the consolidated
  // drd getters to save USB transactions — just run the PD here.
  double wrist_tau[3] = {0.0, 0.0, 0.0};   // Cartesian wrist torque (world frame)
  double wrist_joint_tau[3] = {0.0, 0.0, 0.0};  // joint-space wrist torque
  if (constraints_.wrist_upright_enabled) {
    // Clean upright lock — same design principles as the ring constraint:
    // force-mode, bilateral spring+damper, damping on the SDK's MEASURED joint
    // velocity (rate-independent: no finite-difference, no decimation/ZOH, no
    // LPF), torque-capped. Holds all three wrist joints at the captured upright
    // pose. Overdamped gains (per SDK hold.cpp) so it cannot buzz.
    double j_now[3] = {0.0, 0.0, 0.0};
    double jvel[DHD_MAX_DOF] = {};
    dhdGetWristJointAngles(&j_now[0], &j_now[1], &j_now[2], device_id_);
    dhdGetJointVelocities(jvel, device_id_);  // slots 3,4,5 = wrist roll/pitch/yaw
    if (!constraints_.wrist_upright_homed) {
      for (int i = 0; i < 3; ++i) {
        constraints_.wrist_upright_home[i] = j_now[i];
        constraints_.wrist_upright_omega_filt[i] = 0.0;  // reset filter on home
      }
      constraints_.wrist_upright_homed = true;
    }
    const double Kp_u = constraints_.wrist_upright_stiffness;
    const double Kv_u = constraints_.wrist_upright_damping;
    const double tmax = constraints_.wrist_upright_max_torque;
    // Optional first-order LPF on the (noisy over WSL/usbipd) joint velocity.
    // alpha 1.0 = raw (no filter); lower = smoother. Kills the torque chatter
    // that heavy Kv injects from velocity-sensor noise.
    double a_u = constraints_.wrist_upright_vel_filter_alpha;
    if (a_u < 0.0) a_u = 0.0;
    if (a_u > 1.0) a_u = 1.0;
    const int free_ax = constraints_.wrist_upright_free_axis;
    const double b_free = constraints_.wrist_upright_free_axis_damping;
    for (int i = 0; i < 3; ++i) {
      constraints_.wrist_upright_omega_filt[i] =
          a_u * jvel[3 + i] +
          (1.0 - a_u) * constraints_.wrist_upright_omega_filt[i];
      double t;
      if (i == free_ax) {
        // Released axis: viscous damping only, no position lock (e.g. yaw for
        // the wrist-flexion case). b_free can be ramped by damping_progression.
        t = -b_free * constraints_.wrist_upright_omega_filt[i];
      } else {
        t = -Kp_u * (j_now[i] - constraints_.wrist_upright_home[i])
            - Kv_u * constraints_.wrist_upright_omega_filt[i];
      }
      if (t >  tmax) t =  tmax;
      if (t < -tmax) t = -tmax;
      wrist_joint_tau[i] = t;
    }
  } else if (constraints_.wrist_lock_enabled &&
      constraints_.wrist_lock_joint_space) {
    // Joint-space PD with zero-order-hold decimation. The PD output is
    // recomputed every kJointHoldTicks ticks (default 10 -> 200 Hz) and
    // held constant in between. Two reasons:
    //   1. Wrist mechanical bandwidth is under ~50 Hz; updating PD output
    //      at 2 kHz is unnecessary and gives encoder-quantization
    //      positive-feedback room to ring at audible frequencies.
    //   2. ZOH at 200 Hz mathematically prevents any closed-loop limit
    //      cycle above ~100 Hz: the controller cannot respond fast enough
    //      to sustain it. The buzz (which sounds ~1 kHz) needs the 2 kHz
    //      loop to close on itself.
    // Joint angle is still read every tick so the snapshot stays fresh
    // for downstream publishers; only the PD compute and torque write
    // are decimated.
    constexpr int kJointHoldTicks = 10;  // -> 200 Hz PD update
    double j_now[3] = {0.0, 0.0, 0.0};
    dhdGetWristJointAngles(&j_now[0], &j_now[1], &j_now[2], device_id_);
    if (!constraints_.wrist_joint_homed) {
      for (int i = 0; i < 3; ++i) {
        constraints_.wrist_home_joint[i] = j_now[i];
        constraints_.wrist_joint_prev[i] = j_now[i];
        constraints_.wrist_joint_vel_filt[i] = 0.0;
        constraints_.wrist_joint_tau_hold[i] = 0.0;
      }
      constraints_.wrist_joint_hold_counter = 0;
      constraints_.wrist_joint_homed = true;
      // Capture the upright neutral roll and start the offset at zero. The
      // applied offset then slews from here toward whatever mode is selected,
      // so the very first activation never jumps the roll setpoint.
      constraints_.wrist_neutral_roll = j_now[0];
      constraints_.wrist_roll_offset_current = 0.0;
    }

    // Decimated PD recompute. Otherwise the held torque from last compute
    // is reused (ZOH).
    if (constraints_.wrist_joint_hold_counter <= 0) {
      // dt is the *decimated* update period (5 ms at 200 Hz), not the 2 kHz
      // tick. Using the decimated period keeps the velocity estimate's gain
      // consistent with how often we look at it.
      constexpr double kDecimatedDt = kJointHoldTicks / 2000.0;
      const int free_ax_j = constraints_.wrist_lock_free_axis;
      const double Kp_j = constraints_.wrist_lock_stiffness;
      const double Kv_j = constraints_.wrist_lock_damping;
      const double b_free_j = constraints_.wrist_free_axis_damping;
      const double err_dz = constraints_.wrist_lock_error_deadband;
      constexpr double kRampFracJ = 1.0;
      const double ramp_w_j = err_dz * kRampFracJ;
      // Larger raw deadbands here than the Cartesian path: at a 200 Hz
      // update rate, encoder dither shows up as bigger velocity excursions
      // even though the underlying motion is the same.
      constexpr double kJointVelQuant = 0.10;     // rad/s (~5.7 deg/s)
      constexpr double kJointSilenceTau = 0.010;  // N·m
      constexpr double kJointSilenceVel = 0.10;   // rad/s

      // --- Roll lock-mode offset: slew toward target, clamp to safe range ---
      // Runs once per decimated recompute (200 Hz, kDecimatedDt) so the slew
      // rate stays in rad/s. The roll setpoint below becomes
      //   wrist_home_joint[0] + wrist_roll_offset_current.
      // Slewing bounds the *rate* of torque rise (Kp*slew_rate), so a mode
      // switch ramps in smoothly instead of stepping (a step would apply
      // Kp*offset N*m to the wrist instantly).
      {
        double tgt = wrist_roll_offset_target_.load();
        // Keep the resulting roll setpoint inside the device's measured joint
        // range (minus a margin). Offsets are relative to the captured neutral,
        // so shift the absolute limits by the neutral to get offset bounds.
        if (constraints_.wrist_roll_range_valid) {
          const double lo = (constraints_.wrist_roll_min +
                             constraints_.wrist_roll_range_margin) -
                             constraints_.wrist_neutral_roll;
          const double hi = (constraints_.wrist_roll_max -
                             constraints_.wrist_roll_range_margin) -
                             constraints_.wrist_neutral_roll;
          if (tgt < lo) tgt = lo;
          if (tgt > hi) tgt = hi;
        }
        const double step = constraints_.wrist_roll_slew_rate * kDecimatedDt;
        double delta = tgt - constraints_.wrist_roll_offset_current;
        if (delta >  step) delta =  step;
        if (delta < -step) delta = -step;
        constraints_.wrist_roll_offset_current += delta;
      }

      for (int i = 0; i < 3; ++i) {
        double dj = (j_now[i] - constraints_.wrist_joint_prev[i]) / kDecimatedDt;
        constraints_.wrist_joint_prev[i] = j_now[i];
        if (std::abs(dj) < kJointVelQuant) dj = 0.0;
        constraints_.wrist_joint_vel_filt[i] = dj;

        if (i == free_ax_j) {
          constraints_.wrist_joint_tau_hold[i] = -b_free_j * dj;
          continue;
        }
        // Roll (i==0) is offset by the slewed lock-mode offset; pitch holds at
        // its captured neutral. The offset is zero in upright mode.
        const double setpoint =
            (i == 0) ? constraints_.wrist_home_joint[i] +
                           constraints_.wrist_roll_offset_current
                     : constraints_.wrist_home_joint[i];
        double err = j_now[i] - setpoint;
        if (std::abs(err) < err_dz) err = 0.0;
        double mag = std::abs(err);
        double spring = 0.0;
        if (mag > err_dz) {
          double effective_err = err > 0.0 ? err - err_dz : err + err_dz;
          double scale = 1.0;
          if (ramp_w_j > 0.0 && mag < err_dz + ramp_w_j) {
            double t = (mag - err_dz) / ramp_w_j;
            scale = t * t * (3.0 - 2.0 * t);
          }
          spring = -Kp_j * effective_err * scale;
        }
        double tau_i = spring - Kv_j * dj;
        if (std::abs(tau_i) < kJointSilenceTau &&
            std::abs(dj) < kJointSilenceVel) {
          tau_i = 0.0;
        }
        constraints_.wrist_joint_tau_hold[i] = tau_i;
      }
      constraints_.wrist_joint_hold_counter = kJointHoldTicks;
    }
    --constraints_.wrist_joint_hold_counter;
    for (int i = 0; i < 3; ++i) {
      wrist_joint_tau[i] = constraints_.wrist_joint_tau_hold[i];
    }
  } else if (constraints_.wrist_lock_enabled && got_orientation) {
    // Capture home orientation as a rotation matrix on the first tick.
    if (!constraints_.wrist_homed) {
      for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
          constraints_.wrist_home_rotation[r][c] = R[r][c];
      // Zero the error and omega filter states so the first tick after
      // homing doesn't ramp from stale values (which could push the
      // handle on activation).
      for (int i = 0; i < 3; ++i) {
        constraints_.wrist_lock_error_filt[i] = 0.0;
        constraints_.wrist_lock_omega_filt[i] = 0.0;
      }
      constraints_.wrist_homed = true;
    }

    // Compute the orientation error as a rotation vector in world frame.
    //   R_err = R_current * R_home^T  (rotation from home to current).
    // For small rotations, R_err ≈ I + [err_vec]_×, so the skew-symmetric
    // part of R_err gives the rotation vector directly. This is
    // convention-free: err_vec[0] is rotation about world X (roll),
    // err_vec[1] about Y (pitch), err_vec[2] about Z (yaw), regardless of
    // what Euler convention the SDK uses for the raw angles.
    double R_err[3][3] = {};
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        double s = 0.0;
        for (int k = 0; k < 3; ++k)
          s += R[i][k] * constraints_.wrist_home_rotation[j][k];
        R_err[i][j] = s;
      }
    }
    double err_vec[3];
    err_vec[0] = 0.5 * (R_err[2][1] - R_err[1][2]);  // roll error
    err_vec[1] = 0.5 * (R_err[0][2] - R_err[2][0]);  // pitch error
    err_vec[2] = 0.5 * (R_err[1][0] - R_err[0][1]);  // yaw error

    const int free_ax = constraints_.wrist_lock_free_axis;
    const double Kp = constraints_.wrist_lock_stiffness;
    const double Kv = constraints_.wrist_lock_damping;
    const double b_free = constraints_.wrist_free_axis_damping;
    // LPF on the free-axis omega (suppresses encoder-quantization noise).
    double alpha = constraints_.wrist_free_axis_filter_alpha;
    if (alpha < 0.0) alpha = 0.0;
    if (alpha > 1.0) alpha = 1.0;
    constraints_.wrist_free_axis_omega_filt =
        alpha * omega[free_ax] +
        (1.0 - alpha) * constraints_.wrist_free_axis_omega_filt;

    const double err_deadband = constraints_.wrist_lock_error_deadband;
    // Width of the smooth ramp into full spring stiffness, as a fraction of
    // the deadband. The spring transitions 0 -> full Kp via smoothstep over
    // [deadband, deadband*(1+ramp_frac)]. This eliminates the slope
    // discontinuity at the deadband edge that caused encoder-noise-driven
    // limit-cycle buzz when the handle hovered right on the boundary.
    constexpr double kRampFrac = 1.0;
    const double ramp_w = err_deadband * kRampFrac;
    // LPF on the rotation-vector error so encoder dither can't flutter the
    // signal across the deadband boundary every tick. Clamp alpha into
    // [0, 1]; a fresh activation should also reset the filter state via
    // wrist_homed=false (handled below in the spring loop).
    double err_alpha = constraints_.wrist_lock_error_filter_alpha;
    if (err_alpha < 0.0) err_alpha = 0.0;
    if (err_alpha > 1.0) err_alpha = 1.0;
    double omega_alpha = constraints_.wrist_lock_omega_filter_alpha;
    if (omega_alpha < 0.0) omega_alpha = 0.0;
    if (omega_alpha > 1.0) omega_alpha = 1.0;
    // Per-axis idle gate: when filtered torque on this axis is below
    // kSilenceTorque AND filtered angular velocity is below kSilenceOmega,
    // force the locked-axis torque to exactly zero. This kills the residual
    // sub-mN·m PD output that excites the wrist gimbal at audible frequencies
    // when the handle rests at the deadband edge under gravity bias.
    // Thresholds chosen well below human force-perception (~10 mN·m at the
    // wrist) but above the encoder-quantization torque floor.
    constexpr double kSilenceTorque = 0.008;  // N·m
    constexpr double kSilenceOmega  = 0.05;   // rad/s
    // Raw-omega quantization deadband. Encoder velocity is a finite
    // difference of position; near-zero motion is dominated by quantization
    // dither that's slightly biased by gravity creep. Feeding biased dither
    // through an LPF integrates a slow drift that shows up as accumulating
    // damping torque -> audible buzz that grows over a few hundred ms.
    // Force raw omega to exactly zero below this threshold so the LPF can
    // reach a true zero steady state. Above the threshold, real motion
    // passes through unchanged (no scale factor, no lag past the LPF).
    constexpr double kOmegaQuantization = 0.03;  // rad/s (~1.7 deg/s)
    for (int i = 0; i < 3; ++i) {
      if (i == free_ax) {
        wrist_tau[i] = -b_free * constraints_.wrist_free_axis_omega_filt;
      } else {
        // Damping (-Kv * ω_filt) is always active so motion through the
        // home point feels smooth. Both error and omega are LPF'd to keep
        // encoder quantization from buzzing the motors at gravity-bias
        // equilibrium. The spring ramps in smoothly from the deadband
        // edge: zero inside the deadband, full Kp beyond deadband+ramp_w,
        // smoothstep blend in between. C¹-continuous across the boundary.
        // Pre-deadband the raw error inside the position-deadband zone so
        // biased encoder dither can't drive the spring while the handle is
        // mechanically resting at the edge. NOTE: no LPF on error either —
        // see omega comment below. Phase lag inside the spring loop was
        // pushing the closed-loop poles toward instability at the boundary.
        double err_in = err_vec[i];
        if (std::abs(err_in) < err_deadband) err_in = 0.0;
        constraints_.wrist_lock_error_filt[i] = err_in;
        // Pre-deadband the raw omega so biased encoder dither can't drive
        // the damping torque on a near-stationary handle. NOTE: no LPF
        // here. The LPF was creating ~3 ms of phase lag on the damping
        // term, which destabilized the closed loop near the deadband edge:
        // the PD's own torque excited the wrist's structural mode at ~20 Hz,
        // building into the "buzz that accumulates until it bumps the other
        // deadband" symptom. With raw-omega deadbanding, the noise floor is
        // killed without adding phase lag, so damping stays stabilizing.
        double omega_in = omega[i];
        if (std::abs(omega_in) < kOmegaQuantization) omega_in = 0.0;
        constraints_.wrist_lock_omega_filt[i] = omega_in;
        double err = constraints_.wrist_lock_error_filt[i];
        double mag = std::abs(err);
        double spring = 0.0;
        if (mag > err_deadband) {
          double effective_err =
              err > 0.0 ? err - err_deadband : err + err_deadband;
          double scale = 1.0;
          if (ramp_w > 0.0 && mag < err_deadband + ramp_w) {
            double t = (mag - err_deadband) / ramp_w;  // 0..1
            scale = t * t * (3.0 - 2.0 * t);           // smoothstep
          }
          spring = -Kp * effective_err * scale;
        }
        double tau_i = spring - Kv * constraints_.wrist_lock_omega_filt[i];
        // Idle silence gate: zero out tiny torques at near-zero motion so
        // the motors don't sing while the handle hovers at the deadband
        // edge. Hysteresis is implicit: any real participant input will
        // both raise omega and (if past the deadband) raise spring above
        // these thresholds, instantly re-engaging the PD.
        if (std::abs(tau_i) < kSilenceTorque &&
            std::abs(constraints_.wrist_lock_omega_filt[i]) < kSilenceOmega) {
          tau_i = 0.0;
        }
        wrist_tau[i] = tau_i;
      }
    }
  }

  // Gripper constraint: one-sided wall at the home gap with deadband +
  // smoothstep entry. The hard `if (depth > 0)` test was a deadband-edge
  // limit cycle: encoder dither on the gripper position (~tens of microns)
  // flipped the test on/off every tick, firing the 500 N/m spring
  // intermittently. The gripper motor shares hardware with the wrist
  // gimbal, so the chatter transmitted as audible buzz that grew worse
  // when wrist pitch shifted the natural resting gripper gap onto the
  // boundary. Same C¹ smoothstep + raw-input deadband treatment we use
  // for the wrist lock.
  double fg = 0.0;
  double currentGap = 0.0;
  {
    constexpr double Kgp             = 500.0;  // N/m
    constexpr double MaxGripperForce = 10.0;   // N — closing cap
    constexpr double kGripDeadband   = 0.0008; // m — silence zone past home
    constexpr double kGripRamp       = 0.0008; // m — smoothstep into full Kgp
    dhdGetGripperGap(&currentGap, device_id_);
    double depth = currentGap - home_gripper_gap_;
    // Pre-deadband the raw depth so encoder dither at the boundary
    // doesn't reach the spring at all.
    if (depth < kGripDeadband) {
      fg = 0.0;
    } else {
      double effective_depth = depth - kGripDeadband;
      double scale = 1.0;
      if (kGripRamp > 0.0 && effective_depth < kGripRamp) {
        double t = effective_depth / kGripRamp;       // 0..1
        scale = t * t * (3.0 - 2.0 * t);              // smoothstep
      }
      fg = -Kgp * effective_depth * scale;
      if (fg < -MaxGripperForce) fg = -MaxGripperForce;
    }
  }

  // Stash a snapshot for the ROS executor's publish callbacks. This is
  // the *only* place SDK getters run during steady state — the publish
  // callbacks read from the snapshot under state_mutex_ and never touch
  // the SDK themselves. Buttons + gripper_angle are read every 20th tick
  // (~100 Hz at 2 kHz) since they're slow-changing and the extra USB
  // transactions would tax the haptic loop budget.
  static int snapshot_decim = 0;
  bool    update_slow_fields = false;
  int     button_mask_snap = 0;
  double  gripper_angle_snap = 0.0;
  double  wrist_joint_snap[3] = {0.0, 0.0, 0.0};  // raw joint angles w0,w1,w2 (rad)
  double  ori_rad_snap[3] = {0.0, 0.0, 0.0};      // end-effector Euler (rad)
  double  quat_snap[4] = {0.0, 0.0, 0.0, 1.0};    // orientation quaternion (x,y,z,w)
  bool    ori_snap_valid = false;
  bool    has_gripper_snap = (!hardware_disabled_) && dhdHasGripper(device_id_);
  if (++snapshot_decim >= 20) {
    snapshot_decim = 0;
    update_slow_fields = true;
    if (!hardware_disabled_) {
      button_mask_snap = dhdGetButtonMask(device_id_);
      if (has_gripper_snap) {
        dhdGetGripperAngleRad(&gripper_angle_snap, device_id_);
      }
      // Raw wrist joint angles for recording (w0=roll, w1=pitch, w2=yaw).
      // Read here (~100 Hz) so they are always recorded, regardless of whether
      // a wrist lock is active. Slow-changing, so the decimated rate is plenty.
      dhdGetWristJointAngles(&wrist_joint_snap[0], &wrist_joint_snap[1],
                             &wrist_joint_snap[2], device_id_);
      // End-effector orientation, captured here so it is recorded in EVERY mode
      // (the per-tick wrist_lock path only runs when that lock is active). Euler
      // for the existing field; rotation matrix -> quaternion for the quat topic.
      dhdGetOrientationRad(&ori_rad_snap[0], &ori_rad_snap[1], &ori_rad_snap[2],
                           device_id_);
      double Rm[3][3] = {};
      if (dhdGetOrientationFrame(Rm, device_id_) >= 0) {
        const double tr = Rm[0][0] + Rm[1][1] + Rm[2][2];
        double qx, qy, qz, qw;
        if (tr > 0.0) {
          double s = 0.5 / std::sqrt(tr + 1.0);
          qw = 0.25 / s;
          qx = (Rm[2][1] - Rm[1][2]) * s;
          qy = (Rm[0][2] - Rm[2][0]) * s;
          qz = (Rm[1][0] - Rm[0][1]) * s;
        } else if (Rm[0][0] > Rm[1][1] && Rm[0][0] > Rm[2][2]) {
          double s = 2.0 * std::sqrt(1.0 + Rm[0][0] - Rm[1][1] - Rm[2][2]);
          qw = (Rm[2][1] - Rm[1][2]) / s;
          qx = 0.25 * s;
          qy = (Rm[0][1] + Rm[1][0]) / s;
          qz = (Rm[0][2] + Rm[2][0]) / s;
        } else if (Rm[1][1] > Rm[2][2]) {
          double s = 2.0 * std::sqrt(1.0 + Rm[1][1] - Rm[0][0] - Rm[2][2]);
          qw = (Rm[0][2] - Rm[2][0]) / s;
          qx = (Rm[0][1] + Rm[1][0]) / s;
          qy = 0.25 * s;
          qz = (Rm[1][2] + Rm[2][1]) / s;
        } else {
          double s = 2.0 * std::sqrt(1.0 + Rm[2][2] - Rm[0][0] - Rm[1][1]);
          qw = (Rm[1][0] - Rm[0][1]) / s;
          qx = (Rm[0][2] + Rm[2][0]) / s;
          qy = (Rm[1][2] + Rm[2][1]) / s;
          qz = 0.25 * s;
        }
        quat_snap[0] = qx; quat_snap[1] = qy; quat_snap[2] = qz; quat_snap[3] = qw;
        ori_snap_valid = true;
      }
    }
  }
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    device_snapshot_.pos[0] = pos[0];
    device_snapshot_.pos[1] = pos[1];
    device_snapshot_.pos[2] = pos[2];
    device_snapshot_.vel[0] = vel[0];
    device_snapshot_.vel[1] = vel[1];
    device_snapshot_.vel[2] = vel[2];
    if (got_orientation) {
      device_snapshot_.ori_rad[0] = ang[0];
      device_snapshot_.ori_rad[1] = ang[1];
      device_snapshot_.ori_rad[2] = ang[2];
      device_snapshot_.omega[0] = omega[0];
      device_snapshot_.omega[1] = omega[1];
      device_snapshot_.omega[2] = omega[2];
      device_snapshot_.has_orientation = true;
    }
    device_snapshot_.gripper_gap_m = currentGap;
    device_snapshot_.has_gripper = has_gripper_snap;
    if (update_slow_fields) {
      device_snapshot_.button_mask = button_mask_snap;
      device_snapshot_.gripper_angle_rad = gripper_angle_snap;
      if (!hardware_disabled_) {
        device_snapshot_.wrist_joint_rad[0] = wrist_joint_snap[0];
        device_snapshot_.wrist_joint_rad[1] = wrist_joint_snap[1];
        device_snapshot_.wrist_joint_rad[2] = wrist_joint_snap[2];
        device_snapshot_.has_wrist_joint = true;
      }
      if (ori_snap_valid) {
        // Mode-independent orientation capture (Euler + quaternion), so it is
        // recorded even when no per-tick wrist-lock path is reading orientation.
        device_snapshot_.ori_rad[0] = ori_rad_snap[0];
        device_snapshot_.ori_rad[1] = ori_rad_snap[1];
        device_snapshot_.ori_rad[2] = ori_rad_snap[2];
        device_snapshot_.quat[0] = quat_snap[0];
        device_snapshot_.quat[1] = quat_snap[1];
        device_snapshot_.quat[2] = quat_snap[2];
        device_snapshot_.quat[3] = quat_snap[3];
        device_snapshot_.has_orientation = true;
      }
    }
    device_snapshot_.valid = true;
  }

  // Apply forces + Cartesian torques (passthrough from external) + gripper.
  // When any actuator is held by DRD, use the drd* variant so our commands
  // compose additively with DRD's regulation output instead of overwriting it
  // each tick (which would cause motor chatter and weak hold).
  // Cartesian torques (not wrist joint torques): passing zero torque is neutral
  // at all workspace positions. SDK motor saturation handles hardware limits.
  // In joint-space mode, our PD output is already in joint torques and we'll
  // route it through dhdSetForceAndWristJointTorques* below; the Cartesian
  // tx/ty/tz here only carry external command torques.
  const double tx = cmd.torque.x + wrist_tau[0];
  const double ty = cmd.torque.y + wrist_tau[1];
  const double tz = cmd.torque.z + wrist_tau[2];

  // Skip the SDK write when commanded forces are negligible. Two reasons:
  // (1) Calling drdSetForceAndTorqueAndGripperForce at 2 kHz with zeros makes
  //     DRD interpret the override as "caller has taken over" and shut down
  //     regulation (reproduced with Test B, all gains zero).
  // (2) The PD sees micro-motion from the participant's grip (muscle tremor)
  //     and outputs tiny torques that pass a sub-1 mN threshold. Writing
  //     those to the SDK at 2 kHz makes the motors audibly whir while the
  //     handle is "held still." 5 mN / 5 mN·m is below human force
  //     perception but above ambient sensor noise — hold-still is silent,
  //     real input reaches the device unchanged.
  constexpr double kIdleForceN = 0.005;         // 5 mN
  constexpr double kIdleTorqueNm = 0.005;       // 5 mN·m
  const bool joint_space_active =
      constraints_.wrist_upright_enabled ||
      (constraints_.wrist_lock_enabled && constraints_.wrist_lock_joint_space);
  const bool joint_torques_idle =
      std::abs(wrist_joint_tau[0]) < kIdleTorqueNm &&
      std::abs(wrist_joint_tau[1]) < kIdleTorqueNm &&
      std::abs(wrist_joint_tau[2]) < kIdleTorqueNm;
  const bool idle =
      std::abs(fx) < kIdleForceN && std::abs(fy) < kIdleForceN &&
      std::abs(fz) < kIdleForceN && std::abs(tx) < kIdleTorqueNm &&
      std::abs(ty) < kIdleTorqueNm && std::abs(tz) < kIdleTorqueNm &&
      std::abs(fg) < kIdleForceN &&
      (!joint_space_active || joint_torques_idle);
  if (idle) return;

  int result = 0;
  if (joint_space_active) {
    // Joint-space mode: route the PD output as wrist joint torques so the
    // SDK does not project them through the Jacobian onto the translation
    // motors (which would fight a position hold). External Cartesian wrench
    // torques are dropped in this mode — Unity / Python publishers that need
    // to drive the wrist must switch to joint-space too.
    //   - force mode: use the dhd* variant (no DRD regulation running; the
    //     SDK hold/sphere examples use exactly this call).
    //   - legacy DRD-regulation mode: use the drd* variant so the write
    //     composes additively with DRD's translation regulation.
    result = force_mode_
        ? dhdSetForceAndWristJointTorquesAndGripperForce(
              fx, fy, fz,
              wrist_joint_tau[0], wrist_joint_tau[1], wrist_joint_tau[2],
              fg, device_id_)
        : drdSetForceAndWristJointTorquesAndGripperForce(
              fx, fy, fz,
              wrist_joint_tau[0], wrist_joint_tau[1], wrist_joint_tau[2],
              fg, device_id_);
  } else {
    // Cartesian path. Use dhd* in force mode (and whenever use_drd_api is off).
    result = (haptic_use_drd_api_ && !force_mode_)
        ? drdSetForceAndTorqueAndGripperForce(
              fx, fy, fz,
              tx, ty, tz,
              fg,
              device_id_)
        : dhdSetForceAndTorqueAndGripperForce(
              fx, fy, fz,
              tx, ty, tz,
              fg,
              device_id_);
  }
  if ((result != 0) & (result != DHD_MOTOR_SATURATED)) {
    std::string message = "Cannot set force: ";
    message += dhdErrorGetLastStr();
    Log(message);
    on_error();
  }
}

// Resets constraint homing so offsets are re-captured from the current device
// position on the next 2 kHz tick. Called by Unity when switching scenes.
void Node::rehome_constraints_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  constraints_.homed = false;
  Log("Constraints rehomed via service call.");
  response->success = true;
  response->message = "Constraint offsets will be recaptured on the next haptic tick.";
}
