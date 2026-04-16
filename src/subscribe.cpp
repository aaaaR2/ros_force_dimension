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
  static int drd_drop_count = 0;
  if (!drdIsRunning()) {
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
  double ang[3] = {};    // wrist Euler angles (rad)
  double omega[3] = {};  // Cartesian angular velocity (rad/s)
  bool   got_orientation = false;
  if (constraints_.wrist_lock_enabled) {
    double pg = 0.0;       // gripper gap (unused here)
    double R[3][3] = {};   // rotation matrix (unused here)
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
  double wrist_tau[3] = {0.0, 0.0, 0.0};
  if (constraints_.wrist_lock_enabled && got_orientation) {
    if (!constraints_.wrist_homed) {
      for (int i = 0; i < 3; ++i) constraints_.wrist_home_angles[i] = ang[i];
      constraints_.wrist_homed = true;
    }
    const int free_ax = constraints_.wrist_lock_free_axis;
    const double Kp = constraints_.wrist_lock_stiffness;
    const double Kv = constraints_.wrist_lock_damping;
    const double b_free = constraints_.wrist_free_axis_damping;
    // For small-to-moderate rotations, each Cartesian torque axis maps to
    // the Euler component with the same index. PD-lock the non-free axes,
    // apply viscous damping on the free axis using the SDK's hardware-rate
    // angular velocity. Low-pass filter the free-axis omega so high-b
    // settings don't amplify encoder-quantization noise into tremor.
    double alpha = constraints_.wrist_free_axis_filter_alpha;
    if (alpha < 0.0) alpha = 0.0;
    if (alpha > 1.0) alpha = 1.0;
    constraints_.wrist_free_axis_omega_filt =
        alpha * omega[free_ax] +
        (1.0 - alpha) * constraints_.wrist_free_axis_omega_filt;
    const double err_deadband = constraints_.wrist_lock_error_deadband;
    for (int i = 0; i < 3; ++i) {
      if (i == free_ax) {
        wrist_tau[i] = -b_free * constraints_.wrist_free_axis_omega_filt;
      } else {
        double err = ang[i] - constraints_.wrist_home_angles[i];
        // Deadband: inside this zone, the PD outputs zero torque so small
        // grip offsets do not drive the SDK at 2 kHz with a constant force
        // (which produces an audible low-amplitude motor hum).
        if (std::abs(err) <= err_deadband) {
          wrist_tau[i] = 0.0;
        } else {
          // Shrink the error by the deadband so there's no discontinuity at
          // the zone boundary (torque ramps smoothly from zero).
          err = err > 0.0 ? err - err_deadband : err + err_deadband;
          wrist_tau[i] = -Kp * err - Kv * omega[i];
        }
      }
    }
  }

  // Gripper constraint: dead-zone stiffness (one-sided wall at home gap).
  double fg = 0.0;
  {
    constexpr double Kgp          = 500.0;  // N/m
    constexpr double MaxGripperForce = 10.0;  // N — closing cap
    double currentGap = 0.0;
    dhdGetGripperGap(&currentGap, device_id_);
    const double depth = currentGap - home_gripper_gap_;
    if (depth > 0.0) {
      fg = -Kgp * depth;
      if (fg < -MaxGripperForce) fg = -MaxGripperForce;
    }
  }

  // Apply forces + Cartesian torques (passthrough from external) + gripper.
  // When any actuator is held by DRD, use the drd* variant so our commands
  // compose additively with DRD's regulation output instead of overwriting it
  // each tick (which would cause motor chatter and weak hold).
  // Cartesian torques (not wrist joint torques): passing zero torque is neutral
  // at all workspace positions. SDK motor saturation handles hardware limits.
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
  const bool idle =
      std::abs(fx) < kIdleForceN && std::abs(fy) < kIdleForceN &&
      std::abs(fz) < kIdleForceN && std::abs(tx) < kIdleTorqueNm &&
      std::abs(ty) < kIdleTorqueNm && std::abs(tz) < kIdleTorqueNm &&
      std::abs(fg) < kIdleForceN;
  if (idle) return;

  auto result =
      haptic_use_drd_api_
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
