/** Copyright 2022 Neuromechatronics Lab, Carnegie Mellon University
 *
 *  Created by: a. whit. (nml@whit.contact)
 *
 *  This Source Code Form is subject to the terms of the Mozilla Public
 *  License, v. 2.0. If a copy of the MPL was not distributed with this
 *  file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

// Functionality related to receiving ROS2 messages and applying haptic forces.

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

  // Read position and velocity directly from the SDK (no ROS transport).
  double pos[3] = {};
  double vel[3] = {};
  dhdGetPosition(&pos[0], &pos[1], &pos[2], device_id_);
  dhdGetLinearVelocity(&vel[0], &vel[1], &vel[2], device_id_);

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
  // Uses dhdSetForceAndTorqueAndGripperForce (Cartesian torques, not wrist joint torques)
  // so that passing zero torque is truly neutral at all workspace positions.
  // No explicit force clamping — SDK motor saturation handles hardware limits.
  auto result = dhdSetForceAndTorqueAndGripperForce(
                    fx, fy, fz,
                    cmd.torque.x,
                    cmd.torque.y,
                    cmd.torque.z,
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
