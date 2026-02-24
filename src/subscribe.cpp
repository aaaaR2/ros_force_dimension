/** Copyright 2022 Neuromechatronics Lab, Carnegie Mellon University
 *
 *  Created by: a. whit. (nml@whit.contact)
 *
 *  This Source Code Form is subject to the terms of the Mozilla Public
 *  License, v. 2.0. If a copy of the MPL was not distributed with this
 *  file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

// Functionality related to receiving ROS2 messages.

// Import node header.
#include "node.hpp"

// Import the Force Dimension haptics header.
#include "dhdc.h"

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

// Applies a force+torque wrench to the robotic manipulandum.
// Reads translational forces from message.force and wrist torques from
// message.torque, so the haptic constraint node can command both.
// Gravity compensation is added automatically by the SDK on top of these values.
void Node::force_callback(const ForceMessage message) {
  auto result = hardware_disabled_
                    ? 0
                    : dhdSetForceAndTorqueAndGripperForce(
                          message.force.x, message.force.y, message.force.z,
                          message.torque.x, message.torque.y, message.torque.z,
                          0.0,
                          device_id_);
  if ((result != 0) & (result != DHD_MOTOR_SATURATED)) {
    std::string message = "Cannot set force: ";
    message += hardware_disabled_ ? "unknown error" : dhdErrorGetLastStr();
    Log(message);
    on_error();
  }
}
