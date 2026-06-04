/** Copyright 2022 Neuromechatronics Lab, Carnegie Mellon University (a.whit)
 *
 *  Created by: a. whit. (nml@whit.contact)
 *
 *  This Source Code Form is subject to the terms of the Mozilla Public
 *  License, v. 2.0. If a copy of the MPL was not distributed with this
 *  file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

// Functionality related to publishing ROS2 messages.
//
// Architectural note: every Publish*() callback below reads from
// device_snapshot_ under state_mutex_ — none of them call SDK getters.
// All SDK reads happen on the haptic thread (subscribe.cpp::ApplyHapticForce),
// which stashes the latest device state into device_snapshot_ once per tick.
// Without this separation, the executor thread's SDK calls contend with
// the haptic thread's 2 kHz SDK calls and cause DRD regulation drops,
// especially when ROS-TCP-Endpoint serializes messages for Unity.

// Include guard.
#ifndef FORCE_DIMENSION_PUBLISH_H_
#define FORCE_DIMENSION_PUBLISH_H_

// Import the node header.
#include "node.hpp"

// Import the ROS interface.
#include "rclcpp/rclcpp.hpp"

// Import the Force Dimension haptics library.
#include "dhdc.h"

// Import package headers.
#include "messages.hpp"

void force_dimension::Node::PublishState() {
  sample_number_++;

  // Snapshot the device state once per executor tick. Each Publish*()
  // call below copies from this local snapshot rather than locking and
  // re-reading every time, which keeps lock hold time tiny.
  PublishDeviceState();
  PublishPosition();
  PublishButton();
  PublishGripperGap();
  PublishGripperAngle();
  PublishVelocity();
  PublishOrientation();
  PublishWristJoints();
}

void force_dimension::Node::PublishPosition() {
  if (!IsPublishableSample("position")) return;
  DeviceSnapshot snap;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    snap = device_snapshot_;
  }
  if (!snap.valid) return;  // haptic thread has not produced a sample yet

  auto message = PositionMessage();
  message.x = snap.pos[0];
  message.y = snap.pos[1];
  message.z = snap.pos[2];
  position_publisher_->publish(message);
}

void force_dimension::Node::PublishButton() {
  if (!IsPublishableSample("button")) return;
  DeviceSnapshot snap;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    snap = device_snapshot_;
  }
  if (!snap.valid) return;

  auto message = ButtonMessage();
  message.data = snap.button_mask;
  button_publisher_->publish(message);
}

void force_dimension::Node::PublishGripperGap() {
  if (!IsPublishableSample("gripper_gap")) return;
  DeviceSnapshot snap;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    snap = device_snapshot_;
  }
  if (!snap.valid) return;

  auto message = GripperGapMessage();
  message.data = snap.has_gripper ? snap.gripper_gap_m : -1.0;
  gripper_gap_publisher_->publish(message);
}

void force_dimension::Node::PublishGripperAngle() {
  if (!IsPublishableSample("gripper_angle")) return;
  DeviceSnapshot snap;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    snap = device_snapshot_;
  }
  if (!snap.valid) return;

  auto message = GripperAngleMessage();
  message.data = snap.has_gripper ? snap.gripper_angle_rad : -1.0;
  gripper_angle_publisher_->publish(message);
}

void force_dimension::Node::PublishVelocity() {
  if (!IsPublishableSample("velocity")) return;
  DeviceSnapshot snap;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    snap = device_snapshot_;
  }
  if (!snap.valid) return;

  auto message = VelocityMessage();
  message.x = snap.vel[0];
  message.y = snap.vel[1];
  message.z = snap.vel[2];
  velocity_publisher_->publish(message);
}

void force_dimension::Node::PublishOrientation() {
  if (!IsPublishableSample("orientation")) return;
  DeviceSnapshot snap;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    snap = device_snapshot_;
  }
  if (!snap.valid || !snap.has_orientation) return;

  auto message = OrientationMessage();
  message.x = snap.ori_rad[0];
  message.y = snap.ori_rad[1];
  message.z = snap.ori_rad[2];
  orientation_publisher_->publish(message);
}

void force_dimension::Node::PublishWristJoints() {
  if (!IsPublishableSample("wrist_joint_angles")) return;
  DeviceSnapshot snap;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    snap = device_snapshot_;
  }
  if (!snap.valid || !snap.has_wrist_joint) return;

  // Raw Sigma.7 wrist joint angles (rad): x = w0 (roll), y = w1 (pitch),
  // z = w2 (yaw). Distinct from the derived Euler orientation.
  auto message = WristJointMessage();
  message.x = snap.wrist_joint_rad[0];
  message.y = snap.wrist_joint_rad[1];
  message.z = snap.wrist_joint_rad[2];
  wrist_joint_publisher_->publish(message);
}

bool force_dimension::Node::IsPublishableSample(std::string parameter_name) {
  std::string parameter_path = "feedback_sample_decimation." + parameter_name;
  rclcpp::Parameter parameter = get_parameter(parameter_path);
  int decimation_divisor = parameter.as_int();
  return (decimation_divisor > 0)
             ? ((sample_number_ % decimation_divisor) == 0)
             : false;
}

void force_dimension::Node::PublishDeviceState() {
  if (!IsPublishableSample("state")) return;

  DeviceSnapshot snap;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    snap = device_snapshot_;
  }
  if (!snap.valid) return;

  bool include_position = get_parameter("device_state_metrics.include_position").as_bool();
  bool include_velocity = get_parameter("device_state_metrics.include_velocity").as_bool();
  bool include_orientation = get_parameter("device_state_metrics.include_orientation").as_bool();
  bool include_gripper = get_parameter("device_state_metrics.include_gripper").as_bool();
  bool include_buttons = get_parameter("device_state_metrics.include_buttons").as_bool();

  auto msg = DeviceStateMessage();
  msg.header.stamp = this->now();
  msg.header.frame_id = "haptic_device";

  msg.has_position = include_position;
  msg.has_velocity = include_velocity;
  msg.has_orientation = include_orientation && snap.has_orientation;
  msg.has_gripper = include_gripper && snap.has_gripper;
  msg.has_buttons = include_buttons;

  if (include_position) {
    msg.position.x = snap.pos[0];
    msg.position.y = snap.pos[1];
    msg.position.z = snap.pos[2];
  }
  if (include_orientation && snap.has_orientation) {
    msg.orientation.x = snap.ori_rad[0];
    msg.orientation.y = snap.ori_rad[1];
    msg.orientation.z = snap.ori_rad[2];
  }
  if (include_velocity) {
    msg.velocity.x = snap.vel[0];
    msg.velocity.y = snap.vel[1];
    msg.velocity.z = snap.vel[2];
  }
  if (include_gripper && snap.has_gripper) {
    msg.gripper_gap = snap.gripper_gap_m;
    msg.gripper_angle = snap.gripper_angle_rad;
  }
  if (include_buttons) {
    msg.button_mask = snap.button_mask;
  }

  device_state_publisher_->publish(msg);
}

#endif // FORCE_DIMENSION_PUBLISH_H_
