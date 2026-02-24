/** Copyright 2022 Neuromechatronics Lab, Carnegie Mellon University (a.whit)
 *
 *  Created by: a. whit. (nml@whit.contact)
 *
 *  This Source Code Form is subject to the terms of the Mozilla Public
 *  License, v. 2.0. If a copy of the MPL was not distributed with this
 *  file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

// Functionality related to publishing ROS2 messages.

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

/**
 *
 *
 *
 *
 */

/** Publish state feedback. The state consists of the position, velocity, and
 *  force.
 */
void force_dimension::Node::PublishState() {
  sample_number_++;

  // NEW: Publish synchronized device state with selective metrics
  PublishDeviceState();

  // Keep existing individual publishers for backward compatibility
  PublishPosition();
  PublishButton();
  PublishGripperGap();
  PublishGripperAngle();
  PublishVelocity();
  // publish_velocity();
  // publish_force();
  // publish_button();
  // publish_orientation();
  PublishOrientation();
}

/** Publish the position of the robotic end-effector.
 *
 */
void force_dimension::Node::PublishPosition() {

  // Retrieve the position.
  double px, py, pz;
  auto result =
      hardware_disabled_ ? DHD_NO_ERROR : dhdGetPosition(&px, &py, &pz);
  if (result < DHD_NO_ERROR) {
    std::string message = "Failed to read position: ";
    message += hardware_disabled_ ? "unknown error" : dhdErrorGetLastStr();
    Log(message);
    on_error();
  }

  auto message = PositionMessage();
  message.x = px;
  message.y = py;
  message.z = pz;
  // message.sample_number = sample_number;

  // Publish.
  if (IsPublishableSample("position"))
    position_publisher_->publish(message);
}

/** Check whether or not the current data sample should be published.
 *
 */
bool force_dimension::Node::IsPublishableSample(std::string parameter_name) {

  // Decide whether or not to publish based on decimation of the sample counter.
  std::string parameter_path = "feedback_sample_decimation." + parameter_name;
  // int decimation_divisor;
  rclcpp::Parameter parameter =
      get_parameter(parameter_path); //, decimation_divisor);
  int decimation_divisor = parameter.as_int();
  bool publish = (decimation_divisor > 0)
                     ? ((sample_number_ % decimation_divisor) == 0)
                     : false;
  return publish;
}

/** Publish button events.
 *
 */
void force_dimension::Node::PublishButton() {

  // Read the button mask.
  int result = hardware_disabled_ ? 0 : dhdGetButtonMask(device_id_);

  // Prepare a button message.
  auto message = ButtonMessage();
  message.data = result;

  // Publish.
  if (IsPublishableSample("button"))
    button_publisher_->publish(message);
}

/** Publish gripper opening distance in meters.
 *
 */
void force_dimension::Node::PublishGripperGap() {

  // Read the gripper gap.
  double gap = -1;
  bool has_gripper = hardware_disabled_ ? false : dhdHasGripper(device_id_);
  int result = has_gripper ? dhdGetGripperGap(&gap, device_id_) : 0;
  if ((result != 0) & (result != DHD_TIMEGUARD)) {
    std::string message = "Failed to read gripper gap: ";
    message += hardware_disabled_ ? "unknown error" : dhdErrorGetLastStr();
    Log(message);
    on_error();
  }

  // Prepare a gripper gap message.
  // ROS2 messages have "no constructor with positional arguments for the
  // members".
  auto message = GripperGapMessage();
  message.data = gap;

  // Publish.
  if (IsPublishableSample("gripper_gap"))
    gripper_gap_publisher_->publish(message);
}

/** Publish gripper opening angle in radians.
 *
 */
void force_dimension::Node::PublishGripperAngle() {

  // Read the gripper angle.
  double angle = -1;
  bool has_gripper = hardware_disabled_ ? false : dhdHasGripper(device_id_);
  int result = has_gripper ? dhdGetGripperAngleRad(&angle, device_id_) : 0;
  if (result != 0) {
    std::string message = "Failed to read gripper angle: ";
    message += hardware_disabled_ ? "unknown error" : dhdErrorGetLastStr();
    Log(message);
    on_error();
  }

  // Prepare a gripper angle message.
  // ROS2 messages have "no constructor with positional arguments for the
  // members".
  auto message = GripperAngleMessage();
  message.data = angle;

  // Publish.
  if (IsPublishableSample("gripper_angle"))
    gripper_angle_publisher_->publish(message);
}

/** Publish the velocity of the robotic end-effector.
 *
 */
void force_dimension::Node::PublishVelocity() {

  // Retrieve the velocity.
  double vx, vy, vz;
  auto result =
      hardware_disabled_ ? DHD_NO_ERROR : dhdGetLinearVelocity(&vx, &vy, &vz);
  if (result < DHD_NO_ERROR) {
    std::string message = "Failed to read velocity: ";
    message += hardware_disabled_ ? "unknown error" : dhdErrorGetLastStr();
    Log(message);
    on_error();
  }

  // Prepare a message.
  auto message = VelocityMessage();
  message.x = vx;
  message.y = vy;
  message.z = vz;
  // message.sample_number = sample_number;

  // Publish.
  if (IsPublishableSample("velocity"))
    velocity_publisher_->publish(message);
}

/** Publish the oientation of the robotic end-effector.
 *
 */
// void force_dimension::Node::PublishOrientation() {

//   // Retrieve the angle.
//   double ax, ay, az;
//   bool has_wrist = hardware_disabled_ ? false : dhdHasWrist(device_id_);

//   auto result = has_wrist ? dhdGetOrientationRad(&ax, &ay, &az, device_) : 0;
//   if (result != 0) {
//     std::string message = "Failed to read orientation: ";
//     message += hardware_disabled_ ? "unknown error" : dhdErrorGetLastStr();
//     Log(message);
//     on_error();
//   }

//   // Prepare a message.
//   auto message = OrientationMessage();
//   message.x = ax;
//   message.y = ay;
//   message.z = az;
//   // message.sample_number = sample_number;

//   // Publish.
//   if (IsPublishableSample("orientation"))
//     orientation_publisher_->publish(message);
// }

void force_dimension::Node::PublishOrientation() {
  double px, py, pz;
  double oa, ob, og;

  bool has_wrist = hardware_disabled_ ? false : dhdHasWrist(device_id_);

  auto result =
      has_wrist ? dhdGetPositionAndOrientationRad(&px, &py, &pz, &oa, &ob, &og)
                : 0;
  if (result < 0) {
    std::string message = "Failed to read orientation: ";
    message += hardware_disabled_ ? "unknown error" : dhdErrorGetLastStr();
    Log(message);
    on_error();
  }

  auto message = OrientationMessage();
  message.x = oa;
  message.y = ob;
  message.z = og;

  if (IsPublishableSample("orientation"))
    orientation_publisher_->publish(message);
}

/** Publish synchronized device state with selective metrics.
 *
 *  This method implements selective metric publishing to optimize performance.
 *  Only metrics enabled via device_state_metrics parameters are read from hardware.
 *  Uses atomic combined DHD API calls when available (e.g., dhdGetPositionAndOrientationRad).
 */
void force_dimension::Node::PublishDeviceState() {

  // Check if we should publish this sample based on decimation
  if (!IsPublishableSample("state"))
    return;

  // Read parameters to determine which metrics to include
  bool include_position = get_parameter("device_state_metrics.include_position").as_bool();
  bool include_velocity = get_parameter("device_state_metrics.include_velocity").as_bool();
  bool include_orientation = get_parameter("device_state_metrics.include_orientation").as_bool();
  bool include_gripper = get_parameter("device_state_metrics.include_gripper").as_bool();
  bool include_buttons = get_parameter("device_state_metrics.include_buttons").as_bool();

  // Initialize message
  auto msg = DeviceStateMessage();
  msg.header.stamp = this->now();
  msg.header.frame_id = "haptic_device";

  // Set validity flags
  msg.has_position = include_position;
  msg.has_velocity = include_velocity;
  msg.has_orientation = include_orientation;
  msg.has_gripper = include_gripper;
  msg.has_buttons = include_buttons;

  // Optimization: Use atomic combined getter for position + orientation
  // This reduces DHD API calls when both are enabled
  if (include_position || include_orientation) {
    double px = 0.0, py = 0.0, pz = 0.0;
    double oa = 0.0, ob = 0.0, og = 0.0;
    bool has_wrist = hardware_disabled_ ? false : dhdHasWrist(device_id_);

    auto result = hardware_disabled_
        ? DHD_NO_ERROR
        : (has_wrist
            ? dhdGetPositionAndOrientationRad(&px, &py, &pz, &oa, &ob, &og, device_id_)
            : dhdGetPosition(&px, &py, &pz, device_id_));

    if (result < 0) {
      std::string message = "Failed to read position/orientation for device state: ";
      message += hardware_disabled_ ? "unknown error" : dhdErrorGetLastStr();
      Log(message);
    }

    // Store position data (will be zeros if hardware disabled or read failed)
    if (include_position) {
      msg.position.x = px;
      msg.position.y = py;
      msg.position.z = pz;
    }

    // Store orientation data (only if device has wrist)
    if (include_orientation && has_wrist) {
      msg.orientation.x = oa;
      msg.orientation.y = ob;
      msg.orientation.z = og;
    }
  }

  // Read velocity if enabled
  if (include_velocity) {
    double vx = 0.0, vy = 0.0, vz = 0.0;
    auto result = hardware_disabled_
        ? DHD_NO_ERROR
        : dhdGetLinearVelocity(&vx, &vy, &vz, device_id_);

    if (result < 0) {
      Log("Failed to read velocity for device state");
    }

    msg.velocity.x = vx;
    msg.velocity.y = vy;
    msg.velocity.z = vz;
  }

  // Read gripper data if enabled
  if (include_gripper) {
    double gap = 0.0, angle = 0.0;
    bool has_gripper = hardware_disabled_ ? false : dhdHasGripper(device_id_);

    if (has_gripper) {
      // Read gripper gap
      int result = dhdGetGripperGap(&gap, device_id_);
      if ((result != 0) && (result != DHD_TIMEGUARD)) {
        Log("Failed to read gripper gap for device state");
        gap = 0.0;
      }

      // Read gripper angle
      result = dhdGetGripperAngleRad(&angle, device_id_);
      if (result != 0) {
        Log("Failed to read gripper angle for device state");
        angle = 0.0;
      }
    }

    msg.gripper_gap = gap;
    msg.gripper_angle = angle;
  }

  // Read button state if enabled
  if (include_buttons) {
    int button_mask = hardware_disabled_ ? 0 : dhdGetButtonMask(device_id_);
    msg.button_mask = button_mask;
  }

  // Publish the message
  device_state_publisher_->publish(msg);
}

#endif // FORCE_DIMENSION_PUBLISH_H_