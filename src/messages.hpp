/** Copyright 2022 Neuromechatronics Lab, Carnegie Mellon University (a.whit)
 *
 *  Created by: a. whit. (nml@whit.contact)
 *
 *  This Source Code Form is subject to the terms of the Mozilla Public
 *  License, v. 2.0. If a copy of the MPL was not distributed with this
 *  file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

// ROS2 message definitions.

// Include guard.
#ifndef FORCE_DIMENSION_MESSAGES_H_
#define FORCE_DIMENSION_MESSAGES_H_

// Import message types.
#include "example_interfaces/msg/float64.hpp"
#include "example_interfaces/msg/int32.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "force_dimension_msgs/msg/device_state.hpp"
#include "force_dimension_msgs/msg/raw_sample.hpp"
#include "std_msgs/msg/string.hpp"

// Declare namespace.
namespace force_dimension {

// Message type definitions.

/** Effector position ROS message type definition.
 *
 */
typedef geometry_msgs::msg::Point PositionMessage;

/** Effector velocity ROS message type definition.
 *
 */
typedef geometry_msgs::msg::Vector3 VelocityMessage;

/** Effector force+torque ROS message type definition.
 *  Uses Wrench so the haptic constraint node can command both translational
 *  forces and wrist torques in a single message.
 */
typedef geometry_msgs::msg::Wrench ForceMessage;

/** Wrist orientation ROS message type definition.
 *
 */
typedef geometry_msgs::msg::Vector3 OrientationMessage;

/** Raw Sigma.7 wrist joint angles (rad): x = w0 (roll), y = w1 (pitch),
 *  z = w2 (yaw). The actual gimbal joint rotations from
 *  dhdGetWristJointAngles — distinct from the derived Euler orientation.
 */
typedef geometry_msgs::msg::Vector3 WristJointMessage;

/** Event ROS message type definition.
 *
 */
typedef example_interfaces::msg::Int32 ButtonMessage;

/** Gripper gap ROS message type definition.
 *
 */
typedef example_interfaces::msg::Float64 GripperGapMessage;

/** Gripper angle ROS message type definition.
 *
 */
typedef example_interfaces::msg::Float64 GripperAngleMessage;

/** Synchronized device state ROS message type definition.
 *
 */
typedef force_dimension_msgs::msg::DeviceState DeviceStateMessage;

/** Full-rate raw data-collection sample ROS message type definition. */
typedef force_dimension_msgs::msg::RawSample RawSampleMessage;

/** Wrist lock-mode command/feedback ROS message type definition.
 *  Carries the mode string ("upright" | "left" | "right").
 */
typedef std_msgs::msg::String WristModeMessage;

} // namespace force_dimension

#endif // FORCE_DIMENSION_MESSAGES_H_
