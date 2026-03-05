/** Copyright 2022 Neuromechatronics Lab, Carnegie Mellon University (a.whit)
 *
 *  Created by: a. whit. (nml@whit.contact)
 *
 *  This Source Code Form is subject to the terms of the Mozilla Public
 *  License, v. 2.0. If a copy of the MPL was not distributed with this
 *  file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

// Interface between ROS2 and the Force Dimension SDK for haptic robots.

// Import essential libraries
#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// Include guard.
#ifndef FORCE_DIMENSION_NODE_H_
#define FORCE_DIMENSION_NODE_H_

// Import the ROS interface.
#include "rclcpp/rclcpp.hpp"

// Import message types.
#include "messages.hpp"

// Import workspace constraint state.
#include "constraints.hpp"

// Import the parameter management message info.
#include "rcl_interfaces/msg/set_parameters_result.hpp"

// Import service type for rehome trigger.
#include "std_srvs/srv/trigger.hpp"

/** The ForceDimension namespace.
 */
namespace force_dimension {

/** A ROS2 node for interfacing with a Force Dimension haptic robot.
 *
 *  Periodically emits ROS2 messages containing the sampled position of a Force
 *  Dimension robotic manipulandum (e.g., delta.3, sigma.7, or Novint Falcon).
 *  Accepts ROS2 messages that contain an instantaneous force, or vibration, to
 *  be applied to the manipulandum.
 */
class Node : public rclcpp::Node {

public:
  // Constructor.
  Node(bool, bool);

  // Destructor.
  ~Node();

  // Configures the ROS node by creating publishers, subscriptions, and
  // initializing parameters.
  void on_configure(void);

  // Activates the ROS node by initializing the Force Dimension interface and
  // the publication timer / callback.
  void on_activate(void);

  //
  void on_error(void);

  //
  void on_deactivate(void);

  //  This method is expected to clear all state and return the node to a
  // functionally equivalent state as when first created.
  // transition to Unconfigured.
  void on_cleanup(void);

  //
  // on_shutdown

private:
  //
  // void Log(const char *);
  void Log(std::string);

  // Publishes robot state feedback.
  void PublishState(void);

  // Publishes robot position messages.
  void PublishPosition(void);

  // Publishes robot button messages.
  void PublishButton(void);

  // Publish gripper opening distance in meters.
  void PublishGripperGap(void);

  // Publish gripper opening angle in radians.
  void PublishGripperAngle(void);

  // Publishes robot velocity messages.
  void PublishVelocity(void);

  // Publishes wrist orientation in radians.
  void PublishOrientation(void);

  // Publishes synchronized device state with selective metrics.
  void PublishDeviceState(void);

  //// Publishes robot force messages.
  // void PublishForce(void);

  // Subscribes to ROS messages that indicate an instantaneous force to be
  // applied to the robot endpoint.
  void SubscribeForce(void);

  // Subscribes to ROS messages that indicate an instantaneous force to be
  // applied to the gripper by the robot.
  void SubscribeGripperForce(void);

  // Stores the latest force command received via ROS for the haptic loop.
  void force_callback(const ForceMessage);

  // Applies workspace constraint forces + external commands to the device at 2 kHz.
  void ApplyHapticForce(void);

  // Computes channel + circle dead-zone constraint forces from SDK position/velocity.
  void ComputeConstraintForce(const double pos[3], const double vel[3],
                               double force_out[3]);

  // Service handler: resets constraint homing so offsets are re-captured on the
  // next 2 kHz tick. Unity calls this when switching scenes.
  void rehome_constraints_callback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  // Check whether or not the current data sample should be published.
  bool IsPublishableSample(std::string);

  // Set effector mass.
  double get_effector_mass(void);
  void set_effector_mass(double mass_kg = -1);
  void reset_effector_mass(void);

  // Enable and disable gravity compensation.
  void set_gravity_compensation(bool);
  void set_gravity_compensation();

  // Enable and disable forces.
  void set_enable_force(bool);
  void set_enable_force();

  // Parameters set callback.
  rcl_interfaces::msg::SetParametersResult
  set_parameters_callback(const std::vector<rclcpp::Parameter> &);

private:
  int device_id_;
  float publication_interval_s_;
  bool active_;
  bool configured_;
  int sample_number_;
  bool hardware_disabled_;
  double baseline_effector_mass_kg_;
  double home_gripper_gap_;
  ConstraintState constraints_;        // cached workspace constraint parameters
  ForceMessage last_force_command_;    // additive external forces (zero-order hold), guarded by force_mutex_
  mutable std::mutex force_mutex_;     // protects last_force_command_ between ROS and haptic threads
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr rehome_service_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::thread haptic_thread_;              // SDK-paced haptic loop (tight busy-loop)
  std::atomic<bool> haptic_running_{false};
  rclcpp::Publisher<PositionMessage>::SharedPtr position_publisher_;
  rclcpp::Publisher<ButtonMessage>::SharedPtr button_publisher_;
  rclcpp::Publisher<GripperGapMessage>::SharedPtr gripper_gap_publisher_;
  rclcpp::Publisher<GripperAngleMessage>::SharedPtr gripper_angle_publisher_;
  rclcpp::Publisher<VelocityMessage>::SharedPtr velocity_publisher_;
  // rclcpp::Publisher<ForceMessage>::SharedPtr force_publisher_;
  rclcpp::Subscription<ForceMessage>::SharedPtr force_subscription_;
  rclcpp::Publisher<OrientationMessage>::SharedPtr orientation_publisher_;
  rclcpp::Publisher<DeviceStateMessage>::SharedPtr device_state_publisher_;
  OnSetParametersCallbackHandle::SharedPtr parameters_callback_handle_;
};

} // namespace force_dimension

// Include guard.
#endif
