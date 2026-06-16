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

  // Publishes raw wrist joint angles (w0,w1,w2) in radians.
  void PublishWristJoints(void);

  // Publishes synchronized device state with selective metrics.
  void PublishDeviceState(void);

  // Publishes the applied wrist counteracting torque (joint-space) as a Wrench.
  void PublishAppliedForce(void);

  // Publishes the full-rate raw data-collection sample (sample-time stamped).
  void PublishRawSample(void);

  // Subscribes to ROS messages that indicate an instantaneous force to be
  // applied to the robot endpoint.
  void SubscribeForce(void);

  // Subscribes to ROS messages that indicate an instantaneous force to be
  // applied to the gripper by the robot.
  void SubscribeGripperForce(void);

  // Stores the latest force command received via ROS for the haptic loop.
  void force_callback(const ForceMessage);

  // Subscribes to wrist lock-mode commands (upright / left / right) so the
  // locked roll orientation can be switched live from Unity.
  void SubscribeWristMode(void);

  // Stores the latest wrist lock mode by setting the target roll offset.
  void wrist_mode_callback(const WristModeMessage);

  // Maps a lock-mode string ("upright"|"left"|"right") to a roll offset (rad).
  // Returns the upright offset (0) and logs a warning for unknown strings.
  double mode_to_offset(const std::string &mode) const;

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

  // Snapshot of the latest device state captured by the haptic thread.
  // Read by ROS publish callbacks under state_mutex_ — never call SDK
  // getters from the executor thread (it contends with the haptic loop's
  // 2 kHz SDK calls and causes DRD regulation drops, especially when
  // ROS-TCP-Endpoint serializes messages for Unity).
  struct DeviceSnapshot {
    double pos[3] = {0.0, 0.0, 0.0};
    double vel[3] = {0.0, 0.0, 0.0};
    double ori_rad[3] = {0.0, 0.0, 0.0};   // wrist Euler angles (rad)
    double omega[3] = {0.0, 0.0, 0.0};     // angular velocity (rad/s)
    double wrist_joint_rad[3] = {0.0, 0.0, 0.0};  // raw joint angles: w0,w1,w2 (rad)
    double gripper_gap_m = 0.0;
    double gripper_angle_rad = 0.0;
    int    button_mask = 0;
    // Commanded output this tick (for the raw data-collection record).
    double applied_force[3] = {0.0, 0.0, 0.0};   // Cartesian force fx,fy,fz (N)
    double applied_torque[3] = {0.0, 0.0, 0.0};  // wrist joint torque w0,w1,w2 (N*m)
    // Sample time captured in the haptic loop (ROS clock, nanoseconds). 0 until
    // the first tick. Used as RawSample/DeviceState header.stamp so the stamp
    // is the SAMPLE time, not the (later, jittery) publish time.
    int64_t sample_stamp_ns = 0;
    bool   has_orientation = false;
    bool   has_wrist_joint = false;
    bool   has_gripper = false;
    bool   valid = false;                  // false until first haptic tick lands
  };

private:
  int device_id_;
  float publication_interval_s_;
  bool active_;
  bool configured_;
  int sample_number_;
  bool hardware_disabled_;
  bool haptic_use_drd_api_ = false;  // true: drd* force API (composes with DRD regulation)
  // Force-mode hold (SDK hold/sphere example pattern): after centering with DRD,
  // drdStop(true) and hold every axis in the 2 kHz loop using ONLY dhd* force
  // functions. Eliminates the DRD regulation thread entirely (no "regulation
  // thread stopped" drops). Translation hold comes from channel constraints,
  // gripper from the fg spring, wrist from the joint-space PD. Default false
  // preserves the legacy DRD-regulation behavior.
  bool force_mode_ = false;
  double baseline_effector_mass_kg_;
  double home_gripper_gap_;
  ConstraintState constraints_;        // cached workspace constraint parameters
  ForceMessage last_force_command_;    // additive external forces (zero-order hold), guarded by force_mutex_
  // Latest applied wrist JOINT torque (roll=0,pitch=1,yaw=2, N*m) computed by the
  // 2 kHz/~4 kHz haptic thread, read by the executor thread in PublishAppliedForce().
  // Guarded by force_mutex_ (same boundary-crossing pattern as last_force_command_).
  double applied_wrist_torque_[3] = {0.0, 0.0, 0.0};
  bool   applied_wrist_torque_valid_ = false;  // false until first haptic tick stashes a value
  mutable std::mutex force_mutex_;     // protects last_force_command_ between ROS and haptic threads
  // Target roll offset for the wrist lock mode (rad). Written by the mode
  // topic/param callbacks on the executor thread, read by the 2 kHz haptic
  // loop. A single lock-free scalar — no mutex needed (cf. force_mutex_ which
  // guards a multi-field message).
  std::atomic<double> wrist_roll_offset_target_{0.0};
  double wrist_roll_offset_left_  = 0.0;   // cached "left"  mode offset (rad)
  double wrist_roll_offset_right_ = 0.0;   // cached "right" mode offset (rad)
  DeviceSnapshot device_snapshot_;     // populated by haptic thread, read by publish callbacks
  mutable std::mutex state_mutex_;     // protects device_snapshot_
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr rehome_service_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::thread haptic_thread_;              // SDK-paced haptic loop (tight busy-loop)
  std::atomic<bool> haptic_running_{false};
  rclcpp::Publisher<PositionMessage>::SharedPtr position_publisher_;
  rclcpp::Publisher<ButtonMessage>::SharedPtr button_publisher_;
  rclcpp::Publisher<GripperGapMessage>::SharedPtr gripper_gap_publisher_;
  rclcpp::Publisher<GripperAngleMessage>::SharedPtr gripper_angle_publisher_;
  rclcpp::Publisher<VelocityMessage>::SharedPtr velocity_publisher_;
  rclcpp::Publisher<ForceMessage>::SharedPtr wrench_publisher_;
  rclcpp::Subscription<ForceMessage>::SharedPtr force_subscription_;
  rclcpp::Subscription<WristModeMessage>::SharedPtr wrist_mode_subscription_;
  rclcpp::Publisher<WristModeMessage>::SharedPtr wrist_mode_publisher_;
  rclcpp::Publisher<OrientationMessage>::SharedPtr orientation_publisher_;
  rclcpp::Publisher<WristJointMessage>::SharedPtr wrist_joint_publisher_;
  rclcpp::Publisher<DeviceStateMessage>::SharedPtr device_state_publisher_;
  rclcpp::Publisher<RawSampleMessage>::SharedPtr raw_sample_publisher_;
  OnSetParametersCallbackHandle::SharedPtr parameters_callback_handle_;
};

} // namespace force_dimension

// Include guard.
#endif
