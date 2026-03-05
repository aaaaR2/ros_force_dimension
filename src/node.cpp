/** Copyright 2022 Neuromechatronics Lab, Carnegie Mellon University (a.whit)
 *
 *  Created by: a. whit. (nml@whit.contact)
 *
 *  This Source Code Form is subject to the terms of the Mozilla Public
 *  License, v. 2.0. If a copy of the MPL was not distributed with this
 *  file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

// Implementation of an interface between ROS2 and the Force Dimension SDK for
// haptic robots.

// Import the node header.
#include "node.hpp"

// Import the ROS interface.
#include "rclcpp/rclcpp.hpp"

// Import the Force Dimension haptics library.
#include "dhdc.h"

// Import the Force Dimension robotics library (required for DRD autocenter
// and joint torque control initialisation).
#include "drdc.h"

// Import package headers.
#include "qos.hpp"
#include "topics.hpp"
#include "std_srvs/srv/trigger.hpp"

// Select namespace.
// using namespace force_dimension;
using force_dimension::Node;

// Constructor.
Node::Node(bool configure = true, bool activate = true)
    : rclcpp::Node("force_dimension", "robot") {
  // Initialize default values.
  device_id_ = -1;
  active_ = false;
  configured_ = false;

  // Configure and activate, if requested.
  configure = activate ? true : configure;
  if (configure)
    on_configure();
  if (activate)
    on_activate();
}

// Destructor.
Node::~Node() {
  if (active_)
    on_deactivate();
  if (configured_)
    on_cleanup();
}

// Configures the ROS node by creating publishers, subscriptions, and
// initializing parameters.
void Node::on_configure(void) {

  // Use the default ROS2 Quality-of-Service.
  auto qos = DefaultQoS();

  // Create the position state publisher.
  auto topic = POSITION_FEEDBACK_TOPIC;
  position_publisher_ = create_publisher<PositionMessage>(topic, qos);

  // Create the button state publisher.
  topic = BUTTON_FEEDBACK_TOPIC;
  button_publisher_ = create_publisher<ButtonMessage>(topic, qos);

  // Create the gripper gap state publisher.
  topic = GRIPPER_GAP_FEEDBACK_TOPIC;
  gripper_gap_publisher_ = create_publisher<GripperGapMessage>(topic, qos);

  // Create the gripper angle state publisher.
  topic = GRIPPER_ANGLE_FEEDBACK_TOPIC;
  gripper_angle_publisher_ = create_publisher<GripperAngleMessage>(topic, qos);

  // Create the velocity state publisher.
  topic = VELOCITY_FEEDBACK_TOPIC;
  velocity_publisher_ = create_publisher<VelocityMessage>(topic, qos);

  // Create the orientation state publisher.
  topic = ORIENTATION_FEEDBACK_TOPIC;
  orientation_publisher_ = create_publisher<OrientationMessage>(topic, qos);

  // Create the synchronized device state publisher.
  topic = DEVICE_STATE_FEEDBACK_TOPIC;
  device_state_publisher_ = create_publisher<DeviceStateMessage>(topic, qos);

  //// Create the force state publisher.
  // topic = FORCE_FEEDBACK_TOPIC;
  // force_publisher = create_publisher<force_message>(topic, qos);

  // Initialize ROS2 parameters.
  declare_parameter<float>("sample_interval_s", 0.025);
  declare_parameter<bool>("disable_hardware", false);
  declare_parameter<bool>("gripper.emulate_button", false);
  declare_parameter<int>("feedback_sample_decimation.position", 50);
  declare_parameter<int>("feedback_sample_decimation.velocity", 50);
  declare_parameter<int>("feedback_sample_decimation.button", 50);
  declare_parameter<int>("feedback_sample_decimation.gripper_gap", 50);
  declare_parameter<int>("feedback_sample_decimation.gripper_angle", 50);
  declare_parameter<int>("feedback_sample_decimation.orientation", 50);
  declare_parameter<int>("feedback_sample_decimation.state", 50);
  declare_parameter<bool>("device_state_metrics.include_position", true);
  declare_parameter<bool>("device_state_metrics.include_velocity", true);
  declare_parameter<bool>("device_state_metrics.include_orientation", true);
  declare_parameter<bool>("device_state_metrics.include_gripper", true);
  declare_parameter<bool>("device_state_metrics.include_buttons", true);
  declare_parameter<float>("effector_mass_kg", 0.190000);
  declare_parameter<bool>("gravity_compensation", true);
  declare_parameter<bool>("enable_force", true);

  // Workspace constraint parameters (cached in constraints_ for the 2 kHz loop).
  // Per-axis channel constraints (x=0, y=1, z=2).
  declare_parameter<bool>("constraints.channel_x.enabled", false);
  declare_parameter<double>("constraints.channel_x.half_width", 0.003);
  declare_parameter<bool>("constraints.channel_y.enabled", false);
  declare_parameter<double>("constraints.channel_y.half_width", 0.003);
  declare_parameter<bool>("constraints.channel_z.enabled", false);
  declare_parameter<double>("constraints.channel_z.half_width", 0.003);
  // Circle constraint.
  declare_parameter<bool>("constraints.circle.enabled", false);
  declare_parameter<double>("constraints.circle.radius", 0.11);
  declare_parameter<double>("constraints.home_offset_0", 0.0);  // startup/home offset along circle plane axis 0
  declare_parameter<double>("constraints.home_offset_1", 0.0);  // startup/home offset along circle plane axis 1
  // Shared spring-damper.
  declare_parameter<double>("constraints.stiffness", 2000.0);
  declare_parameter<double>("constraints.damping", 50.0);

  // Create the force control subcription.
  SubscribeForce();

  // Advertise rehome service — resets constraint homing on the next 2 kHz tick.
  rehome_service_ = create_service<std_srvs::srv::Trigger>(
      "~/rehome_constraints",
      std::bind(&Node::rehome_constraints_callback, this,
                std::placeholders::_1, std::placeholders::_2));
}

/** Activates the ROS node by initializing the Force Dimension interface and
 *  the publication timer / callback.
 */
void Node::on_activate(void) {

  // Check to see if hardware has been disabled.
  // This is done once, at the time of activation, and stored in a member
  // variable while active.
  get_parameter("disable_hardware", hardware_disabled_);

  // Open device, autocenter, and enable expert mode for joint torque control.
  // Follows the hold.cpp + autocenter.cpp SDK example pattern.
  Log("Initializing the Force Dimension interface.");
  if (hardware_disabled_) {
    device_id_ = 999;
    home_gripper_gap_ = 0.05;  // 5 cm default for hardware-disabled mode
    Log("Hardware disabled: skipping DRD initialisation.");
  } else {
    // Expert mode must be enabled before opening the device.
    dhdEnableExpertMode();

    // Open device via DRD (required for autocenter and joint torque control).
    device_id_ = drdOpen();
    if (device_id_ < 0) {
      std::string message = "Cannot open Force Dimension device: ";
      message += dhdErrorGetLastStr();
      Log(message);
      on_error();
    }

    if (!drdIsSupported()) {
      Log("Device does not support DRD robotics library.");
      on_error();
    }

    {
      std::string message = "Force Dimension device detected: ";
      message += dhdGetSystemName();
      Log(message);
    }

    // Initialise device if not already done (requires holding near centre).
    if (!drdIsInitialized() && drdAutoInit() < 0) {
      std::string message = "Cannot initialise device: ";
      message += dhdErrorGetLastStr();
      Log(message);
      on_error();
    }

    // Stop any running regulation so we can reconfigure regulate flags.
    drdStop(false);

    // Centre base + wrist; leave gripper at its natural position.
    drdRegulatePos(1);
    drdRegulateRot(dhdHasActiveWrist());
    drdRegulateGrip(0);  // do not centre gripper

    if (drdStart() < 0) {
      std::string message = "Cannot start DRD regulation: ";
      message += dhdErrorGetLastStr();
      Log(message);
      on_error();
    }

    // Move base + wrist to startup position.
    // If circle constraints are configured with a center offset, move to that
    // offset position so the device starts at the circle center.
    double positionCenter[DHD_MAX_DOF] = {};
    {
      // Determine circle plane axes from the first enabled channel (same logic
      // as ComputeConstraintForce auto-home). Read params directly here since
      // constraints_ has not been cached yet.
      bool cx = get_parameter("constraints.channel_x.enabled").as_bool();
      bool cy = get_parameter("constraints.channel_y.enabled").as_bool();
      int primary = cx ? 0 : (cy ? 1 : 2);
      int ax0 = (primary == 0) ? 1 : 0;
      int ax1 = (primary == 2) ? 1 : 2;
      // Clamp offsets to safe Cartesian range for the Sigma.7 workspace.
      // Values beyond ±8 cm risk hitting joint limits and causing DRD errors.
      constexpr double kMaxOffsetM = 0.08;
      auto clamp = [](double v, double lo, double hi) {
        return v < lo ? lo : (v > hi ? hi : v);
      };
      double off0 = clamp(
          get_parameter("constraints.home_offset_0").as_double(),
          -kMaxOffsetM, kMaxOffsetM);
      double off1 = clamp(
          get_parameter("constraints.home_offset_1").as_double(),
          -kMaxOffsetM, kMaxOffsetM);
      positionCenter[ax0] = off0;
      positionCenter[ax1] = off1;
      {
        std::string message = "Moving to startup position: axis";
        message += std::to_string(ax0) + "=" + std::to_string(off0);
        message += " axis" + std::to_string(ax1) + "=" + std::to_string(off1);
        Log(message);
      }
    }
    if (drdMoveTo(positionCenter, true) < 0) {
      std::string message = "Cannot move to centre: ";
      message += dhdErrorGetLastStr();
      Log(message);
      on_error();
    }

    // Stop regulation but keep forces enabled for the haptic loop.
    if (drdStop(true) < 0) {
      std::string message = "Cannot stop DRD regulation: ";
      message += dhdErrorGetLastStr();
      Log(message);
      on_error();
    }

    // Store gripper gap at natural resting position as the home target.
    dhdGetGripperGap(&home_gripper_gap_, device_id_);
    {
      std::string message = "Autocenter complete. Home gripper gap: ";
      message += std::to_string(home_gripper_gap_);
      message += " m";
      Log(message);
    }
  }

  // Enable button emulation, if requested.
  unsigned char val =
      get_parameter("gripper.emulate_button").as_bool() ? DHD_ON : DHD_OFF;
  int result = hardware_disabled_ ? 0 : dhdEmulateButton(val, device_id_);
  if (result != 0) {
    std::string message = "Button emulation failure: ";
    // message += dhdErrorGetLastStr();
    message += hardware_disabled_ ? "unknown error" : dhdErrorGetLastStr();
    Log(message);
    on_error();
  }

  // Apply zero force and zero wrist joint torques.
  result = hardware_disabled_ ? DHD_NO_ERROR
                              : dhdSetForceAndWristJointTorquesAndGripperForce(
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  if (result < DHD_NO_ERROR) {
    std::string message = "Cannot set force: ";
    // message += dhdErrorGetLastStr();
    message += hardware_disabled_ ? "unknown error" : dhdErrorGetLastStr();
    Log(message);
    on_error();
  } else
    Log("Force Dimension interface initialized.");

  // Report the BASELINE communication refresh rate.
  result = hardware_disabled_ ? -1.0 : dhdGetComFreq(device_id_);
  if (result == 0.0) {
    std::string message = "Failure getting communication refresh rate: ";
    message += hardware_disabled_ ? "unknown error" : dhdErrorGetLastStr();
    Log(message);
  } else {
    std::string message = "Communication refresh rate: ";
    message += std::to_string(result);
    message += " kHz";
    Log(message);
  }

  // Initialize the publication interval variable and create the timer.
  // Sets the ROS publication interval for state messages.
  // The sampling rate should be set low enough that any receiving nodes
  // (e.g., a GUI), as well as the ROS system, can handle the message volume.
  auto callback = [this]() { this->PublishState(); };
  float sample_interval_s;
  if (get_parameter("sample_interval_s", sample_interval_s)) {
    int publication_interval_ns = round(sample_interval_s * 1e9);
    std::chrono::nanoseconds publication_interval(publication_interval_ns);
    timer_ = create_wall_timer(publication_interval, callback);
    std::string message = "Sample timer initialized: Interval (ms) = ";
    message += std::to_string(publication_interval_ns * (1e3 / 1e9));
    Log(message);
  } else {
    Log("Failed to initialize sample timer.");
    on_error();
  }

  // Cache workspace constraint parameters into constraints_ for the 2 kHz loop.
  constraints_.channel_enabled[0]    = get_parameter("constraints.channel_x.enabled").as_bool();
  constraints_.channel_half_width[0] = get_parameter("constraints.channel_x.half_width").as_double();
  constraints_.channel_enabled[1]    = get_parameter("constraints.channel_y.enabled").as_bool();
  constraints_.channel_half_width[1] = get_parameter("constraints.channel_y.half_width").as_double();
  constraints_.channel_enabled[2]    = get_parameter("constraints.channel_z.enabled").as_bool();
  constraints_.channel_half_width[2] = get_parameter("constraints.channel_z.half_width").as_double();
  constraints_.circle_enabled              = get_parameter("constraints.circle.enabled").as_bool();
  constraints_.circle_radius               = get_parameter("constraints.circle.radius").as_double();
  constraints_.home_offset[0]              = get_parameter("constraints.home_offset_0").as_double();
  constraints_.home_offset[1]              = get_parameter("constraints.home_offset_1").as_double();
  constraints_.stiffness             = get_parameter("constraints.stiffness").as_double();
  constraints_.damping               = get_parameter("constraints.damping").as_double();
  constraints_.homed                 = false;
  for (int i = 0; i < 3; ++i) constraints_.channel_offset[i] = 0.0;
  constraints_.circle_center[0]      = 0.0;
  constraints_.circle_center[1]      = 0.0;
  {
    std::string message = "Constraints: channel_x=";
    message += constraints_.channel_enabled[0] ? "ON" : "OFF";
    message += " channel_y=";
    message += constraints_.channel_enabled[1] ? "ON" : "OFF";
    message += " channel_z=";
    message += constraints_.channel_enabled[2] ? "ON" : "OFF";
    message += " circle=";
    message += constraints_.circle_enabled ? "ON" : "OFF";
    message += " Kp=" + std::to_string(constraints_.stiffness);
    message += " Kv=" + std::to_string(constraints_.damping);
    Log(message);
  }

  // Launch the haptic loop as a tight busy-loop thread (SDK-paced).
  // dhdSetForceAndTorqueAndGripperForce blocks until the next hardware servo
  // tick (~4 kHz), so the thread runs at the hardware rate without an
  // external timer. This matches the SDK example pattern and avoids the
  // jitter caused by a ROS wall timer firing asynchronously with the SDK.
  haptic_running_ = true;
  haptic_thread_ = std::thread([this]() {
    while (haptic_running_) {
      this->ApplyHapticForce();
    }
  });
  Log("Haptic loop thread started (SDK-paced busy loop)");

  // Reset the sample counter.
  sample_number_ = 0;

  // Get baseline effector mass to be used for gravity compensation.
  baseline_effector_mass_kg_ = get_effector_mass();
  {
    std::string message = "BASELINE EFFECTOR MASS: ";
    message += std::to_string(baseline_effector_mass_kg_);
    message += " kg";
    Log(message);
  }

  // Set effector mass to be used for gravity compensation.
  // Defaults to the ROS parameter value.
  set_effector_mass();

  // Enable gravity compensation.
  // Defaults to the ROS parameter value.
  set_gravity_compensation();

  // Enable or disable forces.
  // Defaults to the ROS parameter value.
  set_enable_force();

  // Add a set parameters callback.
  // Initialize a function pointer to the set_parameters_callback member with
  // one argument placeholder (for the parameter vector).
  auto parameters_callback =
      std::bind(&Node::set_parameters_callback, this, std::placeholders::_1);
  parameters_callback_handle_ =
      this->add_on_set_parameters_callback(parameters_callback);
}

/**
 *
 */
void Node::on_deactivate(void) {

  // Stop the haptic loop thread and publication timer.
  haptic_running_ = false;
  if (haptic_thread_.joinable()) haptic_thread_.join();
  timer_->cancel();
  // timer_->destroy();

  // Close the connection to the Force Dimension device.
  Log("Shutting the Force Dimension interface down.");
  auto result = hardware_disabled_ ? DHD_NO_ERROR : drdClose();
  if (result == DHD_NO_ERROR)
    Log("Force Dimension interface closed.");
  else {
    std::string message = "Unable to close the device connection: ";
    // message += dhdErrorGetLast();
    message += hardware_disabled_ ? "unknown error" : dhdErrorGetLastStr();
    Log(message);
    on_error();
  }
}

/**
 *
 */
void Node::on_cleanup(void) {}

// Error handling callback de-activates and cleans up.
void Node::on_error(void) {
  if (active_)
    on_deactivate();
  if (configured_)
    on_cleanup();
}
