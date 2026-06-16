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

// Real-time scheduling for the haptic thread.
#include <pthread.h>
#include <sched.h>
#include <cerrno>
#include <cmath>
#include <cstring>

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

  // Create the raw wrist joint angle publisher (w0,w1,w2 in rad).
  topic = WRIST_JOINT_FEEDBACK_TOPIC;
  wrist_joint_publisher_ = create_publisher<WristJointMessage>(topic, qos);

  // Create the synchronized device state publisher.
  topic = DEVICE_STATE_FEEDBACK_TOPIC;
  device_state_publisher_ = create_publisher<DeviceStateMessage>(topic, qos);

  // Create the applied-force (wrist torque) feedback publisher.
  topic = FORCE_FEEDBACK_TOPIC;
  wrench_publisher_ = create_publisher<ForceMessage>(topic, qos);

  // Create the full-rate raw data-collection sample publisher.
  topic = RAW_SAMPLE_FEEDBACK_TOPIC;
  raw_sample_publisher_ = create_publisher<RawSampleMessage>(topic, qos);

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
  declare_parameter<int>("feedback_sample_decimation.wrist_joint_angles", 50);
  declare_parameter<int>("feedback_sample_decimation.state", 50);
  declare_parameter<int>("feedback_sample_decimation.force", 50);
  // Raw data-collection sample. Default 1 = publish every timer tick, so at
  // the 2 kHz sample timer (unity_low_latency.yaml) this records at 2 kHz.
  declare_parameter<int>("feedback_sample_decimation.raw_sample", 1);
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

  // Translation position hold (force-mode primary translation hold). Bilateral
  // spring+damper to the captured home with a per-axis force cap (SDK hold
  // pattern) — use this instead of the channel walls when DRD regulation is off.
  declare_parameter<bool>("constraints.translation_hold.enabled", false);
  declare_parameter<double>("constraints.translation_hold.stiffness", 150.0);
  declare_parameter<double>("constraints.translation_hold.damping", 25.0);
  declare_parameter<double>("constraints.translation_hold.max_force", 5.0);

  // Circular track ("ring") guide: confine the effector to a circle of radius
  // ring.radius in the plane orthogonal to ring.height_axis (default Z), free to
  // orbit, height locked. Force-mode primary translation constraint.
  declare_parameter<bool>("constraints.ring.enabled", false);
  declare_parameter<double>("constraints.ring.radius", 0.04);
  declare_parameter<int>("constraints.ring.height_axis", 2);
  declare_parameter<double>("constraints.ring.stiffness", 300.0);
  declare_parameter<double>("constraints.ring.damping", 30.0);
  declare_parameter<double>("constraints.ring.max_force", 10.0);
  declare_parameter<double>("constraints.ring.center_deadzone", 0.005);
  // Ring center planar offset from the autocenter home (the "central cylinder"
  // offset), along the two in-plane axes; and optional start-on-rim placement.
  declare_parameter<double>("constraints.ring.center_offset_0", 0.0);
  declare_parameter<double>("constraints.ring.center_offset_1", 0.0);
  declare_parameter<bool>("constraints.ring.start_on_rim", false);
  declare_parameter<double>("constraints.ring.start_angle_rad", 0.0);
  // Ellipse semi-axes. Default 0.0 = sentinel: fall back to ring.radius.
  declare_parameter<double>("constraints.ring.semi_axis_a", 0.0);
  declare_parameter<double>("constraints.ring.semi_axis_b", 0.0);

  // Plane lock ("2D constraint"): bilateral spring+damper holding ONE axis at
  // its captured home (the ring's height lock as a standalone primitive); the
  // orthogonal plane stays free.
  declare_parameter<bool>("constraints.plane_lock.enabled", false);
  declare_parameter<int>("constraints.plane_lock.axis", 2);
  declare_parameter<double>("constraints.plane_lock.stiffness", 300.0);
  declare_parameter<double>("constraints.plane_lock.damping", 30.0);
  declare_parameter<double>("constraints.plane_lock.max_force", 10.0);

  // Spherical outer boundary: free inside radius (centered at captured home),
  // inward spring + outward-only radial damping past the rim. Keeps the
  // effector off the hard mechanical workspace limits.
  declare_parameter<bool>("constraints.sphere_boundary.enabled", false);
  declare_parameter<double>("constraints.sphere_boundary.radius", 0.05);
  declare_parameter<double>("constraints.sphere_boundary.stiffness", 300.0);
  declare_parameter<double>("constraints.sphere_boundary.damping", 30.0);
  declare_parameter<double>("constraints.sphere_boundary.max_force", 10.0);

  // Clean wrist upright lock (ring design principles: SDK joint velocity,
  // bilateral PD, capped, no decimation). Holds all 3 wrist joints upright.
  declare_parameter<bool>("constraints.wrist_upright.enabled", false);
  declare_parameter<double>("constraints.wrist_upright.stiffness", 1.5);
  declare_parameter<double>("constraints.wrist_upright.damping", 0.08);
  declare_parameter<double>("constraints.wrist_upright.max_torque", 0.25);
  declare_parameter<double>("constraints.wrist_upright.vel_filter_alpha", 1.0);
  declare_parameter<int>("constraints.wrist_upright.free_axis", -1);
  declare_parameter<double>("constraints.wrist_upright.free_axis_damping", 0.0);
  // Free-axis torque cap, decoupled from max_torque (which bounds only the
  // locked joints) so a small stability cap on the lock does not neuter the
  // yaw viscous damping.
  declare_parameter<double>("constraints.wrist_upright.free_axis_max_torque", 0.25);
  // SDK angular-velocity estimator window (ms). The SDK default of 20 ms adds
  // ~10 ms of group delay to dhdGetJointVelocities — the dominant lag that
  // destabilizes -b*omega damping at high b (measured 14 Hz limit cycle).
  // Shorter = less lag, noisier estimate. Applied live by the haptic thread.
  declare_parameter<int>("velocity_estimator.angular_window_ms", 20);

  // Wrist orientation lock: PD on two of three Cartesian rotation axes,
  // leaves the third (default: Z/yaw) free. Applies in the 2 kHz loop.
  declare_parameter<bool>("constraints.wrist_lock.enabled", false);
  declare_parameter<int>("constraints.wrist_lock.free_axis_cartesian", 2);
  declare_parameter<double>("constraints.wrist_lock.stiffness", 0.5);
  declare_parameter<double>("constraints.wrist_lock.damping", 0.05);
  declare_parameter<double>("constraints.wrist_lock.free_axis_damping", 0.0);
  declare_parameter<double>("constraints.wrist_lock.free_axis_filter_alpha", 0.2);
  declare_parameter<double>("constraints.wrist_lock.error_deadband_rad", 0.01);
  declare_parameter<double>("constraints.wrist_lock.error_filter_alpha", 0.15);
  declare_parameter<double>("constraints.wrist_lock.omega_filter_alpha", 0.15);
  // Per-wrist-joint home offsets. SDK joint ordering is hardware-dependent.
  // For the Sigma.7, empirically: slot 3 = roll, 4 = pitch, 5 = yaw.
  // Positive/negative biases the autocenter pose so the participant gets
  // asymmetric rotation range on that joint.
  declare_parameter<double>("constraints.wrist_lock.home_joint_0_rad", 0.0);
  declare_parameter<double>("constraints.wrist_lock.home_joint_1_rad", 0.0);
  declare_parameter<double>("constraints.wrist_lock.home_joint_2_rad", 0.0);

  // Wrist lock MODE: rotate the locked roll setpoint to one of three poses.
  //   upright -> roll offset 0 (captured neutral)
  //   left    -> roll offset roll_offset_left_rad
  //   right   -> roll offset roll_offset_right_rad
  // Switchable live via /robot/wrist_lock/set_mode (std_msgs/String) or this
  // `mode` parameter. The applied offset slews at roll_slew_rate_rad_s and is
  // clamped into the device's measured roll range minus roll_range_margin_rad.
  declare_parameter<std::string>("constraints.wrist_lock.mode", "upright");
  declare_parameter<double>("constraints.wrist_lock.roll_offset_left_rad", -1.40);
  declare_parameter<double>("constraints.wrist_lock.roll_offset_right_rad", 1.40);
  declare_parameter<double>("constraints.wrist_lock.roll_slew_rate_rad_s", 0.5);
  declare_parameter<double>("constraints.wrist_lock.roll_range_margin_rad", 0.05);

  // Selective DRD actuator regulation. Defaults preserve legacy behaviour:
  // translation regulated during homing then released; wrist regulated iff the
  // device has an active wrist; gripper never auto-centred.
  // When actuators.hold_* is true, the DRD regulation is kept active after
  // autocenter so that axis stays locked while the 2 kHz force loop runs.
  declare_parameter<bool>("actuators.regulate_pos", true);
  declare_parameter<bool>("actuators.regulate_rot", true);  // true -> auto = device_has_active_wrist
  declare_parameter<bool>("actuators.regulate_grip", false);
  declare_parameter<bool>("actuators.hold_pos", false);
  declare_parameter<bool>("actuators.hold_rot", false);
  declare_parameter<bool>("actuators.hold_grip", false);

  // When any actuator is held by DRD, the 2 kHz force loop must use the drd*
  // force API instead of dhd*, so our commands add on top of DRD regulation
  // instead of overwriting it each tick (which causes motor chatter).
  declare_parameter<bool>("haptic_loop.use_drd_api", false);

  // Force-mode hold: stop DRD regulation after centering and hold all axes via
  // dhd* force functions in the 2 kHz loop (SDK hold/sphere example pattern).
  // Default true. NOTE: a launch using force mode MUST provide its own
  // translation hold (channel constraints), else translation is left free once
  // DRD regulation stops. The unity_sigma_controller launch helper pins this
  // false for tasks that rely on DRD translation hold; wrist_flexion overrides
  // it true and supplies channel springs.
  declare_parameter<bool>("haptic_loop.force_mode", true);

  // Create the force control subcription.
  SubscribeForce();

  // Subscribe to wrist lock-mode commands (upright/left/right) and advertise
  // the active mode for Unity feedback.
  SubscribeWristMode();
  wrist_mode_publisher_ =
      create_publisher<WristModeMessage>(WRIST_MODE_FEEDBACK_TOPIC, DefaultQoS());

  // Advertise rehome service — resets constraint homing on the next 2 kHz tick.
  rehome_service_ = create_service<std_srvs::srv::Trigger>(
      "~/rehome_constraints",
      std::bind(&Node::rehome_constraints_callback, this,
                std::placeholders::_1, std::placeholders::_2));

  // Mark configured so ~Node()/on_error() run on_cleanup() on shutdown.
  configured_ = true;
}

/** Activates the ROS node by initializing the Force Dimension interface and
 *  the publication timer / callback.
 */
void Node::on_activate(void) {

  // Check to see if hardware has been disabled.
  // This is done once, at the time of activation, and stored in a member
  // variable while active.
  get_parameter("disable_hardware", hardware_disabled_);

  // Cache force-mode EARLY: the device init below (Phase 2) branches on it to
  // decide whether to drdStop() after centering. It must be read before that
  // point, not later with the other haptic-loop params.
  force_mode_ = get_parameter("haptic_loop.force_mode").as_bool();

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

    // Log the USB COM mode for the record. ASYNC (1) is the SDK default and
    // already the lower-jitter option (parallelized read/write); SYNC (0)
    // serializes transactions. There is no SDK-side fix for the usbipd/WSL2
    // per-transaction latency floor (~12 ms velocity-path lag measured).
    {
      const int com_mode = dhdGetComMode(device_id_);
      const char* name = (com_mode == DHD_COM_MODE_SYNC)    ? "SYNC"
                       : (com_mode == DHD_COM_MODE_ASYNC)   ? "ASYNC"
                       : (com_mode == DHD_COM_MODE_VIRTUAL) ? "VIRTUAL"
                       : (com_mode == DHD_COM_MODE_NETWORK) ? "NETWORK"
                                                            : "UNKNOWN";
      Log("USB COM mode: " + std::string(name) + " (" +
          std::to_string(com_mode) + ")");
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

    // Read the device's per-joint angle range so the wrist lock-mode offset can
    // be clamped to the hardware's actual safe roll travel instead of a fixed
    // guess. dhdGetJointAngleRange returns DHD_MAX_DOF arrays indexed by global
    // joint slot; for the Sigma.7, slot 3 = roll (matches positionCenter[3] and
    // the empirical j0=roll mapping used by the 2 kHz PD). Range is pose
    // independent, so reading it here (before autocenter) is fine.
    constexpr int kRollGlobalJoint = 3;
    constexpr int kPitchGlobalJoint = 4;
    constexpr int kYawGlobalJoint = 5;
    constexpr double kRad2Deg = 57.29577951308232;
    constraints_.wrist_roll_range_valid = false;
    {
      double jmin[DHD_MAX_DOF] = {};
      double jmax[DHD_MAX_DOF] = {};
      if (dhdGetJointAngleRange(jmin, jmax, device_id_) >= 0) {
        constraints_.wrist_roll_min = jmin[kRollGlobalJoint];
        constraints_.wrist_roll_max = jmax[kRollGlobalJoint];
        constraints_.wrist_roll_range_valid = true;
        std::string message = "Wrist joint range (rad / deg): roll=[";
        message += std::to_string(jmin[kRollGlobalJoint]) + ", " +
                   std::to_string(jmax[kRollGlobalJoint]) + "] / [" +
                   std::to_string(jmin[kRollGlobalJoint] * kRad2Deg) + ", " +
                   std::to_string(jmax[kRollGlobalJoint] * kRad2Deg) + "]  pitch=[" +
                   std::to_string(jmin[kPitchGlobalJoint]) + ", " +
                   std::to_string(jmax[kPitchGlobalJoint]) + "]  yaw=[" +
                   std::to_string(jmin[kYawGlobalJoint]) + ", " +
                   std::to_string(jmax[kYawGlobalJoint]) + "]";
        Log(message);
      } else {
        Log("Could not read wrist joint angle range; roll lock-mode offset will "
            "slew unclamped (no hardware range check).");
      }
    }

    // Phase 1 — autocenter: ALWAYS regulate all available axes regardless of
    // the user's final actuators.regulate_* preferences. This guarantees the
    // device reaches a known starting pose (translation at home_offset, wrist
    // at zero Euler angles, gripper at natural gap) before we release any
    // axis. Previously, launching with regulate_rot=false meant the wrist was
    // never commanded to its upright orientation, so the 2 kHz loop captured
    // whatever tilt the handle happened to have as "home."
    const bool device_has_wrist = dhdHasActiveWrist();
    drdRegulatePos(true);
    drdRegulateRot(device_has_wrist);
    drdRegulateGrip(false);  // leave gripper at its natural gap during autocenter

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

      // Pre-rotate each wrist joint during autocenter. SDK joint ordering
      // for the Sigma.7: slot 3 = roll, 4 = pitch, 5 = yaw (empirical). Roll
      // and pitch stay clamped to ±40° to stay clear of their joint limits;
      // yaw (w2) has a wider safe range (~±80°) so the wrist-flexion task can
      // start the free yaw axis at a flexed pose (e.g. 75°). The device's own
      // limits still bound the actual drdMoveTo (see logged wrist joint range).
      constexpr double kMaxJointOffsetRad = 0.7;     // roll/pitch (±40°)
      constexpr double kMaxYawOffsetRad   = 1.5708;  // yaw / w2 (±90°, pi/2)
      auto clampJoint = [](double v, double lim) {
        return v < -lim ? -lim : (v > lim ? lim : v);
      };
      double j0 = clampJoint(get_parameter("constraints.wrist_lock.home_joint_0_rad").as_double(), kMaxJointOffsetRad);
      double j1 = clampJoint(get_parameter("constraints.wrist_lock.home_joint_1_rad").as_double(), kMaxJointOffsetRad);
      double j2 = clampJoint(get_parameter("constraints.wrist_lock.home_joint_2_rad").as_double(), kMaxYawOffsetRad);
      positionCenter[3] = j0;
      positionCenter[4] = j1;
      positionCenter[5] = j2;

      // Ring guide: record the ring center (= autocenter base + planar offset)
      // and, if requested, move the device ONTO the rim at startup instead of
      // leaving it at the center. The center is the "central cylinder" axis; its
      // offset and the radius are exposed as parameters.
      if (get_parameter("constraints.ring.enabled").as_bool()) {
        const int rh  = get_parameter("constraints.ring.height_axis").as_int();
        const int ra0 = (rh == 0) ? 1 : 0;   // first in-plane axis
        const int ra1 = (rh == 2) ? 1 : 2;   // second in-plane axis
        const double R   = get_parameter("constraints.ring.radius").as_double();
        // Resolve semi-axes for rim placement (0.0 sentinel falls back to radius).
        double rim_sa = get_parameter("constraints.ring.semi_axis_a").as_double();
        double rim_sb = get_parameter("constraints.ring.semi_axis_b").as_double();
        if (rim_sa <= 0.0) rim_sa = R;
        if (rim_sb <= 0.0) rim_sb = R;
        const double coff0 = get_parameter("constraints.ring.center_offset_0").as_double();
        const double coff1 = get_parameter("constraints.ring.center_offset_1").as_double();
        // Capture the base center + offset separately so center_offset_0/1 stays
        // live-tunable (the parameter callback recomputes ring_center = base + offset).
        constraints_.ring_base[0] = positionCenter[ra0];
        constraints_.ring_base[1] = positionCenter[ra1];
        constraints_.ring_center_offset[0] = coff0;
        constraints_.ring_center_offset[1] = coff1;
        const double cc0 = positionCenter[ra0] + coff0;
        const double cc1 = positionCenter[ra1] + coff1;
        constraints_.ring_center[0] = cc0;
        constraints_.ring_center[1] = cc1;
        constraints_.ring_center_valid = true;
        const bool start_on_rim =
            get_parameter("constraints.ring.start_on_rim").as_bool();
        if (start_on_rim) {
          const double start_ang = get_parameter("constraints.ring.start_angle_rad").as_double();
          positionCenter[ra0] = clamp(cc0 + rim_sa * std::cos(start_ang), -kMaxOffsetM, kMaxOffsetM);
          positionCenter[ra1] = clamp(cc1 + rim_sb * std::sin(start_ang), -kMaxOffsetM, kMaxOffsetM);
        }
        std::string message = "Ring guide: center=(";
        message += std::to_string(cc0) + ", " + std::to_string(cc1);
        message += ") semi_axis_a=" + std::to_string(rim_sa);
        message += " semi_axis_b=" + std::to_string(rim_sb);
        message += start_on_rim ? " (start ON rim)" : " (start at center)";
        Log(message);
      }

      {
        std::string message = "Moving to startup position: axis";
        message += std::to_string(ax0) + "=" + std::to_string(off0);
        message += " axis" + std::to_string(ax1) + "=" + std::to_string(off1);
        message += " wrist=[" + std::to_string(j0);
        message += ", " + std::to_string(j1);
        message += ", " + std::to_string(j2) + "] rad";
        Log(message);
      }
    }
    if (drdMoveTo(positionCenter, true) < 0) {
      std::string message = "Cannot move to centre: ";
      message += dhdErrorGetLastStr();
      Log(message);
      on_error();
    }

    // Phase 2 — apply runtime hold settings. All axes were regulated for
    // autocenter; now release the ones the launch file does NOT want DRD to
    // hold at runtime. The 2 kHz force loop owns any released axis.
    // When nothing is held, this matches the legacy drdStop(true) behaviour
    // used by Task 3 / pong launches.
    if (force_mode_) {
      // Force-mode (SDK hold/sphere pattern): regulation only served to center
      // the device above. Release every axis and STOP the regulation thread —
      // forces stay enabled. From here the 2 kHz dhd force loop holds everything:
      // translation via channel constraints, wrist via the joint-space PD, and
      // gripper via the fg spring. With no regulation thread there is nothing
      // for high-rate force writes to "take over", so no drop/restart cycle.
      drdRegulatePos(false);
      drdRegulateRot(false);
      drdRegulateGrip(false);
      if (drdStop(true) < 0) {
        std::string message = "Cannot stop DRD regulation (force mode): ";
        message += dhdErrorGetLastStr();
        Log(message);
        on_error();
      } else {
        Log("Force mode: DRD regulation stopped after centering; forces enabled.");
      }
    } else {
      const bool hold_pos = get_parameter("actuators.hold_pos").as_bool();
      const bool hold_rot = get_parameter("actuators.hold_rot").as_bool();
      const bool hold_grip_param = get_parameter("actuators.hold_grip").as_bool();
      const bool reg_grip_param = get_parameter("actuators.regulate_grip").as_bool();
      // Only honor hold_grip if the launch file also asked to regulate the grip
      // (legacy launches default regulate_grip=false — keep them untouched).
      const bool hold_grip = hold_grip_param && reg_grip_param;
      if (!hold_pos) drdRegulatePos(false);
      if (!hold_rot) drdRegulateRot(false);
      if (hold_grip) drdRegulateGrip(true);   // engage grip regulation for held grip
      else           drdRegulateGrip(false);
      if (!hold_pos && !hold_rot && !hold_grip) {
        if (drdStop(true) < 0) {
          std::string message = "Cannot stop DRD regulation: ";
          message += dhdErrorGetLastStr();
          Log(message);
          on_error();
        }
      } else {
        std::string message = "DRD holding actuators: pos=";
        message += hold_pos ? "1" : "0";
        message += " rot=";
        message += hold_rot ? "1" : "0";
        message += " grip=";
        message += hold_grip ? "1" : "0";
        Log(message);
      }
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
  constraints_.translation_hold_enabled   = get_parameter("constraints.translation_hold.enabled").as_bool();
  constraints_.translation_hold_stiffness = get_parameter("constraints.translation_hold.stiffness").as_double();
  constraints_.translation_hold_damping   = get_parameter("constraints.translation_hold.damping").as_double();
  constraints_.translation_hold_max_force = get_parameter("constraints.translation_hold.max_force").as_double();
  constraints_.ring_enabled          = get_parameter("constraints.ring.enabled").as_bool();
  constraints_.ring_radius           = get_parameter("constraints.ring.radius").as_double();
  constraints_.ring_height_axis      = get_parameter("constraints.ring.height_axis").as_int();
  constraints_.ring_stiffness        = get_parameter("constraints.ring.stiffness").as_double();
  constraints_.ring_damping          = get_parameter("constraints.ring.damping").as_double();
  constraints_.ring_max_force        = get_parameter("constraints.ring.max_force").as_double();
  constraints_.ring_center_deadzone  = get_parameter("constraints.ring.center_deadzone").as_double();
  // Two-read back-compat for ellipse semi-axes: 0.0 sentinel falls back to ring.radius.
  {
    const double r_bc = get_parameter("constraints.ring.radius").as_double();
    double sa = get_parameter("constraints.ring.semi_axis_a").as_double();
    double sb = get_parameter("constraints.ring.semi_axis_b").as_double();
    if (sa <= 0.0) sa = r_bc;
    if (sb <= 0.0) sb = r_bc;
    constraints_.ring_semi_axis_a = sa;
    constraints_.ring_semi_axis_b = sb;
  }
  constraints_.plane_lock_enabled   = get_parameter("constraints.plane_lock.enabled").as_bool();
  constraints_.plane_lock_axis      = get_parameter("constraints.plane_lock.axis").as_int();
  constraints_.plane_lock_stiffness = get_parameter("constraints.plane_lock.stiffness").as_double();
  constraints_.plane_lock_damping   = get_parameter("constraints.plane_lock.damping").as_double();
  constraints_.plane_lock_max_force = get_parameter("constraints.plane_lock.max_force").as_double();
  constraints_.sphere_boundary_enabled   = get_parameter("constraints.sphere_boundary.enabled").as_bool();
  constraints_.sphere_boundary_radius    = get_parameter("constraints.sphere_boundary.radius").as_double();
  constraints_.sphere_boundary_stiffness = get_parameter("constraints.sphere_boundary.stiffness").as_double();
  constraints_.sphere_boundary_damping   = get_parameter("constraints.sphere_boundary.damping").as_double();
  constraints_.sphere_boundary_max_force = get_parameter("constraints.sphere_boundary.max_force").as_double();
  constraints_.wrist_upright_enabled    = get_parameter("constraints.wrist_upright.enabled").as_bool();
  constraints_.wrist_upright_stiffness  = get_parameter("constraints.wrist_upright.stiffness").as_double();
  constraints_.wrist_upright_damping    = get_parameter("constraints.wrist_upright.damping").as_double();
  constraints_.wrist_upright_max_torque = get_parameter("constraints.wrist_upright.max_torque").as_double();
  constraints_.wrist_upright_vel_filter_alpha = get_parameter("constraints.wrist_upright.vel_filter_alpha").as_double();
  constraints_.wrist_upright_free_axis = get_parameter("constraints.wrist_upright.free_axis").as_int();
  constraints_.wrist_upright_free_axis_damping = get_parameter("constraints.wrist_upright.free_axis_damping").as_double();
  constraints_.wrist_upright_free_axis_max_torque =
      get_parameter("constraints.wrist_upright.free_axis_max_torque").as_double();
  constraints_.velocity_angular_window_ms =
      static_cast<int>(get_parameter("velocity_estimator.angular_window_ms").as_int());
  constraints_.velocity_angular_window_applied = -1;  // force (re)apply on first tick
  constraints_.wrist_upright_homed   = false;
  constraints_.homed                 = false;
  for (int i = 0; i < 3; ++i) constraints_.channel_offset[i] = 0.0;
  constraints_.circle_center[0]      = 0.0;
  constraints_.circle_center[1]      = 0.0;
  constraints_.wrist_lock_enabled    = get_parameter("constraints.wrist_lock.enabled").as_bool();
  constraints_.wrist_lock_free_axis  = get_parameter("constraints.wrist_lock.free_axis_cartesian").as_int();
  constraints_.wrist_lock_stiffness  = get_parameter("constraints.wrist_lock.stiffness").as_double();
  constraints_.wrist_lock_damping    = get_parameter("constraints.wrist_lock.damping").as_double();
  constraints_.wrist_free_axis_damping = get_parameter("constraints.wrist_lock.free_axis_damping").as_double();
  constraints_.wrist_free_axis_filter_alpha = get_parameter("constraints.wrist_lock.free_axis_filter_alpha").as_double();
  constraints_.wrist_lock_error_deadband = get_parameter("constraints.wrist_lock.error_deadband_rad").as_double();
  constraints_.wrist_lock_error_filter_alpha = get_parameter("constraints.wrist_lock.error_filter_alpha").as_double();
  constraints_.wrist_lock_omega_filter_alpha = get_parameter("constraints.wrist_lock.omega_filter_alpha").as_double();
  constraints_.wrist_free_axis_omega_filt = 0.0;
  for (int i = 0; i < 3; ++i) {
    constraints_.wrist_lock_error_filt[i] = 0.0;
    constraints_.wrist_lock_omega_filt[i] = 0.0;
  }
  constraints_.wrist_homed           = false;
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      constraints_.wrist_home_rotation[r][c] = (r == c) ? 1.0 : 0.0;
  if (constraints_.wrist_lock_enabled) {
    std::string message = "Wrist lock: free_axis=";
    message += std::to_string(constraints_.wrist_lock_free_axis);
    message += " Kp=" + std::to_string(constraints_.wrist_lock_stiffness);
    message += " Kv=" + std::to_string(constraints_.wrist_lock_damping);
    message += " b_free=" + std::to_string(constraints_.wrist_free_axis_damping);
    message += " alpha=" + std::to_string(constraints_.wrist_free_axis_filter_alpha);
    message += " deadband=" + std::to_string(constraints_.wrist_lock_error_deadband);
    Log(message);
  }

  // Wrist lock-mode (upright/left/right) configuration. The slew rate + margin
  // live in the constraint struct (read by the haptic loop); the left/right
  // offsets are cached on the node so mode_to_offset() can map a mode string.
  constraints_.wrist_roll_slew_rate =
      get_parameter("constraints.wrist_lock.roll_slew_rate_rad_s").as_double();
  constraints_.wrist_roll_range_margin =
      get_parameter("constraints.wrist_lock.roll_range_margin_rad").as_double();
  wrist_roll_offset_left_ =
      get_parameter("constraints.wrist_lock.roll_offset_left_rad").as_double();
  wrist_roll_offset_right_ =
      get_parameter("constraints.wrist_lock.roll_offset_right_rad").as_double();
  // Set the initial target offset from the `mode` parameter. The haptic loop
  // slews the applied offset from 0 (upright) toward this after first homing.
  {
    const std::string mode =
        get_parameter("constraints.wrist_lock.mode").as_string();
    const double offset = mode_to_offset(mode);
    wrist_roll_offset_target_.store(offset);
    std::string message = "Wrist lock initial mode: " + mode +
        " (roll offset " + std::to_string(offset) + " rad, slew " +
        std::to_string(constraints_.wrist_roll_slew_rate) + " rad/s)";
    Log(message);
    // Roll (joint 0) must be a LOCKED axis for the offset to take effect; if
    // it is the free axis the offset is silently ignored, so warn loudly.
    if (offset != 0.0 && constraints_.wrist_lock_free_axis == 0) {
      Log("WARNING: wrist_lock free_axis=0 (roll) but a non-upright mode is "
          "set — the roll offset will be ignored because roll is free.");
    }
  }

  // Cache which SDK force API the 2 kHz loop should use.
  haptic_use_drd_api_ = get_parameter("haptic_loop.use_drd_api").as_bool();
  // force_mode_ was already cached at the top of on_activate (Phase 2 needs it).
  if (force_mode_) {
    Log("Haptic loop: FORCE MODE — DRD regulation stopped after centering; "
        "all axes held via dhd* force functions (no regulation thread).");
  } else if (haptic_use_drd_api_) {
    Log("Haptic loop: using drd* force API (composes with DRD regulation)");
  }
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

  // Promote the haptic thread to real-time priority so the kernel does
  // not preempt it for normal-priority work (ros2 bag disk I/O, network
  // RX). Without this, heavy I/O can starve the SDK's microsecond timing
  // and the DRD regulation thread silently exits ("drop"). Requires
  // CAP_SYS_NICE — start the launch with `sudo -E` OR run once:
  //   sudo setcap cap_sys_nice+ep $(which python3) <or the node binary>
  // If the capability is missing, we log a warning and continue at
  // SCHED_OTHER (the watchdog will still catch drops).
  {
    sched_param sp{};
    sp.sched_priority = 80;  // SCHED_FIFO range is 1..99; 80 is plenty
    int rc = pthread_setschedparam(
        haptic_thread_.native_handle(), SCHED_FIFO, &sp);
    if (rc == 0) {
      Log("Haptic loop thread elevated to SCHED_FIFO prio 80");
    } else {
      std::string warn = "Could not set SCHED_FIFO on haptic thread (";
      warn += std::strerror(rc);
      warn += "); running at default priority. To fix, run with sudo or "
              "grant CAP_SYS_NICE to the node binary.";
      Log(warn);
    }
  }
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

  // Mark active so ~Node()/on_error() run on_deactivate() on shutdown:
  // this joins the haptic thread and closes the device. Without it the
  // joinable std::thread is destroyed -> std::terminate -> abort (exit -6),
  // which appears as "terminate called without an active exception".
  active_ = true;
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
