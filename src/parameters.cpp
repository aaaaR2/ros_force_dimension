/** Functionality related to ROS2 parameters.
 */

/** Copyright 2022 Neuromechatronics Lab, Carnegie Mellon University (a.whit)
 *
 *  Created by: a. whit. (nml@whit.contact)
 *
 *  This Source Code Form is subject to the terms of the Mozilla Public
 *  License, v. 2.0. If a copy of the MPL was not distributed with this
 *  file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

// Include guard.
#ifndef FORCE_DIMENSION_PARAMETERS_H_
#define FORCE_DIMENSION_PARAMETERS_H_

// Import the node header.
#include "node.hpp"

// Import the ROS interface.
#include "rclcpp/rclcpp.hpp"

/** Parameter change callback.
 *
 *  This function is called any time a ROS2 parameter value changes.
 */
rcl_interfaces::msg::SetParametersResult
force_dimension::Node::set_parameters_callback(
    const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto &parameter : parameters) {
    if (parameter.get_name() == "effector_mass_kg")
      set_effector_mass(parameter.as_double());
    if (parameter.get_name() == "gravity_compensation")
      set_gravity_compensation(parameter.as_bool());
    if (parameter.get_name() == "enable_force")
      set_enable_force(parameter.as_bool());
    // Workspace constraint parameters — update cached values for 2 kHz loop.
    if (parameter.get_name() == "constraints.channel_x.enabled")
      constraints_.channel_enabled[0] = parameter.as_bool();
    if (parameter.get_name() == "constraints.channel_x.half_width")
      constraints_.channel_half_width[0] = parameter.as_double();
    if (parameter.get_name() == "constraints.channel_y.enabled")
      constraints_.channel_enabled[1] = parameter.as_bool();
    if (parameter.get_name() == "constraints.channel_y.half_width")
      constraints_.channel_half_width[1] = parameter.as_double();
    if (parameter.get_name() == "constraints.channel_z.enabled")
      constraints_.channel_enabled[2] = parameter.as_bool();
    if (parameter.get_name() == "constraints.channel_z.half_width")
      constraints_.channel_half_width[2] = parameter.as_double();
    if (parameter.get_name() == "constraints.circle.enabled")
      constraints_.circle_enabled = parameter.as_bool();
    if (parameter.get_name() == "constraints.circle.radius")
      constraints_.circle_radius = parameter.as_double();
    if (parameter.get_name() == "constraints.stiffness")
      constraints_.stiffness = parameter.as_double();
    if (parameter.get_name() == "constraints.damping")
      constraints_.damping = parameter.as_double();
    // Translation position hold (force-mode), live-tunable for bring-up.
    if (parameter.get_name() == "constraints.translation_hold.enabled")
      constraints_.translation_hold_enabled = parameter.as_bool();
    if (parameter.get_name() == "constraints.translation_hold.stiffness")
      constraints_.translation_hold_stiffness = parameter.as_double();
    if (parameter.get_name() == "constraints.translation_hold.damping")
      constraints_.translation_hold_damping = parameter.as_double();
    if (parameter.get_name() == "constraints.translation_hold.max_force")
      constraints_.translation_hold_max_force = parameter.as_double();
    // Circular track ("ring") guide, live-tunable for bring-up.
    if (parameter.get_name() == "constraints.ring.enabled")
      constraints_.ring_enabled = parameter.as_bool();
    if (parameter.get_name() == "constraints.ring.radius") {
      constraints_.ring_radius = parameter.as_double();
      // Back-compat: setting radius also sets both semi-axes so orbit.launch.py
      // (which sets ring.radius) works unchanged without specifying semi_axis_a/b.
      constraints_.ring_semi_axis_a = parameter.as_double();
      constraints_.ring_semi_axis_b = parameter.as_double();
    }
    if (parameter.get_name() == "constraints.ring.semi_axis_a")
      constraints_.ring_semi_axis_a = parameter.as_double();
    if (parameter.get_name() == "constraints.ring.semi_axis_b")
      constraints_.ring_semi_axis_b = parameter.as_double();
    if (parameter.get_name() == "constraints.ring.stiffness")
      constraints_.ring_stiffness = parameter.as_double();
    if (parameter.get_name() == "constraints.ring.damping")
      constraints_.ring_damping = parameter.as_double();
    if (parameter.get_name() == "constraints.ring.max_force")
      constraints_.ring_max_force = parameter.as_double();
    if (parameter.get_name() == "constraints.ring.center_deadzone")
      constraints_.ring_center_deadzone = parameter.as_double();
    // Clean wrist upright lock, live-tunable for bring-up.
    if (parameter.get_name() == "constraints.wrist_upright.enabled")
      constraints_.wrist_upright_enabled = parameter.as_bool();
    if (parameter.get_name() == "constraints.wrist_upright.stiffness")
      constraints_.wrist_upright_stiffness = parameter.as_double();
    if (parameter.get_name() == "constraints.wrist_upright.damping")
      constraints_.wrist_upright_damping = parameter.as_double();
    if (parameter.get_name() == "constraints.wrist_upright.max_torque")
      constraints_.wrist_upright_max_torque = parameter.as_double();
    if (parameter.get_name() == "constraints.wrist_upright.vel_filter_alpha")
      constraints_.wrist_upright_vel_filter_alpha = parameter.as_double();
    if (parameter.get_name() == "constraints.wrist_upright.free_axis")
      constraints_.wrist_upright_free_axis = parameter.as_int();
    if (parameter.get_name() == "constraints.wrist_upright.free_axis_damping")
      constraints_.wrist_upright_free_axis_damping = parameter.as_double();
    // Wrist orientation lock (live-tunable via ROS parameter service).
    if (parameter.get_name() == "constraints.wrist_lock.stiffness")
      constraints_.wrist_lock_stiffness = parameter.as_double();
    if (parameter.get_name() == "constraints.wrist_lock.damping")
      constraints_.wrist_lock_damping = parameter.as_double();
    if (parameter.get_name() == "constraints.wrist_lock.free_axis_damping")
      constraints_.wrist_free_axis_damping = parameter.as_double();
    if (parameter.get_name() == "constraints.wrist_lock.free_axis_filter_alpha")
      constraints_.wrist_free_axis_filter_alpha = parameter.as_double();
    if (parameter.get_name() == "constraints.wrist_lock.error_deadband_rad")
      constraints_.wrist_lock_error_deadband = parameter.as_double();
    if (parameter.get_name() == "constraints.wrist_lock.error_filter_alpha")
      constraints_.wrist_lock_error_filter_alpha = parameter.as_double();
    if (parameter.get_name() == "constraints.wrist_lock.omega_filter_alpha")
      constraints_.wrist_lock_omega_filter_alpha = parameter.as_double();
    if (parameter.get_name() == "constraints.wrist_lock.enabled")
      constraints_.wrist_lock_enabled = parameter.as_bool();
    if (parameter.get_name() == "constraints.wrist_lock.joint_space") {
      constraints_.wrist_lock_joint_space = parameter.as_bool();
      // Re-home on mode switch so the captured reference is fresh.
      constraints_.wrist_homed = false;
      constraints_.wrist_joint_homed = false;
    }
    // Wrist lock-mode (upright/left/right). Switching `mode` retargets the
    // slewed roll offset; it must NOT re-home (that would move the neutral).
    if (parameter.get_name() == "constraints.wrist_lock.roll_slew_rate_rad_s")
      constraints_.wrist_roll_slew_rate = parameter.as_double();
    if (parameter.get_name() == "constraints.wrist_lock.roll_range_margin_rad")
      constraints_.wrist_roll_range_margin = parameter.as_double();
    if (parameter.get_name() == "constraints.wrist_lock.roll_offset_left_rad")
      wrist_roll_offset_left_ = parameter.as_double();
    if (parameter.get_name() == "constraints.wrist_lock.roll_offset_right_rad")
      wrist_roll_offset_right_ = parameter.as_double();
    if (parameter.get_name() == "constraints.wrist_lock.mode")
      wrist_roll_offset_target_.store(mode_to_offset(parameter.as_string()));
  }
  return result;
}

#endif // FORCE_DIMENSION_PARAMETERS_H_
