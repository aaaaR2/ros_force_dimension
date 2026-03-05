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
  }
  return result;
}

#endif // FORCE_DIMENSION_PARAMETERS_H_
