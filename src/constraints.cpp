/** Workspace constraint force computation for the Force Dimension haptic node.
 *
 *  Implements per-axis channel (bilateral wall) and circle (radial boundary)
 *  dead-zone constraints, following the SDK constraints.cpp convention:
 *    - Zero force inside the dead-zone boundary.
 *    - Spring along the constraint normal outside the boundary.
 *    - Damping projected onto the normal only (tangential motion is free).
 *    - No explicit force clamping; SDK motor saturation handles limits.
 */

/** Copyright 2026 Neuromechatronics Lab, Carnegie Mellon University
 *
 *  This Source Code Form is subject to the terms of the Mozilla Public
 *  License, v. 2.0. If a copy of the MPL was not distributed with this
 *  file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

// Include guard.
#ifndef FORCE_DIMENSION_CONSTRAINTS_CPP_
#define FORCE_DIMENSION_CONSTRAINTS_CPP_

#include "node.hpp"
#include <cmath>

// Scope namespace elements.
using force_dimension::Node;

/** Compute workspace constraint forces from current SDK position and velocity.
 *
 *  Reads from constraints_ (cached ROS2 parameters). On the first call,
 *  auto-captures channel offsets and circle center from the device position
 *  (device autocenters to ~(0,0,0) on startup via DRD).
 *
 *  Channels: each axis (X=0, Y=1, Z=2) has an independent bilateral dead-zone
 *  wall. Any subset can be enabled independently at runtime.
 *
 *  Circle: radial boundary in the XY plane (axes 0 and 1).
 *
 *  @param pos       Current device position [x, y, z] in meters.
 *  @param vel       Current device velocity [vx, vy, vz] in m/s.
 *  @param force_out Output constraint force [fx, fy, fz] in Newtons.
 */
void Node::ComputeConstraintForce(const double pos[3], const double vel[3],
                                  double force_out[3]) {
  force_out[0] = 0.0;
  force_out[1] = 0.0;
  force_out[2] = 0.0;

  // Auto-home: capture channel offsets and circle center from the first
  // valid SDK position read. Since the device autocenters to (0,0,0) on
  // startup, these will be near zero — but we capture exactly for accuracy.
  if (!constraints_.homed) {
    for (int i = 0; i < 3; ++i)
      constraints_.channel_offset[i] = pos[i];

    // Determine the circle plane: the two axes orthogonal to the first
    // enabled channel axis (or default X=0 if none enabled).
    int primary_axis = 0;
    for (int i = 0; i < 3; ++i) {
      if (constraints_.channel_enabled[i]) { primary_axis = i; break; }
    }
    int ax0 = (primary_axis == 0) ? 1 : 0;
    int ax1 = (primary_axis == 2) ? 1 : 2;
    constraints_.circle_plane_axes[0] = ax0;
    constraints_.circle_plane_axes[1] = ax1;
    // Circle center = device position + home_offset, so it tracks the
    // configured offset position regardless of any residual drdMoveTo error.
    constraints_.circle_center[0] = pos[ax0] + constraints_.home_offset[0];
    constraints_.circle_center[1] = pos[ax1] + constraints_.home_offset[1];
    constraints_.homed = true;

    {
      std::string message = "Constraints homed: offsets=(";
      message += std::to_string(constraints_.channel_offset[0]);
      message += ", ";
      message += std::to_string(constraints_.channel_offset[1]);
      message += ", ";
      message += std::to_string(constraints_.channel_offset[2]);
      message += ") circle_plane=";
      message += std::to_string(ax0);
      message += std::to_string(ax1);
      message += " circle_center=(";
      message += std::to_string(constraints_.circle_center[0]);
      message += ", ";
      message += std::to_string(constraints_.circle_center[1]);
      message += ")";
      Log(message);
    }
  }

  const double Kp = constraints_.stiffness;
  const double Kv = constraints_.damping;

  // --- Per-axis channel constraints (bilateral dead-zone walls) ---
  // Normal direction is the axis itself. Damping on the normal component only.
  for (int ax = 0; ax < 3; ++ax) {
    if (!constraints_.channel_enabled[ax]) continue;

    const double displacement = pos[ax] - constraints_.channel_offset[ax];
    const double half_width   = constraints_.channel_half_width[ax];
    const double depth_pos    = displacement - half_width;
    const double depth_neg    = -displacement - half_width;

    if (depth_pos > 0.0) {
      // Past the positive wall — spring pushes inward (-). Damp only outward velocity (vel > 0).
      const double damp = (vel[ax] > 0.0) ? -Kv * vel[ax] : 0.0;
      force_out[ax] += -Kp * depth_pos + damp;
    } else if (depth_neg > 0.0) {
      // Past the negative wall — spring pushes inward (+). Damp only outward velocity (vel < 0).
      const double damp = (vel[ax] < 0.0) ? -Kv * vel[ax] : 0.0;
      force_out[ax] +=  Kp * depth_neg + damp;
    }
  }

  // --- Circle constraint (radial dead-zone boundary in the task plane) ---
  // Plane is orthogonal to the primary channel axis (e.g. YZ when channel is X).
  // Normal is the outward radial direction. Damping projected onto it only.
  // Circle uses the same global stiffness/damping as the channels.
  if (constraints_.circle_enabled) {
    const int ax0 = constraints_.circle_plane_axes[0];
    const int ax1 = constraints_.circle_plane_axes[1];

    const double dp0 = pos[ax0] - constraints_.circle_center[0];
    const double dp1 = pos[ax1] - constraints_.circle_center[1];
    const double r   = std::sqrt(dp0 * dp0 + dp1 * dp1);
    const double depth = r - constraints_.circle_radius;

    if (depth > 0.0 && r > 1e-9) {
      // Outward unit normal in the task plane.
      const double n0 = dp0 / r;
      const double n1 = dp1 / r;

      // Radial velocity component (projection onto outward normal).
      const double v_radial = vel[ax0] * n0 + vel[ax1] * n1;

      // Spring inward + damping only when moving outward (v_radial > 0).
      const double damp = (v_radial > 0.0) ? Kv * v_radial : 0.0;
      const double f_mag = -(Kp * depth + damp);

      force_out[ax0] += f_mag * n0;
      force_out[ax1] += f_mag * n1;
    }
  }
}

#endif // FORCE_DIMENSION_CONSTRAINTS_CPP_
