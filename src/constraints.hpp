/** Copyright 2026 Neuromechatronics Lab, Carnegie Mellon University
 *
 *  This Source Code Form is subject to the terms of the Mozilla Public
 *  License, v. 2.0. If a copy of the MPL was not distributed with this
 *  file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

// Workspace constraint configuration for the Force Dimension haptic node.
// Fields are cached from ROS2 parameters and read at 2 kHz in ApplyHapticForce.
// Updated at runtime via the parameter callback — no get_parameter() in the hot path.

#ifndef FORCE_DIMENSION_CONSTRAINTS_H_
#define FORCE_DIMENSION_CONSTRAINTS_H_

namespace force_dimension {

/** Cached state for channel and circle workspace constraints.
 *
 *  channels: up to 3 independent bilateral dead-zone walls, one per Cartesian
 *            axis (index 0=X, 1=Y, 2=Z). Each can be enabled independently.
 *  circle:   radial dead-zone boundary in the XY task plane (axes 0 and 1).
 *
 *  All constraints share stiffness and damping. Damping is projected onto the
 *  constraint normal only (tangential motion free), matching SDK constraints.cpp.
 *  No explicit force clamping — SDK motor saturation handles limits.
 */
struct ConstraintState {

  // Per-axis channel constraints (index 0=X, 1=Y, 2=Z).
  bool   channel_enabled[3]    = {false, false, false};
  double channel_offset[3]     = {0.0, 0.0, 0.0};           // m, auto-captured on first read
  double channel_half_width[3] = {0.003, 0.003, 0.003};      // m, dead-zone half-width (±)

  // Circle constraint (radial boundary in the plane orthogonal to the primary channel axis).
  // Plane axes are always the two axes that are NOT the primary channel axis (0=X by default).
  // circle_plane_axes[0] and [1] are set at home time from the active channel config.
  bool   circle_enabled       = false;
  double circle_radius        = 0.11;        // m
  double circle_center[2]     = {0.0, 0.0};  // in-plane center = device pos + offset, auto-set at home
  double circle_center_offset[2] = {0.0, 0.0}; // m, fixed offset applied on top of homed position
  int    circle_plane_axes[2] = {1, 2};      // default: YZ plane (channel on X)

  // Shared spring-damper
  double stiffness = 2000.0; // N/m
  double damping   = 50.0;   // N/(m/s), projected onto constraint normal

  // True once offsets and circle_center have been captured from the first
  // valid SDK position read. Reset to false on re-activation.
  bool homed = false;
};

} // namespace force_dimension

#endif // FORCE_DIMENSION_CONSTRAINTS_H_
