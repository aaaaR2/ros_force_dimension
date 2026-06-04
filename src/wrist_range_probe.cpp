/** Copyright 2026 Carnegie Mellon University Neuromechatronics Lab
 *
 *  This Source Code Form is subject to the terms of the Mozilla Public
 *  License, v. 2.0. If a copy of the MPL was not distributed with this
 *  file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

// Standalone diagnostic for the wrist-flexion lock modes.
//
// Prints the Sigma.7 wrist joint angle RANGE (per-axis min/max) and the CURRENT
// wrist joint angles, in both radians and degrees. Use this to decide the
// left/right roll lock offsets: it shows where roll "starts" (neutral) and how
// far the device can rotate each way, so you can choose a symmetric ±offset or
// an asymmetric range that respects the hardware limits.
//
// SDK joint-slot convention for the Sigma.7 (empirical, matches the node):
//   slot 3 = roll, slot 4 = pitch, slot 5 = yaw.
//
// Usage (device attached; deactivate conda first to avoid the libstdc++ clash):
//   ros2 run force_dimension wrist_range_probe
//   # or directly:
//   ./install/force_dimension/lib/force_dimension/wrist_range_probe

#include <cstdio>

#include "dhdc.h"
#include "drdc.h"

namespace {
constexpr double kRad2Deg = 57.29577951308232;
constexpr int kRoll = 3;
constexpr int kPitch = 4;
constexpr int kYaw = 5;

void print_axis(const char *name, double lo, double hi, double current) {
  std::printf("  %-6s range [% .4f, % .4f] rad   [% 8.2f, % 8.2f] deg"
              "   current % .4f rad (% 7.2f deg)\n",
              name, lo, hi, lo * kRad2Deg, hi * kRad2Deg, current,
              current * kRad2Deg);
}
}  // namespace

int main() {
  // Expert mode must be enabled before opening the device (joint-level access).
  dhdEnableExpertMode();

  const int id = drdOpen();
  if (id < 0) {
    std::printf("ERROR: cannot open Force Dimension device: %s\n",
                dhdErrorGetLastStr());
    return 1;
  }
  std::printf("Device detected: %s (id %d)\n", dhdGetSystemName(), id);

  if (!drdIsSupported()) {
    std::printf("WARNING: device does not support the DRD library; reading raw "
                "values anyway.\n");
  }

  // Initialise if needed so the joint angles are meaningful. AutoInit requires
  // the handle to be near the workspace centre. We continue even on failure —
  // the static joint RANGE is still valid, only the CURRENT angles may be off.
  if (!drdIsInitialized()) {
    std::printf("Device not initialised; running drdAutoInit() "
                "(hold the handle near centre)...\n");
    if (drdAutoInit() < 0) {
      std::printf("WARNING: drdAutoInit failed (%s). Range is still valid; "
                  "current angles may be uncalibrated.\n",
                  dhdErrorGetLastStr());
    }
  }

  double jmin[DHD_MAX_DOF] = {};
  double jmax[DHD_MAX_DOF] = {};
  if (dhdGetJointAngleRange(jmin, jmax, id) < 0) {
    std::printf("ERROR: dhdGetJointAngleRange failed: %s\n",
                dhdErrorGetLastStr());
    drdClose(id);
    return 1;
  }

  double r = 0.0, p = 0.0, y = 0.0;
  dhdGetWristJointAngles(&r, &p, &y, id);

  std::printf("\nWrist joint angle range + current (slots 3/4/5):\n");
  print_axis("roll", jmin[kRoll], jmax[kRoll], r);
  print_axis("pitch", jmin[kPitch], jmax[kPitch], p);
  print_axis("yaw", jmin[kYaw], jmax[kYaw], y);

  std::printf("\nTo lock 90 deg from neutral on roll, the offset is ~%.3f rad "
              "(1.571). Confirm it stays inside the roll range above (minus a "
              "small margin) before using it as roll_offset_left/right_rad.\n",
              1.5707963267948966);

  drdClose(id);
  return 0;
}
