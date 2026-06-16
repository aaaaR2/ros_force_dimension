/** Copyright 2022 Neuromechatronics Lab, Carnegie Mellon University (a.whit)
 *
 *  Created by: a. whit. (nml@whit.contact)
 *
 *  This Source Code Form is subject to the terms of the Mozilla Public
 *  License, v. 2.0. If a copy of the MPL was not distributed with this
 *  file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

// Interface between ROS2 and the Force Dimension SDK for haptic robots.

// Include guard.
#ifndef FORCE_DIMENSION_TOPICS_H_
#define FORCE_DIMENSION_TOPICS_H_

/** Topic for publishing robot position feedback.
 *
 */

namespace force_dimension {

template <typename MessageT> struct topic_data {
  char name;
  MessageT message;
};

/** Topic for publishing robot position feedback.
 *
 */
const char POSITION_FEEDBACK_TOPIC[] = "feedback/position";

/** Topic for publishing button press feedback.
 *
 */
const char BUTTON_FEEDBACK_TOPIC[] = "feedback/button";

/** Topic for publishing robot velocity feedback.
 *
 */
const char VELOCITY_FEEDBACK_TOPIC[] = "feedback/velocity";

/** Topic for publishing robot force feedback.
 *
 */
const char FORCE_FEEDBACK_TOPIC[] = "feedback/force";

/** Topic for subscribing to robot force commands.
 *
 */
const char FORCE_COMMAND_TOPIC[] = "command/force";

/** Topic for gripper gap width feedback.
 *
 */
const char GRIPPER_GAP_FEEDBACK_TOPIC[] = "feedback/gripper_gap";

/** Topic for gripper angle feedback.
 *
 */
const char GRIPPER_ANGLE_FEEDBACK_TOPIC[] = "feedback/gripper_angle";

/** Topic for wrist orientation feedback.
 *
 */
const char ORIENTATION_FEEDBACK_TOPIC[] = "feedback/orientation";

/** Topic for raw wrist joint angles (rad): x=w0 roll, y=w1 pitch, z=w2 yaw.
 *
 */
const char WRIST_JOINT_FEEDBACK_TOPIC[] = "feedback/wrist_joint_angles";

/** Topic for synchronized device state feedback.
 *
 */
const char DEVICE_STATE_FEEDBACK_TOPIC[] = "feedback/state";

/** Topic for the full-rate raw data-collection sample stream
 *  (force_dimension_msgs/RawSample), recorded into the session mcap.
 */
const char RAW_SAMPLE_FEEDBACK_TOPIC[] = "feedback/raw_sample";

/** Topic for selecting the wrist lock mode (upright / left / right).
 *  Subscribed as std_msgs/String; Unity publishes here to switch the locked
 *  roll orientation live within a recording session.
 */
const char WRIST_MODE_COMMAND_TOPIC[] = "wrist_lock/set_mode";

/** Topic for publishing the active wrist lock mode (feedback for Unity). */
const char WRIST_MODE_FEEDBACK_TOPIC[] = "wrist_lock/mode";

} // namespace force_dimension

#endif // FORCE_DIMENSION_TOPICS_H_
