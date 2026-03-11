# Copyright 2026 Duatic AG
#
# Redistribution and use in source and binary forms, with or without modification, are permitted provided that
# the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this list of conditions, and
#    the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions, and
#    the following disclaimer in the documentation and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or
#    promote products derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
# WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
# PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
# TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from duatic_gamepad_interface.controllers.base_controller import BaseController
import math
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint



class JointTrajectoryController(BaseController):
    """Handles joint trajectory control using the gamepad"""

    MIRRORED_BASES = {"shoulder_rotation", "forearm_rotation", "wrist_rotation"}

    def __init__(self, node, duatic_robots_helper):
        super().__init__(node, duatic_robots_helper)
        self.node.get_logger().info("Initializing joint trajectory controller.")

        # Discover all relevant components
        self.arms = self.duatic_robots_helper.get_component_names("arm")
        self.hips = self.duatic_robots_helper.get_component_names("hip")
        self.all_components = self.arms + self.hips

        found_topics = self.duatic_jtc_helper.find_topics_for_controller(
            "joint_trajectory_controller", "joint_trajectory", self.all_components
        )
        response = self.duatic_jtc_helper.process_topics_and_extract_joint_names(found_topics)
        self.topic_to_joint_names = response[0]
        self.topic_to_commanded_positions = response[1]
        for topic, joint_names in self.topic_to_joint_names.items():
            self.topic_to_commanded_positions[topic] = [0.0] * len(joint_names)

        # Build mirrored joints
        self.mirror = self.node.get_parameter("mirror").get_parameter_value().bool_value
        self.mirrored_joints = []
        self._build_mirrored_joints()

        # Focus management
        self.last_dpad_msg = {"x": 0.0, "y": 0.0, "buttons": [0]*15}
        self.focused_component = "arm_left" if "arm_left" in self.all_components else (self.all_components[0] if self.all_components else "")

        # Create publishers for each joint trajectory topic
        self.joint_trajectory_publishers = {}
        for topic in self.topic_to_joint_names.keys():
            self.joint_trajectory_publishers[topic] = self.node.create_publisher(
                JointTrajectory, topic, 10
            )

        self.prefix_to_joints = {}
        self.is_joystick_idle = True

        # Dominant axis tracking
        self.dominant_axis_threshold = 0.6
        self.active_axes = {
            "left_joystick": {"x": False, "y": False},
            "right_joystick": {"x": False, "y": False},
        }

        if self.mirrored_joints:
            self.node.get_logger().info(f"Mirrored joints: {self.mirrored_joints}")

        self.node.get_logger().info("Joint Trajectory Controller initialized.")

    def get_low_level_controllers(self):
        """Returns specific LLC names based on focus."""
        if self.mirror:
            # Mirror mode requires both arms
            return ["joint_trajectory_controller_arm_left", "joint_trajectory_controller_arm_right"]

        # Return the specific controller for the focus
        for topic in self.topic_to_joint_names.keys():
            if f"_{self.focused_component}/" in topic or topic.endswith(f"_{self.focused_component}"):
                # Extract the controller name part: joint_trajectory_controller_arm_left
                segments = topic.strip("/").split("/")
                if len(segments) >= 2:
                    return [segments[-2]]

        return ["joint_trajectory_controller"]

    def reset(self):
        """Reset commanded positions to current joint states for all topics."""
        joint_states = self.duatic_robots_helper.get_joint_states()

        for topic, joint_names in self.topic_to_joint_names.items():
            self.topic_to_commanded_positions[topic] = [
                joint_states.get(joint, 0.0) for joint in joint_names
            ]

    def _update_focus(self, joy_msg):
        """Update focus based on D-Pad input (axes 6,7 or buttons 11-14)."""
        dpad_x = 0.0
        dpad_y = 1.0 if (len(joy_msg.buttons) > 11 and joy_msg.buttons[11] and not self.last_dpad_msg["buttons"][11]) else (0.0)
        
        # Check Axes 6/7 (Standard for many controllers)
        if len(joy_msg.axes) > 7:
            if abs(joy_msg.axes[6]) > 0.5: dpad_x = joy_msg.axes[6]
            if abs(joy_msg.axes[7]) > 0.5: dpad_y = joy_msg.axes[7]
            
        # Check Buttons (Fallback or overlay)
        # 11=Up, 12=Down, 13=Left, 14=Right
        if len(joy_msg.buttons) > 14:
            if joy_msg.buttons[11] and not self.last_dpad_msg["buttons"][11]: dpad_y = 1.0
            if joy_msg.buttons[12] and not self.last_dpad_msg["buttons"][12]: dpad_y = -1.0
            if joy_msg.buttons[13] and not self.last_dpad_msg["buttons"][13]: dpad_x = 1.0
            if joy_msg.buttons[14] and not self.last_dpad_msg["buttons"][14]: dpad_x = -1.0

        new_focus = None
        if dpad_y > 0.5 and self.last_dpad_msg["y"] <= 0.5:
            new_focus = "hip"
        elif dpad_y < -0.5 and self.last_dpad_msg["y"] >= -0.5:
            new_focus = "platform"
        elif dpad_x > 0.5 and self.last_dpad_msg["x"] <= 0.5:
            new_focus = "arm_left"
        elif dpad_x < -0.5 and self.last_dpad_msg["x"] >= -0.5:
            new_focus = "arm_right"

        self.last_dpad_msg["x"] = dpad_x
        self.last_dpad_msg["y"] = dpad_y
        if len(joy_msg.buttons) > 14:
            self.last_dpad_msg["buttons"] = list(joy_msg.buttons)

        if new_focus and new_focus in self.all_components:
            self.node.get_logger().info(f"Switching focus to: {new_focus}")
            self.focused_component = new_focus
            self.reset()
            # Trigger LLC sync through main controller manager
            if hasattr(self.node, "controller_manager"):
                self.node.controller_manager.trigger_llc_sync()
            self.node.gamepad_feedback.send_feedback(intensity=0.5)


    def process_input(self, msg):
        """Processes joystick input, integrates over dt, and clamps the commanded positions."""
        # Focus selection is independent of deadman in HLC mode
        self._update_focus(msg)

        super().process_input(msg)

        any_axis_active = False
        deadzone = 0.1

        # Determine which axes are currently dominant
        left_x = msg.axes[self.node.axis_mapping["left_joystick"]["x"]] if len(msg.axes) > self.node.axis_mapping["left_joystick"]["x"] else 0.0
        left_y = msg.axes[self.node.axis_mapping["left_joystick"]["y"]] if len(msg.axes) > self.node.axis_mapping["left_joystick"]["y"] else 0.0
        right_x = msg.axes[self.node.axis_mapping["right_joystick"]["x"]] if len(msg.axes) > self.node.axis_mapping["right_joystick"]["x"] else 0.0
        right_y = msg.axes[self.node.axis_mapping["right_joystick"]["y"]] if len(msg.axes) > self.node.axis_mapping["right_joystick"]["y"] else 0.0

        self._update_dominant_axes(left_x, left_y, right_x, right_y, deadzone)

        # Process each topic
        for topic, joint_names in self.topic_to_joint_names.items():
            arm_name = self.get_arm_from_topic(topic)

            # Normal mode: Only process the focused component
            if not self.mirror and arm_name != self.focused_component:
                continue
            
            # Mirror mode: Only process Arm Left (which then mirrors to Arm Right)
            if self.mirror and arm_name == "arm_right":
                continue

            # Safety: Only move if deadman is active
            if not self.node.deadman_active:
                continue


            commanded_positions = self.topic_to_commanded_positions[topic]
            for i, joint_name in enumerate(joint_names):
                axis_val = 0.0
                effective_deadzone = deadzone

                # Joint Mapping
                if i == 0:
                    axis_val = left_x
                    if self.active_axes["left_joystick"]["y"] and not self.active_axes["left_joystick"]["x"]: effective_deadzone = self.dominant_axis_threshold
                elif i == 1:
                    axis_val = left_y
                    if self.active_axes["left_joystick"]["x"] and not self.active_axes["left_joystick"]["y"]: effective_deadzone = self.dominant_axis_threshold
                elif i == 2:
                    axis_val = right_y
                    if self.active_axes["right_joystick"]["x"] and not self.active_axes["right_joystick"]["y"]: effective_deadzone = self.dominant_axis_threshold
                elif i == 3:
                    axis_val = right_x
                    if self.active_axes["right_joystick"]["y"] and not self.active_axes["right_joystick"]["x"]: effective_deadzone = self.dominant_axis_threshold
                elif i == 4:
                    left_trigger = msg.axes[self.node.axis_mapping["triggers"]["left"]]
                    right_trigger = msg.axes[self.node.axis_mapping["triggers"]["right"]]
                    axis_val = right_trigger - left_trigger
                elif i == 5:
                    move_left = msg.buttons[self.node.button_mapping["wrist_rotation_left"]] == 1
                    move_right = msg.buttons[self.node.button_mapping["wrist_rotation_right"]] == 1
                    if move_left: axis_val = -1.0
                    elif move_right: axis_val = 1.0

                if self.mirror and joint_name in self.mirrored_joints:
                    axis_val = -axis_val

                if abs(axis_val) > effective_deadzone:
                    current_position = self.duatic_robots_helper.get_joint_value_from_states(joint_name)
                    commanded_positions[i] += axis_val * self.node.dt
                    offset = commanded_positions[i] - current_position
                    if abs(offset) > self.joint_pos_offset_tolerance:
                        commanded_positions[i] = current_position + math.copysign(self.joint_pos_offset_tolerance, offset)
                        self.node.gamepad_feedback.send_feedback(intensity=1.0)
                    any_axis_active = True

            self.topic_to_commanded_positions[topic] = commanded_positions

        # Publish
        if any_axis_active:
            self.is_joystick_idle = False
            for topic, publisher in self.joint_trajectory_publishers.items():
                arm_name = self.get_arm_from_topic(topic)
                if self.mirror:
                    self.publish_joint_trajectory(self.topic_to_commanded_positions[topic], publisher, self.topic_to_joint_names[topic])
                elif arm_name == self.focused_component:
                    self.publish_joint_trajectory(self.topic_to_commanded_positions[topic], publisher, self.topic_to_joint_names[topic])
        elif not any_axis_active and not self.is_joystick_idle:
            for topic, publisher in self.joint_trajectory_publishers.items():
                arm_name = self.get_arm_from_topic(topic)
                if self.mirror or arm_name == self.focused_component:
                    self.publish_joint_trajectory(self.topic_to_commanded_positions[topic], publisher, self.topic_to_joint_names[topic])
            self.is_joystick_idle = True


    def publish_joint_trajectory(
        self, target_positions, publisher, joint_names, speed_percentage=1.0
    ):
        """Publishes a joint trajectory message for the given positions using the provided publisher."""

        if not joint_names:
            self.node.get_logger().error("No joint names available. Cannot publish trajectory.")
            return

        if not target_positions:
            self.node.get_logger().error("No trajectory points available to publish.")
            return

        # Clamp speed percentage between 1 and 100
        speed_percentage = max(1.0, min(100.0, speed_percentage))

        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = joint_names
        point = JointTrajectoryPoint()
        point.positions = target_positions
        point.velocities = [0.0] * len(joint_names)
        point.accelerations = [0.0] * len(joint_names)
        time_in_sec = self.node.dt
        sec = int(time_in_sec)
        nanosec = int((time_in_sec - sec) * 1e9)
        point.time_from_start.sec = sec
        point.time_from_start.nanosec = nanosec
        trajectory_msg.points.append(point)
        publisher.publish(trajectory_msg)

    def _update_dominant_axes(self, left_x, left_y, right_x, right_y, deadzone):
        """Update which axes are currently active to determine dominant axis behavior."""
        # Update active axes based on current input
        self.active_axes["left_joystick"]["x"] = abs(left_x) > deadzone
        self.active_axes["left_joystick"]["y"] = abs(left_y) > deadzone
        self.active_axes["right_joystick"]["x"] = abs(right_x) > deadzone
        self.active_axes["right_joystick"]["y"] = abs(right_y) > deadzone

    def _build_mirrored_joints(self):
        """Build list of joints that should be mirrored based on MIRRORED_BASES."""
        all_joints = []
        for joint_names in self.topic_to_joint_names.values():
            all_joints.extend(joint_names)

        for base in self.MIRRORED_BASES:
            found = []
            for joint in all_joints:
                if joint.endswith("/" + base) or joint.endswith("_" + base) or joint == base:
                    found.append(joint)
            if len(found) == 2:
                found_sorted = sorted(found)
                self.mirrored_joints.append(found_sorted[1])

