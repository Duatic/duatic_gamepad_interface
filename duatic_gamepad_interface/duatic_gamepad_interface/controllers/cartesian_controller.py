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


from geometry_msgs.msg import PoseStamped

from tf_transformations import quaternion_from_euler, quaternion_multiply
from duatic_gamepad_interface.controllers.base_controller import BaseController

from duatic_dynaarm_extensions.duatic_helpers.duatic_marker_helper import DuaticMarkerHelper
from duatic_dynaarm_extensions.duatic_helpers.duatic_pinocchio_helper import DuaticPinocchioHelper
from duatic_dynaarm_extensions.duatic_helpers.duatic_robots_helper import DuaticRobotsHelper


class CartesianController(BaseController):
    """Handles Cartesian control mode and publishes a visualization marker."""

    def __init__(self, node, duatic_robots_helper):
        super().__init__(node, duatic_robots_helper)

        self.node.get_logger().info("Initializing cartesian controller.")

        self.ee_frame = "flange"
        self.current_pose = None
        self.scale = 0.05
        self.mirror = self.node.get_parameter("mirror").get_parameter_value().bool_value

        self.needed_low_level_controllers = [
            "cartesian_pose_controller",
        ]

        self.arms = self.duatic_robots_helper.get_component_names("arm")
        found_topics = self.duatic_jtc_helper.find_topics_for_controller(
            "cartesian_pose_controller", "target_pose", self.arms
        )

        response = self.duatic_jtc_helper.process_topics_and_extract_joint_names(found_topics)
        self.topic_to_joint_names = response[0]
        self.topic_to_commanded_poses = response[1]
        for topic, _ in self.topic_to_joint_names.items():
            self.topic_to_commanded_poses[topic] = PoseStamped()

        # Create publishers for each pose controller topic
        self.cartesian_publishers = {}
        for topic in self.topic_to_commanded_poses.keys():
            self.cartesian_publishers[topic] = self.node.create_publisher(PoseStamped, topic, 10)
            self.node.get_logger().debug(f"Created publisher for topic: {topic}")

        if len(self.arms) >= 2:
            self.base_frame = "tbase"
            self.pin_helper = DuaticPinocchioHelper(self.node, robot_type="Alpha")
        else:
            self.base_frame = "world"
            self.pin_helper = DuaticPinocchioHelper(self.node)

        self.marker_helper = DuaticMarkerHelper(self.node)

        self.node.get_logger().info("Cartesian controller initialized.")

    def _get_name_for_arm(self, arm_name, frame_name):
        """Get frame name for specific arm"""
        if arm_name:
            frame_with_arm = f"{arm_name}/{frame_name}"
            return frame_with_arm

        return f"{frame_name}"

    def reset(self):
        """Resets the current_pose to the current one"""
        self.marker_helper.clear_markers()
        current_joint_values = self.duatic_robots_helper.get_joint_states()

        for topic in self.topic_to_commanded_poses.keys():
            arm_name = self.get_arm_from_topic(topic)
            frame_name = self._get_name_for_arm(arm_name, self.ee_frame)
            self.topic_to_commanded_poses[topic] = self.pin_helper.get_fk_as_pose_stamped(
                current_joint_values, frame_name, self.base_frame
            )
            self.topic_to_commanded_poses[topic].header.frame_id = self.base_frame

    def process_input(self, msg):
        """Processes joystick input and updates Cartesian position for all arms."""
        super().process_input(msg)

        # Initialize poses if not already done
        first_topic = list(self.topic_to_commanded_poses.keys())[0]
        if self.topic_to_commanded_poses[first_topic].header.frame_id == "":
            print("Resetting poses...")
            self.reset()

        # Get input values
        x = msg.axes[self.node.axis_mapping["left_joystick"]["y"]]
        y = -1 * msg.axes[self.node.axis_mapping["left_joystick"]["x"]]
        z = msg.axes[self.node.axis_mapping["right_joystick"]["y"]]
        roll = msg.axes[self.node.axis_mapping["right_joystick"]["x"]]
        pitch = float(msg.buttons[self.node.button_mapping["wrist_rotation_left"]]) - float(
            msg.buttons[self.node.button_mapping["wrist_rotation_right"]]
        )
        yaw = float(msg.axes[self.node.axis_mapping["triggers"]["left"]] > 0.5) - float(
            msg.axes[self.node.axis_mapping["triggers"]["right"]] > 0.5
        )

        # Prioritization logic
        if abs(pitch) > 1e-4:
            lx, ly = 0.0, 0.0
        elif abs(x) > abs(y) and abs(x) > 1e-4:
            lx, ly = x, 0.0
        elif abs(y) > 1e-4:
            lx, ly = 0.0, y
        else:
            lx, ly = 0.0, 0.0

        if abs(pitch) > 1e-4:
            lz, d_roll = 0.0, 0.0
        elif abs(z) > abs(roll) and abs(z) > 1e-4:
            lz, d_roll = z, 0.0
        elif abs(roll) > 1e-4:
            lz, d_roll = 0.0, roll
        else:
            lz, d_roll = 0.0, 0.0

        # Scaling
        linear_speed = 0.2
        angular_speed = 0.3

        # Process each arm/topic
        for topic, current_pose in self.topic_to_commanded_poses.items():

            arm_name = self.get_arm_from_topic(topic)

            if not self.mirror and arm_name == "arm_right":
                continue

            # Apply mirroring for right arm if mirror is enabled
            if self.mirror and arm_name == "arm_right":
                # Mirror the Y-axis movement and yaw rotation for the right arm
                mirror_lx = lx
                mirror_ly = -ly  # Mirror Y movement
                mirror_lz = lz
                mirror_d_roll = d_roll
                mirror_pitch = pitch
                mirror_yaw = -yaw  # Mirror yaw rotation
            else:
                # Use original values for left arm or when mirroring is disabled
                mirror_lx = lx
                mirror_ly = ly
                mirror_lz = lz
                mirror_d_roll = d_roll
                mirror_pitch = pitch
                mirror_yaw = yaw

            # Update Position
            current_pose.pose.position.x += mirror_lx * linear_speed * self.scale
            current_pose.pose.position.y += mirror_ly * linear_speed * self.scale
            current_pose.pose.position.z += mirror_lz * linear_speed * self.scale

            # Update Orientation (Apply Incremental Rotations)
            d_roll_scaled = mirror_d_roll * angular_speed * self.scale
            pitch_scaled = mirror_pitch * angular_speed * self.scale
            yaw_scaled = mirror_yaw * angular_speed * self.scale

            q_roll = quaternion_from_euler(d_roll_scaled, 0, 0)
            q_pitch = quaternion_from_euler(0, pitch_scaled, 0)
            q_yaw = quaternion_from_euler(0, 0, yaw_scaled)

            current_q = current_pose.pose.orientation
            q_current = [current_q.x, current_q.y, current_q.z, current_q.w]
            q_new = quaternion_multiply(q_current, q_roll)
            q_new = quaternion_multiply(q_new, q_pitch)
            q_new = quaternion_multiply(q_new, q_yaw)
            norm = (q_new[0] ** 2 + q_new[1] ** 2 + q_new[2] ** 2 + q_new[3] ** 2) ** 0.5
            q_new = [q / norm for q in q_new]

            current_pose.pose.orientation.x = q_new[0]
            current_pose.pose.orientation.y = q_new[1]
            current_pose.pose.orientation.z = q_new[2]
            current_pose.pose.orientation.w = q_new[3]

            # Update header
            current_pose.header.frame_id = self.base_frame
            current_pose.header.stamp = self.node.get_clock().now().to_msg()

            # Publish to the corresponding arm
            self.cartesian_publishers[topic].publish(current_pose)

            # Create markers for visualization
            self.marker_helper.create_pose_markers(current_pose, self.base_frame, arm_name + "_")
