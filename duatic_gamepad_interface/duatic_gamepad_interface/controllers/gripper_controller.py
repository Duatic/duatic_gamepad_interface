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

from rclpy.qos import QoSProfile
from std_msgs.msg import Float64MultiArray


class GripperController(BaseController):
    """Handles gripper control."""

    def __init__(self, node, duatic_robots_helper):
        super().__init__(node, duatic_robots_helper)

        self.needed_low_level_controllers = ["gripper_controller"]

        # Gripper publisher setup
        self.gripper_topic_suffix = "gripper_controller/commands"
        self.gripper_pub = None
        self._setup_gripper_publisher()

        # Track gripper state and button press
        self._gripper_open = False
        self._last_button_state = False

    def _setup_gripper_publisher(self):
        # Get node namespace
        ns = self.node.get_namespace().rstrip("/")
        all_topics = [t[0] for t in self.node.get_topic_names_and_types()]

        # Look for namespaced topic
        self.gripper_topic = None
        for topic in all_topics:
            if topic.endswith(self.gripper_topic_suffix):
                self.gripper_topic = topic
                break

        if self.gripper_topic:
            qos_profile = QoSProfile(depth=1)
            self.gripper_pub = self.node.create_publisher(
                Float64MultiArray, self.gripper_topic, qos_profile
            )
            self.node.get_logger().info(f"Gripper publisher created on {self.gripper_topic}")
        else:
            self.node.get_logger().warn(
                f"Gripper topic {self.gripper_topic_suffix} not available in namespace {ns}."
            )

    def send_gripper_command(self, position: float):
        """Send a command to the gripper if publisher is available."""
        if self.gripper_pub is not None:
            msg = Float64MultiArray()
            msg.data = [position]
            self.gripper_pub.publish(msg)
            self.node.get_logger().info(f"Sent gripper command: {position}")
        else:
            self.node.get_logger().warn("Gripper publisher not available.")

    def process_input(self, joy_msg):
        # Toggle gripper state with button 0
        if hasattr(joy_msg, "buttons") and len(joy_msg.buttons) > 0:
            button_pressed = bool(joy_msg.buttons[0])
            if button_pressed and not self._last_button_state:
                # Toggle state
                self._gripper_open = not self._gripper_open
                position = 1.0 if self._gripper_open else 0.0
                self.send_gripper_command(position)
            self._last_button_state = button_pressed
