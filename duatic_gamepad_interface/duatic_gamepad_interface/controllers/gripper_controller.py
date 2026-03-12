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

import rclpy
from rclpy.qos import QoSProfile

from std_msgs.msg import Float64MultiArray


class GripperController(BaseController):
    """Handles gripper control."""

    def __init__(self, node, duatic_robots_helper):
        super().__init__(node, duatic_robots_helper)

        self.needed_low_level_controllers = ["gripper_controller"]
        self.gripper_topic_suffix = "gripper_controller/commands"

        # Dictionary to store publishers: {component_name: publisher}
        self.gripper_publishers = {}
        self._setup_gripper_publishers()

        # Track gripper state per component
        self._gripper_states = {}
        self._last_button_state = False

    def _setup_gripper_publishers(self):
        """Discover and create publishers for all available grippers with retries."""
        max_retries = 20
        retry_count = 0
        qos_profile = QoSProfile(depth=1)

        while retry_count < max_retries:
            all_topics = [t[0] for t in self.node.get_topic_names_and_types()]
            found_any = False

            for topic in all_topics:
                if topic.endswith(self.gripper_topic_suffix):
                    # Extract component name from topic e.g. /arm_left/gripper_controller/commands
                    component = self.get_arm_from_topic(topic)
                    if component and component not in self.gripper_publishers:
                        self.gripper_publishers[component] = self.node.create_publisher(
                            Float64MultiArray, topic, qos_profile
                        )
                        self._gripper_states[component] = False
                        self.node.get_logger().info(
                            f"Gripper publisher created for {component} on {topic}"
                        )
                        found_any = True

            # If we already have some publishers, we can be more lenient, but let's try to find all possible ones
            # For now, if we found at least one or enough retries passed, we continue
            if self.gripper_publishers:
                # Give it a bit more time if we think there should be more, but don't block forever
                if retry_count > 5:
                    break

            rclpy.spin_once(self.node, timeout_sec=0.2)
            retry_count += 1

    def send_gripper_command(self, component, position: float):
        """Send a command to a specific gripper."""
        pub = self.gripper_publishers.get(component)
        if pub:
            msg = Float64MultiArray()
            msg.data = [position]
            pub.publish(msg)
            self.node.get_logger().info(f"Sent gripper command to {component}: {position}")
        else:
            self.node.get_logger().warn(f"No gripper publisher for {component}")

    def process_input(self, joy_msg):
        # Safety: Only process gripper if deadman is active
        if not self.node.deadman_active:
            return

        # Determine current focus

        current_hlc = self.node.controller_manager.get_current_controller()
        if not current_hlc:
            return

        focus = current_hlc.get_focus()

        # Toggle gripper state with button 0
        if hasattr(joy_msg, "buttons") and len(joy_msg.buttons) > 0:
            button_pressed = bool(joy_msg.buttons[0])
            if button_pressed and not self._last_button_state:
                # Toggle state for focused component
                is_open = not self._gripper_states.get(focus, False)
                self._gripper_states[focus] = is_open
                position = 1.0 if is_open else 0.0
                self.send_gripper_command(focus, position)
            self._last_button_state = button_pressed
