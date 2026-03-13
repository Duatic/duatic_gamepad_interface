#!/usr/bin/env python3

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

import os
import threading
import yaml
import argparse
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from sensor_msgs.msg import Joy

from duatic_gamepad_interface.controller_manager import ControllerManager
from duatic_gamepad_interface.utils.gamepad_feedback import GamepadFeedback
from duatic_dynaarm_extensions.duatic_helpers.duatic_robots_helper import DuaticRobotsHelper


class GamepadInterface(Node):
    """Processes joystick input"""

    def __init__(self):
        super().__init__("gamepad_interface")

        self.latest_joy_msg = None
        self.joy_lock = threading.Lock()
        self.last_menu_button_state = 0
        self.move_command_active = False  # Track if move_home or move_sleep was executed
        self.deadman_active = False  # Track deadman switch state
        self.last_deadman_state = False  # Track previous deadman state for edge detection

        # Publishers
        self.move_home_pub = self.create_publisher(Bool, "move_home", 10)
        self.move_sleep_pub = self.create_publisher(Bool, "move_sleep", 10)

        # Subscribers
        self.create_subscription(Joy, "joy", self.joy_callback, 10)

        # Load gamepad mappings from YAML
        config_path = os.path.join(
            get_package_share_directory("duatic_gamepad_interface"),
            "config",
            "gamepad_config.yaml",
        )
        with open(config_path) as file:
            config = yaml.safe_load(file)["gamepad_node"]["ros__parameters"]

        # Load configurations
        self.button_mapping = config["button_mapping"]
        self.axis_mapping = config["axis_mapping"]
        self.get_logger().info(f"Loaded gamepad config: {self.button_mapping}, {self.axis_mapping}")

        self.duatic_robots_helper = DuaticRobotsHelper(self)
        self.duatic_robots_helper.wait_for_robot()
        self.controller_manager = ControllerManager(self, self.duatic_robots_helper)

        self.gamepad_feedback = GamepadFeedback(self)

        # Set the timing based on simulation or real hardware
        self.dt = self.duatic_robots_helper.get_dt()

        self.create_timer(self.dt, self.process_joy_input)
        self.get_logger().info("Gamepad Interface Initialized.")

    def set_dt(self):

        self.is_simulation = self.duatic_robots_helper.check_simulation_mode()

        if self.is_simulation:
            self.dt = 0.05
            self.get_logger().info("Using simulation timing: dt=0.05s (20Hz)")
        else:
            self.dt = 0.001
            self.get_logger().info("Using real hardware timing: dt=0.001s (1000Hz)")

    def joy_callback(self, msg: Joy):
        """Store latest joystick message."""
        with self.joy_lock:
            self.latest_joy_msg = msg

    def process_joy_input(self):
        """Process latest joystick input."""
        with self.joy_lock:
            msg = self.latest_joy_msg  # Get the latest stored joystick input

        if msg is None:
            return

        # Check deadman switch and track state changes
        current_deadman_state = msg.buttons[self.button_mapping["dead_man_switch"]] == 1
        deadman_just_released = self.deadman_active and not current_deadman_state
        deadman_just_pressed = not self.deadman_active and current_deadman_state
        self.deadman_active = current_deadman_state

        if deadman_just_pressed:
            self._reset_current_controller()

        # 2. Handle Move Command Blocking (Safety Check)
        can_move = self.deadman_active and not self.controller_manager.is_freeze_active

        if not can_move:
            if self.move_command_active or deadman_just_released:
                self._stop_move_commands()
        else:
            # Allowed to move -> Process Home/Sleep commands
            move_home_pressed = msg.buttons[self.button_mapping["move_home"]]
            move_sleep_pressed = msg.buttons[self.button_mapping["move_sleep"]]

            if move_home_pressed:
                self.move_home_pub.publish(Bool(data=True))
                self.move_command_active = True
                return
            if move_sleep_pressed:
                self.move_sleep_pub.publish(Bool(data=True))
                self.move_command_active = True
                return

            if self.move_command_active:
                self._stop_move_commands()

        # Use dynamically loaded menu button index
        switch_controller_index = self.button_mapping["switch_controller"]
        # Ensure switching happens only on button press (down event) and not while held down
        if msg.buttons[switch_controller_index] == 1 and self.last_menu_button_state == 0:
            self.controller_manager.switch_to_next_controller()
            # Reset current controller after switching
            self._reset_current_controller()

        # Wait until button is released (0) before allowing another switch
        # And don't execute anything else when the button is pressed
        self.last_menu_button_state = msg.buttons[switch_controller_index]
        if self.last_menu_button_state:
            return

        self.controller_manager.gripper_controller.process_input(msg)

        # Now get the current active controller from the controller manager:
        current_controller = self.controller_manager.get_current_controller()

        if current_controller is not None:
            if self.controller_manager.is_freeze_active:
                # If freeze is active, we don't process any input
                current_controller.reset()
            else:
                # Pass deadman state so controller knows if it can move
                current_controller.process_input(msg)

    def _stop_move_commands(self):
        """Stop move_home and move_sleep commands and reset controller."""
        self.move_home_pub.publish(Bool(data=False))
        self.move_sleep_pub.publish(Bool(data=False))
        self.move_command_active = False
        # Reset current controller after move commands are stopped
        self._reset_current_controller()

    def _reset_current_controller(self):
        """Reset the current active controller if it exists."""
        current_controller = self.controller_manager.get_current_controller()
        if current_controller is not None:
            current_controller.reset()


def main(args=None):

    parser = argparse.ArgumentParser()
    parsed_args, unknown = parser.parse_known_args()  # ← ignore ROS args

    rclpy.init(args=unknown)  # ← pass remaining args to rclpy
    node = GamepadInterface()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
