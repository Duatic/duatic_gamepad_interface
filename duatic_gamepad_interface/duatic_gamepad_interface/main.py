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
        self.dpad_mapping = config.get("dpad_mapping", {})
        self.last_dpad_state = {"x": 0.0, "y": 0.0, "buttons": {}}
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

        # Handle focus switching (independent of deadman)
        self._update_focus(msg)

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

    def _update_focus(self, joy_msg):
        """Update focus based on D-Pad input from config/gamepad_config.yaml."""
        if not self.dpad_mapping:
            return

        dpad_x = 0.0
        dpad_y = 0.0

        # Check Axes (Standard for Xbox/Hat-Switch D-Pads)
        axes_cfg = self.dpad_mapping.get("axes", {})
        ax_x = axes_cfg.get("x")
        ax_y = axes_cfg.get("y")

        if ax_x is not None and len(joy_msg.axes) > ax_x:
            if abs(joy_msg.axes[ax_x]) > 0.5:
                dpad_x = joy_msg.axes[ax_x]
        if ax_y is not None and len(joy_msg.axes) > ax_y:
            if abs(joy_msg.axes[ax_y]) > 0.5:
                dpad_y = joy_msg.axes[ax_y]

        # Check Buttons (Standard for PS4/PS5 D-Pads)
        btns_cfg = self.dpad_mapping.get("buttons", {})
        up_idx = btns_cfg.get("up")
        down_idx = btns_cfg.get("down")
        left_idx = btns_cfg.get("left")
        right_idx = btns_cfg.get("right")

        def btn_pressed(idx):
            if idx is None or len(joy_msg.buttons) <= idx:
                return False
            return joy_msg.buttons[idx] == 1 and self.last_dpad_state["buttons"].get(idx, 0) == 0

        if btn_pressed(up_idx):
            dpad_y = 1.0
        if btn_pressed(down_idx):
            dpad_y = -1.0
        if btn_pressed(left_idx):
            dpad_x = 1.0
        if btn_pressed(right_idx):
            dpad_x = -1.0

        # Determine target focus
        targets = self.dpad_mapping.get("focus_targets", {})
        new_focus = None

        if dpad_y > 0.5 and self.last_dpad_state["y"] <= 0.5:
            new_focus = targets.get("up")
        elif dpad_y < -0.5 and self.last_dpad_state["y"] >= -0.5:
            new_focus = targets.get("down")
        elif dpad_x > 0.5 and self.last_dpad_state["x"] <= 0.5:
            new_focus = targets.get("left")
        elif dpad_x < -0.5 and self.last_dpad_state["x"] >= -0.5:
            new_focus = targets.get("right")

        # Update last state
        self.last_dpad_state["x"] = dpad_x
        self.last_dpad_state["y"] = dpad_y
        for key, idx in btns_cfg.items():
            if idx is not None and len(joy_msg.buttons) > idx:
                self.last_dpad_state["buttons"][idx] = joy_msg.buttons[idx]

        if new_focus:
            current_controller = self.controller_manager.get_current_controller()
            if current_controller:
                # Check if component exists in the robot
                all_components = self.duatic_robots_helper.get_component_names("arm") + \
                                 self.duatic_robots_helper.get_component_names("hip")
                
                # Special case for "platform" which might be handled by mecanum
                if new_focus in all_components or new_focus == "platform":
                    self.get_logger().info(f"Switching focus to: {new_focus}")
                    current_controller.set_focus(new_focus)
                    current_controller.reset()
                    self.controller_manager.trigger_llc_sync()
                    self.gamepad_feedback.send_feedback(intensity=0.5)


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
