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


from duatic_dynaarm_extensions.duatic_helpers.duatic_controller_helper import DuaticControllerHelper
from duatic_dynaarm_extensions.duatic_helpers.duatic_robots_helper import DuaticRobotsHelper
from duatic_gamepad_interface.controllers.joint_trajectory_controller import (
    JointTrajectoryController,
)
from duatic_gamepad_interface.controllers.freedrive_controller import FreedriveController
from duatic_gamepad_interface.controllers.mecanum_controller import MecanumController
from duatic_gamepad_interface.controllers.gripper_controller import GripperController


class ControllerManager:
    """Handle controllers"""

    def __init__(self, node, duatic_robots_helper: DuaticRobotsHelper):
        self.node = node

        self.active_high_level_controller_index = -1
        self.active_low_level_controllers = []
        self.is_freeze_active = True  # Assume freeze is active until proven otherwise
        self.emergency_button_was_pressed = False

        self.duatic_controller_helper = DuaticControllerHelper(self.node)

        # Initialize all potential controllers
        self.all_potential_controllers = {
            0: FreedriveController(self.node, duatic_robots_helper),
            1: JointTrajectoryController(self.node, duatic_robots_helper),
            # 2: CartesianController(self.node, duatic_robots_helper),
            2: MecanumController(self.node, duatic_robots_helper),
        }

        self.gripper_controller = GripperController(self.node, duatic_robots_helper)

        # Check which controllers are actually available and create filtered list
        self.all_high_level_controllers = {}
        self._filter_available_controllers()

        self.node.create_timer(0.2, self.check_active_low_level_controllers)

    def _filter_available_controllers(self):
        """Check which controllers are available and filter out unavailable ones."""
        self.node.get_logger().info("Checking controller availability...")

        # Get all available low-level controllers from the system
        self.duatic_controller_helper.wait_for_controller_data()
        all_system_controllers = self.duatic_controller_helper.get_all_controllers()

        if not all_system_controllers:
            self.node.get_logger().warn("No controllers found in the system!")
            return

        self.node.get_logger().info(f"Available system controllers: {all_system_controllers}")

        available_controllers = {}
        new_index = 0

        for idx, controller in self.all_potential_controllers.items():
            controller_name = controller.__class__.__name__

            if hasattr(controller, "get_low_level_controllers"):
                required_controllers = controller.get_low_level_controllers()

                # Check if all required low-level controllers are available
                all_required_available = True
                missing_controllers = []

                for required in required_controllers:
                    # Check if any system controller contains the required controller name
                    controller_found = any(
                        required in system_controller
                        for system_controller in all_system_controllers
                    )
                    if not controller_found:
                        all_required_available = False
                        missing_controllers.append(required)

                if all_required_available:
                    available_controllers[new_index] = controller
                    self.node.get_logger().info(
                        f"✅ {controller_name} (index {new_index}) - Available (requires: {required_controllers})"
                    )
                    new_index += 1
                else:
                    self.node.get_logger().warn(
                        f"❌ {controller_name} - Unavailable (missing: {missing_controllers})"
                    )
            else:
                self.node.get_logger().warn(
                    f"❌ {controller_name} - No get_low_level_controllers method"
                )

        self.all_high_level_controllers = available_controllers

        if not self.all_high_level_controllers:
            self.node.get_logger().error(
                "No controllers are available! Check your controller configuration."
            )
        else:
            controller_names = [
                c.__class__.__name__ for c in self.all_high_level_controllers.values()
            ]
            self.node.get_logger().info(f"Available high-level controllers: {controller_names}")

    def get_current_controller(self):
        if self.active_high_level_controller_index < 0:
            return None

        if self.active_high_level_controller_index not in self.all_high_level_controllers:
            return None

        return self.all_high_level_controllers[self.active_high_level_controller_index]

    def wait_for_controller_loaded(self, controller_name, timeout=60.0):
        return self.duatic_controller_helper.wait_for_controller_loaded(controller_name, timeout)

    def wait_for_controller_data(self):
        return self.duatic_controller_helper.wait_for_controller_data()

    def check_active_low_level_controllers(self):
        """Checks which controllers are active and updates the state machine."""

        self.is_freeze_active = self.duatic_controller_helper.is_freeze_active()

        if self.is_freeze_active and not self.emergency_button_was_pressed:
            self.node.get_logger().warn(
                "       ⚠️   Emergency stop is ACTIVE!   ⚠️           \n"
                "\t\t\t\t\tTo deactivate: Hold Left Stick Button (LSB) or L1 for ~4s."
            )
            self.emergency_button_was_pressed = True
            self.is_freeze_active = True
        elif not self.is_freeze_active and self.emergency_button_was_pressed:
            self.node.get_logger().warn("    ✅   Emergency stop is DEACTIVATED!   ✅           ")
            self.is_freeze_active = False
            self.emergency_button_was_pressed = False

        active_low_level_controllers = self.duatic_controller_helper.get_active_controllers()
        if not active_low_level_controllers or len(active_low_level_controllers) <= 0:
            self.node.get_logger().warn("No active controller found.", throttle_duration_sec=30.0)
            self.active_high_level_controller_index = -1
            self.active_low_level_controllers.clear()
            return

        # Compare the lists of active_low_level_controllers
        if active_low_level_controllers == self.active_low_level_controllers:
            self.node.get_logger().debug(
                f"Active low-level controllers remain unchanged: {active_low_level_controllers}"
            )
            return

        # Only set high-level controller if none was set before
        if self.active_high_level_controller_index >= 0:
            self.node.get_logger().debug(
                f"High level controller already set: {self.active_high_level_controller_index}"
            )
            return

        # Find the best matching controller (one with most required controllers satisfied)
        best_controller_idx = -1
        best_controller_score = 0

        for idx, high_level_controller in self.all_high_level_controllers.items():
            if hasattr(high_level_controller, "get_low_level_controllers"):
                required = set(high_level_controller.get_low_level_controllers())
                active = set(active_low_level_controllers)

                # Check if all required controllers are "contained" in active controllers
                required_found = True
                for req_controller in required:
                    controller_found = any(
                        req_controller in active_controller for active_controller in active
                    )
                    if not controller_found:
                        required_found = False
                        break

                self.node.get_logger().debug(
                    f"Checking controller {idx}: required={required}, active={active}, all_required_found={required_found}"
                )

                # If all required controllers are found, check if this is the best match
                if required_found:
                    score = len(required)  # Score based on number of required controllers
                    if score > best_controller_score:
                        best_controller_idx = idx
                        best_controller_score = score

        # Activate the best matching controller
        if best_controller_idx >= 0:
            self.active_high_level_controller_index = best_controller_idx
            best_controller = self.all_high_level_controllers[best_controller_idx]
            best_controller.reset()
            self.node.get_logger().info(
                f"Activated high level controller index: {best_controller_idx}"
            )

    def switch_to_next_controller(self):
        """Switch to the next high-level controller. Only switch low-level controller if needed."""

        num_controllers = len(self.all_high_level_controllers)
        if num_controllers <= 0:
            self.node.get_logger().warn("No available controllers to switch to!")
            return

        # Get the current index in the available controllers
        current_idx = self.active_high_level_controller_index

        # Get list of available indices and find next one
        available_indices = sorted(self.all_high_level_controllers.keys())

        if current_idx not in available_indices:
            # If current index is invalid, start with first available
            next_high_level_controller_index = available_indices[0]
        else:
            # Find current position and get next one (wrap around)
            current_position = available_indices.index(current_idx)
            next_position = (current_position + 1) % len(available_indices)
            next_high_level_controller_index = available_indices[next_position]

        next_high_level_controller = self.all_high_level_controllers[
            next_high_level_controller_index
        ]
        next_low_level_controllers = next_high_level_controller.get_low_level_controllers()
        active_low_level_controllers = self.duatic_controller_helper.get_active_controllers()

        if not next_low_level_controllers:
            self.node.get_logger().warn(
                "No low-level controller defined for the next high-level controller."
            )
            return

        # Always update the high-level controller index
        self.active_high_level_controller_index = next_high_level_controller_index
        self.node.get_logger().info(
            f"Switching to high-level controller index: {next_high_level_controller_index} ({next_high_level_controller.__class__.__name__})"
        )

        matching_controllers = self.duatic_controller_helper.get_all_controllers(
            next_low_level_controllers
        )

        controllers_to_activate = []
        for controller in matching_controllers:
            # Only switch low-level controller if it is different
            if controller in active_low_level_controllers:
                self.node.get_logger().debug(f"Already using controller: {controller}")
            else:
                controllers_to_activate.append(controller)
                self.node.get_logger().debug(f"Will activate controller: {controller}")

        controllers_to_deactivate = []
        for controller in active_low_level_controllers:
            if controller not in matching_controllers:
                controllers_to_deactivate.append(controller)
                self.node.get_logger().debug(f"Will deactivate controller: {controller}")

        self.duatic_controller_helper.switch_controller(
            controllers_to_activate, controllers_to_deactivate
        )

        for idx, high_level_controller in self.all_high_level_controllers.items():
            high_level_controller.reset()
