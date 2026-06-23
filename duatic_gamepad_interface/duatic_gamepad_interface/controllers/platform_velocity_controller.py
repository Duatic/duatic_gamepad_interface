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

import math
from duatic_gamepad_interface.controllers.base_controller import BaseController

from geometry_msgs.msg import TwistStamped


class PlatformVelocityController(BaseController):
    """Handles platform velocity controller drive mode."""

    def __init__(self, node, duatic_robots_helper):
        super().__init__(node, duatic_robots_helper)
        self.needed_capabilities = ["mobility"]
        self.node.get_logger().info("Initializing platform velocity controller.")

        self.base_needed_llcs = ["platform_velocity_controller"]
        self.potential_freeze_llcs = [
            "freeze_controller_hip",
            "freeze_controller_arm_left",
            "freeze_controller_arm_right",
        ]

        # Create publisher for platform drive
        self.twist_publisher = self.node.create_publisher(TwistStamped, "cmd_vel_smoothed", 10)

        def _param(name, default):
            if self.node.has_parameter(name):
                return self.node.get_parameter(name).value
            return self.node.declare_parameter(name, default).value

        # Get control parameters from ROS parameters
        self.max_vel = _param("max_vel", 0.6)        # m/s
        self.accel_limit = _param("accel_limit", 0.5)  # m/s²
        self.decel_limit = _param("decel_limit", 1.0)  # m/s²
        self.deadzone = _param("deadzone", 0.05)

        # Current velocity state for acceleration limiting
        self.current_linear_x = 0.0
        self.current_linear_y = 0.0
        self.current_angular_z = 0.0

        # Time tracking for acceleration calculations
        self.last_time = self.node.get_clock().now()

        # Controller state
        self.is_initialized = False

        self.node.get_logger().info("Platform velocity controller initialized")

    def get_low_level_controllers(self):
        """Returns the names of low-level controllers needed for platform velocity mode."""
        needed = list(self.base_needed_llcs)

        available_freezes = self.duatic_controller_helper.get_all_controllers(
            self.potential_freeze_llcs
        )
        needed.extend(available_freezes)

        return needed

    def _is_valid_float(self, value):
        """Check if a value is a valid finite float."""
        return isinstance(value, (int, float)) and math.isfinite(value)

    def _clamp_value(self, value, min_val=-1.0, max_val=1.0):
        """Clamp value to specified range and ensure it's valid."""
        if not self._is_valid_float(value):
            return 0.0
        return max(min_val, min(max_val, value))

    def reset(self):
        """Reset commanded positions to current joint states for all topics."""
        self._send_zero_command()

    def _apply_acceleration_limit(self, target_vel, current_vel, dt):
        """Apply acceleration limiting to smooth velocity changes."""
        if not self._is_valid_float(target_vel):
            target_vel = 0.0
        if not self._is_valid_float(current_vel):
            current_vel = 0.0
        if not self._is_valid_float(dt) or dt <= 0:
            return current_vel

        if abs(target_vel) < abs(current_vel) or target_vel * current_vel < 0:
            max_vel_change = self.decel_limit * dt
        else:
            max_vel_change = self.accel_limit * dt

        vel_diff = target_vel - current_vel

        if not self._is_valid_float(vel_diff) or not self._is_valid_float(max_vel_change):
            return 0.0

        if abs(vel_diff) <= max_vel_change:
            return target_vel
        else:
            result = current_vel + math.copysign(max_vel_change, vel_diff)
            return result if self._is_valid_float(result) else 0.0

    def process_input(self, joy_msg):
        """Process joystick input and convert to platform velocity commands."""
        if not self.node.deadman_active:
            return

        current_time = self.node.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        if not self.is_initialized:
            self.is_initialized = True
            dt = 0.02
            self._send_zero_command()
            return

        if dt <= 0.001 or dt > 0.1:
            dt = 0.02

        if not joy_msg:
            self._send_zero_command()
            return

        if not hasattr(joy_msg, "axes") or len(joy_msg.axes) < 3:
            self._send_zero_command()
            return

        try:
            left_stick_x = self._clamp_value(joy_msg.axes[0])
            left_stick_y = self._clamp_value(joy_msg.axes[1])
            right_stick_x = self._clamp_value(joy_msg.axes[2])
        except (IndexError, TypeError):
            self._send_zero_command()
            return

        if abs(left_stick_x) < self.deadzone:
            left_stick_x = 0.0
        if abs(left_stick_y) < self.deadzone:
            left_stick_y = 0.0
        if abs(right_stick_x) < self.deadzone:
            right_stick_x = 0.0

        target_linear_x = left_stick_y * self.max_vel
        target_linear_y = left_stick_x * self.max_vel
        target_angular_z = right_stick_x * self.max_vel

        self.current_linear_x = self._apply_acceleration_limit(
            target_linear_x, self.current_linear_x, dt
        )
        self.current_linear_y = self._apply_acceleration_limit(
            target_linear_y, self.current_linear_y, dt
        )
        self.current_angular_z = self._apply_acceleration_limit(
            target_angular_z, self.current_angular_z, dt
        )

        if not (
            self._is_valid_float(self.current_linear_x)
            and self._is_valid_float(self.current_linear_y)
            and self._is_valid_float(self.current_angular_z)
        ):
            self._send_zero_command()
            return

        self._send_twist_command(
            self.current_linear_x, self.current_linear_y, self.current_angular_z
        )

    def _send_zero_command(self):
        """Send a zero velocity command to stop the robot safely."""
        self.current_linear_x = 0.0
        self.current_linear_y = 0.0
        self.current_angular_z = 0.0
        self._send_twist_command(0.0, 0.0, 0.0)

    def _send_twist_command(self, linear_x, linear_y, angular_z):
        """Send a twist command with proper validation."""
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.node.get_clock().now().to_msg()
        twist_msg.header.frame_id = "base_link"

        twist_msg.twist.linear.x = linear_x
        twist_msg.twist.linear.y = linear_y
        twist_msg.twist.linear.z = 0.0

        twist_msg.twist.angular.x = 0.0
        twist_msg.twist.angular.y = 0.0
        twist_msg.twist.angular.z = angular_z

        self.twist_publisher.publish(twist_msg)
