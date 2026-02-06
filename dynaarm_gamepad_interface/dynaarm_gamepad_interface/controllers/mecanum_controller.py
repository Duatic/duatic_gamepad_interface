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
from dynaarm_gamepad_interface.controllers.base_controller import BaseController

from geometry_msgs.msg import TwistStamped


class MecanumController(BaseController):
    """Handles mecanum drive mode."""

    def __init__(self, node, duatic_robots_helper):
        super().__init__(node, duatic_robots_helper)
        self.node.get_logger().info("Initializing mecanum controller.")

        self.needed_low_level_controllers = ["mecanum_drive_controller"]

        # Create publisher for mecanum drive
        self.twist_publisher = self.node.create_publisher(TwistStamped, "cmd_vel", 10)

        # Get control parameters from ROS parameters
        self.max_vel = self.node.declare_parameter("max_vel", 0.6).value
        self.max_accel = self.node.declare_parameter("max_accel", 0.1).value  # m/s²

        # Deadzone for joystick input
        self.deadzone = self.node.declare_parameter("deadzone", 0.25).value

        # Current velocity state for acceleration limiting - ensure valid initialization
        self.current_linear_x = 0.0
        self.current_linear_y = 0.0
        self.current_angular_z = 0.0

        # Time tracking for acceleration calculations
        self.last_time = self.node.get_clock().now()

        # Controller state
        self.is_initialized = False

        self.node.get_logger().info("Mecanum controller initialized")

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

    def _apply_acceleration_limit(self, target_vel, current_vel, max_accel, dt):
        """Apply acceleration limiting to smooth velocity changes."""
        # Extra validation to prevent NaN propagation
        if not self._is_valid_float(target_vel):
            target_vel = 0.0
        if not self._is_valid_float(current_vel):
            current_vel = 0.0
        if not self._is_valid_float(dt) or dt <= 0:
            return current_vel

        vel_diff = target_vel - current_vel
        max_vel_change = max_accel * dt

        # Additional safety check
        if not self._is_valid_float(vel_diff) or not self._is_valid_float(max_vel_change):
            return 0.0

        if abs(vel_diff) <= max_vel_change:
            return target_vel
        else:
            result = current_vel + math.copysign(max_vel_change, vel_diff)
            # Final validation of result
            return result if self._is_valid_float(result) else 0.0

    def process_input(self, joy_msg):
        """Process joystick input and convert to mecanum drive commands."""

        # Calculate time delta
        current_time = self.node.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # Handle first call initialization
        if not self.is_initialized:
            self.is_initialized = True
            dt = 0.02  # Use default timestep for first iteration
            # Send zero command on first call
            self._send_zero_command()
            return

        # Skip if time delta is too small or too large
        if dt <= 0.001 or dt > 0.1:
            dt = 0.02  # Default to 50Hz

        if not joy_msg:
            self.node.get_logger().warn("Joystick message missing (joy_msg is None or empty)")
            self._send_zero_command()
            return

        if not hasattr(joy_msg, "axes"):
            self.node.get_logger().warn("Joystick message missing 'axes' attribute")
            self._send_zero_command()
            return

        if len(joy_msg.axes) < 3:
            self.node.get_logger().warn(
                f"Joystick message has insufficient axes count: {len(joy_msg.axes)} (expected at least 3)"
            )
            self._send_zero_command()
            return

        # Get joystick axes (normalized -1.0 to 1.0) with validation
        try:
            left_stick_x = self._clamp_value(joy_msg.axes[0])  # Left stick X - strafe left/right
            left_stick_y = self._clamp_value(joy_msg.axes[1])  # Left stick Y - forward/backward
            right_stick_x = self._clamp_value(joy_msg.axes[2])  # Right stick X - rotation
        except (IndexError, TypeError) as e:
            self.node.get_logger().warn(f"Error reading joystick axes: {e}")
            self._send_zero_command()
            return

        # Apply deadzone to prevent drift
        if abs(left_stick_x) < self.deadzone:
            left_stick_x = 0.0
        if abs(left_stick_y) < self.deadzone:
            left_stick_y = 0.0
        if abs(right_stick_x) < self.deadzone:
            right_stick_x = 0.0

        # Calculate target velocities
        target_linear_x = left_stick_y * self.max_vel
        target_linear_y = left_stick_x * self.max_vel
        target_angular_z = right_stick_x * self.max_vel

        # Apply acceleration limiting with extra safety
        self.current_linear_x = self._apply_acceleration_limit(
            target_linear_x, self.current_linear_x, self.max_accel, dt
        )
        self.current_linear_y = self._apply_acceleration_limit(
            target_linear_y, self.current_linear_y, self.max_accel, dt
        )
        self.current_angular_z = self._apply_acceleration_limit(
            target_angular_z, self.current_angular_z, self.max_accel, dt
        )

        # Final validation of calculated values
        if not (
            self._is_valid_float(self.current_linear_x)
            and self._is_valid_float(self.current_linear_y)
            and self._is_valid_float(self.current_angular_z)
        ):
            self.node.get_logger().warn("Invalid velocity values calculated, sending zero command")
            self._send_zero_command()
            return

        # Create and send twist message
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
        # Create TwistStamped message
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.node.get_clock().now().to_msg()
        twist_msg.header.frame_id = "base_link"

        # Map joystick inputs to robot velocities
        twist_msg.twist.linear.x = linear_x
        twist_msg.twist.linear.y = linear_y
        twist_msg.twist.linear.z = 0.0

        twist_msg.twist.angular.x = 0.0
        twist_msg.twist.angular.y = 0.0
        twist_msg.twist.angular.z = angular_z

        # Publish the command
        self.twist_publisher.publish(twist_msg)
