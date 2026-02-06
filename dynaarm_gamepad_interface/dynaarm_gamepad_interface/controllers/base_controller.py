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

from dynaarm_extensions.duatic_helpers.duatic_robots_helper import DuaticRobotsHelper
from dynaarm_extensions.duatic_helpers.duatic_jtc_helper import DuaticJTCHelper


class BaseController:
    """Base class for all controllers, providing logging and common methods."""

    def __init__(self, node, duatic_robots_helper: DuaticRobotsHelper):
        self.node = node
        self.log_printed = False  # Track whether the log was printed
        self.needed_low_level_controllers = None
        self.joint_pos_offset_tolerance = 0.1

        self.duatic_robots_helper = duatic_robots_helper
        self.duatic_jtc_helper = DuaticJTCHelper(self.node)

    def get_low_level_controllers(self):
        """Returns the name of the low-level controller this controller is based on."""
        return self.needed_low_level_controllers

    def process_input(self, joy_msg):
        """Override this in child classes."""
        pass

    def reset(self):
        """Reset controller state when switching back to this controller."""
        self.log_printed = False  # Reset logging state

    def get_arm_from_topic(self, topic):
        """Extract arm name from topic like '/joint_trajectory_controller_arm_left/joint_trajectory'"""
        if "arm_left" in topic:
            return "arm_left"
        elif "arm_right" in topic:
            return "arm_right"
        return ""
