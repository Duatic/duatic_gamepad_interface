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

from sensor_msgs.msg import JoyFeedback
from rclpy.node import Node


class GamepadFeedback:
    """
    A helper class to send force-feedback (rumble) commands via the ROS joy feedback mechanism.
    """

    def __init__(self, node: Node):
        """
        Initializes the GamepadFeedback instance.

        :param node: The ROS node used for creating publishers and logging.
        :param topic: The topic to publish JoyFeedbackArray messages on.
        """
        self.node = node
        self.publisher = self.node.create_publisher(JoyFeedback, "joy/set_feedback", 10)

    def send_feedback(self, intensity: float):
        """
        Sends a rumble feedback command.

        :param intensity: A value between 0 and 1 indicating the feedback intensity.
        """

        feedback = JoyFeedback()
        feedback.type = JoyFeedback.TYPE_RUMBLE
        feedback.id = 0
        feedback.intensity = intensity
        self.publisher.publish(feedback)
