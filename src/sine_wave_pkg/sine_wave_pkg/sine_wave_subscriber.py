#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Copyright (c) 2023 Alaa
# This code is licensed under the MIT License.
# See the LICENSE file in the project root for license terms.

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64
from sine_wave_pkg.sine_wave_parameters import sine_wave_params
import math


class SineWaveSubscriber(Node):

    def __init__(self):
        super().__init__('sine_wave_subscriber')

        self.param_listener = sine_wave_params.ParamListener(self)
        self.params = self.param_listener.get_params()

        self.sine_wave_topic = self.params.topic_name

        self.sine_wave_subscriber = self.create_subscription(
            Float64, self.sine_wave_topic, self.sine_wave_callback, 10)
        self.sine_wave_subscriber

        self.get_logger().info('SineWaveSubscriber node has been started.')

    def sine_wave_callback(self, msg):
        data = msg.data
        inverse_sine = math.asin(data)
        data_in_degrees = math.degrees(inverse_sine)
        self.get_logger().info(f'Received sine wave data: {data_in_degrees} degrees')


def main(args=None):
    rclpy.init(args=args)
    node = SineWaveSubscriber()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
