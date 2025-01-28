#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Copyright (c) 2023 Alaa
# This code is licensed under the MIT License.
# See the LICENSE file in the project root for license terms.

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64


class SineWaveSubscriber(Node):

    def __init__(self):
        super().__init__('sine_wave_subscriber')
        self.string_subscriber = self.create_subscription(
            String, 'string', self.string_callback, 10)
        self.string_subscriber

        self.declare_parameter('topic_name', '/sine_wave')

        self.sine_wave_topic = self.get_parameter(
            'topic_name').get_parameter_value().string_value

        # self.sine_wave_topic = 'sine_wave'
        self.sine_wave_subscriber = self.create_subscription(
            Float64, self.sine_wave_topic, self.sine_wave_callback, 10)
        self.sine_wave_subscriber

        self.get_logger().info('SineWaveSubscriber node has been started.')

    def string_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

    def sine_wave_callback(self, msg):
        self.get_logger().info('Sine wave value: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = SineWaveSubscriber()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
