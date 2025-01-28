#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Copyright (c) 2023 Alaa
# This code is licensed under the MIT License.
# See the LICENSE file in the project root for license terms.

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64
from rcl_interfaces.msg import SetParametersResult
import math


class SineWavePublisher(Node):

    def __init__(self):
        super().__init__('sine_wave_publisher')
        self.string_publisher = self.create_publisher(String, 'string', 10)
        self.string_timer = self.create_timer(1.0, self.string_publish_message)

        self.declare_parameter('topic_name', '/sine_wave')
        self.declare_parameter('amplitude', 1.0)
        self.declare_parameter('phase', 0.0)
        self.declare_parameter('angular_frequency', 1.0)
        self.declare_parameter('frequency', 10.0)

        self.sine_wave_topic = self.get_parameter(
            'topic_name').get_parameter_value().string_value
        self.amplitude = self.get_parameter(
            'amplitude').get_parameter_value().double_value
        self.phase = self.get_parameter(
            'phase').get_parameter_value().double_value
        self.angular_frequency = self.get_parameter(
            'angular_frequency').get_parameter_value().double_value
        self.frequency = self.get_parameter(
            'frequency').get_parameter_value().double_value

        self.add_on_set_parameters_callback(self.parameter_callback)

        # self.amplitude = 1.0
        # self.phase = 0.0
        # self.angular_frequency = 1.0
        # self.frequency = 10.0
        # self.sine_wave_topic = 'sine_wave'

        self.sine_wave_publisher = self.create_publisher(
            Float64, self.sine_wave_topic, 10)
        self.sine_wave_timer = self.create_timer(
            1.0 / self.frequency, self.sine_wave_publish_message)
        self.angle = 0.0

        self.get_logger().info('SineWavePublisher node has been started.')

    def string_publish_message(self):
        msg = String()
        msg.data = 'Hello, world!'
        self.string_publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

    def sine_wave_publish_message(self):
        msg = Float64()
        msg.data = self.amplitude * \
            math.sin(self.angular_frequency *
                     self.angle + self.phase)
        self.sine_wave_publisher.publish(msg)
        self.angle += 0.1

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'amplitude':
                self.amplitude = param.value
            elif param.name == 'phase':
                self.phase = param.value
            elif param.name == 'angular_frequency':
                self.angular_frequency = param.value
            elif param.name == 'frequency':
                self.frequency = param.value
            self.get_logger().info('Parameter %s has been set to: %s' %
                                   (param.name, param.value))
        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    node = SineWavePublisher()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
