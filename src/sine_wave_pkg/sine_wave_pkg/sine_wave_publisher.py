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
from sine_wave_pkg.sine_wave_parameters import sine_wave_params


class SineWavePublisher(Node):

    def __init__(self):
        super().__init__('sine_wave_publisher')

        self.param_listener = sine_wave_params.ParamListener(self)
        self.params = self.param_listener.get_params()

        self.amplitude = self.params.amplitude
        self.phase = self.params.phase
        self.angular_frequency = self.params.angular_frequency
        self.frequency = self.params.frequency
        self.sine_wave_topic = self.params.topic_name

        self.add_on_set_parameters_callback(self.parameter_callback)

        self.sine_wave_publisher = self.create_publisher(
            Float64, self.sine_wave_topic, 10)
        self.sine_wave_timer = self.create_timer(
            1.0 / self.frequency, self.sine_wave_publish_message)
        self.angle = 0.0

        self.get_logger().info('SineWavePublisher node has been started.')

    def update_timer(self, new_frequency):
        self.frequency = new_frequency
        self.sine_wave_timer.cancel()
        self.sine_wave_timer = self.create_timer(
            1.0 / self.frequency, self.sine_wave_publish_message)

    def sine_wave_publish_message(self):

        if self.param_listener.is_old(self.params):
            self.params = self.param_listener.get_params()

        msg = Float64()
        msg.data = self.amplitude * \
            math.sin(self.angular_frequency * self.angle + self.phase)
        self.sine_wave_publisher.publish(msg)
        self.angle += 0.0174533

        self.update_timer(self.frequency)

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
