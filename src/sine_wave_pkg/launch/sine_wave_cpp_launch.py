#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Copyright (c) 2023 Alaa
# This code is licensed under the MIT License.
# See the LICENSE file in the project root for license terms.

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value="/home/alaa/sine_wave_ws/src/sine_wave_pkg/params/sine_wave_params.yaml",
                description="Path to the configuration file to load",
            ),
            Node(
                package="sine_wave_pkg",
                executable="sine_wave_publisher_cpp",
                name="sine_wave_publisher",
                output="screen",
                parameters=[LaunchConfiguration("config_file")],
            ),
            Node(
                package="sine_wave_pkg",
                executable="sine_wave_subscriber.py",
                name="sine_wave_subscriber",
                output="screen",
                parameters=[LaunchConfiguration("config_file")],
            ),
        ]
    )
