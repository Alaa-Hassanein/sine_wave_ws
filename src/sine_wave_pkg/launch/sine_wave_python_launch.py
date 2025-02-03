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

def get_workspace_directory(current_path):

        path_parts = current_path.split(os.sep)

        if "install" in path_parts:
            workspace_index = path_parts.index("install") - 1
        elif "src" in path_parts:
            workspace_index = path_parts.index("src") - 1
        else:
            raise ValueError("Workspace directory not found in the path")

        workspace_directory = os.sep.join(path_parts[: workspace_index + 1])
        return workspace_directory
    
workspace_directory = get_workspace_directory(os.path.abspath(__file__))
params_dir = os.path.join(workspace_directory, "src", "sine_wave_pkg", "params","sine_wave_params.yaml")

def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=params_dir,
                description="Path to the configuration file to load",
            ),
            Node(
                package="sine_wave_pkg",
                executable="sine_wave_publisher.py",
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
