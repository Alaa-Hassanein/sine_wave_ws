# -*- coding: utf-8 -*-
# Copyright (c) 2023 Alaa
# This code is licensed under the MIT License.
# See the LICENSE file in the project root for license terms.

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

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
test_dir = os.path.join(workspace_directory, "install", "sine_wave_pkg", "lib","sine_wave_pkg","test_sine_wave_publisher.py")



def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['python3', '-m', 'pytest', test_dir],
            output='screen'
        ),
    ])