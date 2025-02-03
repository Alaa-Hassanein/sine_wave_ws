# sine_wave_ws

## Description
This repository contains a ROS 2 workspace for generating and publishing sine wave data. It includes nodes for generating sine wave parameters and publishing the sine wave data.

## Repository Structure

## Building the Workspace
To build the workspace, follow these steps:

1. Clone the repository:
    ```sh
    git clone https://github.com/yourusername/sine_wave_ws.git

    cd sine_wave_ws
    
    git submodule update --init --recursive
    ```

2. Install dependencies:
    ```sh
    rosdep install --from-paths src --ignore-src -r -y
    ```

3. Build the workspace:
    ```sh
    colcon build 
    ```

## Running the Nodes
To run the nodes, follow these steps:

1. Source the setup script:
    ```sh
    source install/setup.bash
    ```

2. Launch the python sine wave publisher node:
    ```sh
    ros2 launch sine_wave_pkg sine_wave_python_launch.py
    ```
3. Launch the cpp sine wave publisher node:
    ```sh
    ros2 launch sine_wave_pkg sine_wave_cpp_launch.py
4. Run server and client
    ```sh
    ros2 run sine_wave_pkg gray_image_server.py

    ros2 run sine_wave_pkg gray_image_client.py 
    ```
## TEST
1. Launch sine wave publsisher pytest
    ```sh
    ros2 launch sine_wave_pkg test_sine_wave_publisher_launch.py 
    ```

## Example Usage
After launching the sine wave publisher node, you can visualize the sine wave data using `rqt_plot` adn change parameters with `rqt_reconfigure`:

```sh
ros2 run rqt_plot rqt_plot /sine_wave/data

ros2 run rqt_reconfigure rqt_reconfigure
```
