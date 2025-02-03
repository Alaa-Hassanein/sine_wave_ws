import pytest
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from ament_index_python.packages import get_package_share_directory
from sine_wave_publisher import SineWavePublisher
import yaml
import os


@pytest.fixture(scope='module')
def rclpy_init_shutdown():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node(rclpy_init_shutdown):
    node = SineWavePublisher()


    yield node
    node.destroy_node()

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
    




def test_sine_wave_publish_message(node):
    msg = Float64()
    node.sine_wave_publish_message()
    assert isinstance(msg.data, float)


def test_update_timer(node):
    old_timer = node.sine_wave_timer
    node.update_timer(2.0)
    assert node.frequency == 2.0
    assert node.sine_wave_timer != old_timer


def test_parameter_callback(node):
    params = [
        rclpy.parameter.Parameter(
            'amplitude', rclpy.Parameter.Type.DOUBLE, 2.0),
        rclpy.parameter.Parameter('phase', rclpy.Parameter.Type.DOUBLE, 1.0),
        rclpy.parameter.Parameter(
            'angular_frequency', rclpy.Parameter.Type.DOUBLE, 0.5),
        rclpy.parameter.Parameter(
            'frequency', rclpy.Parameter.Type.DOUBLE, 0.5)
    ]
    result = node.parameter_callback(params)
    assert result.successful
    assert node.amplitude == params[0].value
    assert node.phase == params[1].value
    assert node.angular_frequency == params[2].value
    assert node.frequency == params[3].value
