from setuptools import find_packages, setup
from glob import glob
import os

from generate_parameter_library_py.setup_helper import generate_parameter_module

generate_parameter_module(
  "sine_wave_parameters", # python module name for parameter library
  os.path.join(os.getenv('AMENT_PREFIX_PATH').split(os.pathsep)[0],
                                       'share', 'sine_wave_pkg','params', 'sine_wave_prams_sturcture.yaml'), # path to input yaml file
)
package_name = 'sine_wave_pkg'

setup(
    name=package_name,
    version='0.0.2',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/params', glob('params/*.yaml')),
        ('share/' + package_name + '/images', glob('images/*.jpg')),
        #('share/' + package_name + '/srv', glob('srv/*.srv')),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alaa Hassanein',
    maintainer_email='alaahg81@gmail.com',
    description='This package contains ROS2 nodes for publishing and subscribing to sine wave values.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'sine_wave_publisher = sine_wave_pkg.sine_wave_publisher:main',
        'sine_wave_subscriber = sine_wave_pkg.sine_wave_subscriber:main',
        ],
    },
)
