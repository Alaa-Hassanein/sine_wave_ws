from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'sine_wave_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/params', glob('params/*.yaml')),
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
