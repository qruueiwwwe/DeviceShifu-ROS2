from setuptools import setup
import os
from glob import glob

package_name = 'deviceshifu_ros2_driver'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='miku',
    maintainer_email='your_email@example.com',
    description='DeviceShifu ROS2 driver package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'driver_node = deviceshifu_ros2_driver.driver_node:main',
        ],
    },
)