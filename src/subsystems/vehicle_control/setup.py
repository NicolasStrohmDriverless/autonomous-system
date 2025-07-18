import os
from setuptools import setup, find_packages

here = os.path.abspath(os.path.dirname(__file__))
os.chdir(here)

package_name = 'vehicle_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=[package_name, f"{package_name}.*"]),
    install_requires=['setuptools', 'rclpy', 'numpy'],
    zip_safe=True,
    maintainer='Dev',
    maintainer_email='dev@example.com',
    description='Vehicle control skeleton nodes',
    license='MIT',
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/launch', []),
    ],
    entry_points={
        'console_scripts': [
            'car_state_node = vehicle_control.car_state_node:main',
            'mapping_node = vehicle_control.mapping_node:main',
            'slam_node = vehicle_control.slam_node:main',
            'vehicle_controll_node = vehicle_control.vehicle_controll_node:main',
        ],
    },
)
