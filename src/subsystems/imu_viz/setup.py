import os
from setuptools import setup

here = os.path.abspath(os.path.dirname(__file__))
os.chdir(here)

package_name = 'imu_viz'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools', 'numpy', 'rclpy'],
    zip_safe=True,
    maintainer='Dev',
    maintainer_email='dev@example.com',
    description='Standalone IMU visualization node used by multiple subsystems',
    license='MIT',
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
    ],
    entry_points={
        'console_scripts': [
            'imu_viz_node = imu_viz.imu_viz_node:main',
        ],
    },
)
