import os
from setuptools import setup, find_packages

here = os.path.abspath(os.path.dirname(__file__))
os.chdir(here)

package_name = 'art_slam'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=[package_name, f"{package_name}.*"]),
    install_requires=[
        'setuptools',
        'rclpy',
        'numpy',
        'scipy',
    ],
    zip_safe=True,
    maintainer='Nicolas Müller',
    maintainer_email='nicolas.mueller@strohmleitung.de',
    description='Simple cone SLAM example',
    license='MIT',
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            [f'resource/{package_name}'],
        ),
        (f'share/{package_name}', ['package.xml', 'launch/slam.launch.py']),
    ],
    entry_points={
        'console_scripts': [
            'art_slam_node = art_slam.art_slam_node:main',
        ],
    },
)
