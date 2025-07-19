import os
from setuptools import setup, find_packages

here = os.path.abspath(os.path.dirname(__file__))
os.chdir(here)

package_name = 'pathfinding'

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
    description='Pathfinding subsystem for cone detection',
    license='MIT',
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            [f'resource/{package_name}'],
        ),
        (f'share/{package_name}', ['package.xml']),
        # install steering wheel image for runtime access
        (f'share/{package_name}/resource', ['resource/f1_wheel.jpg']),
    ],
    entry_points={
        'console_scripts': [
            'pathfinding_node = pathfinding.pathfinding_node:main',
        ],
    },
)
