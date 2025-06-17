import os
from setuptools import setup

here = os.path.abspath(os.path.dirname(__file__))
os.chdir(here)

package_name = 'path_viz'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dev',
    maintainer_email='dev@example.com',
    description='Camera pose and path visualization node',
    license='MIT',
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
    ],
    entry_points={
        'console_scripts': [
            'path_viz_node = path_viz.path_viz_node:main',
        ],
    },
)
