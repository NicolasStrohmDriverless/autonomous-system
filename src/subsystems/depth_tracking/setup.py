import os
from setuptools import setup

here = os.path.abspath(os.path.dirname(__file__))
os.chdir(here)

package_name = 'depth_tracking'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nicolas Müller',
    maintainer_email='nicolas.mueller@strohmleitung.de',
    description='Standalone depth tracking node',
    license='MIT',
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            [f'resource/{package_name}'],
        ),
        (f'share/{package_name}', ['package.xml']),
    ],
    entry_points={
        'console_scripts': [
            'depth_node = depth_tracking.depth_node:main',
        ],
    },
)
