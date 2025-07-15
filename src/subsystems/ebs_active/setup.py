import os
from setuptools import setup

here = os.path.abspath(os.path.dirname(__file__))
os.chdir(here)

package_name = 'ebs_active'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools', 'rclpy'],
    zip_safe=True,
    maintainer='Dev',
    maintainer_email='dev@example.com',
    description='Emergency brake system activation node',
    license='MIT',
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
    ],
    entry_points={
        'console_scripts': [
            'ebs_active_node = ebs_active.ebs_active_node:main',
        ],
    },
)
