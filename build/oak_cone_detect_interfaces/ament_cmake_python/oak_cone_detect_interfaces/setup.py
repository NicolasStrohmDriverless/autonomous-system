from setuptools import find_packages
from setuptools import setup

setup(
    name='oak_cone_detect_interfaces',
    version='0.0.1',
    packages=find_packages(
        include=('oak_cone_detect_interfaces', 'oak_cone_detect_interfaces.*')),
)
