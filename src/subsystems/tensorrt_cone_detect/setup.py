# src/subsystems/tensorrt_cone_detect/setup.py
import os
from setuptools import setup

# ensure we run in the package directory
here = os.path.abspath(os.path.dirname(__file__))
os.chdir(here)

package_name = 'tensorrt_cone_detect'

def list_rel_files(subdir):
    """Return list of file paths like 'subdir/filename' for everything in subdir/"""
    result = []
    dirpath = os.path.join(here, subdir)
    if not os.path.isdir(dirpath):
        return result
    for fname in os.listdir(dirpath):
        fpath = os.path.join(dirpath, fname)
        if os.path.isfile(fpath):
            # path *relative* to setup.py
            result.append(os.path.join(subdir, fname))
    return result

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nicolas Müller',
    maintainer_email='nicolas.mueller@strohmleitung.de',
    description='OAK-D basierte Kegelerkennung mit DepthAI und YOLO in Python',
    license='MIT',
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            [f'resource/{package_name}'],
        ),
        # 2) package.xml
        (
            f'share/{package_name}',
            ['package.xml'],
        ),
        # 3) launch files
        (
            f'share/{package_name}/launch',
            list_rel_files('launch'),
        ),
        # 4) config files
        (
            f'share/{package_name}/config',
            list_rel_files('config'),
        ),
        # 5) other resources
        (
            f'share/{package_name}/resource',
            list_rel_files('resource'),
        ),
    ],
    # In setup.py
    entry_points={
        'console_scripts': [
            'detection_node = tensorrt_cone_detect.detection_node:main',
            'depth_node = depth_tracking.depth_node:main',
            'imu_viz_node = tensorrt_cone_detect.imu_viz_node:main',
            'multi_node_main = tensorrt_cone_detect.multi_node_main:main',
        ],
    },
)
