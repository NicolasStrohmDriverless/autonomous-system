from setuptools import setup

package_name = 'sort_tracking'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools','scipy','filterpy'],
    zip_safe=True,
    maintainer='AutoBot',
    maintainer_email='noreply@example.com',
    description='SORT tracking utilities',
    license='MIT',
    data_files=[
        ('share/ament_index/resource_index/packages',[f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
    ],
    entry_points={'console_scripts': []},
)
