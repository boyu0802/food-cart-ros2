from setuptools import find_packages, setup
from glob import glob

package_name = 'nt_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'pyntcore'],
    zip_safe=True,
    maintainer='ros2',
    maintainer_email='boyu0802chen@gmail.com',
    description='NetworkTables 4 bridge between ROS2 and FRC RoboRIO (team 6998 food cart)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'nt_bridge_node = nt_bridge.nt_bridge_node:main',
        ],
    },
)
