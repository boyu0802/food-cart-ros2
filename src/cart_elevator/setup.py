from setuptools import find_packages, setup
from glob import glob

package_name = 'cart_elevator'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros2',
    maintainer_email='boyu0802chen@gmail.com',
    description='Elevator interaction stack for the food cart.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Floor detectors
            'floor_v1_apriltag = cart_elevator.floor.floor_v1_apriltag:main',
            'floor_v2_time = cart_elevator.floor.floor_v2_time:main',
            'floor_v3_accel = cart_elevator.floor.floor_v3_accel:main',
            'floor_v4_fusion = cart_elevator.floor.floor_v4_fusion:main',
            # Test helpers
            'fake_tag_publisher = cart_elevator.test_helpers.fake_tag:main',
            'fake_elevator_imu = cart_elevator.test_helpers.fake_imu:main',
        ],
    },
)
