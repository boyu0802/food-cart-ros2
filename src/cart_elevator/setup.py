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
        ('share/' + package_name + '/data/digit_templates',
         glob('data/digit_templates/*.png')),
        ('share/' + package_name + '/test/data',
         glob('test/data/*.mp4')),
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
            # Direction detectors
            'direction_v1_digit = cart_elevator.direction.direction_v1_digit:main',
            'direction_v2_flow = cart_elevator.direction.direction_v2_flow:main',
            'direction_v3_fusion = cart_elevator.direction.direction_v3_fusion:main',
            # Docking
            'dock_controller = cart_elevator.dock.dock_controller:main',
            # Door state
            'door_state_detector = cart_elevator.door.door_state_detector:main',
            # Mission supervisor
            'cart_supervisor = cart_elevator.supervisor.cart_supervisor:main',
            # Per-floor map swap + AMCL relocalize helper
            'map_swap_node = cart_elevator.maps.map_swap_node:main',
            # Test helpers
            'fake_tag_publisher = cart_elevator.test_helpers.fake_tag:main',
            'fake_elevator_imu = cart_elevator.test_helpers.fake_imu:main',
            'fake_direction_image = cart_elevator.test_helpers.fake_direction_image:main',
            'fake_dock_sim = cart_elevator.test_helpers.fake_dock_sim:main',
            'fake_door_depth = cart_elevator.test_helpers.fake_door_depth:main',
            'mission_sim_driver = cart_elevator.test_helpers.mission_sim_driver:main',
            'video_replay = cart_elevator.test_helpers.video_replay:main',
            'direction_debug_view = cart_elevator.test_helpers.direction_debug_view:main',
        ],
    },
)
