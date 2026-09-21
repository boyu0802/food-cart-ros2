from setuptools import find_packages, setup

package_name = 'simulation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/sim.launch.py', 'launch/nav_sim.launch.py']),
        ('share/' + package_name + '/config', ['config/gz_bridge.yaml']),
        ('share/' + package_name + '/worlds', ['worlds/school.sdf']),
        ('share/' + package_name + '/models/food_cart', ['models/food_cart/model.sdf']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Food Cart Team',
    maintainer_email='user@example.com',
    description='Gazebo simulation for Food Cart robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'elevator_node = plugins.elevator_controller.elevator_node:main',
            'sim_dashboard = plugins.sim_dashboard.dashboard_node:main',
        ],
    },
)