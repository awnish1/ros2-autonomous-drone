from setuptools import find_packages, setup

package_name = 'drone_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='awnish-kumar',
    maintainer_email='awnish-kumar@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'dynamic_autonomy = drone_controller.dynamic_autonomy:main',
            'dynamic_navigation_preview = drone_controller.dynamic_navigation_preview:main',
            'follow_astar = drone_controller.follow_astar:main',
            'safe_navigation = drone_controller.safe_navigation:main',
            'obstacle_detector = drone_controller.obstacle_detector:main',
            'goto_xyz = drone_controller.goto_xyz:main',
            'full_mission = drone_controller.full_mission:main',
            'auto_land = drone_controller.auto_land:main',
            'pid_takeoff = drone_controller.pid_takeoff:main',
            'auto_takeoff = drone_controller.auto_takeoff:main',
            'altitude_monitor = drone_controller.altitude_monitor:main'
        ],
    },
)
