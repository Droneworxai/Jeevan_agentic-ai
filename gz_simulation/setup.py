from setuptools import setup
import os
from glob import glob

package_name = 'simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'robot_description/urdf'), glob('robot_description/urdf/*.urdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jeevan',
    maintainer_email='jeevan@example.com',
    description='EcoWeeder Simulation Package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_planner = simulation.mission_planner:main',
            'boundary_manager = simulation.boundary_manager:main',
            'weed_manager = simulation.weed_manager:main',
            'mission_monitor = simulation.mission_monitor:main',
            'test_bot = simulation.test_bot:main',
        ],
    },
)
