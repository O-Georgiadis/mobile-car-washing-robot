import os
from setuptools import find_packages, setup
from glob import glob


package_name = 'car_washing_robot_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='johnp',
    maintainer_email='johnp@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'simple_controller = car_washing_robot_controller.simple_controller:main',
            'lidar_test_node = car_washing_robot_controller.lidar_test_node:main',
        ],
    },
)
