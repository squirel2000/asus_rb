from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'follow_me_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'action'), glob(os.path.join('action', '*.action'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='asus',
    maintainer_email='user@todo.todo',
    description='A package to make a TurtleBot3 follow a user.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'user_tracking_visual_node = follow_me_bot.user_tracking_visual_node:main',
            'user_tracking_controller_node = follow_me_bot.user_tracking_controller_node:main',
            'behavior_coordinator_node = follow_me_bot.behavior_coordinator_node:main',
        ],
    },
)
