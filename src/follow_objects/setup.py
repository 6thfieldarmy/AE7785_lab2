#!usr/bin/env python
from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'follow_objects'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='worren',
    maintainer_email='worren@todo.todo',
    description='enable turtlebot3 to rotate following a targeted ball',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = follow_objects.my_node:main'
            'find_object2 = follow_objects.find_object2:main'
            'turn_turtlebot = follow_objects.turn_turtlebot:main'
        ],
    },
)
