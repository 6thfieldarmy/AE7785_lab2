#!/usr/bin/env python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='follow_objects',
            executable='find_object2',
            name='find_object2'
        ),
        Node(
            package='follow_objects',
            executable='turn_turtlebot',
            name='turn_turtlebot'
        ),
    ])
