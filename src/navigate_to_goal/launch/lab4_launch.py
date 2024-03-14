from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='navigate_to_goal',
            namespace='',
            executable='get_object_range',
            name='get_object_range'
        ),
        Node(
            package='navigate_to_goal',
            namespace='',
            executable='get_robot_location',
            name='get_robot_location'
        ),    
        Node(
            package='navigate_to_goal',
            namespace='',
            executable='go_to_goal',
            name='go_to_goal'
        ), 
    ]) 
