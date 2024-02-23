#!/usr/bin/env python
#import rospy
from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from std_msgs.msg import Float32

import sys

import numpy as np

class turn_turtlebot_node(Node):
    def __init__(self):		
		# Creates the node.
        super().__init__('turn_turtlebot_node')
        self._turn_dir_subscriber = self.create_subscription(
				Int16,
                '/turn_dir',
                self.turn_dir_callback,
				10)
        self._move_dir_subscriber = self.create_subscription(
				Float32,
                '/turn_dir_linear',
                self.move_dir_callback,
				10)
        self.publisher_2 = self.create_publisher(Twist, '/cmd_vel', 5)
        
    def turn_dir_callback(self, turn_dir_msg):
            self.motion_cmd = Twist()
            if turn_dir_msg.data <= 3 and turn_dir_msg.data >= -3:
                self.motion_cmd.angular.z = 0.0
            elif turn_dir_msg.data == 100 or turn_dir_msg.data is None:
                self.motion_cmd.angular.z = 0.0
            elif turn_dir_msg.data > 3 and turn_dir_msg.data < 35:
                self.motion_cmd.angular.z = -(turn_dir_msg.data - 3)/20*0.5
            elif turn_dir_msg.data < -3 and turn_dir_msg.data > -35:
                self.motion_cmd.angular.z = -(turn_dir_msg.data + 3)/20*0.5
            else:
                self.motion_cmd.angular.z = 0.0

            # if turn_dir_msg.data == 0:
            #     self.get_logger().info('Turning left cmd copied')
            #     print("Turning left")
            #     self.rot.angular.z = 0.15
            #     self.publisher_2.publish(self.rot)
            # elif turn_dir_msg.data == 1:
            #     self.get_logger().info('Turning right cmd copied')
            #     print("Turning right")
            #     self.rot.angular.z = -0.15
            #     self.publisher_2.publish(self.rot)
            # elif turn_dir_msg.data == 10:
            #     self.get_logger().info('Turning right cmd copied')
            #     print("Turning right")
            #     self.rot.angular.z = -0.3
            #     self.publisher_2.publish(self.rot)
            # elif turn_dir_msg.data == 11:
            #     self.get_logger().info('Turning right cmd copied')
            #     print("Turning right")
            #     self.rot.angular.z = -0.45
            #     self.publisher_2.publish(self.rot)
            # elif turn_dir_msg.data == 4:
            #     self.get_logger().info('Turning left cmd copied')
            #     print("Turning left")
            #     self.rot.angular.z = 0.3
            #     self.publisher_2.publish(self.rot)
            # elif turn_dir_msg.data == 5:
            #     self.get_logger().info('Turning left cmd copied')
            #     print("Turning left")
            #     self.rot.angular.z = 0.45
            #     self.publisher_2.publish(self.rot)

            # else:
            #     self.rot.angular.z = 0.0
            #     self.publisher_2.publish(self.rot)
    def move_dir_callback(self, turn_dir_linear_msg):
            #keep distance at 0.4
            if turn_dir_linear_msg.data <= 0.43 and turn_dir_linear_msg.data >= 0.37:
                self.motion_cmd.linear.x = 0.0
            elif turn_dir_linear_msg.data == 100 or turn_dir_linear_msg.data is None:
                self.motion_cmd.linear.x = 0.0
            elif turn_dir_linear_msg.data > 0.43 :
                self.motion_cmd.linear.x = min((turn_dir_linear_msg.data - 0.43)/0.2*0.1,0.1)
            elif turn_dir_linear_msg.data < 0.37 :
                self.motion_cmd.linear.x = min((turn_dir_linear_msg.data - 0.37)/0.2*0.1,-0.1)
            else:
                self.motion_cmd.linear.x = 0.0

            self.publisher_2.publish(self.motion_cmd)    

    
def main():
	rclpy.init() #init routine needed for ROS2.
	turn_cmd = turn_turtlebot_node() #Create class object to be used.
   #print("turn_turtlebot_node running")

	rclpy.spin(turn_cmd) # Trigger callback processing.
		

	#Clean up and shutdown.
	turn_cmd.destroy_node()  
	rclpy.shutdown()


if __name__ == '__main__':
	main()


		
