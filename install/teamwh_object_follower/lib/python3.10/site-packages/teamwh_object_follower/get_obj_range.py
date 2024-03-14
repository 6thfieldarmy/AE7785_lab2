#from imutils.video import VideoStream
import numpy as np
#import argparse
#import cv2
#import imutils
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import LaserScan
#from cv_bridge import CvBridge
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
import sys

#define a node to find the distance from the ball detected by the camera
#should subsribe the camera to get pixel of the center of detected ball, and then translate it into angle, fit it to the Laser scan data and get the corresponding distance data

class get_obj_distance_node(Node):
    def  __init__(self):
        super().__init__('get_obj_distance_node')
        image_qos_profile = QoSProfile(
		    reliability=QoSReliabilityPolicy.BEST_EFFORT,
		    history=QoSHistoryPolicy.KEEP_LAST,
		    durability=QoSDurabilityPolicy.VOLATILE,
		    depth=1
		)
        self._laserscan_subscriber = self.create_subscription(
				LaserScan,
                '/scan',
                self.laserscan_callback,
				image_qos_profile)
        
        self._turn_agl_subscriber = self.create_subscription(
				Int16,
                '/turn_dir',
                self.turn_agl_callback,
				10)
        
        self.publisher_2 = self.create_publisher(Float32, 'turn_dir_linear', 5)


    def laserscan_callback(self, LaserScan):
        self.angle_min = LaserScan.angle_min
        self.angle_max = LaserScan.angle_max
        self.ranges = LaserScan.ranges  # the scan starts at angle min, then turn left(ccw)
        self.angle_increment = LaserScan.angle_increment
        # print("angle_min =  %.2f" % self.angle_min)
        # print("angle_max =  %.2f" % self.angle_max)
        # print("angle_increment =  %.2f" % self.angle_increment)

    def turn_agl_callback(self, turn_dir_msg):
        if turn_dir_msg.data == 100:
            print("Circle undetected")
            obj_agle_rad = 100
        elif turn_dir_msg.data >= 0:
            obj_agle_rad = 2*np.pi - turn_dir_msg.data/180*np.pi            
        elif turn_dir_msg.data < 0:
            obj_agle_rad = -turn_dir_msg.data/180*np.pi
        else:
            print("exception happens")
            obj_agle_rad = 100

        if obj_agle_rad != 100:
            if obj_agle_rad < self.angle_min:
                obj_index = int(0)
            elif obj_agle_rad > self.angle_max:
                obj_index = int(len(self.ranges) - 1)
            else:
                obj_index = np.round((obj_agle_rad - self.angle_min)/self.angle_increment)
                obj_index = int(obj_index)
            if obj_index == len(self.ranges):
                obj_index -= 1
            # print(obj_index) #debug
            # print(len(self.ranges))
            turn_dir_linear_msg = Float32()
            turn_dir_linear_msg.data = self.ranges[obj_index]
            print("distance from object: %.3f" % turn_dir_linear_msg.data)
            self.publisher_2.publish(turn_dir_linear_msg)
        else:
            turn_dir_linear_msg = Float32()
            turn_dir_linear_msg.data = 100.0
            self.publisher_2.publish(turn_dir_linear_msg)

def main():
    rclpy.init()
    god = get_obj_distance_node()
    while True:
        rclpy.spin_once(god, timeout_sec=0.05)

    

    god.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
	main()   