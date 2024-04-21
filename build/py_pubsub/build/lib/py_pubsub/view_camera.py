#!/usr/bin/env python
from imutils.video import VideoStream
import numpy as np
#import argparse
import cv2
import imutils
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from std_msgs.msg import Int16
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
import sys

class view_camera_node(Node):
    def __init_(self):
        super().__init__('view_camera_node')

        image_qos_profile = QoSProfile(
		    reliability=QoSReliabilityPolicy.BEST_EFFORT,
		    history=QoSHistoryPolicy.KEEP_LAST,
		    durability=QoSDurabilityPolicy.VOLATILE,
		    depth=1
		)

        #Declare that the minimal_video_subscriber node is subcribing to the /camera/image/compressed topic.
        self._video_subscriber = self.create_subscription(
				CompressedImage,
				'/image/compressed',
				self._image_callback,
                image_qos_profile)
        self._video_subscriber # Prevents unused variable warning.

    def _image_callback(self, CompressedImage):	
		# The "CompressedImage" is transformed to a color image in BGR space and is store in "_imgBGR"
        self._imgBGR = CvBridge().compressed_imgmsg_to_cv2(CompressedImage, "bgr8")
			# Display the image in a window
        self.show_image(self._imgBGR)

    def show_image(self, img):
        cv2.imshow(self._titleOriginal, img)
		# Cause a slight delay so image is displayed
        self._user_input=cv2.waitKey(50) #Use OpenCV keystroke grabber for delay.

    def get_user_input(self):
        return self._user_input

def main():
	rclpy.init() #init routine needed for ROS2.
	video_viewer = view_camera_node('view_camera_node') #Create class object to be used.

	while rclpy.ok():
		rclpy.spin_once(video_viewer) # Trigger callback processing.
		if video_viewer.get_user_input() == ord('q'):
			cv2.destroyAllWindows()
			break

	#Clean up and shutdown.
	video_viewer.destroy_node()  
	rclpy.shutdown()


if __name__ == '__main__':
	main()
