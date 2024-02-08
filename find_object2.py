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

class find_object_node(Node):
    def __init_(self):
        super().__init__('find_object_node')

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

        self.publisher_0 = self.create_publisher(Int16, 'turn_dir', 10)
        self.publisher_1 = self.create_publisher(CompressedImage, 'processed_img', 10)

        while(True):
            frame = self._imgBGR
            if frame is None:
                break
            frame = frame = imutils.resize(frame, width=600, height=480)
            processing = cv2.dilate(frame,None,iterations=3)
            processing = cv2.erode(processing,None,iterations=3)
            processing = cv2.erode(processing,None,iterations=3)  
            processing = cv2.dilate(processing,None,iterations=3)

            gray = cv2.cvtColor(processing, cv2.COLOR_BGR2GRAY)
            circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 50, param1=60, param2=40, minRadius=50, maxRadius=300)
            if circles is not None:
                circles = np.uint16(np.around(circles))
            for i in circles[0,:]:
                cv2.circle(frame,(i[0],i[1]),i[2],(0,0,255),2)
                cv2.circle(frame,(i[0],i[1]),2,(0,0,255),3)
                if i[0] > 320 : #turn right
                    cv2.circle(frame, (575, 25), 20, (255,0,0), 21)
                    self._turn_direction = 1 #1 for right, 0 for left
                elif i[0] < 280 :
                    cv2.circle(frame, (25, 25), 20, (255,0,0), 21)
                    self._turn_direction = 0 #1 for right, 0 for left

            self._processed_image = frame
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    def _image_callback(self, CompressedImage):	
		# The "CompressedImage" is transformed to a color image in BGR space and is store in "_imgBGR"
        self._imgBGR = CvBridge().compressed_imgmsg_to_cv2(CompressedImage, "bgr8")
        if self._imgBGR is not None:
            self.get_logger().info('camera image subscribing...')

    def _turn_dir_publisher(self):
        turn_dir_msg = Int16()
        turn_dir_msg.data = self._turn_direction
        self.publisher_0.publish(turn_dir_msg)
        if turn_dir_msg.data == 0:
            self.get_logger().info('Turning left')
        elif turn_dir_msg.data == 1:
            self.get_logger().info('Turning right')
        else:
            self.get_logger().info('find object direction publisher failed.')

    def _processed_image_publisher(self):
        processed_image_msg = CompressedImage()
        processed_image_msg.data = CvBridge().cv2_to_imgmsg(self._processed_image, encoding="passthrough")
        self.publisher_1.publish(processed_image_msg)

    def get_user_input(self):
        self._user_input=cv2.waitKey(50)
        return self._user_input

def main():
	rclpy.init() #init routine needed for ROS2.
	video_subscriber = find_object_node() #Create class object to be used.

	while rclpy.ok():
		rclpy.spin_once(video_subscriber) # Trigger callback processing.
		if video_subscriber.get_user_input() == ord('q'):
			cv2.destroyAllWindows()
			break

	#Clean up and shutdown.
	video_subscriber.destroy_node()  
	rclpy.shutdown()


if __name__ == '__main__':
	main()