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
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
import sys

#chech remote control

class find_object_node(Node):
    def __init__(self):
        super().__init__('find_object_node')

        image_qos_profile = QoSProfile(
		    reliability=QoSReliabilityPolicy.BEST_EFFORT,
		    history=QoSHistoryPolicy.KEEP_LAST,
		    durability=QoSDurabilityPolicy.VOLATILE,
		    depth=1
		)
        # timer_period = 1 #sec
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        # self.i = 0

        #Declare that the minimal_video_subscriber node is subcribing to the /camera/image/compressed topic.
        self._video_subscriber = self.create_subscription(
				CompressedImage,
				'/image_raw/compressed',
				self._image_callback,
                image_qos_profile)
        self._video_subscriber # Prevents unused variable warning.
        
        self.publisher_0 = self.create_publisher(Int16, 'turn_dir', 5)
        self.publisher_1 = self.create_publisher(CompressedImage, 'processed_img', 5)

    def _image_callback(self, CompressedImage):	
        # The "CompressedImage" is transformed to a color image in BGR space and is store in "_imgBGR"
        # The FOV of the bot's camera is 62.2deg or +-0.543 rad, mapped to 640 horizontal pixels linearly
        # even within the FOV, the ball can only be correctly detected in the field of +- 20 deg

        self._imgBGR = CvBridge().compressed_imgmsg_to_cv2(CompressedImage, "bgr8")
        if self._imgBGR is not None:
            self.get_logger().info('camera image subscribing...')
            frame = self._imgBGR
            
            frame = imutils.resize(frame, width=640, height=480)
            processing = cv2.dilate(frame,None,iterations=3)
            processing = cv2.erode(processing,None,iterations=3)
            processing = cv2.erode(processing,None,iterations=3)  
            processing = cv2.dilate(processing,None,iterations=3)

            gray = cv2.cvtColor(processing, cv2.COLOR_BGR2GRAY)
            circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 50, param1=60, param2=40, minRadius=30, maxRadius=480)
            if circles is not None:
                circles = np.uint16(np.around(circles))
                sorted_circles = sorted(circles[0], key=lambda x:x[2], reverse=True)
                i = sorted_circles[0]
                cv2.circle(frame,(i[0],i[1]),i[2],(0,0,255),2)
                cv2.circle(frame,(i[0],i[1]),2,(0,0,255),3)


                # notice right = positive angle, left = negative angle
                self._turn_direction = np.round(((i[0] - 320)/320)*31.1) # angles where the center of the ball located, rounded to integer degree
                print(self._turn_direction)

                # if i[0] > 350 and i[0] < 420 : #turn right
                #     cv2.circle(frame, (575, 25), 20, (255,0,0), 21)
                #     self._turn_direction = 1 #1 for right, 0 for left
                #     print("to the right 1st")
                # elif i[0] >= 420 and i[0] < 490 :
                #     cv2.circle(frame, (25, 25), 20, (255,0,0), 21)
                #     self._turn_direction = 10 #1 for right, 0 for left
                #     print("to the left 2nd")
                # elif i[0] >= 490 :
                #     cv2.circle(frame, (25, 25), 20, (255,0,0), 21)
                #     self._turn_direction = 11 #1 for right, 0 for left
                #     print("to the left 3rd")
                
                # elif i[0] < 290 and i[0] >= 220 :
                #     cv2.circle(frame, (25, 25), 20, (255,0,0), 21)
                #     self._turn_direction = 0 #1 for right, 0 for left
                #     print("to the left 1st")
                # elif i[0] < 220 and i[0] >= 150 :
                #     cv2.circle(frame, (25, 25), 20, (255,0,0), 21)
                #     self._turn_direction = 4 #1 for right, 0 for left
                #     print("to the left 2nd")
                # elif i[0] < 150 :
                #     cv2.circle(frame, (25, 25), 20, (255,0,0), 21)
                #     self._turn_direction = 5 #1 for right, 0 for left
                #     print("to the left 3rd")

                # else:
                #     self._turn_direction = 2
                #     print("no need to move")
            else:
                print("circle undetected")
                self._turn_direction = 100 #nothing happened
            self._processed_image = frame

            turn_dir_msg = Int16()
            turn_dir_msg.data = int(self._turn_direction)
            self.publisher_0.publish(turn_dir_msg)

            # processed_image_msg = CompressedImage()
            # processed_image_msg.data = CvBridge().cv2_to_imgmsg(self._processed_image, encoding="passthrough")
            # self.publisher_1.publish(processed_image_msg)

        else:
            print("self._imgBGR convert failed.")

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args) #init routine needed for ROS2.
    video_subscriber = find_object_node() #Create class object to be used.
    while True:
        rclpy.spin_once(video_subscriber, timeout_sec=0.05) # Trigger callback processing.
	#Clean up and shutdown.
    video_subscriber.destroy_node()  
    rclpy.shutdown()


if __name__ == '__main__':
	main()
