import cv2
import csv
import numpy as np
import os


import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image # for gazebo
from cv_bridge import CvBridge
from std_msgs.msg import Int16
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
import sys

# This node should read the images(frames) from robot camera, and use the KNN models to recognize if a sign is shown in the robot's field of view
# It should publish the turn order(integer) acoording to the sign found

# 1 = 90 deg turn left
# 2 = 90 deg turn right
# 3, 4 = 180 deg turn back
# 5 = reach goal
gazebo = False
Directory = '2024Simgs'



resize_h = int(240 * .1)
resize_w = int(320 * .1)
shape = resize_w*resize_h*3

def findObjects(image):
    contours, hierarchy = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)  
    cont = sorted(contours, key=cv2.contourArea, reverse=True)[:1]  
    x = y = w = h = 1
    # Find the center of mass of the blob if there are any
    if len(cont) > 0:
        M = cv2.moments(cont[0])
        if M['m00'] > 250:
            x, y, w, h = cv2.boundingRect(cont[0])
    return x, y, w, h


def filter_colors(image):
    # Convert the image to HSV color space
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Define color ranges for red, green, and blue in HSV
    if not gazebo:
        lower_red = np.array([0, 100, 45])
        upper_red = np.array([225, 250, 255])  # for real
    else:
        lower_red = np.array([10, 100, 20])
        upper_red = np.array([25, 255, 255])  # for gazebo
    lower_green = np.array([36, 0, 0])
    upper_green = np.array([86, 255, 255])
    lower_blue = np.array([100,150,0])  # Decreased lower limit for blue
    upper_blue = np.array([130,255,255])  # Increased upper limit for blue

    # Create masks for red, green, and blue colors
    mask_red = cv2.inRange(hsv, lower_red, upper_red)
    mask_green = cv2.inRange(hsv, lower_green, upper_green)
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

    # Combine the masks
    mask = cv2.bitwise_or(mask_red, mask_green)
    mask = cv2.bitwise_or(mask, mask_blue)

    # Apply the mask to the image
    filtered_image = cv2.bitwise_and(image, image, mask=mask)
 # Perform morphological operations to reduce noise
    kernel = np.ones((3,3),np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # Convert non-zero pixels to white in the final filtered image
    filtered_image = cv2.bitwise_and(filtered_image, filtered_image, mask=mask)


    return filtered_image

def preprocess(img):
    # downsample to speed up
    img = img[60:180, 60:260]
    img_rgb = img
    # filter the background and noise
    img = filter_colors(img)
    mask = cv2.inRange(img, np.array([0, 0, 0]), np.array([0, 0, 0]))
    img[mask == 255] = [255, 255, 255]

   
    img = cv2.Canny(img, 100, 150)
    element = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    fix = cv2.dilate(img, element, iterations=1)
    img = cv2.morphologyEx(fix, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    x, y, w, h = findObjects(img)
    img = img_rgb[y:y + h, x:x + w]
    img = cv2.resize(img, (resize_w, resize_h))

    return img





class camera_situation_perception_node(Node):
    def __init__(self):
        super().__init__('camera_situation_perception_node')

        image_qos_profile = QoSProfile(
		    reliability=QoSReliabilityPolicy.BEST_EFFORT,
		    history=QoSHistoryPolicy.KEEP_LAST,
		    durability=QoSDurabilityPolicy.VOLATILE,
		    depth=1
		)
        #Declare that the minimal_video_subscriber node is subcribing to the /camera/image/compressed topic.
        self._video_subscriber = self.create_subscription(
                # Image,
				CompressedImage,  #for gazebo
			    '/image_raw/compressed',    #for gazebo
                #'/camera/image_raw',  
				self._image_callback,
                image_qos_profile)
        self._video_subscriber # Prevents unused variable warning.

        timer_period = 0.1
        self.image_processing = self.create_timer(timer_period, self.image_processing_callback)
        
        self.publisher_0 = self.create_publisher(Int16, 'turn_dir', 5)
        #self.publisher_1 = self.create_publisher(CompressedImage, 'processed_img', 5)

        # load the model
        self.knn_model = Directory+' knnModel'

        self.knn_test = cv2.ml.KNearest_create()
        self.model = self.knn_test.load(self.knn_model)
        self._turn_direction = 0
        self._turn_directions = []


    #def _image_callback(self, Image):	
    def _image_callback(self, CompressedImage):	
        # The "CompressedImage" is transformed to a color image in BGR space and is store in "_imgBGR"
        # The FOV of the bot's camera is 62.2deg or +-0.543 rad, mapped to 640 horizontal pixels linearly
        # even within the FOV, the ball can only be correctly detected in the field of +- 20 deg

        self._imgBGR = CvBridge().compressed_imgmsg_to_cv2(CompressedImage, "bgr8") # real robot
        #self._imgBGR = CvBridge().imgmsg_to_cv2(Image, "bgr8") # for gazebo only
        if self._imgBGR is not None:
            #self.get_logger().info('camera image subscribing...')
            #self._imgBGR = cv2.resize(self._imgBGR, (410, 308)) # 320*240 original
            cv2.imwrite("./output.png", self._imgBGR)
        else:
            print("camera image not found.")

    def image_processing_callback(self):
        k = 5 # not sure what this is
        image_loc = './output.png'
        if os.path.exists(image_loc):
            original_img = cv2.imread("./output.png", 1)
            # cv2.imshow('image window', original_img)
            # cv2.waitKey(0)
            # cv2.destroyAllWindows()
            test_img = np.array(preprocess(original_img))

            test_img = test_img.flatten().reshape(1,shape)
            test_img = test_img.astype(np.float32)
            ret, _, _, _ = self.model.findNearest(test_img, k)
            # To add tolerance for vision errors, average ret result with previous 5 results
            self._turn_directions.append(ret)
            if len(self._turn_directions) > 5:
                most_common = max(set(self._turn_directions), key=self._turn_directions.count)
                self._turn_direction = most_common # 1left 2right 3back 4back 5stop
                self._turn_directions = []
                if self._turn_direction == 1:
                    direction_order = "turn LEFT"
                elif self._turn_direction == 2:
                    direction_order = "turn RIGHT"
                elif self._turn_direction == 3:
                    direction_order = "turn BACK"
                elif self._turn_direction == 4:
                    direction_order = "turn BACK"
                elif self._turn_direction == 5:
                    direction_order = "GOAL !!!"
                elif self._turn_direction == 0:
                    direction_order = "No sign detected"
                else:
                    direction_order = "Unknown turn direction"

                print(direction_order)
        else:
            print("No output.png")
            self._turn_direction = 0
            
        turn_dir_msg = Int16()
        turn_dir_msg.data = int(self._turn_direction)
        self.publisher_0.publish(turn_dir_msg)


def main(args=None):


    rclpy.init(args=args) #init routine needed for ROS2.
    sp_node = camera_situation_perception_node() #Create class object to be used.
    while True:
        rclpy.spin_once(sp_node, timeout_sec=0.1) # Trigger callback processing.
	#Clean up and shutdown.
    video_subscriber.destroy_node()  
    rclpy.shutdown()


if __name__ == '__main__':
	main()