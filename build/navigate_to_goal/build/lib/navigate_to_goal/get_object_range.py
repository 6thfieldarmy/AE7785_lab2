import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String
import numpy as np

from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import LaserScan
import cv2
from cv_bridge import CvBridge

class get_object_range(Node):
    def __init__(self):
        
        super().__init__('get_object_range')
        qos_profile = QoSProfile(
            
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
		      history=QoSHistoryPolicy.KEEP_LAST,
          durability=QoSDurabilityPolicy.VOLATILE,
		      depth=1
        )
        # self._img_subscription = self.create_subscription(
        #     CompressedImage,
        #     '/image_raw/compressed',
        #     self._image_callback,
        #     qos_profile
        # )

        self._lidar_subscription = self.create_subscription(
            LaserScan,
             '/scan',
              self.lidar_callback,
              qos_profile)
        self.pt_msg = Point()
        self.publisher1 = self.create_publisher(Twist,'get_object_range/range',qos_profile)
        # self.publisher2 = self.create_publisher(CompressedImage,'get_object_range/image',qos_profile)

    # def _image_callback(self, CompressedImage):
    #     self.find_object(CompressedImage)

    # def find_object(self, CompressedImage):
    #     frame = CvBridge().compressed_imgmsg_to_cv2(CompressedImage,"bgr8")
    #     hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # # Apply Canny edge detection
    #     edges = cv2.Canny(hsv, 50, 150)
    # # Find contours
    #     contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
    #     for cnt in contours:
    #       # Get the approximate polygon of the contour
    #       epsilon = 0.05 * cv2.arcLength(cnt, True)
    #       approx = cv2.approxPolyDP(cnt, epsilon, True)
  
    #       # If the polygon has 4 vertices, it is a rectangle
    #       if len(approx) == 4:
    #           # Calculate the center point of the rectangle
    #           M = cv2.moments(cnt)
    #           self.pt_msg.x = int(M["m10"] / M["m00"])
    #           self.pt_msg.y = int(M["m01"] / M["m00"])
    #           self.pt_msg.z = float(1)
    #           self.get_logger().info("Camera found: x=%d, y=%d", self.pt_msg.x, self.pt_msg.y)
    #       else:
    #           self.pt_msg.x = float(0)
    #           self.pt_msg.y = float(0)
    #           self.pt_msg.z = float(0)
        

    #     self.publisher2.publish(CvBridge().cv2_to_compressed_imgmsg(frame))

    def lidar_callback(self, LaserScan):
        twist_msg = Twist()
        rg = LaserScan.ranges
        rg_left = np.flip(rg[0:20]) #first 20 elements
        rg_right = np.flip(rg[len(rg) - 21 : len(rg) - 1]) #last 20 elements
        rg = np.append(rg_left,rg_right)
        if min(rg) <= 0.5:
            twist_msg.angular.z = 0.1*(np.argmin(rg)-20)/40
            print(rg)
            print("minimum dist: %.4f , orientation: %d" % min(rg), np.argmin(rg)-20 )
            print(twist_msg.angular.z)

        # for i in range(0,len(rg)):
        #     if rg[i] > LaserScan.range_max:
        #         rg[i] = 0.25
        #     elif rg[i] < LaserScan.range_min:
        #         rg[i] = 0.25
        # if self.pt_msg.z == float(1):
        #     #self.get_logger().info("Camera found: x=%f, y=%f", self.pt_msg.x, self.pt_msg.y)
        #     loc = round(self.pt_msg.x / 320 * 60)
        #     if(loc >=30):
        #         loc =29
        #     elif(loc<=0):
        #         loc = 0
        #         dist = rg[loc]
        #     twist_msg.linear.x = float(dist)
        #     twist_msg.angular.z = -0.005*(self.pt_msg.x-float(160))
        # else:
        #     #self.get_logger().info("Camera found nothing")
        #     twist_msg.linear.x = 1.0
        #     twist_msg.angular.z = 0.0
            
        self.publisher1.publish(twist_msg)

def main():
    rclpy.init()
    ranger = get_object_range()
    while rclpy.ok():
        rclpy.spin_once(ranger) # Trigger callback processing.
    ranger.destroy_node()    
    rclpy.shutdown()

if __name__ == '__main__':
	main()


        