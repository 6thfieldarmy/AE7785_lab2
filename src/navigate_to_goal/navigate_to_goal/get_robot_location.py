import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String
from std_msgs.msg import Float64

from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

import numpy as np
import math
import time

class get_robot_location(Node):
    def __init__(self):
        super().__init__('get_robot_location')
        qos_profile = QoSProfile(
            
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
		      history=QoSHistoryPolicy.KEEP_LAST,
          durability=QoSDurabilityPolicy.VOLATILE,
		      depth=1
        )

        self._odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self._odom_callback,
            qos_profile
        )
        self.Init = True
        self.Init_pos = Point()
        self.Init_pos.x = 0.0
        self.Init_pos.y = 0.0
        self.Init_ang = 0.0
        self.globalPos = Point()
        self.publisher1 = self.create_publisher(Point,'get_robot_location/position',qos_profile)
        self.publisher2 = self.create_publisher(Float64,'get_robot_location/orientation',qos_profile)

    def _odom_callback(self, msg):
        self.get_robot_location(msg)

    def get_robot_location(self, msg):
        position = msg.pose.pose.position
        
        #Orientation uses the quaternion aprametrization.
        #To get the angular position along the z-axis, the following equation is required.
        q = msg.pose.pose.orientation
        orientation = np.arctan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

        if self.Init:
            #The initial data is stored to by subtracted to all the other values as we want to start at position (0,0) and orientation 0
            self.Init = False
            self.Init_ang = orientation
            self.globalAng = self.Init_ang
            Mrot = np.matrix([[np.cos(self.Init_ang), np.sin(self.Init_ang)],[-np.sin(self.Init_ang), np.cos(self.Init_ang)]])        
            self.Init_pos.x = Mrot.item((0,0))*position.x + Mrot.item((0,1))*position.y
            self.Init_pos.y = Mrot.item((1,0))*position.x + Mrot.item((1,1))*position.y
            self.Init_pos.z = position.z
        Mrot = np.matrix([[np.cos(self.Init_ang), np.sin(self.Init_ang)],[-np.sin(self.Init_ang), np.cos(self.Init_ang)]])        

        #We subtract the initial values
        self.globalPos.x = Mrot.item((0,0))*position.x + Mrot.item((0,1))*position.y - self.Init_pos.x
        self.globalPos.y = Mrot.item((1,0))*position.x + Mrot.item((1,1))*position.y - self.Init_pos.y
        self.globalAng = orientation - self.Init_ang #+left -right; radian (e.g. 转向正左为 1.57)
        self.globalorientation = Float64()
        self.globalorientation.data = self.globalAng
    
        self.get_logger().info('Transformed global pose is x:%.2f, y:%.2f, a:%.2f' % (self.globalPos.x,self.globalPos.y,self.globalAng))
        self.publisher1.publish(self.globalPos)
        self.publisher2.publish(self.globalorientation)

        # if self.init_flag:
        #   self.init_x = round(msg.pose.pose.position.x,3)
        #   self.init_y = round(msg.pose.pose.position.y,3)
        #   self.init_flag = 0
        # else:
        #   self.loc_msg.x = round(msg.pose.pose.position.x,3) - self.init_x
        #   self.loc_msg.y = round(msg.pose.pose.position.y,3) - self.init_y
        # #print(self.loc_msg.x, self.loc_msg.y)
        # self.publisher1.publish(self.loc_msg)

def main(args=None):
    rclpy.init(args=args)
    get_robot_location_node = get_robot_location()
    rclpy.spin(get_robot_location_node)
    # while True:
    #     rclpy.spin_once(get_robot_location_node)
    #     time.sleep(0.1)
        
    get_robot_location_node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()