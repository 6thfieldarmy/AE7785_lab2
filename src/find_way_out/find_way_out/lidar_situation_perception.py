import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
import numpy as np
from std_msgs.msg import Int16
from sensor_msgs.msg import LaserScan


class lidar_situation_perception(Node):
    def __init__(self):
        
        super().__init__('lidar_situation_perception')
        qos_profile = QoSProfile(
            
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
		    history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
		    depth=1
        )

        self._lidar_subscription = self.create_subscription(
            LaserScan,
             '/scan',
              self.lidar_callback,
              qos_profile)
        
        self.publisher1 = self.create_publisher(Int16, 'collide_warning', qos_profile)


    def lidar_callback(self, LaserScan):
        # I made a lot of assumption here:
        # 1. the maze is composed of 3*6 squares, and the robot should be able to read signs at the center of any squares(in at least one direction)
        # 2. the robot starts at the center of a square, and orient to the direction without wall(no need to turn initially)
        # 3. the robot only turn 90 or 180 deg
        # then this node will warn when obstacles(expected to be wall) presents in front of the turtlebot(+- 15 deg) in 0.46 m (measured in maze)


        lidar_msg = Int16()
        rg = LaserScan.ranges
        danger_zone = 10 # this is a very ideal condition, only needs to focus on the front
        rg_left = np.flip(rg[0:danger_zone]) #first danger_zone elements
        rg_right = np.flip(rg[len(rg) - 1 - danger_zone : len(rg) - 1]) #last danger_zone elements
        rg = np.append(rg_left,rg_right)
        if min(rg) <= 0.55:  #enable obstacles avoidance logic about a foot from the wall
            lidar_msg.data = 1 
            print("Collide Warning")
            # print(rg)
            # print("minimum dist: %.4f , orientation: %d" % (min(rg), np.nanargmin(rg)-20) )
            #print(self.lidar_msg.angular.z)
        else:
            lidar_msg.data = 2
            print("Move forward")
         # 1 = stop and turn; 2 = move forward; 0 = unexpected error
        self.publisher1.publish(lidar_msg)
            

def main(args=None):
    rclpy.init(args=args) #init routine needed for ROS2.
    my_node = lidar_situation_perception() #Create class object to be used.
    rclpy.spin(my_node) # Trigger callback processing.
	#Clean up and shutdown.
    my_node.destroy_node()  
    rclpy.shutdown()
    

if __name__ == '__main__':
	main()