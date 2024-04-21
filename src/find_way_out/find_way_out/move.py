import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
import numpy as np
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist
import time
from std_msgs.msg import Float64
from threading import Thread

class move(Node):
    def __init__(self):
        
        super().__init__('move')
        qos_profile = QoSProfile(
            
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
		    history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
		    depth=1
        )
        self.direction_subscription = self.create_subscription(
            Int16,
            '/turn_dir',
            self.direction_callback,
            10)

        self._lidar_subscription = self.create_subscription(
            Int16,
             '/collide_warning',
              self.lidar_callback,
              qos_profile)
        
        self._orientation_subscription = self.create_subscription(  # ros2 run navigate_to_goal get_robot_location   # to get the odom info
            Float64,
             '/get_robot_location/orientation',
              self.orient_callback,
              qos_profile)
        
        self.publisher1 = self.create_publisher(Twist,'/cmd_vel',5)
        self.cmd = Twist()
        # self.turn_start = False     use odom to determine the turn situation
        # self.turn_complete = False
        # self._turn_saved_direction = 0
        self.timer_ = self.create_timer(0.2, self.move)

        self._turn_direction = 0
        self._collide_warning = 0
        self.ang = 0

        self._mode = 1 # 1 = moving forward; 2 = read sigh and turn; 3 = stop at the goal; 0 = exception


    
    def direction_callback(self, msg):
        #self.get_logger().info('sign = (%d)' % msg)
        self._turn_direction = msg.data
        #print("direction: %d" % self._turn_direction)

    def orient_callback(self, msg):
        self.ang = msg.data
        if self.ang < 0:
            self.ang = np.pi*2 + self.ang  # 0 to 2 pi
        elif self.ang > np.pi*2:
            self.ang = -np.pi*2 + self.ang
        #self.get_logger().info('orientation a = (%.2f)' % self.ang)

    def lidar_callback(self, msg):
        self._collide_warning = msg.data
        #print("Collide or not: %d" % self._collide_warning)
        
            
    def turn_degrees(self):
        err_tol_ang = 0.05
        start_turn = True
        #self.cmd.linear.x = 0.0
        while self._mode == 2:
            if start_turn:
                start_angle = self.ang # once every loop
                start_turn = False
                print("Start Angle Recorded: %.2f" % start_angle)

            if self._turn_direction == 1: #left
                turn_angle = np.pi/2                
                target_angle = start_angle + turn_angle
                if target_angle < 0:
                    target_angle = np.pi*2 + target_angle  # 0 to 2 pi
                elif target_angle > np.pi*2:
                    target_angle = -np.pi*2 + target_angle
                print("Turning LEFT to ang. = %.2f" % (target_angle ))
                while True:
                    self.cmd.angular.z = 0.15
                    self.publisher1.publish(self.cmd)
                    if abs(target_angle - self.ang) <= err_tol_ang:
                        self._mode = 1
                        self.cmd.angular.z = 0.0
                        self.publisher1.publish(self.cmd)
                        break
                print("Finish Turn")
                break
                # else:
                #     print("Turning Left")

            elif self._turn_direction == 2: #right
                turn_angle = -np.pi/2                
                target_angle = start_angle + turn_angle
                if target_angle < 0:
                    target_angle = np.pi*2 + target_angle  # 0 to 2 pi
                elif target_angle > np.pi*2:
                    target_angle = -np.pi*2 + target_angle
                print("Turning RIGHT to ang. = %.2f" % (target_angle ))
                while True:
                    self.cmd.angular.z = -0.15
                    self.publisher1.publish(self.cmd)
                    if abs(target_angle - self.ang) <= err_tol_ang:
                        self._mode = 1
                        self.cmd.angular.z = 0.0
                        self.publisher1.publish(self.cmd)
                        break
                print("Finish Turn")
                break
                
                # else:
                #     print("Turning Right")
                
            elif self._turn_direction == 3 or self._turn_direction == 4: #back
                turn_angle = -np.pi                
                target_angle = start_angle + turn_angle
                if target_angle < 0:
                    target_angle = np.pi*2 + target_angle  # 0 to 2 pi
                elif target_angle > np.pi*2:
                    target_angle = -np.pi*2 + target_angle
                print("Turning BACK to ang. = %.2f" % (target_angle ))
                while True:
                    self.cmd.angular.z = -0.15
                    self.publisher1.publish(self.cmd)
                    if abs(target_angle - self.ang) <= err_tol_ang:
                        self._mode = 1
                        self.cmd.angular.z = 0.0
                        self.publisher1.publish(self.cmd)
                        break
                print("Finish Turn")
                break
                # else:
                #     print("Turning Back")
                
            elif self._turn_direction == 5: #goal
                self._mode == 3
                print("Goal!!!")
                self.idle()
                break
            else:
                print("sign undetected")
                self.cmd.linear.x = 0.0
                self.cmd.angular.z = 0.0
                break                
            #else:
                #TODO about to hit the wall, but cannot find any recognizable sign, should move back to find a position where the sign can be obsreved
            # self.publisher1.publish(self.cmd)
            # time.sleep(0.1)


    def idle(self):
        while(True):
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.0
            self.publisher1.publish(self.cmd)

    def move(self):
        if self._collide_warning == 1 and self._mode == 1:
            self._mode = 2 # stop and read signs
            self.cmd.linear.x = 0.0
            print("Looking For Sign")
            turn_action = Thread(target=self.turn_degrees)
            turn_action.start()
        elif self._collide_warning == 2 and self._mode == 1:
            self.cmd.linear.x = 0.12
            self.cmd.angular.z = 0.0
            print("Moving Forward")
        self.publisher1.publish(self.cmd)


def main(args=None):
    rclpy.init(args=args) #init routine needed for ROS2.
    mv_node = move() #Create class object to be used.
    while True:
        rclpy.spin_once(mv_node, timeout_sec=0.1) # Trigger callback processing.
	#Clean up and shutdown.
    video_subscriber.destroy_node()  
    rclpy.shutdown()
    

if __name__ == '__main__':
	main()