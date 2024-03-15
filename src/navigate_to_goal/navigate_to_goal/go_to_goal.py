import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Float64

from geometry_msgs.msg import Point, Twist
import math
import numpy as np
import time

class go_to_goal(Node):
    def __init__(self):
        super().__init__('go_to_goal')
        self.waypoint1_arrived = False
        self.waypoint2_arrived = False
        self.waypoint3_arrived = False

        self.sleep1_flag = True
        self.sleep2_flag = True

        

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
        self._range_subscription = self.create_subscription(
            Twist,
             '/get_object_range/range',
              self.range_callback,
              qos_profile)
        
        self._loc_subscription = self.create_subscription(
            Point,
             '/get_robot_location/position',
              self.loc_callback,
              qos_profile)
        
        self._orientation_subscription = self.create_subscription(
            Float64,
             '/get_robot_location/orientation',
              self.orient_callback,
              qos_profile)
        
        self.publisher1 = self.create_publisher(Twist,'/cmd_vel', 5)
        # self.publisher2 = self.create_publisher(CompressedImage,'/go_to_goal/compressed',qos_profile)
        
        #time.sleep(5) #for me to get ready 
        self.timer_ = self.create_timer(0.2, self.go_goal)
        #self.time_flag = 0

    def range_callback(self, twist_msg):
        #self.get_logger().info('obstacle avoidance turn cmd = (%.4f)' % twist_msg.angular.z)
        self.obs_avoidance_turn = twist_msg.angular.z
        self.obs_avoidance_move = twist_msg.linear.x
    
    def loc_callback(self, msg):
        #self.get_logger().info('location x = (%.2f)' % msg.x)
        self.loc_x = msg.x
        self.loc_y = msg.y
        

    def orient_callback(self, msg):
        #self.get_logger().info('orientation a = (%.2f)' % msg.data)
        self.ang = msg.data

    # def turn_90_degrees(self):
    #     self.cmd.linear.x = 0.0
    #     current_angle = 0.0
    #     angular_speed = 0.1 
    #     relative_angle = math.pi / 2
    #     self.get_logger().info('turnning 90 degrees')
    #     current_time = self.get_clock().now()
    #     elapsed_time = current_time - self.start_time
    #     current_angle = angular_speed * elapsed_time.seconds
    #     if current_angle < relative_angle:
    #         self.cmd.angular.z = angular_speed
    #     else:
    #         self.cmd.angular.z = 0.0

    def go_goal(self):
        self.cmd = Twist()
        err_tol = 0.08
        err_tol_ang = 0.06
        self.cmd.linear.x = 0.0  #initial
        self.cmd.angular.z = 0.0

        if 1.5 - 0.5*err_tol <= self.loc_x < 1.5 + 0.5*err_tol and 0.0 - 0.5*err_tol <= self.loc_y <= 0.0 + 0.5*err_tol:
            self.waypoint1_arrived = True
            if self.sleep1_flag:
                self.sleep1_flag = False
                self.cmd.linear.x = 0.0
                self.cmd.angular.z = 0.0
                print("First waypoint arrived! Rest 10 sec!")
                self.publisher1.publish(self.cmd)
                time.sleep(10)
        if 1.5 - 0.5*err_tol <= self.loc_x < 1.5 + 0.5*err_tol and 1.4 - 0.5*err_tol  <= self.loc_y <= 1.4 + 0.5*err_tol:
            self.waypoint2_arrived = True
            if self.sleep2_flag:
                self.sleep2_flag = False
                self.cmd.linear.x = 0.0
                self.cmd.angular.z = 0.0
                print("Second waypoint arrived! Rest 10 sec!")
                self.publisher1.publish(self.cmd)
                time.sleep(10)
            #TODO stop 10 sec
        if 0.0 - 0.5*err_tol <= self.loc_x < 0.0 + 0.5*err_tol and 1.4 - 0.5*err_tol <= self.loc_y <= 1.4 + 0.5*err_tol:
            self.waypoint3_arrived = True
            print("We made it! Destination Arrived!")
            return

        if not self.waypoint1_arrived:            
              
            if (self.obs_avoidance_turn == 0) and self.obs_avoidance_move == 0:    #go goal logic
                self.get_logger().info('moving to 1st point')  
                rel_pos_x = 1.5 - self.loc_x
                rel_pos_y = 0 - self.loc_y
                rel_target_angle = np.arctan2(rel_pos_y , rel_pos_x)
                
                if self.ang > rel_target_angle + err_tol_ang: #means need to turn right-
                    self.cmd.angular.z = max(-0.6, 0.6*(rel_target_angle - self.ang )/(0.4) )
                elif self.ang < rel_target_angle - err_tol_ang: #means need to turn left+
                    self.cmd.angular.z = min(0.6, 0.6*(rel_target_angle - self.ang )/(0.4) )
                else:   #right orientation
                    self.cmd.linear.x = 0.1
            else:
                self.cmd.angular.z = self.obs_avoidance_turn
                self.cmd.linear.x = self.obs_avoidance_move
                self.get_logger().info("Avoiding Obstacles")

        elif self.waypoint1_arrived and not self.waypoint2_arrived:
            
            if (self.obs_avoidance_turn == 0) and self.obs_avoidance_move == 0:    #go goal logic
                self.get_logger().info('moving to 2nd point') 
                rel_pos_x = 1.5 - self.loc_x
                rel_pos_y = 1.4 - self.loc_y
                rel_target_angle = np.arctan2(rel_pos_y , rel_pos_x)
                
                if self.ang > rel_target_angle + err_tol_ang: #means need to turn right-
                    self.cmd.angular.z = max(-0.6, 0.6*(rel_target_angle - self.ang )/(0.4) )
                elif self.ang < rel_target_angle - err_tol_ang: #means need to turn left+
                    self.cmd.angular.z = min(0.6, 0.6*(rel_target_angle - self.ang )/(0.4) )
                else:   #right orientation
                    self.cmd.linear.x = 0.1
            else:
                self.cmd.angular.z = self.obs_avoidance_turn
                self.cmd.linear.x = self.obs_avoidance_move
                self.get_logger().info("Avoiding Obstacles")

        elif self.waypoint1_arrived and self.waypoint2_arrived and not self.waypoint3_arrived:
            if (self.obs_avoidance_turn == 0) and self.obs_avoidance_move == 0:    #go goal logic
                self.get_logger().info('moving to 3rd point') 
                rel_pos_x = 0.0 - self.loc_x
                rel_pos_y = 1.4 - self.loc_y
                rel_target_angle = np.arctan2(rel_pos_y , rel_pos_x) #this is likely to be a negative number
                #err = self.ang - rel_target_angle
                ang_diff = rel_target_angle - self.ang
                if ang_diff > np.pi:
                    ang_diff = ang_diff - 2*np.pi
                elif ang_diff < -np.pi:
                    ang_diff = 2*np.pi + ang_diff
                    
                if self.ang > rel_target_angle + err_tol_ang: #means need to turn right-
                    self.cmd.angular.z = max(-0.6, 0.6*(ang_diff )/(0.4) )
                    print("3rd waypoint angular turn = %.2f" % self.cmd.angular.z )
                elif self.ang < rel_target_angle - err_tol_ang: #means need to turn left+
                    self.cmd.angular.z = min(0.6, 0.6*(ang_diff )/(0.4) )
                    print("3rd waypoint angular turn = %.2f" % self.cmd.angular.z )
                else:   #right orientation
                    self.cmd.angular.z = 0.0
                    self.cmd.linear.x = 0.1
            else:
                self.cmd.angular.z = self.obs_avoidance_turn
                self.cmd.linear.x = self.obs_avoidance_move
                self.get_logger().info("Avoiding Obstacles")

        # elif self.loc_x == float(1.5) and self.loc_y == float(-1.5):
        #     if(self.time_flag == 0):
        #         self.start_time = self.get_clock().now()
        #         self.time_flag = 1
        #     self.turn_90_degrees()
        # elif self.loc_x <= float(1.5) and self.loc_y == float(-1.5):
        #     self.time_flag = 0
        #     if (self.range > float(0.2)):    
        #         self.cmd.linear.x = 0.1
        #         self.get_logger().info('moving to 3rd point')
        #     else:
        #         self.cmd.linear.x = 0.0
        #         self.get_logger().info('object detected!')
        elif self.waypoint3_arrived:
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.0
            self.get_logger().info('Stop.')

        else:
            print("else condition")
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.0   

        self.publisher1.publish(self.cmd)
        #print(self.cmd)

def main():
    rclpy.init()
    go_goal_node = go_to_goal()
    rclpy.spin(go_goal_node) 
    go_goal_node.destroy_node()    
    rclpy.shutdown()

if __name__ == '__main__':
	main()
   