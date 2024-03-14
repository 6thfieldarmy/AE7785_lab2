import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String

from geometry_msgs.msg import Point, Twist
import math

class go_to_goal(Node):
    def __init__(self):
        super().__init__('go_to_goal')
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
            Point,
             '/get_object_range/range',
              self.range_callback,
              qos_profile)
        
        self._loc_subscription = self.create_subscription(
            Point,
             '/get_robot_location/position',
              self.loc_callback,
              qos_profile)
        
        self.publisher1 = self.create_publisher(Twist,'go_to_goal/cmd_vel',qos_profile)
        # self.publisher2 = self.create_publisher(CompressedImage,'/go_to_goal/compressed',qos_profile)
        self.timer_ = self.create_timer(0.5, self.go_goal)
        self.time_flag = 0

    def range_callback(self, msg):
        self.get_logger().info('object range = (%.2f)' % msg.x)
        self.range = msg.x
    
    def loc_callback(self, msg):
        self.get_logger().info('location x = (%.2f)' % msg.x)
        self.loc_x = msg.x
        self.loc_y = msg.y

    def turn_90_degrees(self):
        self.cmd.linear.x = 0.0
        current_angle = 0.0
        angular_speed = 0.1 
        relative_angle = math.pi / 2
        self.get_logger().info('turnning 90 degrees')
        current_time = self.get_clock().now()
        elapsed_time = current_time - self.start_time
        current_angle = angular_speed * elapsed_time.seconds
        if current_angle < relative_angle:
            self.cmd.angular.z = angular_speed
        else:
            self.cmd.angular.z = 0.0

    def go_goal(self):
        self.cmd = Twist()
        if self.loc_x < float(1.5) and self.loc_y == 0:    
            if (self.range > float(0.2)):    
                self.cmd.linear.x = 0.1
                self.get_logger().info('moving to 1st point')
            else:
                self.cmd.linear.x = 0.0
                self.get_logger().info('object detected!')
        elif self.loc_x == float(1.5) and self.loc_y == 0:
            if(self.time_flag == 0):
                self.start_time = self.get_clock().now()
                self.time_flag = 1
            self.turn_90_degrees()
            #TODO: need stop for 10 seconds
        elif self.loc_x == float(1.5) and self.loc_y > float(-1.5):
            self.time_flag = 0
            if (self.range > float(0.2)):    
                self.cmd.linear.x = 0.1
                self.get_logger().info('moving to 2nd point')
            else:
                self.cmd.linear.x = 0.0
                self.get_logger().info('object detected!')
            # ???? what blue box?
        elif self.loc_x == float(1.5) and self.loc_y == float(-1.5):
            if(self.time_flag == 0):
                self.start_time = self.get_clock().now()
                self.time_flag = 1
            self.turn_90_degrees()
        elif self.loc_x <= float(1.5) and self.loc_y == float(-1.5):
            self.time_flag = 0
            if (self.range > float(0.2)):    
                self.cmd.linear.x = 0.1
                self.get_logger().info('moving to 3rd point')
            else:
                self.cmd.linear.x = 0.0
                self.get_logger().info('object detected!')
        else:
            self.cmd.linear.x = 0.0
            self.get_logger().info('Stop.')

        self.publisher1.publish(self.cmd)

def main():
    rclpy.init()
    go_goal_node = go_to_goal()
    rclpy.spin(go_goal_node) 
    go_goal_node.destroy_node()    
    rclpy.shutdown()
   