import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String

from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry

class get_robot_location(Node):
    def __init__(self):
        super().__init__('get_object_range')
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
        self.init_x = 0.0
        self.init_y = 0.0
        self.init_flag = 1
        self.loc_msg = Point()
        self.publisher1 = self.create_publisher(Point,'get_robot_location/position',qos_profile)

    def _odom_callback(self, msg):
        self.get_robot_location(msg)

    def get_robot_location(self, msg):
        if self.init_flag:
          self.init_x = round(msg.pose.pose.position.x,3)
          self.init_y = round(msg.pose.pose.position.y,3)
          self.init_flag = 0
        else:
          self.loc_msg.x = round(msg.pose.pose.position.x,3) - self.init_x
          self.loc_msg.y = round(msg.pose.pose.position.y,3) - self.init_y
        #print(self.loc_msg.x, self.loc_msg.y)
        self.publisher1.publish(self.loc_msg)

def main(args=None):
    rclpy.init(args=args)
    get_robot_location_node = get_robot_location()
    rclpy.spin(get_robot_location_node)
    get_robot_location_node.destroy_node()
    rclpy.shutdown()
