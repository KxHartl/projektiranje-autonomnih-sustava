import rclpy
from rclpy.node import Node

from turtlesim.msg import Pose
from geometry_msgs.msg import TwistWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler

import random
import math


class TurtlesimOdometry(Node):

    def __init__(self):
        super().__init__('odometry')
        self.subscription = self.create_subscription(Pose, '/turtle1/pose', self.twist_callback, 10)
        self.publisher_ = self.create_publisher(TwistWithCovarianceStamped, 'turtle1/sensors/twist', 10)
        self.pose_publisher_ = self.create_publisher(PoseStamped, 'turtle1/sensors/pose_from_twist', 10)
        self.subscription

        self.declare_parameter("odometry_frequency", 20)
        self.declare_parameter("error_vx_systematic", 0.1)
        self.declare_parameter("error_vx_random", 0.01)
        self.declare_parameter("error_angular_systematic", 0.01)
        self.declare_parameter("error_angular_random", 0.01)
        
        self.timer_period_ = 1 / float(self.get_parameter("odometry_frequency").value) #seconds
        self.timer = self.create_timer(self.timer_period_, self.timer_callback)
        
        self.msg_out = TwistWithCovarianceStamped()
        self.pose_msg = PoseStamped()
        
        self.theta_ = 0.0
        self.x_ = 5.544445
        self.y_ = 5.544445
        
        self.last_time_ = None
        
    def twist_callback(self, msg: Pose) -> None:
        #calcular dt for odometry
        #load parameters
        #add noise 
        #calculate pose
        #publish twist
        #publish pose  
        mu_vx = 0.0
        sigma_vx = 0.0
        
        mu_angular = 0.0
        sigma_angular = 0.0
        
    
        
    def timer_callback(self):
        self.publisher_.publish(self.msg_out)
        self.pose_publisher_.publish(self.pose_msg)

def main(args=None):
    rclpy.init(args=args)
    odometry = TurtlesimOdometry()
    rclpy.spin(odometry)
    odometry.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()