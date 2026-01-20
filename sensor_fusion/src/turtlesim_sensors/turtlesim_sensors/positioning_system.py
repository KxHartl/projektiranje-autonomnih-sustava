import rclpy
from rclpy.node import Node

from turtlesim.msg import Pose
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf_transformations import quaternion_from_euler

import random


class TurtlesimPositioningSystem(Node):

    def __init__(self):
        super().__init__('positioning_system')
        self.subscription = self.create_subscription(Pose, '/turtle1/pose', self.positioning_callback, 10)
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, 'turtle1/sensors/pose', 10)
        self.subscription

        self.declare_parameter("positioning_frequency", 1)
        self.declare_parameter("error_x_systematic", 0.0)
        self.declare_parameter("error_x_random", 0.2)
        self.declare_parameter("error_y_systematic", 0.0)
        self.declare_parameter("error_y_random", 0.2)
        self.declare_parameter("error_yaw_systematic", 0.0)
        self.declare_parameter("error_yaw_random", 0.2)
        self.declare_parameter("frame_id", "map")
        
        timer_period = 1 / float(self.get_parameter("positioning_frequency").value)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.msg_out = PoseWithCovarianceStamped()
        
    def positioning_callback(self, msg: Pose) -> None:
        # Load parameters so dynamic reconfigure takes effect without restart
        mu_x = float(self.get_parameter("error_x_systematic").value)
        sigma_x = float(self.get_parameter("error_x_random").value)
        mu_y = float(self.get_parameter("error_y_systematic").value)
        sigma_y = float(self.get_parameter("error_y_random").value)
        mu_yaw = float(self.get_parameter("error_yaw_systematic").value)
        sigma_yaw = float(self.get_parameter("error_yaw_random").value)
        frame_id = self.get_parameter("frame_id").value

        # Add systematic bias and Gaussian noise to simulate GPS-like measurements
        noisy_x = msg.x + mu_x + random.gauss(0.0, sigma_x)
        noisy_y = msg.y + mu_y + random.gauss(0.0, sigma_y)
        noisy_yaw = msg.theta + mu_yaw + random.gauss(0.0, sigma_yaw)

        # Convert yaw-only to quaternion
        q_x, q_y, q_z, q_w = quaternion_from_euler(0.0, 0.0, noisy_yaw)

        # Fill outgoing message
        self.msg_out.header.stamp = self.get_clock().now().to_msg()
        self.msg_out.header.frame_id = frame_id
        self.msg_out.pose.pose.position.x = noisy_x
        self.msg_out.pose.pose.position.y = noisy_y
        self.msg_out.pose.pose.position.z = 0.0
        self.msg_out.pose.pose.orientation.x = q_x
        self.msg_out.pose.pose.orientation.y = q_y
        self.msg_out.pose.pose.orientation.z = q_z
        self.msg_out.pose.pose.orientation.w = q_w

        # Covariance: 6x6 flattened row-major (x, y, z, roll, pitch, yaw)
        cov = [0.0] * 36
        cov[0] = sigma_x ** 2           # var(x)
        cov[7] = sigma_y ** 2           # var(y)
        cov[14] = 1e-9                  # var(z) minimal since we fix z=0
        cov[21] = 1e-9                  # var(roll)
        cov[28] = 1e-9                  # var(pitch)
        cov[35] = sigma_yaw ** 2        # var(yaw)
        self.msg_out.pose.covariance = cov
        
    def timer_callback(self):
        self.publisher_.publish(self.msg_out)

def main(args=None):
    rclpy.init(args=args)
    positioning_system = TurtlesimPositioningSystem()
    rclpy.spin(positioning_system)
    positioning_system.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()