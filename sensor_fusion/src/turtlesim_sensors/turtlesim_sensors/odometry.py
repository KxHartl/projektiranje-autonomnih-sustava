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
        self.declare_parameter("frame_id", "odom")
        
        self.timer_period_ = 1 / float(self.get_parameter("odometry_frequency").value) #seconds
        self.timer = self.create_timer(self.timer_period_, self.timer_callback)
        
        self.msg_out = TwistWithCovarianceStamped()
        self.pose_msg = PoseStamped()
        
        self.theta_ = 0.0
        self.x_ = 5.544445
        self.y_ = 5.544445
        
        self.last_time_ = None
        
    def twist_callback(self, msg: Pose) -> None:
        # Calculate dt for odometry integration
        current_time = self.get_clock().now()
        if self.last_time_ is None:
            self.last_time_ = current_time
            return
        
        dt = (current_time - self.last_time_).nanoseconds / 1e9  # Convert to seconds
        self.last_time_ = current_time
        
        # Load parameters so dynamic reconfigure takes effect
        mu_vx = float(self.get_parameter("error_vx_systematic").value)
        sigma_vx = float(self.get_parameter("error_vx_random").value)
        mu_angular = float(self.get_parameter("error_angular_systematic").value)
        sigma_angular = float(self.get_parameter("error_angular_random").value)
        frame_id = self.get_parameter("frame_id").value
        
        # Calculate forward velocity and angular velocity from position change
        # Approximate velocity = distance / dt
        dx = msg.x - self.x_
        dy = msg.y - self.y_
        distance = math.sqrt(dx**2 + dy**2)
        v_linear = distance / dt if dt > 0 else 0.0
        
        # Angular velocity = angle_change / dt
        d_theta = msg.theta - self.theta_
        # Normalize angle difference to [-pi, pi]
        d_theta = math.atan2(math.sin(d_theta), math.cos(d_theta))
        v_angular = d_theta / dt if dt > 0 else 0.0
        
        # Add systematic bias and Gaussian noise to simulate odometry-like measurements
        noisy_vx = v_linear + mu_vx + random.gauss(0.0, sigma_vx)
        noisy_v_angular = v_angular + mu_angular + random.gauss(0.0, sigma_angular)
        
        # Integrate noisy velocity to update odometry pose
        self.x_ += noisy_vx * math.cos(self.theta_) * dt
        self.y_ += noisy_vx * math.sin(self.theta_) * dt
        self.theta_ += noisy_v_angular * dt
        
        # Fill twist message with covariance
        self.msg_out.header.stamp = current_time.to_msg()
        self.msg_out.header.frame_id = frame_id
        self.msg_out.twist.twist.linear.x = noisy_vx
        self.msg_out.twist.twist.linear.y = 0.0
        self.msg_out.twist.twist.linear.z = 0.0
        self.msg_out.twist.twist.angular.x = 0.0
        self.msg_out.twist.twist.angular.y = 0.0
        self.msg_out.twist.twist.angular.z = noisy_v_angular
        
        # Covariance: 6x6 flattened row-major (vx, vy, vz, wx, wy, wz)
        twist_cov = [0.0] * 36
        twist_cov[0] = sigma_vx ** 2       # var(vx)
        twist_cov[7] = 1e-9                # var(vy) minimal
        twist_cov[14] = 1e-9               # var(vz) minimal
        twist_cov[21] = 1e-9               # var(wx) minimal
        twist_cov[28] = 1e-9               # var(wy) minimal
        twist_cov[35] = sigma_angular ** 2 # var(wz)
        self.msg_out.twist.covariance = twist_cov
        
        # Fill pose message from integrated odometry
        self.pose_msg.header.stamp = current_time.to_msg()
        self.pose_msg.header.frame_id = frame_id
        self.pose_msg.pose.position.x = self.x_
        self.pose_msg.pose.position.y = self.y_
        self.pose_msg.pose.position.z = 0.0
        q_x, q_y, q_z, q_w = quaternion_from_euler(0.0, 0.0, self.theta_)
        self.pose_msg.pose.orientation.x = q_x
        self.pose_msg.pose.orientation.y = q_y
        self.pose_msg.pose.orientation.z = q_z
        self.pose_msg.pose.orientation.w = q_w
        
    
        
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