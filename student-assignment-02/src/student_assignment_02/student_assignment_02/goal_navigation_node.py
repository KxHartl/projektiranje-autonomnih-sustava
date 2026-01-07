#!/usr/bin/env python3
"""
Goal Navigation Node
ROS 2 node for robot navigation along planned path
"""

import rclpy
from rclpy.node import Node
import math

from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
from student_assignment_02.utils import normalize_angle


class GoalNavigationNode(Node):
    """ROS 2 Goal Navigation Node"""

    def __init__(self):
        super().__init__('goal_navigation_node')
        
        # Subscriptions
        self.path_sub = self.create_subscription(
            Path,
            '/planned_path',
            self.path_callback,
            rclpy.qos.qos_profile_system_default
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # Timer for movement loop
        self.move_timer = self.create_timer(
            0.1,  # 100 ms
            self.move_callback
        )
        
        # Declare parameters
        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('angular_speed', 0.5)
        self.declare_parameter('waypoint_tolerance', 0.15)
        self.declare_parameter('angle_tolerance', 0.2)
        
        # Get parameters
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.waypoint_tolerance = self.get_parameter('waypoint_tolerance').value
        self.angle_tolerance = self.get_parameter('angle_tolerance').value
        
        # State
        self.current_path = None
        self.path_received = False
        self.current_waypoint_idx = 0
        self.is_moving = False
        
        # Simulated robot pose
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        
        # Logging
        self.get_logger().info('Goal Navigation Node initialized')
        self.get_logger().info(f'  Linear speed: {self.linear_speed:.2f} m/s')
        self.get_logger().info(f'  Angular speed: {self.angular_speed:.2f} rad/s')
        self.get_logger().info(f'  Waypoint tolerance: {self.waypoint_tolerance:.2f} m')
        self.get_logger().info(f'  Angle tolerance: {self.angle_tolerance:.2f} rad')

    def path_callback(self, msg: Path):
        """Callback for planned path subscription"""
        self.current_path = msg
        self.path_received = True
        self.current_waypoint_idx = 0
        self.is_moving = True
        
        self.get_logger().info(
            f'✅ New path received with {len(msg.poses)} points'
        )
        
        # Initialize robot position at first waypoint
        if msg.poses:
            self.robot_x = msg.poses[0].pose.position.x
            self.robot_y = msg.poses[0].pose.position.y

    def move_callback(self):
        """Timer callback for movement loop"""
        if not self.path_received or not self.is_moving or not self.current_path.poses:
            self.stop_robot()
            return
        
        # Check if all waypoints are reached
        if self.current_waypoint_idx >= len(self.current_path.poses):
            self.get_logger().info('✅ Goal reached! Movement finished.')
            self.stop_robot()
            self.is_moving = False
            self.path_received = False
            return
        
        # Get current target waypoint
        target_pose = self.current_path.poses[self.current_waypoint_idx].pose
        target_x = target_pose.position.x
        target_y = target_pose.position.y
        
        # Calculate distance to target
        dx = target_x - self.robot_x
        dy = target_y - self.robot_y
        distance = math.sqrt(dx * dx + dy * dy)
        
        # Check if waypoint reached
        if distance < self.waypoint_tolerance:
            self.current_waypoint_idx += 1
            if (self.current_waypoint_idx % 10 == 0 or 
                self.current_waypoint_idx == len(self.current_path.poses)):
                self.get_logger().info(
                    f'✅ Waypoint {self.current_waypoint_idx} reached ({distance:.2f} m)'
                )
            return
        
        # Calculate angle to target
        target_angle = math.atan2(dy, dx)
        angle_diff = normalize_angle(target_angle - self.robot_theta)
        
        # Create velocity command
        cmd_vel = Twist()
        cmd_vel.linear.z = 0.0
        cmd_vel.angular.x = 0.0
        cmd_vel.angular.y = 0.0
        
        # Control logic:
        # 1. If angle error is large, rotate in place
        # 2. Otherwise, move forward with small angle correction
        
        if abs(angle_diff) > self.angle_tolerance:
            # Rotate to face target
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = (
                self.angular_speed if angle_diff > 0 else -self.angular_speed
            )
        else:
            # Move forward with angle correction
            cmd_vel.linear.x = self.linear_speed
            # PD control for angle
            cmd_vel.angular.z = max(
                -self.angular_speed,
                min(self.angular_speed, angle_diff * 0.5)
            )
        
        # Publish velocity command
        self.cmd_vel_pub.publish(cmd_vel)
        
        # Simulate robot motion
        dt = 0.1  # Timer interval
        self.robot_x += cmd_vel.linear.x * math.cos(self.robot_theta) * dt
        self.robot_y += cmd_vel.linear.x * math.sin(self.robot_theta) * dt
        self.robot_theta = normalize_angle(self.robot_theta + cmd_vel.angular.z * dt)
        
        # Log every 20 iterations
        if self.current_waypoint_idx % 20 == 0:
            self.get_logger().debug(
                f'[WP {self.current_waypoint_idx}] '
                f'Robot: ({self.robot_x:.2f}, {self.robot_y:.2f}, {self.robot_theta:.2f} rad) | '
                f'Target: ({target_x:.2f}, {target_y:.2f}) | '
                f'Distance: {distance:.2f} m | '
                f'Angle diff: {angle_diff:.2f} rad'
            )

    def stop_robot(self):
        """Stop robot by sending zero velocity"""
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.linear.y = 0.0
        cmd_vel.linear.z = 0.0
        cmd_vel.angular.x = 0.0
        cmd_vel.angular.y = 0.0
        cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel)


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    node = GoalNavigationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
