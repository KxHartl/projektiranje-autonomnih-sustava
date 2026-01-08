#!/usr/bin/env python3
"""
Simple Path Follower - OD NULE

Radio:
1. Prima /planned_path
2. Prima robot poziciju iz TF
3. Sljedi putanju do kraja
4. Šalje /cmd_vel komande
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
from tf2_ros import TransformListener, Buffer
import math
from typing import Optional, Tuple


class SimplePathFollower(Node):
    
    def __init__(self):
        super().__init__('simple_path_follower')
        
        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscribers
        self.path_sub = self.create_subscription(Path, '/planned_path', self.on_path, 10)
        
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Stanje
        self.current_path = None
        self.path_index = 0
        self.is_following = False
        
        # Parametri
        self.declare_parameter('linear_speed', 0.3)
        self.declare_parameter('angular_speed', 1.0)
        self.declare_parameter('distance_tolerance', 0.1)
        
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.distance_tolerance = self.get_parameter('distance_tolerance').value
        
        # Timer za slijeđenje
        self.follow_timer = self.create_timer(0.05, self.follow_callback)  # 20Hz
        
        self.get_logger().info('Simple Path Follower started')
    
    def on_path(self, msg: Path):
        """Primanje nove putanje"""
        if len(msg.poses) == 0:
            return
        
        self.current_path = msg
        self.path_index = 0
        self.is_following = True
        
        self.get_logger().info(f'New path: {len(msg.poses)} poses')
    
    def get_robot_pos(self) -> Optional[Tuple[float, float]]:
        """Dohvati robot poziciju"""
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            return (tf.transform.translation.x, tf.transform.translation.y)
        except:
            return None
    
    def follow_callback(self):
        """Timer callback za slijeđenje putanje"""
        # Ako nema putanje, zaustavi
        if not self.is_following or not self.current_path:
            cmd = Twist()
            self.cmd_pub.publish(cmd)
            return
        
        # Ako je putanja gotova, zaustavi
        if self.path_index >= len(self.current_path.poses):
            self.get_logger().info('Path complete')
            self.is_following = False
            cmd = Twist()
            self.cmd_pub.publish(cmd)
            return
        
        # Dohvati robot poziciju
        robot_pos = self.get_robot_pos()
        if not robot_pos:
            return
        
        # Dohvati trenutnu ciljnu točku
        target = self.current_path.poses[self.path_index]
        target_x = target.pose.position.x
        target_y = target.pose.position.y
        
        # Izračunaj udaljenost
        dx = target_x - robot_pos[0]
        dy = target_y - robot_pos[1]
        distance = math.sqrt(dx*dx + dy*dy)
        
        # Ako je dovoljno blizu, idi na sljedeću točku
        if distance < self.distance_tolerance:
            self.path_index += 1
            return
        
        # Kreiraj komandu
        cmd = Twist()
        
        # Linearna brzina proporcionalno udaljenosti
        cmd.linear.x = min(self.linear_speed, distance * 0.5)
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        
        # Kutna brzina - rotacija prema cilju
        angle = math.atan2(dy, dx)
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = min(self.angular_speed, abs(angle) * 0.5) * (1 if angle > 0 else -1)
        
        # Objavi
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = SimplePathFollower()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
