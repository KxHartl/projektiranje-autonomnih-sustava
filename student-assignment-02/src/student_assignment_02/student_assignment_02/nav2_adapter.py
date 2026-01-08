#!/usr/bin/env python3
"""
Nav2 Adapter Node - JEDNOSTAVNA VERZIJA
Samo sljedi putanju i šalje /cmd_vel komande

FIX:
1. Sluša /planned_path (od A* planera)
2. SVE POZICIJE SU U MAP FRAMEU
3. Direktno sljedi putanju
4. Šalje /cmd_vel komande robotu

BEZ:
- /follow_path akcije
- Controller Servera
- Lifecycle Managera
- Kruga

SAMO GEOMETRIJA:
1. Robot je u map frameu (TF lookup)
2. Putanja je u map frameu (/planned_path)
3. Izračuna vektor do sljedeće točke
4. Šalje komandu
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
from tf2_ros import TransformListener, Buffer
import math


class Nav2Adapter(Node):
    """Adapter koji sljedi putanju - JEDNOSTAVNO"""
    
    def __init__(self):
        super().__init__('nav2_adapter')
        
        self.get_logger().info(
            '\n' + '='*80 +
            '\n[NAV2 ADAPTER] Inicijalizacija' +
            '\n- Sluša: /planned_path (od A* planera)' +
            '\n- Šalje: /cmd_vel (robotu)' +
            '\n- SVE U MAP FRAMEU' +
            '\n- NEMA KRUGA' +
            '\n' + '='*80 + '\n'
        )
        
        # Subscribe na A* putanju
        self.path_sub = self.create_subscription(
            Path,
            '/planned_path',
            self.path_callback,
            10
        )
        self.get_logger().info('[INIT] Subscribe /planned_path ✓')
        
        # Publisher za cmd_vel
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        self.get_logger().info('[INIT] Publisher /cmd_vel ✓')
        
        # TF lookup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.get_logger().info('[INIT] TF Listener ✓')
        
        # Stanje
        self.current_path = None
        self.path_index = 0
        self.is_following = False
        
        # Parametri
        self.declare_parameter('linear_speed', 0.3)
        self.declare_parameter('angular_speed', 1.0)
        self.declare_parameter('distance_tolerance', 0.15)
        
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.distance_tolerance = self.get_parameter('distance_tolerance').value
        
        self.get_logger().info(
            f'[INIT] Parametri: linear={self.linear_speed}, '
            f'angular={self.angular_speed}, tolerance={self.distance_tolerance}'
        )
        
        # Timer za sljeđenje
        self.timer = self.create_timer(0.1, self.follow_timer)
        self.get_logger().info('[INIT] Timer 0.1s ✓')
        self.get_logger().info('='*80 + '\n')
    
    def path_callback(self, msg: Path):
        """Primanje putanje od A* planera"""
        if len(msg.poses) == 0:
            self.get_logger().warn('[PATH] Prazna putanja!')
            self.is_following = False
            return
        
        self.current_path = msg
        self.path_index = 0
        self.is_following = True
        
        # Info
        total_length = 0.0
        for i in range(len(msg.poses) - 1):
            p1 = msg.poses[i].pose.position
            p2 = msg.poses[i + 1].pose.position
            dx = p2.x - p1.x
            dy = p2.y - p1.y
            total_length += math.sqrt(dx*dx + dy*dy)
        
        self.get_logger().info(
            f'[PATH] Primljena putanja: {len(msg.poses)} točaka, '
            f'dužina {total_length:.2f}m, frame={msg.header.frame_id}'
        )
        self.get_logger().info('[FOLLOW] Počinjem sljjeđenje putanje!\n')
    
    def follow_timer(self):
        """Timer za sljeđenje putanje"""
        
        # Ako nema putanje, zaustavi robota
        if not self.is_following or self.current_path is None:
            cmd = Twist()
            self.cmd_pub.publish(cmd)
            return
        
        # Ako je kraj putanje, zaustavi i završi
        if self.path_index >= len(self.current_path.poses):
            self.get_logger().info('[DONE] Putanja završena! Robot zaustavljen.\n')
            self.is_following = False
            cmd = Twist()
            self.cmd_pub.publish(cmd)
            return
        
        # Dohvati robot poziciju iz TF-a
        try:
            tf = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            robot_x = tf.transform.translation.x
            robot_y = tf.transform.translation.y
        except Exception as e:
            self.get_logger().error(f'[TF] Lookup failed: {e}')
            cmd = Twist()
            self.cmd_pub.publish(cmd)
            return
        
        # Dohvati trenutnu ciljnu točku
        target_pose = self.current_path.poses[self.path_index]
        target_x = target_pose.pose.position.x
        target_y = target_pose.pose.position.y
        
        # Izračuna udaljenost
        dx = target_x - robot_x
        dy = target_y - robot_y
        distance = math.sqrt(dx*dx + dy*dy)
        
        # DEBUG: svakih 10 iteracija
        if self.path_index % 10 == 0:
            self.get_logger().debug(
                f'[FOLLOW] Točka {self.path_index}: '
                f'robot=({robot_x:.2f},{robot_y:.2f}), '
                f'target=({target_x:.2f},{target_y:.2f}), '
                f'dist={distance:.2f}m'
            )
        
        # Ako je blizu, idi na sljedeću točku
        if distance < self.distance_tolerance:
            self.path_index += 1
            return
        
        # Izračuna smjer
        angle_to_target = math.atan2(dy, dx)
        
        # Kreiraj Twist komandu
        cmd = Twist()
        cmd.linear.x = min(self.linear_speed, distance * 0.5)  # Usporavanje blizu cilja
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = angle_to_target * 0.3  # Rotacija prema cilju
        
        # Objavi
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = Nav2Adapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Zaustavi robota
        cmd = Twist()
        node.cmd_pub.publish(cmd)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
