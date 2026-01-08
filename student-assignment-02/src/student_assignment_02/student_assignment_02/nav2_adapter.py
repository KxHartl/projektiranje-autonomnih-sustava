#!/usr/bin/env python3
"""
Nav2 Adapter Node - Sljedi putanju i zaustavlja se na cilju

FIX:
1. Sluša /planned_path (od A* planera)
2. SVE POZICIJE U MAP FRAMEU
3. Direktno sljedi putanju
4. Šalje /cmd_vel komande robotu
5. ZAUSTAVLJA SE NA CILJU
6. Praći svaki korak
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
from tf2_ros import TransformListener, Buffer
import math


class Nav2Adapter(Node):
    """Adapter koji sljedi putanju i zaustavlja se na cilju"""
    
    def __init__(self):
        super().__init__('nav2_adapter')
        
        self.get_logger().info(
            '\n' + '='*80 +
            '\n[NAV2 ADAPTER] Inicijalizacija' +
            '\n- Sluša: /planned_path (od A* planera)' +
            '\n- Šalje: /cmd_vel (robotu)' +
            '\n- SVE U MAP FRAMEU' +
            '\n- PRATI PUTANJU I ZAUSTAVLJA SE' +
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
        self.goal_reached = False
        self.last_log_index = -5
        
        # Parametri
        self.declare_parameter('linear_speed', 0.3)
        self.declare_parameter('angular_speed', 1.0)
        self.declare_parameter('distance_tolerance', 0.15)
        self.declare_parameter('goal_tolerance', 0.2)
        
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.distance_tolerance = self.get_parameter('distance_tolerance').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        
        self.get_logger().info(
            f'[INIT] Parametri: linear={self.linear_speed}m/s, '
            f'angular={self.angular_speed}rad/s, distance_tol={self.distance_tolerance}m, '
            f'goal_tol={self.goal_tolerance}m'
        )
        
        # Timer za sljedićenje
        self.timer = self.create_timer(0.05, self.follow_timer)  # 20 Hz
        self.get_logger().info('[INIT] Timer 0.05s (20Hz) ✓\n')
    
    def path_callback(self, msg: Path):
        """Primanje putanje od A* planera"""
        if len(msg.poses) == 0:
            self.get_logger().warn('[PATH] Prazna putanja!')
            self.is_following = False
            return
        
        self.current_path = msg
        self.path_index = 0
        self.is_following = True
        self.goal_reached = False
        self.last_log_index = -5
        
        # Izračuna dužinu
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
        self.get_logger().info('[FOLLOW] Počinjem sljedićenje putanje...\n')
    
    def follow_timer(self):
        """Timer za sljedićenje putanje"""
        
        # Ako nema putanje, zaustavi robota
        if not self.is_following or self.current_path is None:
            cmd = Twist()
            self.cmd_pub.publish(cmd)
            return
        
        # Ako je kraj putanje, zaustavi
        if self.path_index >= len(self.current_path.poses):
            if not self.goal_reached:
                self.get_logger().info('[DONE] Putanja završena! Robot zaustavljen.\n')
                self.goal_reached = True
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
            # self.get_logger().error(f'[TF] Lookup failed: {type(e).__name__}')
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
        
        # Log svakih 5 točaka
        if self.path_index >= self.last_log_index + 5:
            self.get_logger().info(
                f'[FOLLOW] Točka {self.path_index}/{len(self.current_path.poses)}: '
                f'robot=({robot_x:.2f},{robot_y:.2f}), '
                f'target=({target_x:.2f},{target_y:.2f}), '
                f'dist={distance:.3f}m'
            )
            self.last_log_index = self.path_index
        
        # Ako je blizu, idi na sljedeću točku
        if distance < self.distance_tolerance:
            self.path_index += 1
            return
        
        # Kreiraj Twist komandu
        cmd = Twist()
        
        # Linearna brzina: proporcionalno udaljenosti
        # Usporavanje blizu cilja
        if distance < 0.5:
            cmd.linear.x = max(0.05, distance * 0.3)
        else:
            cmd.linear.x = self.linear_speed
        
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        
        # Kutna brzina: rotacija prema cilju
        # atan2 (dy, dx) je kut prema cilju
        angle_to_target = math.atan2(dy, dx)
        
        # Kutna brzina proporcionalno kut razlici
        # Ograniči da ne bude prevelika
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = min(self.angular_speed, abs(angle_to_target) * 0.5) * (1 if angle_to_target > 0 else -1)
        
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
