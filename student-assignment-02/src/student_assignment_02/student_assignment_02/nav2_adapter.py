#!/usr/bin/env python3
"""
Nav2 Adapter Node
Povezuče A* putanju s Nav2 kontrolerom
Nav2 automatski sljedi putanju
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import Path
from nav2_msgs.action import FollowPath
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
import time


class Nav2Adapter(Node):
    """Adapter koji povezuje A* putanju s Nav2 kontrolerom"""
    
    def __init__(self):
        super().__init__('nav2_adapter')
        
        # QoS profil
        qos = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscribe na A* putanju
        self.path_subscription = self.create_subscription(
            Path,
            'astar_path',
            self.path_callback,
            qos
        )
        
        # Action client za Nav2 controller
        self.follow_path_client = ActionClient(
            self,
            FollowPath,
            'follow_path'
        )
        
        # Timer za periodičko slanjegledanja
        self.timer = self.create_timer(0.5, self.timer_callback)
        
        # Skladištenje
        self.current_path: Path = None
        self.last_sent_time = 0.0
        self.goal_handle = None
        self.send_interval = 1.0  # Pošalji putanju svakih 1 sekunde
        
        self.get_logger().info(
            'Nav2 Adapter inicijaliziran\n'
            'Slusa na astar_path i slanja Nav2 FollowPath akciji'
        )
    
    def path_callback(self, msg: Path):
        """Prima putanju od A* planera"""
        if len(msg.poses) > 0:
            self.current_path = msg
            self.get_logger().info(f'Nova A* putanja primljena: {len(msg.poses)} točaka')
            # Odmah pošalji Nav2-u
            self.send_path_to_nav2()
    
    def send_path_to_nav2(self):
        """Pošalji putanju Nav2 FollowPath akciji"""
        if not self.current_path or len(self.current_path.poses) == 0:
            return
        
        # Provjeri ima li dostatan razmak od zadnjeg slanja
        current_time = time.time()
        if current_time - self.last_sent_time < self.send_interval:
            return
        
        # Provjeri je li server dostupan
        if not self.follow_path_client.server_is_ready():
            self.get_logger().warn('Nav2 FollowPath server nije dostupan')
            return
        
        # Kreiraj goal
        goal = FollowPath.Goal()
        goal.path = self.current_path
        
        # Pošalji goal
        self.get_logger().info(f'Slanjem putanje Nav2-u (putanja: {len(goal.path.poses)} točaka)')
        future = self.follow_path_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)
        
        self.last_sent_time = current_time
    
    def goal_response_callback(self, future):
        """Callback kada Nav2 prihvati goal"""
        self.goal_handle = future.result()
        
        if not self.goal_handle.accepted:
            self.get_logger().error('Nav2 FollowPath goal ODBIJEN')
            return
        
        self.get_logger().info('Nav2 FollowPath goal PRIHVAĆEN')
        
        # Dodaj callback za rezultat
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)
    
    def goal_result_callback(self, future):
        """Callback kada je Nav2 završio slijeđenje"""
        result = future.result().result
        
        if result:
            self.get_logger().info('Nav2 je uspješno slijedio putanju do cilja!')
        else:
            self.get_logger().warn('Nav2 je prekinuo slijeđenje')
    
    def timer_callback(self):
        """Timer callback - periodički provjera i pošalji putanju"""
        # Ako nema aktivnog goal-a i imamo putanju, pošalji je
        if self.goal_handle is None and self.current_path:
            self.send_path_to_nav2()
        # Ako je goal završen, provjeri je li novija putanja dostupna
        elif self.goal_handle and self.goal_handle.status != 4:  # 4 = SUCCEEDED
            # Goal je završen ili je prekinut, spreman za novu putanju
            if self.current_path:
                self.goal_handle = None
                self.send_path_to_nav2()


def main(args=None):
    rclpy.init(args=args)
    node = Nav2Adapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
