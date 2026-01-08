#!/usr/bin/env python3
"""
Nav2 Adapter Node
Povezuče A* putanju s Nav2 kontrolerom
Nav2 automatski sljedi putanju
Takođe sluša /goal_pose iz RViza (2D Goal Pose tool)
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
        
        # Subscribe na goal pose iz RViza (2D Goal Pose tool)
        self.goal_pose_subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_pose_callback,
            10
        )
        
        # Action client za Nav2 controller
        self.follow_path_client = ActionClient(
            self,
            FollowPath,
            'follow_path'
        )
        
        # Publisher za goal koja se prosleđuje A*-u
        self.astar_goal_publisher = self.create_publisher(
            PoseStamped,
            '/a_star_goal',
            10
        )
        
        # Timer za periodičko slanjegledanja
        self.timer = self.create_timer(0.5, self.timer_callback)
        
        # Skladištenje
        self.current_path: Path = None
        self.last_goal_pose: PoseStamped = None
        self.last_sent_time = 0.0
        self.goal_handle = None
        self.send_interval = 1.0  # Pošalji putanju svakih 1 sekunde
        self.path_received = False
        
        self.get_logger().info(
            '\n' +
            '='*60 +
            '\nNav2 Adapter inicijaliziran\n' +
            '- Slusa na /astar_path od A* planera\n' +
            '- Slusa na /goal_pose iz RViza (2D Goal Pose tool)\n' +
            '- Koristi Nav2 FollowPath akciju za sljenje\n' +
            '\nKorak 1: Postavite GOAL u RViz-u (2D Goal Pose)\n' +
            'Korak 2: A* planira putanju\n' +
            'Korak 3: Nav2 automatski sljedi putanju\n' +
            '='*60 +
            '\n'
        )
    
    def goal_pose_callback(self, msg: PoseStamped):
        """Prima goal pose iz RViza (2D Goal Pose tool)"""
        self.last_goal_pose = msg
        self.get_logger().info(
            f'\n>>> NOVA GOAL POSE iz RViza: '
            f'({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})'
        )
        
        # Proslijedi goal A* planeru na /a_star_goal
        self.astar_goal_publisher.publish(msg)
    
    def path_callback(self, msg: Path):
        """Prima putanju od A* planera"""
        if len(msg.poses) > 0:
            self.current_path = msg
            self.path_received = True
            self.get_logger().info(
                f'\n>>> A* PUTANJA PRIMLJENA: {len(msg.poses)} točaka, '
                f'dužina: {self.calculate_path_length(msg):.2f}m'
            )
            # Odmah pošalji Nav2-u
            self.send_path_to_nav2()
        else:
            self.get_logger().warn('\n>>> A* PUTANJA JE PRAZNA!')
    
    def calculate_path_length(self, path: Path) -> float:
        """Izračuna duljinu putanje"""
        if len(path.poses) < 2:
            return 0.0
        
        total_length = 0.0
        for i in range(len(path.poses) - 1):
            p1 = path.poses[i].pose.position
            p2 = path.poses[i + 1].pose.position
            dx = p2.x - p1.x
            dy = p2.y - p1.y
            total_length += (dx**2 + dy**2)**0.5
        
        return total_length
    
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
            self.get_logger().debug('Nav2 FollowPath server nije dostupan, čekam...')
            return
        
        # Kreiraj goal
        goal = FollowPath.Goal()
        goal.path = self.current_path
        
        # Pošalji goal
        self.get_logger().info(
            f'\n>>> SLANJE PUTANJE NAV2 ({len(goal.path.poses)} točaka)'
        )
        future = self.follow_path_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)
        
        self.last_sent_time = current_time
    
    def goal_response_callback(self, future):
        """Callback kada Nav2 prihvati goal"""
        self.goal_handle = future.result()
        
        if not self.goal_handle.accepted:
            self.get_logger().error('\n>>> Nav2 FollowPath goal ODBIJEN')
            return
        
        self.get_logger().info('\n>>> Nav2 FollowPath goal PRIHVAĆEN - ROBOT SE KREĆE!')
        
        # Dodaj callback za rezultat
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)
    
    def goal_result_callback(self, future):
        """Callback kada je Nav2 završio slijeđenje"""
        result = future.result().result
        
        if result:
            self.get_logger().info(
                '\n>>> SLJENJE DOVRŠENO - ROBOT JE STIGAO NA CILJ!'
            )
        else:
            self.get_logger().warn(
                '\n>>> SLJENJE PREKINUTO - Postavite novi GOAL u RViz-u'
            )
    
    def timer_callback(self):
        """Timer callback - periodički provjera i pošalji putanju"""
        # Ako nema aktivnog goal-a i imamo putanju, pošalji je
        if self.goal_handle is None and self.current_path and self.path_received:
            self.send_path_to_nav2()
        # Ako je goal završen, provjeri je li novija putanja dostupna
        elif self.goal_handle:
            # Provjerite je li goal završen ili neuspješan
            # Status: 0=UNKNOWN, 1=ACCEPTED, 2=EXECUTING, 3=CANCELING, 4=SUCCEEDED, 5=CANCELED, 6=ABORTED
            if self.goal_handle.status in [4, 5, 6]:  # SUCCEEDED, CANCELED, ABORTED
                if self.current_path and self.path_received:
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
