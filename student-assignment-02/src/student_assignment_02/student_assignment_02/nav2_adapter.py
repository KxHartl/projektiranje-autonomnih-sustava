#!/usr/bin/env python3
"""
Nav2 Adapter Node - FIXED VERSION
Povezuče A* putanju s Nav2 kontrolerom
Nav2 automatski sljedi putanju
Također sluša /goal_pose iz RViza (2D Goal Pose tool)

KRITIČNO FIX:
- Uklonjeni callback groups (uzrokovali su blocking)
- Dodan direktan path send bez async wait
- Dodan server.server_is_ready() check s retry logikom
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import Path
from nav2_msgs.action import FollowPath
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
import time
import threading


class Nav2Adapter(Node):
    """Adapter koji povezuje A* putanju s Nav2 kontrolerom"""
    
    def __init__(self):
        super().__init__('nav2_adapter')
        
        self.get_logger().info(
            '\n' +
            '='*80 +
            '\n[ADAPTER] INICIJALIZACIJA' +
            '\n' +
            '- Sluša: /planned_path (od A* planera)' +
            '\n' +
            '- Sluša: /goal_pose (iz RViza - 2D Goal Pose tool)' +
            '\n' +
            '- Akcija: follow_path (Nav2 kontroler DWB)' +
            '\n' +
            '='*80 +
            '\n'
        )
        
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
            '/planned_path',
            self.path_callback,
            qos
        )
        self.get_logger().info('[ADAPTER] ✓ Subscribe na /planned_path')
        
        # Subscribe na goal pose iz RViza
        self.goal_pose_subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_pose_callback,
            10
        )
        self.get_logger().info('[ADAPTER] ✓ Subscribe na /goal_pose')
        
        # Action client za Nav2 controller
        # KRITIČNO: BEZ callback_group!
        self.follow_path_client = ActionClient(
            self,
            FollowPath,
            'follow_path'
        )
        self.get_logger().info('[ADAPTER] ✓ Action client za follow_path')
        
        # Skladištenje
        self.current_path: Path = None
        self.last_goal_pose: PoseStamped = None
        self.goal_handle = None
        self.path_received = False
        self.follow_path_ready = False
        self.sending_goal = False
        
        # Pokreni provjeru follow_path servera u zasebnoj niti
        self.check_thread = threading.Thread(target=self._check_follow_path_thread, daemon=True)
        self.check_thread.start()
        
        self.get_logger().info('[ADAPTER] ✓ Inicijalizacija gotova')
        self.get_logger().info('')
    
    def _check_follow_path_thread(self):
        """
        Provjeri follow_path akciju u zasebnoj niti
        """
        timeout = time.time() + 30  # 30 sekundi
        
        while time.time() < timeout:
            if self.follow_path_client.server_is_ready():
                self.get_logger().info('[ADAPTER] ✓✓✓ /follow_path akcija DOSTUPNA!')
                self.follow_path_ready = True
                return
            
            self.get_logger().debug('[ADAPTER] Čekam /follow_path akciju...')
            time.sleep(1.0)
        
        self.get_logger().error('[ADAPTER] ✗ /follow_path akcija NIJE dostupna nakon 30s')
        self.follow_path_ready = False
    
    def goal_pose_callback(self, msg: PoseStamped):
        """Prima goal pose iz RViza"""
        self.last_goal_pose = msg
        self.get_logger().info(
            f'[GOAL] Nova goal pose iz RViza: '
            f'({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})'
        )
    
    def path_callback(self, msg: Path):
        """Prima putanju od A* planera"""
        if len(msg.poses) > 0:
            self.current_path = msg
            self.path_received = True
            length = self.calculate_path_length(msg)
            
            self.get_logger().info(
                f'[PATH] ✓ Primljena putanja: {len(msg.poses)} točaka, '
                f'dužina: {length:.2f}m'
            )
            
            # Odmah pošalji Nav2-u
            self.send_path_to_nav2()
        else:
            self.get_logger().warn('[PATH] ✗ Putanja je PRAZNA!')
    
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
        """
        Pošalji putanju Nav2 FollowPath akciji
        KRITIČNO: Ovo je DIREKTNO slanje, ne async!
        """
        if self.sending_goal:
            self.get_logger().debug('[SEND] Već se šalje goal, preskakam')
            return
        
        if not self.current_path or len(self.current_path.poses) == 0:
            self.get_logger().debug('[SEND] Nema putanje za slanje')
            return
        
        # Provjeri je li server dostupan
        if not self.follow_path_client.server_is_ready():
            self.get_logger().warn(
                '[SEND] ✗ Nav2 /follow_path server NIJE dostupan!'
            )
            self.follow_path_ready = False
            return
        
        self.sending_goal = True
        
        # Kreiraj goal
        goal = FollowPath.Goal()
        goal.path = self.current_path
        
        self.get_logger().info(
            f'[SEND] ✓ Pošinjam slanje putanje ({len(goal.path.poses)} točaka)'
        )
        
        try:
            # DIREKTNO slanje - bez threading problema!
            future = self.follow_path_client.send_goal_async(goal)
            future.add_done_callback(self.goal_response_callback)
            self.get_logger().info('[SEND] ✓ Goal poslana')
        except Exception as e:
            self.get_logger().error(f'[SEND] ✗ Greška pri slanju: {e}')
            self.sending_goal = False
    
    def goal_response_callback(self, future):
        """
        Callback kada Nav2 odgovori na zahtjev za putanju
        """
        try:
            self.goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f'[NAV2] ✗ future.result() greška: {e}')
            self.sending_goal = False
            return
        
        self.sending_goal = False
        
        if not self.goal_handle:
            self.get_logger().error('[NAV2] ✗ Goal handle je None!')
            return
        
        if not self.goal_handle.accepted:
            self.get_logger().error(
                '[NAV2] ✗ Nav2 je ODBIO putanju (goal rejected)'
            )
            return
        
        self.get_logger().info(
            '[NAV2] ✓✓✓ PRIHVAĆENO - robot počinje slijediti putanju!'
        )
        
        # Dodaj callback za rezultat
        try:
            result_future = self.goal_handle.get_result_async()
            result_future.add_done_callback(self.goal_result_callback)
        except Exception as e:
            self.get_logger().error(f'[NAV2] ✗ get_result_async greška: {e}')
    
    def goal_result_callback(self, future):
        """
        Callback kada je Nav2 završio slijeđenje
        """
        try:
            result = future.result()
            if result and result.result:
                self.get_logger().info(
                    '[DONE] ✓ SLIJEĐENJE DOVRŠENO - robot je stigao na cilj!'
                )
            else:
                self.get_logger().warn(
                    '[DONE] ✗ Slijeđenje je prekinuto ili nije dostupno'
                )
            
            self.goal_handle = None
        except Exception as e:
            self.get_logger().warn(f'[DONE] Greška u rezultatu: {e}')
            self.goal_handle = None


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
