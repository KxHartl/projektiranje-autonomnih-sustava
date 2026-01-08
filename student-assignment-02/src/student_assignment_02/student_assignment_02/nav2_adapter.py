#!/usr/bin/env python3
"""
Nav2 Adapter Node
Poezuče A* putanju s Nav2 kontrolerom
Nav2 automatski sljedi putanju
Također sluša /goal_pose iz RViza (2D Goal Pose tool)

TOPIKI:
- Subscribe: /planned_path (od A* planera)
- Subscribe: /goal_pose (iz RViza)
- Publish: /follow_path (Nav2 FollowPath akcija)

DEBUG: Dodani detaljniji ispis za pronalaženje greške!
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
            '- Redoslijed:' +
            '\n  1. RViz: 2D Goal Pose tool' +
            '\n  2. A* planer: racuna putanju od base_link do goal (SVAKI PUT!)' +
            '\n  3. A*: publikuje /planned_path' +
            '\n  4. Adapter: hvata i šalje Nav2-u' +
            '\n  5. Nav2 DWB: robot se kreće!' +
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
        
        # Subscribe na A* putanju - ISPRAVLJENA TOPIKA!
        self.path_subscription = self.create_subscription(
            Path,
            '/planned_path',  # OVO JE ISPRAVNA TOPIKA OD A*!
            self.path_callback,
            qos
        )
        self.get_logger().info('[ADAPTER] ✓ Subscribe na /planned_path')
        
        # Subscribe na goal pose iz RViza (2D Goal Pose tool)
        self.goal_pose_subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_pose_callback,
            10
        )
        self.get_logger().info('[ADAPTER] ✓ Subscribe na /goal_pose')
        
        # Action client za Nav2 controller
        self.follow_path_client = ActionClient(
            self,
            FollowPath,
            'follow_path'
        )
        self.get_logger().info('[ADAPTER] ✓ Action client za follow_path')
        
        # Timer za periodičko slanjegledanja
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.get_logger().info('[ADAPTER] ✓ Timer kreiran')
        
        # Skladištenje
        self.current_path: Path = None
        self.last_goal_pose: PoseStamped = None
        self.last_sent_time = 0.0
        self.goal_handle = None
        self.send_interval = 1.0  # Pošalji putanju svakih 1 sekunde
        self.path_received = False
        self.follow_path_ready = False
        
        # Čekaj FollowPath akciju
        self._wait_for_follow_path()
        
        self.get_logger().info('[ADAPTER] ✓ Spreman!')
        self.get_logger().info('')
    
    def _wait_for_follow_path(self, timeout: int = 10):
        """
        Čekaj da FollowPath akcija postane dostupna
        """
        self.get_logger().info('[ADAPTER] Čekam /follow_path akciju od Nav2...')
        start = time.time()
        
        while not self.follow_path_client.server_is_ready():
            if time.time() - start > timeout:
                self.get_logger().error('[ADAPTER] ✗ /follow_path akcija NIJE dostupna!')
                self.get_logger().error('[ADAPTER] Provjeri je li nav2_controller pokrenut!')
                self.follow_path_ready = False
                return
            
            self.get_logger().info('[ADAPTER]   ... čekam ...')
            time.sleep(0.5)
        
        self.get_logger().info('[ADAPTER] ✓ /follow_path akcija dostupna!')
        self.follow_path_ready = True
    
    def goal_pose_callback(self, msg: PoseStamped):
        """Prima goal pose iz RViza (2D Goal Pose tool)"""
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
        """Pošalji putanju Nav2 FollowPath akciji"""
        if not self.current_path or len(self.current_path.poses) == 0:
            self.get_logger().debug('[SEND] Nema putanje za slanje')
            return
        
        # Provjeri je li dovoljno vremena prošlo od zadnjeg slanja
        current_time = time.time()
        if current_time - self.last_sent_time < self.send_interval:
            self.get_logger().debug('[SEND] Preskačem - premalo vremena od zadnjeg slanja')
            return
        
        # Provjeri je li server dostupan
        if not self.follow_path_client.server_is_ready():
            self.get_logger().warn(
                '[SEND] ✗ Nav2 /follow_path server NIJE dostupan!'
            )
            self.follow_path_ready = False
            return
        
        # Kreiraj goal
        goal = FollowPath.Goal()
        goal.path = self.current_path
        
        self.get_logger().info(
            f'[SEND] ✓ Slanje putanje Nav2 ({len(goal.path.poses)} točaka)'
        )
        self.get_logger().info(
            f'[SEND]   Akcija: follow_path'
        )
        self.get_logger().info(
            f'[SEND]   Kontroler: DWB'
        )
        
        # Pošalji goal - VAŽNO: Ovo je asinkreno slanje!
        try:
            future = self.follow_path_client.send_goal_async(
                goal,
                feedback_callback=self.feedback_callback
            )
            future.add_done_callback(self.goal_response_callback)
            self.last_sent_time = current_time
            
            self.get_logger().info('[SEND] ✓ Goal poslana, čekam potvrdu...')
        except Exception as e:
            self.get_logger().error(f'[SEND] ✗ Greška pri slanju: {e}')
    
    def goal_response_callback(self, future):
        """Callback kada Nav2 prihvati goal"""
        try:
            self.goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f'[NAV2] ✗ Greška u future.result(): {e}')
            return
        
        if not self.goal_handle:
            self.get_logger().error('[NAV2] ✗ Goal handle je None!')
            return
        
        if not self.goal_handle.accepted:
            self.get_logger().error(
                '[NAV2] ✗ Nav2 je ODBIO putanju (goal rejected)'
            )
            return
        
        self.get_logger().info('[NAV2] ✓ PRIHVAĆENO - robot počinje slijediti putanju!')
        
        # Dodaj callback za rezultat
        try:
            result_future = self.goal_handle.get_result_async()
            result_future.add_done_callback(self.goal_result_callback)
        except Exception as e:
            self.get_logger().error(f'[NAV2] ✗ Greška pri get_result_async: {e}')
    
    def feedback_callback(self, feedback_msg):
        """
        Callback za feedback tijekom izvršavanja
        """
        try:
            feedback = feedback_msg.feedback
            if hasattr(feedback, 'distance_remaining'):
                dist = feedback.distance_remaining
                if dist > 0:
                    self.get_logger().debug(
                        f'[FEEDBACK] Preostala distanca: {dist:.2f}m'
                    )
        except:
            pass  # Ignoriraj greške u feedback-u
    
    def goal_result_callback(self, future):
        """Callback kada je Nav2 završio slijeđenje"""
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
    
    def timer_callback(self):
        """Timer callback - periodički provjera i pošalji putanju"""
        # Ako nema aktivnog goal-a i imamo putanju, pošalji je
        if self.goal_handle is None and self.current_path and self.path_received:
            self.send_path_to_nav2()
        # Ako je goal završen, provjeri je li novija putanja dostupna
        elif self.goal_handle:
            try:
                # Status: 0=UNKNOWN, 1=ACCEPTED, 2=EXECUTING, 3=CANCELING, 4=SUCCEEDED, 5=CANCELED, 6=ABORTED
                if self.goal_handle.status in [4, 5, 6]:  # SUCCEEDED, CANCELED, ABORTED
                    if self.current_path and self.path_received:
                        self.goal_handle = None
                        self.send_path_to_nav2()
            except:
                pass  # Ignoriraj greške u status provjeri


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
