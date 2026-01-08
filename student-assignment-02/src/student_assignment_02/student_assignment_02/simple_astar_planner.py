#!/usr/bin/env python3
"""
Simple A* Path Planner - OD NULE

Radio:
1. Prima /goal_pose iz RViza
2. Prima /map
3. Svaki put kada se robot pomakne - planira novu putanju
4. Objavljuje /planned_path u MAP frameu
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformListener, Buffer
from heapq import heappush, heappop
import math
from typing import List, Tuple, Optional


class SimpleAStarPlanner(Node):
    
    def __init__(self):
        super().__init__('simple_astar_planner')
        
        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscribers
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.on_map, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.on_goal, 10)
        
        # Publishers
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        
        # Stanje
        self.map_data = None
        self.map_info = None
        self.goal_x = None
        self.goal_y = None
        self.last_robot_pos = None
        
        # Timer za planiranje
        self.plan_timer = self.create_timer(0.2, self.plan_callback)  # 5Hz
        
        self.get_logger().info('Simple A* Planner started')
    
    def on_map(self, msg: OccupancyGrid):
        """Primanje mape"""
        self.map_data = msg.data
        self.map_info = msg.info
        self.get_logger().info(
            f'Map received: {msg.info.width}x{msg.info.height}, '
            f'origin=({msg.info.origin.position.x:.1f}, {msg.info.origin.position.y:.1f})'
        )
    
    def on_goal(self, msg: PoseStamped):
        """Primanje goal-a"""
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.get_logger().info(f'Goal: ({self.goal_x:.1f}, {self.goal_y:.1f})')
    
    def get_robot_pos(self) -> Optional[Tuple[float, float]]:
        """Dohvati robot poziciju"""
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            return (tf.transform.translation.x, tf.transform.translation.y)
        except:
            return None
    
    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """World -> Grid"""
        if not self.map_info:
            return (0, 0)
        gx = int((x - self.map_info.origin.position.x) / self.map_info.resolution)
        gy = int((y - self.map_info.origin.position.y) / self.map_info.resolution)
        gx = max(0, min(gx, self.map_info.width - 1))
        gy = max(0, min(gy, self.map_info.height - 1))
        return (gx, gy)
    
    def grid_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        """Grid -> World"""
        if not self.map_info:
            return (0.0, 0.0)
        x = self.map_info.origin.position.x + (gx + 0.5) * self.map_info.resolution
        y = self.map_info.origin.position.y + (gy + 0.5) * self.map_info.resolution
        return (x, y)
    
    def is_free(self, gx: int, gy: int) -> bool:
        """Je li ćelija slobodna?"""
        if not self.map_data or not self.map_info:
            return False
        if gx < 0 or gx >= self.map_info.width or gy < 0 or gy >= self.map_info.height:
            return False
        idx = gy * self.map_info.width + gx
        return self.map_data[idx] < 50
    
    def heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """Heuristika - Euklidska udaljenost"""
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
    
    def astar(self, start: Tuple[int, int], goal: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
        """A* algoritam"""
        if not self.is_free(start[0], start[1]) or not self.is_free(goal[0], goal[1]):
            return None
        
        open_set = []
        heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        
        while open_set:
            _, current = heappop(open_set)
            
            if current == goal:
                # Rekonstruiraj putanju
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                path.reverse()
                return path
            
            # Provjeri sve susjede (4-connected)
            for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (-1, -1), (1, -1), (-1, 1)]:
                neighbor = (current[0] + dx, current[1] + dy)
                
                if not self.is_free(neighbor[0], neighbor[1]):
                    continue
                
                tentative_g = g_score[current] + math.sqrt(dx*dx + dy*dy)
                
                if neighbor in g_score and tentative_g >= g_score[neighbor]:
                    continue
                
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score = tentative_g + self.heuristic(neighbor, goal)
                heappush(open_set, (f_score, neighbor))
        
        return None
    
    def plan_callback(self):
        """Timer callback za planiranje"""
        if not self.map_data or self.goal_x is None:
            return
        
        # Dohvati robot poziciju
        robot_pos = self.get_robot_pos()
        if robot_pos is None:
            return
        
        # Ako se robot premalo pomakao, ne replaniraj
        if self.last_robot_pos and math.sqrt((robot_pos[0] - self.last_robot_pos[0])**2 + 
                                              (robot_pos[1] - self.last_robot_pos[1])**2) < 0.05:
            return
        
        self.last_robot_pos = robot_pos
        
        # Planiraj
        start_grid = self.world_to_grid(robot_pos[0], robot_pos[1])
        goal_grid = self.world_to_grid(self.goal_x, self.goal_y)
        
        path_grid = self.astar(start_grid, goal_grid)
        if not path_grid:
            self.get_logger().warn('No path found')
            return
        
        # Konvertiraj u world koordinate
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for gx, gy in path_grid:
            wx, wy = self.grid_to_world(gx, gy)
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        
        self.path_pub.publish(path_msg)
        self.get_logger().info(f'Path published: {len(path_msg.poses)} poses')


def main(args=None):
    rclpy.init(args=args)
    node = SimpleAStarPlanner()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
