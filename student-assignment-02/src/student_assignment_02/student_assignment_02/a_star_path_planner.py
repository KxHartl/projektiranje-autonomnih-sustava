#!/usr/bin/env python3
"""
A* Path Planner Node - KONTINUIRNA REPLANIRANJE

Korisnik zeli: Kontinuirno se racuna od robot pozicije do cilja

FIX:
1. Dodaj timer koji kontinuirno replaniraj (svake 0.5s)
2. Svaki put koristi TRENUTNU robot poziciju (base_link TF)
3. Koristi zadnji primljeni goal
4. SVE U MAP FRAMEU
5. Nema transformacija
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Point, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from tf2_ros import TransformListener, Buffer
from tf2_geometry_msgs import do_transform_point
import numpy as np
from heapq import heappush, heappop
from typing import List, Tuple, Optional
import math
import traceback


class AStarPathPlanner(Node):
    """ROS2 čvor za A* planiranje putanje - KONTINUIRNA REPLANIRANJE"""
    
    def __init__(self):
        super().__init__('a_star_path_planner')
        
        # TF Buffer i Listener za base_link transformaciju
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # QoS profil
        qos = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscribe na mapu
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            qos
        )
        
        # Subscribe na goal pose iz RViza (2D Goal Pose tool)
        self.goal_subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_pose_callback,
            10
        )
        
        # Publisher za putanju
        self.path_publisher = self.create_publisher(
            Path,
            '/planned_path',
            qos
        )
        
        # Publisher za vizualizaciju pretraživanja
        self.visualization_publisher = self.create_publisher(
            MarkerArray,
            '/path_planning_visualization',
            qos
        )
        
        # Publisher za čelnu frontu pretraživanja
        self.frontier_publisher = self.create_publisher(
            MarkerArray,
            '/planning_frontier',
            qos
        )
        
        # Publisher za inflation buffer vizualizaciju
        self.inflation_buffer_publisher = self.create_publisher(
            MarkerArray,
            '/inflation_buffer_visualization',
            qos
        )
        
        self.map_data = None
        self.map_metadata = None
        self.marker_id_counter = 0
        
        # Parametri planiranja
        self.declare_parameter('inflation_radius', 1)
        self.declare_parameter('allow_diagonal', True)
        self.declare_parameter('max_iterations', 50000)
        self.declare_parameter('search_radius', -1)
        self.declare_parameter('inflation_distance_m', 0.5)
        self.declare_parameter('inflation_cost_threshold', 60)
        
        self.inflation_radius = self.get_parameter('inflation_radius').value
        self.allow_diagonal = self.get_parameter('allow_diagonal').value
        self.max_iterations = self.get_parameter('max_iterations').value
        self.search_radius = self.get_parameter('search_radius').value
        self.inflation_distance_m = self.get_parameter('inflation_distance_m').value
        self.inflation_cost_threshold = self.get_parameter('inflation_cost_threshold').value
        
        # Trenutni goal
        self.current_goal_x = 0.0
        self.current_goal_y = 0.0
        self.goal_set = False
        self.last_robot_x = 0.0
        self.last_robot_y = 0.0
        
        # Mapa s inflacijom
        self.inflated_map = None
        
        self.get_logger().info('='*80)
        self.get_logger().info('A* Path Planner Node: Started - KONTINUIRNA REPLANIRANJE')
        self.get_logger().info(f'Max iterations: {self.max_iterations}')
        self.get_logger().info(f'Inflation distance: {self.inflation_distance_m}m')
        self.get_logger().info('[DEBUG] Sluša na /goal_pose')
        self.get_logger().info('[DEBUG] TIMER: Replaniraj svake 0.5s od robot pozicije')
        self.get_logger().info('[DEBUG] FRAME: Sve putanje su u MAP frameu')
        self.get_logger().info('='*80)
        
        # NOVO: Timer za kontinuirnu replaniranje
        self.planning_timer = self.create_timer(0.5, self.planning_timer_callback)
    
    def get_robot_position(self) -> Tuple[float, float]:
        """Dohvati base_link poziciju iz TF tree-a"""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y
            return (robot_x, robot_y)
            
        except Exception as e:
            self.get_logger().debug(
                f'[TF_FAIL] lookup_transform failed: {type(e).__name__}'
            )
            return (0.0, 0.0)
    
    def goal_pose_callback(self, msg: PoseStamped):
        """Primanje goal pose iz RViza"""
        self.current_goal_x = msg.pose.position.x
        self.current_goal_y = msg.pose.position.y
        self.goal_set = True
        
        self.get_logger().info(
            f'[GOAL] Nova goal pose: ({self.current_goal_x:.2f}, {self.current_goal_y:.2f})'
        )
    
    def map_callback(self, msg: OccupancyGrid):
        """Primanje mape"""
        self.map_data = msg.data
        self.map_metadata = msg.info
        self.inflated_map = self.create_inflated_map()
        
        self.get_logger().info(
            f'Mapa primljena: {msg.info.width}x{msg.info.height}, '
            f'origin: ({msg.info.origin.position.x:.2f}, {msg.info.origin.position.y:.2f})'
        )
    
    def planning_timer_callback(self):
        """Timer za kontinuirnu replaniranje"""
        if not self.goal_set or self.map_data is None:
            return
        
        # Dohvati TRENUTNU robot poziciju
        robot_x, robot_y = self.get_robot_position()
        
        # Ako se robot premalo pomakao, ne replaniraj
        dist_moved = math.sqrt(
            (robot_x - self.last_robot_x)**2 + 
            (robot_y - self.last_robot_y)**2
        )
        if dist_moved < 0.05:  # Trebam se pomakao bar 5cm
            return
        
        self.last_robot_x = robot_x
        self.last_robot_y = robot_y
        
        # Planiraj od TRENUTNE robot pozicije do goal-a
        self.plan_and_publish(robot_x, robot_y, self.current_goal_x, self.current_goal_y)
    
    def _get_inflation_cells(self) -> int:
        if not self.map_metadata:
            return 0
        cells = int(self.inflation_distance_m / self.map_metadata.resolution)
        return max(cells, 1)
    
    def create_inflated_map(self) -> List[int]:
        if not self.map_metadata or not self.map_data:
            return self.map_data
        
        width = self.map_metadata.width
        height = self.map_metadata.height
        resolution = self.map_metadata.resolution
        inflation_cells = self._get_inflation_cells()
        inflated = list(self.map_data)
        
        for y in range(height):
            for x in range(width):
                idx = y * width + x
                if inflated[idx] < self.inflation_cost_threshold:
                    min_dist = self._min_distance_to_obstacle(x, y, inflation_cells)
                    if min_dist < inflation_cells:
                        inflated[idx] = 60 + int((inflation_cells - min_dist) * 10)
        
        return inflated
    
    def _min_distance_to_obstacle(self, x: int, y: int, max_dist: int) -> float:
        if not self.map_metadata or not self.map_data:
            return max_dist
        
        width = self.map_metadata.width
        height = self.map_metadata.height
        
        from collections import deque
        queue = deque([(x, y, 0)])
        visited = set([(x, y)])
        
        while queue:
            cx, cy, dist = queue.popleft()
            idx = cy * width + cx
            
            if self.map_data[idx] >= self.inflation_cost_threshold:
                return dist
            
            if dist >= max_dist:
                return max_dist
            
            for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
                nx, ny = cx + dx, cy + dy
                if 0 <= nx < width and 0 <= ny < height and (nx, ny) not in visited:
                    visited.add((nx, ny))
                    queue.append((nx, ny, dist + 1))
        
        return max_dist
    
    def plan_and_publish(self, robot_x: float, robot_y: float, goal_x: float, goal_y: float):
        """Planiraj putanju i objavi je"""
        # Konvertuj world koordinate u grid
        start_grid = self.world_to_grid(robot_x, robot_y)
        goal_grid = self.world_to_grid(goal_x, goal_y)
        
        # Planis putanju
        path, explored_cells, frontier = self.plan_path_astar(start_grid, goal_grid)
        
        if path:
            self.publish_path(path)
        else:
            self.get_logger().warn('[PLAN] Putanja nije pronađena!')
        
        self.visualize_search(explored_cells, frontier)
    
    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """Konvertuj world koordinate (u MAP frameu) u grid koordinate"""
        if not self.map_metadata:
            return (0, 0)
        
        origin_x = self.map_metadata.origin.position.x
        origin_y = self.map_metadata.origin.position.y
        resolution = self.map_metadata.resolution
        
        grid_x = int((x - origin_x) / resolution)
        grid_y = int((y - origin_y) / resolution)
        
        grid_x = max(0, min(grid_x, self.map_metadata.width - 1))
        grid_y = max(0, min(grid_y, self.map_metadata.height - 1))
        
        return (grid_x, grid_y)
    
    def grid_to_world(self, grid_x: int, grid_y: int) -> Tuple[float, float]:
        """Konvertuj grid koordinate u world koordinate (u MAP frameu)"""
        if not self.map_metadata:
            return (0.0, 0.0)
        
        origin_x = self.map_metadata.origin.position.x
        origin_y = self.map_metadata.origin.position.y
        resolution = self.map_metadata.resolution
        
        world_x = origin_x + (grid_x + 0.5) * resolution
        world_y = origin_y + (grid_y + 0.5) * resolution
        
        return (world_x, world_y)
    
    def is_valid_cell(self, x: int, y: int, use_inflation: bool = True) -> bool:
        if not self.map_metadata:
            return False
        
        if x < 0 or x >= self.map_metadata.width or y < 0 or y >= self.map_metadata.height:
            return False
        
        index = y * self.map_metadata.width + x
        
        if index < 0 or index >= len(self.map_data):
            return False
        
        if use_inflation and self.inflated_map:
            cell_value = self.inflated_map[index]
        else:
            cell_value = self.map_data[index]
        
        return cell_value < 50
    
    def get_neighbors(self, cell: Tuple[int, int]) -> List[Tuple[int, int]]:
        x, y = cell
        neighbors = []
        
        four_neighbors = [
            (x + 1, y), (x - 1, y),
            (x, y + 1), (x, y - 1)
        ]
        
        if self.allow_diagonal:
            four_neighbors.extend([
                (x + 1, y + 1), (x + 1, y - 1),
                (x - 1, y + 1), (x - 1, y - 1)
            ])
        
        for nx, ny in four_neighbors:
            if self.is_valid_cell(nx, ny, use_inflation=True):
                neighbors.append((nx, ny))
        
        return neighbors
    
    def heuristic(self, cell: Tuple[int, int], goal: Tuple[int, int]) -> float:
        dx = cell[0] - goal[0]
        dy = cell[1] - goal[1]
        return math.sqrt(dx * dx + dy * dy)
    
    def get_cost(self, cell1: Tuple[int, int], cell2: Tuple[int, int]) -> float:
        dx = cell2[0] - cell1[0]
        dy = cell2[1] - cell1[1]
        return math.sqrt(dx * dx + dy * dy)
    
    def plan_path_astar(
        self, 
        start: Tuple[int, int], 
        goal: Tuple[int, int]
    ) -> Tuple[Optional[List[Tuple[int, int]]], List[Tuple[int, int]], List[Tuple[int, int]]]:
        
        if not self.is_valid_cell(start[0], start[1], use_inflation=True):
            return None, [], []
        
        if not self.is_valid_cell(goal[0], goal[1], use_inflation=True):
            return None, [], []
        
        open_set = []
        heappush(open_set, (0, start))
        
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}
        explored = []
        frontier_cells = []
        
        iteration = 0
        max_iter = self.max_iterations
        
        while open_set and iteration < max_iter:
            iteration += 1
            current_f, current = heappop(open_set)
            frontier_cells.append(current)
            
            if current == goal:
                path = []
                node = goal
                while node in came_from:
                    path.append(node)
                    node = came_from[node]
                path.append(start)
                path.reverse()
                return path, explored, frontier_cells
            
            explored.append(current)
            
            for neighbor in self.get_neighbors(current):
                tentative_g_score = g_score[current] + self.get_cost(current, neighbor)
                
                if neighbor in g_score and tentative_g_score >= g_score[neighbor]:
                    continue
                
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                heappush(open_set, (f_score[neighbor], neighbor))
        
        return None, explored, frontier_cells
    
    def publish_path(self, path: List[Tuple[int, int]]):
        """Objavi putanju u MAP frameu"""
        ros_path = Path()
        ros_path.header.frame_id = 'map'
        ros_path.header.stamp = self.get_clock().now().to_msg()
        
        for grid_pos in path:
            world_x, world_y = self.grid_to_world(grid_pos[0], grid_pos[1])
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = world_x
            pose.pose.position.y = world_y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            ros_path.poses.append(pose)
        
        self.path_publisher.publish(ros_path)
    
    def visualize_search(self, explored: List[Tuple[int, int]], frontier: List[Tuple[int, int]]):
        explored_markers = MarkerArray()
        
        for idx, (gx, gy) in enumerate(explored):
            if idx % 10 != 0:  # Svaki 10. marker
                continue
            
            world_x, world_y = self.grid_to_world(gx, gy)
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'explored_cells'
            marker.id = idx // 10
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = world_x
            marker.pose.position.y = world_y
            marker.pose.position.z = 0.0
            marker.scale.x = self.map_metadata.resolution * 0.8
            marker.scale.y = self.map_metadata.resolution * 0.8
            marker.scale.z = self.map_metadata.resolution * 0.8
            marker.color.r = 0.5
            marker.color.g = 0.5
            marker.color.b = 0.5
            marker.color.a = 0.3
            explored_markers.markers.append(marker)
            
            if len(explored_markers.markers) > 100:  # Limit
                break
        
        self.visualization_publisher.publish(explored_markers)


def main(args=None):
    rclpy.init(args=args)
    node = AStarPathPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
