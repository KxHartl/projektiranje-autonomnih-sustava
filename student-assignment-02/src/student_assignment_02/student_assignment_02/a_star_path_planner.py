#!/usr/bin/env python3
"""
A* Path Planner Node
Koristi A* algoritam za planiranje putanje na 2D mapi
Vizualizira pretraživanje prostora u RViz-u
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
import numpy as np
from heapq import heappush, heappop
from typing import List, Tuple, Optional
import math


class AStarPathPlanner(Node):
    """ROS2 čvor za A* planiranje putanje"""
    
    def __init__(self):
        super().__init__('a_star_path_planner')
        
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
        
        # Publisher za čelnu frontu pretraživanja (opened čvorovi)
        self.frontier_publisher = self.create_publisher(
            MarkerArray,
            '/planning_frontier',
            qos
        )
        
        self.map_data = None
        self.map_metadata = None
        self.marker_id_counter = 0
        
        # Parametri planiranja
        self.declare_parameter('inflation_radius', 1)  # u stanicama
        self.declare_parameter('allow_diagonal', True)
        self.declare_parameter('goal_x', 5.0)
        self.declare_parameter('goal_y', 5.0)
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        
        self.inflation_radius = self.get_parameter('inflation_radius').value
        self.allow_diagonal = self.get_parameter('allow_diagonal').value
        
        self.get_logger().info('A* Path Planner Node: Started')
    
    def map_callback(self, msg: OccupancyGrid):
        """Primanje mape i planiranje putanje"""
        self.map_data = msg.data
        self.map_metadata = msg.info
        
        self.get_logger().info(
            f'Mapa primljena: {msg.info.width}x{msg.info.height}, '
            f'rezolucija: {msg.info.resolution:.3f} m/stanica'
        )
        
        # Uzmi parametre
        goal_x = self.get_parameter('goal_x').value
        goal_y = self.get_parameter('goal_y').value
        start_x = self.get_parameter('start_x').value
        start_y = self.get_parameter('start_y').value
        
        # Pretvori world koordinate u grid koordinate
        start_grid = self.world_to_grid(start_x, start_y)
        goal_grid = self.world_to_grid(goal_x, goal_y)
        
        self.get_logger().info(
            f'Planiranje putanje od {start_grid} do {goal_grid}'
        )
        
        # Planiraj putanju
        path, explored_cells, frontier = self.plan_path_astar(start_grid, goal_grid)
        
        if path:
            self.publish_path(path)
            self.get_logger().info(f'Putanja pronađena! Dužina: {len(path)} stanica')
        else:
            self.get_logger().warn('Putanja nije pronađena!')
        
        # Vizualiziraj pretraživanje
        self.visualize_search(explored_cells, frontier)
    
    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """Pretvori world koordinate u grid koordinate"""
        if not self.map_metadata:
            return (0, 0)
        
        origin_x = self.map_metadata.origin.position.x
        origin_y = self.map_metadata.origin.position.y
        resolution = self.map_metadata.resolution
        
        grid_x = int((x - origin_x) / resolution)
        grid_y = int((y - origin_y) / resolution)
        
        # Klamp u granice
        grid_x = max(0, min(grid_x, self.map_metadata.width - 1))
        grid_y = max(0, min(grid_y, self.map_metadata.height - 1))
        
        return (grid_x, grid_y)
    
    def grid_to_world(self, grid_x: int, grid_y: int) -> Tuple[float, float]:
        """Pretvori grid koordinate u world koordinate"""
        if not self.map_metadata:
            return (0.0, 0.0)
        
        origin_x = self.map_metadata.origin.position.x
        origin_y = self.map_metadata.origin.position.y
        resolution = self.map_metadata.resolution
        
        world_x = origin_x + (grid_x + 0.5) * resolution
        world_y = origin_y + (grid_y + 0.5) * resolution
        
        return (world_x, world_y)
    
    def is_valid_cell(self, x: int, y: int, threshold: int = 50) -> bool:
        """Provjeri je li stanica valjana (slobodna)"""
        if not self.map_metadata or not self.map_data:
            return False
        
        # Provjeri granice
        if x < 0 or x >= self.map_metadata.width or y < 0 or y >= self.map_metadata.height:
            return False
        
        # Pretvori u index
        index = y * self.map_metadata.width + x
        
        if index < 0 or index >= len(self.map_data):
            return False
        
        # Provjeri vrijednost stanice
        cell_value = self.map_data[index]
        
        return cell_value < threshold  # < 50 = slobodno
    
    def get_neighbors(self, cell: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Dobij sve valjane susjede stanice"""
        x, y = cell
        neighbors = []
        
        # 4-povezanost (gore, dolje, lijevo, desno)
        four_neighbors = [
            (x + 1, y), (x - 1, y),  # Desno, lijevo
            (x, y + 1), (x, y - 1)   # Gore, dolje
        ]
        
        # Dodaj dijagonalne ako je omogućeno
        if self.allow_diagonal:
            four_neighbors.extend([
                (x + 1, y + 1), (x + 1, y - 1),
                (x - 1, y + 1), (x - 1, y - 1)
            ])
        
        for nx, ny in four_neighbors:
            if self.is_valid_cell(nx, ny):
                neighbors.append((nx, ny))
        
        return neighbors
    
    def heuristic(self, cell: Tuple[int, int], goal: Tuple[int, int]) -> float:
        """Heuristička funkcija (Euklidska distanca)"""
        dx = cell[0] - goal[0]
        dy = cell[1] - goal[1]
        return math.sqrt(dx * dx + dy * dy)
    
    def get_cost(self, cell1: Tuple[int, int], cell2: Tuple[int, int]) -> float:
        """Trošak kretanja između dvije stanice"""
        dx = cell2[0] - cell1[0]
        dy = cell2[1] - cell1[1]
        # Dijagonala = sqrt(2), ortogonalno = 1
        return math.sqrt(dx * dx + dy * dy)
    
    def plan_path_astar(
        self, 
        start: Tuple[int, int], 
        goal: Tuple[int, int]
    ) -> Tuple[Optional[List[Tuple[int, int]]], List[Tuple[int, int]], List[Tuple[int, int]]]:
        """
        A* algoritam za planiranje putanje
        
        Returns:
            (path, explored_cells, frontier)
            - path: lista koordinata od starta do cilja
            - explored_cells: sve istražene stanice
            - frontier: stanice na čelnoj fronti
        """
        
        # Provjeri validnost starta i cilja
        if not self.is_valid_cell(start[0], start[1]):
            self.get_logger().error('Start nije valjana stanica!')
            return None, [], []
        
        if not self.is_valid_cell(goal[0], goal[1]):
            self.get_logger().error('Cilj nije valjana stanica!')
            return None, [], []
        
        # Open set - koristi heap za efikasnost
        open_set = []
        heappush(open_set, (0, start))
        
        # Pratimo gdje smo došli
        came_from = {}
        
        # g_score = cijenu puta do stanice
        g_score = {start: 0}
        
        # f_score = g + h
        f_score = {start: self.heuristic(start, goal)}
        
        # Za vizualizaciju
        explored = []
        frontier_cells = []
        
        iteration = 0
        max_iterations = 10000
        
        while open_set and iteration < max_iterations:
            iteration += 1
            
            current_f, current = heappop(open_set)
            
            # Čuva za vizualizaciju
            frontier_cells.append(current)
            
            if current == goal:
                # Rekonstruiraj putanju
                path = []
                node = goal
                while node in came_from:
                    path.append(node)
                    node = came_from[node]
                path.append(start)
                path.reverse()
                
                self.get_logger().info(f'A* završio u {iteration} iteracija, istraživao {len(explored)} stanica')
                return path, explored, frontier_cells
            
            explored.append(current)
            
            # Provjeri sve susjede
            for neighbor in self.get_neighbors(current):
                tentative_g_score = g_score[current] + self.get_cost(current, neighbor)
                
                # Ako smo već našli bolji put do susjedne stanice
                if neighbor in g_score and tentative_g_score >= g_score[neighbor]:
                    continue
                
                # Ovo je najbolji put do susjedne stanice
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                
                # Dodaj u open set
                heappush(open_set, (f_score[neighbor], neighbor))
        
        self.get_logger().warn(f'Nema putanje! Iteracija: {iteration}')
        return None, explored, frontier_cells
    
    def publish_path(self, path: List[Tuple[int, int]]):
        """Objavi pronađenu putanju"""
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
    
    def visualize_search(
        self, 
        explored: List[Tuple[int, int]], 
        frontier: List[Tuple[int, int]]
    ):
        """Vizualiziraj pretraživanje u RViz-u"""
        
        # Vizualizacija istrživanih stanica
        explored_markers = MarkerArray()
        
        for idx, (gx, gy) in enumerate(explored):
            if idx % 5 != 0:  # Prikaži svaku 5. stanicu
                continue
            
            world_x, world_y = self.grid_to_world(gx, gy)
            
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'explored_cells'
            marker.id = idx // 5
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
        
        self.visualization_publisher.publish(explored_markers)
        
        # Vizualizacija čelne fronte
        frontier_markers = MarkerArray()
        
        for idx, (gx, gy) in enumerate(frontier[-1000:]):  # Samo zadnjih 1000
            world_x, world_y = self.grid_to_world(gx, gy)
            
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'frontier'
            marker.id = idx
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose.position.x = world_x
            marker.pose.position.y = world_y
            marker.pose.position.z = 0.0
            
            marker.scale.x = self.map_metadata.resolution * 0.6
            marker.scale.y = self.map_metadata.resolution * 0.6
            marker.scale.z = self.map_metadata.resolution * 0.6
            
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.5
            
            frontier_markers.markers.append(marker)
        
        self.frontier_publisher.publish(frontier_markers)


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
