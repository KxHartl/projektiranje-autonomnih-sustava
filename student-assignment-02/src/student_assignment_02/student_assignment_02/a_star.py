"""
A* path planning algorithm
Adapted from: https://github.com/AtsushiSakai/PythonRobotics
"""

import heapq
import math
from dataclasses import dataclass
from typing import List, Tuple, Set


@dataclass
class Node:
    """Node class for A* algorithm"""
    x: int
    y: int
    cost: float = float('inf')  # g_cost
    parent_idx: int = -1

    def __lt__(self, other):
        """Less than operator for heap"""
        return self.cost < other.cost

    def __eq__(self, other):
        """Equality operator"""
        if isinstance(other, Node):
            return self.x == other.x and self.y == other.y
        return False

    def __hash__(self):
        """Hash function"""
        return hash((self.x, self.y))


class AStar:
    """A* Path Planning Algorithm"""

    def __init__(self, resolution: float = 1.0, robot_radius: float = 0.0):
        """
        Initialize A* planner
        
        Args:
            resolution: Grid resolution
            robot_radius: Robot radius for collision checking
        """
        self.resolution = resolution
        self.robot_radius = robot_radius
        self.SX = 1.0  # x-axis resolution
        self.SY = 1.0  # y-axis resolution
        self.MOTION = [
            [1, 0, 1],      # x, y, cost
            [0, 1, 1],
            [-1, 0, 1],
            [0, -1, 1],
            [-1, -1, math.sqrt(2)],
            [-1, 1, math.sqrt(2)],
            [1, -1, math.sqrt(2)],
            [1, 1, math.sqrt(2)],
        ]

    def planning(self,
                 start_x: int,
                 start_y: int,
                 goal_x: int,
                 goal_y: int,
                 occupancy_grid: List[List[int]],
                 obstacle_value: int = 1) -> Tuple[List[Tuple[int, int]], List[Tuple[int, int]]]:
        """
        A* path planning
        
        Args:
            start_x: Start x coordinate (grid index)
            start_y: Start y coordinate (grid index)
            goal_x: Goal x coordinate (grid index)
            goal_y: Goal y coordinate (grid index)
            occupancy_grid: Occupancy grid (0=free, >0=obstacle)
            obstacle_value: Value threshold for obstacles
            
        Returns:
            path: List of (x, y) coordinates from start to goal
            path_xy: Same as path (for compatibility)
        """
        
        # Get grid dimensions
        x_width = len(occupancy_grid)
        y_width = len(occupancy_grid[0]) if x_width > 0 else 0
        
        # Initialize open and closed sets
        open_set = {}  # {(x, y): Node}
        closed_set = {}  # {(x, y): Node}
        
        # Initialize start node
        start_node = Node(x=start_x, y=start_y, cost=0)
        open_set[(start_x, start_y)] = start_node
        
        # Priority queue: (f_cost, counter, (x, y))
        pq = [(self._calc_heuristic(start_x, start_y, goal_x, goal_y), 0, (start_x, start_y))]
        counter = 1
        
        path = []
        iterations = 0
        max_iterations = 100000
        
        while len(pq) > 0 and iterations < max_iterations:
            iterations += 1
            
            # Get node with minimum f_cost
            _, _, (current_x, current_y) = heapq.heappop(pq)
            current_node = open_set[(current_x, current_y)]
            
            # Check if goal is reached
            if current_x == goal_x and current_y == goal_y:
                # Reconstruct path
                path = self._calc_path(closed_set, current_node, start_node)
                break
            
            # Remove from open set, add to closed set
            del open_set[(current_x, current_y)]
            closed_set[(current_x, current_y)] = current_node
            
            # Expand neighbors
            for i, (dx, dy, move_cost) in enumerate(self.MOTION):
                new_x = current_x + dx
                new_y = current_y + dy
                
                # Check bounds
                if new_x < 0 or new_x >= x_width or new_y < 0 or new_y >= y_width:
                    continue
                
                # Check if occupied
                if occupancy_grid[new_x][new_y] >= obstacle_value:
                    continue
                
                # Check if in closed set
                if (new_x, new_y) in closed_set:
                    continue
                
                # Calculate costs
                new_g_cost = current_node.cost + move_cost
                new_h_cost = self._calc_heuristic(new_x, new_y, goal_x, goal_y)
                new_f_cost = new_g_cost + new_h_cost
                
                # Check if new node or better path
                if (new_x, new_y) not in open_set:
                    new_node = Node(x=new_x, y=new_y, cost=new_g_cost, parent_idx=-1)
                    new_node.parent_idx = id(current_node)
                    open_set[(new_x, new_y)] = new_node
                    heapq.heappush(pq, (new_f_cost, counter, (new_x, new_y)))
                    counter += 1
                else:
                    existing_node = open_set[(new_x, new_y)]
                    if new_g_cost < existing_node.cost:
                        existing_node.cost = new_g_cost
                        existing_node.parent_idx = id(current_node)
                        heapq.heappush(pq, (new_f_cost, counter, (new_x, new_y)))
                        counter += 1
        
        # Convert path to list of tuples
        path_xy = [(node.x, node.y) for node in path]
        return path_xy, path_xy

    @staticmethod
    def _calc_heuristic(x: int, y: int, goal_x: int, goal_y: int) -> float:
        """Calculate heuristic cost (Euclidean distance)"""
        dx = x - goal_x
        dy = y - goal_y
        return math.sqrt(dx * dx + dy * dy)

    @staticmethod
    def _calc_path(closed_set: dict, current_node: Node, start_node: Node) -> List[Node]:
        """Reconstruct path from closed set"""
        path = [current_node]
        current = current_node
        
        while current.x != start_node.x or current.y != start_node.y:
            parent_id = current.parent_idx
            found = False
            
            for node in closed_set.values():
                if id(node) == parent_id:
                    path.append(node)
                    current = node
                    found = True
                    break
            
            if not found:
                break
        
        path.reverse()
        return path
