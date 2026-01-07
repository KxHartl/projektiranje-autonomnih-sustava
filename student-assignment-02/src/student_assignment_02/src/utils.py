"""
Utility functions for path planning and navigation
"""

import math
from typing import Tuple, List
import numpy as np
from nav_msgs.msg import OccupancyGrid


def occupancy_grid_to_numpy(grid_msg: OccupancyGrid, occupancy_threshold: int = 50) -> np.ndarray:
    """
    Convert ROS OccupancyGrid message to numpy 2D array
    
    Args:
        grid_msg: nav_msgs.OccupancyGrid message
        occupancy_threshold: Threshold for occupied cells (default 50)
        
    Returns:
        2D numpy array where 0=free, 1=occupied, -1=unknown
    """
    width = grid_msg.info.width
    height = grid_msg.info.height
    
    # Convert to 2D array and transpose for proper indexing
    grid = np.zeros((width, height), dtype=np.uint8)
    
    for i, cell_value in enumerate(grid_msg.data):
        if cell_value < 0:  # Unknown
            continue
        x = i % width
        y = i // width
        
        # Threshold: values >= threshold are obstacles
        if cell_value >= occupancy_threshold:
            grid[x, y] = 1
        else:
            grid[x, y] = 0
    
    return grid


def world_to_grid(world_x: float, world_y: float, 
                  map_origin_x: float, map_origin_y: float,
                  resolution: float) -> Tuple[int, int]:
    """
    Convert world coordinates to grid indices
    
    Args:
        world_x, world_y: World coordinates in meters
        map_origin_x, map_origin_y: Map origin in meters
        resolution: Map resolution in meters/cell
        
    Returns:
        (grid_x, grid_y): Grid indices
    """
    grid_x = int((world_x - map_origin_x) / resolution)
    grid_y = int((world_y - map_origin_y) / resolution)
    return grid_x, grid_y


def grid_to_world(grid_x: int, grid_y: int,
                  map_origin_x: float, map_origin_y: float,
                  resolution: float) -> Tuple[float, float]:
    """
    Convert grid indices to world coordinates
    
    Args:
        grid_x, grid_y: Grid indices
        map_origin_x, map_origin_y: Map origin in meters
        resolution: Map resolution in meters/cell
        
    Returns:
        (world_x, world_y): World coordinates in meters
    """
    world_x = (grid_x + 0.5) * resolution + map_origin_x
    world_y = (grid_y + 0.5) * resolution + map_origin_y
    return world_x, world_y


def is_grid_valid(grid_x: int, grid_y: int,
                  width: int, height: int) -> bool:
    """
    Check if grid coordinates are within bounds
    
    Args:
        grid_x, grid_y: Grid coordinates
        width, height: Grid dimensions
        
    Returns:
        True if valid, False otherwise
    """
    return 0 <= grid_x < width and 0 <= grid_y < height


def check_collision(grid_x: int, grid_y: int,
                   grid: np.ndarray,
                   robot_radius_cells: int = 0) -> bool:
    """
    Check if position collides with obstacles
    
    Args:
        grid_x, grid_y: Grid coordinates
        grid: 2D occupancy grid (0=free, 1=occupied)
        robot_radius_cells: Robot radius in grid cells
        
    Returns:
        True if collision, False otherwise
    """
    width, height = grid.shape
    
    if not is_grid_valid(grid_x, grid_y, width, height):
        return True
    
    # Check cell itself
    if grid[grid_x, grid_y] > 0:
        return True
    
    # Check radius around cell
    for dx in range(-robot_radius_cells, robot_radius_cells + 1):
        for dy in range(-robot_radius_cells, robot_radius_cells + 1):
            check_x = grid_x + dx
            check_y = grid_y + dy
            
            if is_grid_valid(check_x, check_y, width, height):
                if grid[check_x, check_y] > 0:
                    return True
            else:
                return True
    
    return False


def normalize_angle(angle: float) -> float:
    """
    Normalize angle to [-pi, pi]
    
    Args:
        angle: Angle in radians
        
    Returns:
        Normalized angle in [-pi, pi]
    """
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def quaternion_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    """
    Convert yaw angle to quaternion
    
    Args:
        yaw: Yaw angle in radians
        
    Returns:
        (qx, qy, qz, qw) quaternion
    """
    half_yaw = yaw * 0.5
    qx = 0.0
    qy = 0.0
    qz = math.sin(half_yaw)
    qw = math.cos(half_yaw)
    return qx, qy, qz, qw


def yaw_from_quaternion(qx: float, qy: float, qz: float, qw: float) -> float:
    """
    Convert quaternion to yaw angle
    
    Args:
        qx, qy, qz, qw: Quaternion components
        
    Returns:
        Yaw angle in radians
    """
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (qw * qx + qy * qz)
    cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
    
    # Pitch (y-axis rotation)
    sinp = 2 * (qw * qy - qz * qx)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)
    
    # Yaw (z-axis rotation)
    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    return yaw
