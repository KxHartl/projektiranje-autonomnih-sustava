#!/usr/bin/env python3
"""
Path Planning Node - A* Algorithm
ROS 2 node for path planning using A* algorithm
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import math
import numpy as np

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from visualization_msgs.msg import MarkerArray, Marker
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point

from student_assignment_02.a_star import AStar
from student_assignment_02.utils import (
    occupancy_grid_to_numpy,
    world_to_grid,
    grid_to_world,
    yaw_from_quaternion,
    quaternion_from_yaw,
    normalize_angle
)


class PathPlanningNode(Node):
    """ROS 2 Path Planning Node using A* Algorithm"""

    def __init__(self):
        super().__init__('path_planning_node')
        
        # TF Buffer and Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # QoS profile for map subscription
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscriptions
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            sensor_qos
        )
        
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            rclpy.qos.qos_profile_system_default
        )
        
        self.initial_pose_sub = self.create_subscription(
            PoseStamped,
            '/initialpose',
            self.initial_pose_callback,
            rclpy.qos.qos_profile_system_default
        )
        
        # Publishers
        self.path_pub = self.create_publisher(
            Path,
            '/planned_path',
            10
        )
        
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/visualization_marker_array',
            10
        )
        
        # State
        self.current_map = None
        self.map_received = False
        self.plan_count = 0
        
        # Robot pose
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.robot_pose_initialized = False
        
        # A* planner
        self.planner = AStar()
        
        # Logging
        self.get_logger().info('Path Planning Node initialized')
        self.get_logger().info('Waiting for 2D Goal Pose from RViz...')
        self.get_logger().info('  1. Set 2D Goal Pose tool in RViz')
        self.get_logger().info('  2. Click and drag in map')

    def map_callback(self, msg: OccupancyGrid):
        """Callback for map subscription"""
        self.current_map = msg
        self.map_received = True
        
        if self.plan_count == 0:
            self.get_logger().info(
                f'Map received: {msg.info.width} x {msg.info.height}, '
                f'resolution: {msg.info.resolution:.2f} m/cell'
            )
            self.get_logger().info(
                f'Origin: ({msg.info.origin.position.x:.2f}, {msg.info.origin.position.y:.2f})'
            )

    def initial_pose_callback(self, msg: PoseStamped):
        """Callback for initial pose (2D Pose Estimate)"""
        self.robot_x = msg.pose.position.x
        self.robot_y = msg.pose.position.y
        self.robot_pose_initialized = True
        
        # Extract yaw from quaternion
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        self.robot_theta = yaw_from_quaternion(qx, qy, qz, qw)
        
        self.get_logger().info(
            f'Initial pose set: ({self.robot_x:.2f}, {self.robot_y:.2f}, {self.robot_theta:.2f} rad)'
        )

    def goal_callback(self, msg: PoseStamped):
        """Callback for goal pose (2D Goal Pose)"""
        if not self.map_received:
            self.get_logger().warn('Map not received yet. Waiting for map...')
            return
        
        # Get robot pose from TF
        self.update_robot_pose_from_tf()
        
        # If TF not available, use initialized pose
        if not self.robot_pose_initialized:
            self.get_logger().warn(
                'Robot pose not available. Using default (0, 0)'
            )
            self.robot_x = 0.0
            self.robot_y = 0.0
            self.robot_theta = 0.0
        
        # Convert world coordinates to grid indices
        start_grid_x, start_grid_y = world_to_grid(
            self.robot_x,
            self.robot_y,
            self.current_map.info.origin.position.x,
            self.current_map.info.origin.position.y,
            self.current_map.info.resolution
        )
        
        goal_grid_x, goal_grid_y = world_to_grid(
            msg.pose.position.x,
            msg.pose.position.y,
            self.current_map.info.origin.position.x,
            self.current_map.info.origin.position.y,
            self.current_map.info.resolution
        )
        
        self.plan_count += 1
        self.get_logger().info(f'\n=== [Plan {self.plan_count}] ===')
        self.get_logger().info(
            f'Robot position: ({self.robot_x:.2f} m, {self.robot_y:.2f} m)'
        )
        self.get_logger().info(
            f'Goal position: ({msg.pose.position.x:.2f} m, {msg.pose.position.y:.2f} m)'
        )
        self.get_logger().info(
            f'Grid: start ({start_grid_x}, {start_grid_y}) -> '
            f'goal ({goal_grid_x}, {goal_grid_y})'
        )
        
        # Execute planning
        self.plan_path(
            start_grid_x, start_grid_y,
            goal_grid_x, goal_grid_y
        )

    def plan_path(self, start_x: int, start_y: int, goal_x: int, goal_y: int):
        """Execute A* path planning"""
        # Convert map to occupancy grid
        try:
            grid = occupancy_grid_to_numpy(self.current_map, occupancy_threshold=50)
        except Exception as e:
            self.get_logger().error(f'Failed to convert map: {e}')
            return
        
        # Check start and goal validity
        width, height = grid.shape
        if not (0 <= start_x < width and 0 <= start_y < height):
            self.get_logger().warn('Start position out of bounds')
            return
        
        if not (0 <= goal_x < width and 0 <= goal_y < height):
            self.get_logger().warn('Goal position out of bounds')
            return
        
        if grid[start_x, start_y] > 0:
            self.get_logger().warn('Start position occupied')
            return
        
        if grid[goal_x, goal_y] > 0:
            self.get_logger().warn('Goal position occupied')
            return
        
        # Run A* algorithm
        try:
            path_grid, _ = self.planner.planning(
                start_x, start_y,
                goal_x, goal_y,
                grid.tolist(),
                obstacle_value=1
            )
        except Exception as e:
            self.get_logger().error(f'A* planning failed: {e}')
            return
        
        if not path_grid:
            self.get_logger().warn('Path not found')
            return
        
        self.get_logger().info(f'✓ Path found! Length: {len(path_grid)} nodes')
        
        # Convert grid path to world coordinates
        world_path = Path()
        world_path.header.frame_id = self.current_map.header.frame_id
        world_path.header.stamp = self.get_clock().now().to_msg()
        
        for grid_x, grid_y in path_grid:
            world_x, world_y = grid_to_world(
                grid_x,
                grid_y,
                self.current_map.info.origin.position.x,
                self.current_map.info.origin.position.y,
                self.current_map.info.resolution
            )
            
            pose = PoseStamped()
            pose.header = world_path.header
            pose.pose.position = Point(x=world_x, y=world_y, z=0.0)
            pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            world_path.poses.append(pose)
        
        # Publish path
        self.path_pub.publish(world_path)
        
        # Publish visualization markers
        self.visualize_path(path_grid, start_x, start_y, goal_x, goal_y)
        
        self.get_logger().info('============\n')

    def visualize_path(self, path_grid, start_x: int, start_y: int,
                       goal_x: int, goal_y: int):
        """Create and publish visualization markers"""
        markers = MarkerArray()
        
        # Path line strip (green)
        path_marker = Marker()
        path_marker.header.frame_id = self.current_map.header.frame_id
        path_marker.header.stamp = self.get_clock().now().to_msg()
        path_marker.ns = 'path'
        path_marker.id = 0
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        path_marker.scale.x = 0.1  # Line width
        path_marker.color.r = 0.0
        path_marker.color.g = 1.0
        path_marker.color.b = 0.0
        path_marker.color.a = 1.0
        path_marker.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        
        for grid_x, grid_y in path_grid:
            world_x, world_y = grid_to_world(
                grid_x,
                grid_y,
                self.current_map.info.origin.position.x,
                self.current_map.info.origin.position.y,
                self.current_map.info.resolution
            )
            point = Point(x=world_x, y=world_y, z=0.05)
            path_marker.points.append(point)
        
        markers.markers.append(path_marker)
        
        # Start marker (green sphere)
        start_marker = Marker()
        start_marker.header = path_marker.header
        start_marker.ns = 'start'
        start_marker.id = 1
        start_marker.type = Marker.SPHERE
        start_marker.action = Marker.ADD
        start_marker.scale.x = 0.3
        start_marker.scale.y = 0.3
        start_marker.scale.z = 0.3
        start_marker.color.r = 0.0
        start_marker.color.g = 1.0
        start_marker.color.b = 0.0
        start_marker.color.a = 1.0
        
        start_world_x, start_world_y = grid_to_world(
            start_x,
            start_y,
            self.current_map.info.origin.position.x,
            self.current_map.info.origin.position.y,
            self.current_map.info.resolution
        )
        start_marker.pose.position = Point(x=start_world_x, y=start_world_y, z=0.0)
        start_marker.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        
        markers.markers.append(start_marker)
        
        # Goal marker (red sphere)
        goal_marker = Marker()
        goal_marker.header = path_marker.header
        goal_marker.ns = 'goal'
        goal_marker.id = 2
        goal_marker.type = Marker.SPHERE
        goal_marker.action = Marker.ADD
        goal_marker.scale.x = 0.3
        goal_marker.scale.y = 0.3
        goal_marker.scale.z = 0.3
        goal_marker.color.r = 1.0
        goal_marker.color.g = 0.0
        goal_marker.color.b = 0.0
        goal_marker.color.a = 1.0
        
        goal_world_x, goal_world_y = grid_to_world(
            goal_x,
            goal_y,
            self.current_map.info.origin.position.x,
            self.current_map.info.origin.position.y,
            self.current_map.info.resolution
        )
        goal_marker.pose.position = Point(x=goal_world_x, y=goal_world_y, z=0.0)
        goal_marker.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        
        markers.markers.append(goal_marker)
        
        # Publish markers
        self.marker_pub.publish(markers)

    def update_robot_pose_from_tf(self):
        """Update robot pose from TF"""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )
            self.robot_x = transform.transform.translation.x
            self.robot_y = transform.transform.translation.y
            
            qx = transform.transform.rotation.x
            qy = transform.transform.rotation.y
            qz = transform.transform.rotation.z
            qw = transform.transform.rotation.w
            self.robot_theta = yaw_from_quaternion(qx, qy, qz, qw)
            
            self.robot_pose_initialized = True
        except Exception as e:
            self.get_logger().debug(f'TF lookup failed: {e}')


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    node = PathPlanningNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
