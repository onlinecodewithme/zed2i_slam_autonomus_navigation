#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import Path, OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import PointCloud2
import tf2_ros
from tf2_geometry_msgs import do_transform_pose

import numpy as np
import math
from enum import Enum
import heapq
import threading
from typing import List, Tuple, Dict, Set, Optional

class PlannerAlgorithm(Enum):
    """Supported path planning algorithms"""
    ASTAR = 0
    RRT = 1
    RRT_STAR = 2
    POTENTIAL_FIELD = 3
    INSPECTION = 4  # Special planner for inspection patterns

class PathPlannerNode(Node):
    """
    ROS2 node for path planning and generation with obstacle avoidance
    """
    def __init__(self):
        super().__init__('path_planner')
        
        # Declare parameters
        self.declare_parameter('algorithm', 'astar')
        self.declare_parameter('map_resolution', 0.1)  # meters per cell
        self.declare_parameter('map_width', 20.0)  # meters
        self.declare_parameter('map_height', 20.0)  # meters
        self.declare_parameter('inflation_radius', 0.5)  # meters
        self.declare_parameter('goal_tolerance', 0.5)  # meters
        self.declare_parameter('update_rate', 1.0)  # Hz
        self.declare_parameter('path_topic', '/path')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('pointcloud_topic', '/zed2i/zed_node/point_cloud/cloud_registered')
        self.declare_parameter('inspection_patterns', True)
        
        # Get parameters
        algorithm_str = self.get_parameter('algorithm').value
        self.map_resolution = self.get_parameter('map_resolution').value
        self.map_width = self.get_parameter('map_width').value
        self.map_height = self.get_parameter('map_height').value
        self.inflation_radius = self.get_parameter('inflation_radius').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.update_rate = self.get_parameter('update_rate').value
        self.path_topic = self.get_parameter('path_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.goal_topic = self.get_parameter('goal_topic').value
        self.pointcloud_topic = self.get_parameter('pointcloud_topic').value
        self.inspection_patterns = self.get_parameter('inspection_patterns').value
        
        # Convert algorithm string to enum
        self.algorithm = PlannerAlgorithm.ASTAR
        if algorithm_str == 'rrt':
            self.algorithm = PlannerAlgorithm.RRT
        elif algorithm_str == 'rrt_star':
            self.algorithm = PlannerAlgorithm.RRT_STAR
        elif algorithm_str == 'potential_field':
            self.algorithm = PlannerAlgorithm.POTENTIAL_FIELD
        elif algorithm_str == 'inspection':
            self.algorithm = PlannerAlgorithm.INSPECTION
        
        # Initialize map
        self.initialize_map()
        
        # Initialize TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Create publishers
        self.path_pub = self.create_publisher(
            Path,
            self.path_topic,
            10
        )
        
        self.map_pub = self.create_publisher(
            OccupancyGrid,
            '/local_costmap',
            10
        )
        
        self.markers_pub = self.create_publisher(
            MarkerArray,
            '/path_visualization',
            10
        )
        
        # Create subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10
        )
        
        self.goal_sub = self.create_subscription(
            PoseStamped,
            self.goal_topic,
            self.goal_callback,
            10
        )
        
        # Create QoS profile for PointCloud sensor data
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            self.pointcloud_topic,
            self.pointcloud_callback,
            sensor_qos
        )
        
        # Initialize state variables
        self.current_pose = None
        self.goal_pose = None
        self.last_plan_time = self.get_clock().now()
        self.need_replanning = False
        
        # Processing lock to prevent concurrent processing
        self.processing_lock = threading.Lock()
        
        # Create timer for path planning
        update_period = 1.0 / self.update_rate
        self.planning_timer = self.create_timer(update_period, self.planning_callback)
        
        self.get_logger().info(f'Path planner node initialized with {algorithm_str} algorithm')
    
    def initialize_map(self):
        """
        Initialize the local grid map
        """
        # Calculate map dimensions in cells
        self.map_width_cells = int(self.map_width / self.map_resolution)
        self.map_height_cells = int(self.map_height / self.map_resolution)
        
        # Create empty occupancy grid
        self.occupancy_grid = np.zeros((self.map_height_cells, self.map_width_cells), dtype=np.int8)
        
        # Origin is the center of the map (robot-centric)
        self.map_origin_x = -self.map_width / 2.0
        self.map_origin_y = -self.map_height / 2.0
        
        # Inflation kernel for obstacles
        inflation_radius_cells = int(self.inflation_radius / self.map_resolution)
        y, x = np.ogrid[-inflation_radius_cells:inflation_radius_cells+1, -inflation_radius_cells:inflation_radius_cells+1]
        self.inflation_kernel = x**2 + y**2 <= inflation_radius_cells**2
    
    def odom_callback(self, msg):
        """
        Callback for odometry data
        """
        self.current_pose = msg.pose.pose
    
    def goal_callback(self, msg):
        """
        Callback for goal pose
        """
        self.goal_pose = msg.pose
        self.need_replanning = True
        self.get_logger().info(f'New goal received: ({self.goal_pose.position.x:.2f}, {self.goal_pose.position.y:.2f})')
    
    def pointcloud_callback(self, msg):
        """
        Callback for point cloud data
        """
        # Point cloud processing would go here
        # For simplicity, not implemented in this example
        # This would normally:
        # 1. Convert point cloud to robot frame
        # 2. Project points to 2D grid
        # 3. Update occupancy grid
        pass
    
    def planning_callback(self):
        """
        Main callback for path planning
        """
        if self.current_pose is None or self.goal_pose is None:
            return
        
        # Check if we need to replan
        current_time = self.get_clock().now()
        time_since_last_plan = (current_time - self.last_plan_time).nanoseconds / 1e9
        
        if not self.need_replanning and time_since_last_plan < 1.0:  # Replan at minimum every 1 second
            return
        
        # Acquire lock to prevent concurrent processing
        if not self.processing_lock.acquire(blocking=False):
            return
        
        try:
            # Update map from sensor data (would be more complex in real implementation)
            self.update_map_from_sensors()
            
            # Plan path
            path = self.plan_path()
            
            if path:
                # Publish path
                self.publish_path(path)
                
                # Publish visualization
                self.publish_path_visualization(path)
                
                # Reset replanning flag
                self.need_replanning = False
                self.last_plan_time = current_time
            else:
                self.get_logger().warn('Path planning failed!')
        finally:
            # Release lock
            self.processing_lock.release()
    
    def update_map_from_sensors(self):
        """
        Update local map from sensor data
        """
        # In a real implementation, this would process point cloud data
        # and update the occupancy grid based on obstacle detections
        
        # For demo purposes, add some obstacles
        # Clear map
        self.occupancy_grid.fill(0)
        
        # Add some demo obstacles (in a real system, this would come from sensor data)
        for i in range(5):
            # Random obstacles
            x = np.random.uniform(-self.map_width/3, self.map_width/3)
            y = np.random.uniform(-self.map_height/3, self.map_height/3)
            
            # Convert to grid coordinates
            grid_x, grid_y = self.world_to_grid(x, y)
            
            # Add obstacle with inflation
            self.add_obstacle(grid_x, grid_y)
        
        # Publish updated map
        self.publish_map()
    
    def add_obstacle(self, grid_x, grid_y):
        """
        Add an obstacle to the map with inflation
        
        Args:
            grid_x: x coordinate in grid cells
            grid_y: y coordinate in grid cells
        """
        # Check bounds
        if (0 <= grid_x < self.map_width_cells and 
            0 <= grid_y < self.map_height_cells):
            
            # Set obstacle cell
            self.occupancy_grid[grid_y, grid_x] = 100
            
            # Apply inflation
            x_min = max(0, grid_x - self.inflation_kernel.shape[1]//2)
            x_max = min(self.map_width_cells, grid_x + self.inflation_kernel.shape[1]//2 + 1)
            y_min = max(0, grid_y - self.inflation_kernel.shape[0]//2)
            y_max = min(self.map_height_cells, grid_y + self.inflation_kernel.shape[0]//2 + 1)
            
            kernel_x_min = max(0, -grid_x + self.inflation_kernel.shape[1]//2)
            kernel_x_max = min(self.inflation_kernel.shape[1], self.map_width_cells - grid_x + self.inflation_kernel.shape[1]//2)
            kernel_y_min = max(0, -grid_y + self.inflation_kernel.shape[0]//2)
            kernel_y_max = min(self.inflation_kernel.shape[0], self.map_height_cells - grid_y + self.inflation_kernel.shape[0]//2)
            
            kernel_section = self.inflation_kernel[kernel_y_min:kernel_y_max, kernel_x_min:kernel_x_max]
            
            # Mark cells in inflation radius as occupied (with lower cost)
            self.occupancy_grid[y_min:y_max, x_min:x_max][kernel_section] = 50
    
    def plan_path(self):
        """
        Plan a path from current pose to goal pose
        
        Returns:
            List of PoseStamped representing the path
        """
        # Convert poses to grid coordinates
        start_x, start_y = self.world_to_grid(
            self.current_pose.position.x,
            self.current_pose.position.y
        )
        
        goal_x, goal_y = self.world_to_grid(
            self.goal_pose.position.x,
            self.goal_pose.position.y
        )
        
        # Check if start or goal are out of bounds or in obstacles
        if (not self.is_valid_cell(start_x, start_y) or 
            not self.is_valid_cell(goal_x, goal_y)):
            self.get_logger().warn('Start or goal position is invalid or in obstacle!')
            return []
        
        # Choose planning algorithm
        if self.algorithm == PlannerAlgorithm.ASTAR:
            path_grid = self.plan_astar(start_x, start_y, goal_x, goal_y)
        elif self.algorithm == PlannerAlgorithm.RRT:
            path_grid = self.plan_rrt(start_x, start_y, goal_x, goal_y)
        elif self.algorithm == PlannerAlgorithm.RRT_STAR:
            path_grid = self.plan_rrt_star(start_x, start_y, goal_x, goal_y)
        elif self.algorithm == PlannerAlgorithm.POTENTIAL_FIELD:
            path_grid = self.plan_potential_field(start_x, start_y, goal_x, goal_y)
        elif self.algorithm == PlannerAlgorithm.INSPECTION:
            path_grid = self.plan_inspection(start_x, start_y, goal_x, goal_y)
        else:
            path_grid = self.plan_astar(start_x, start_y, goal_x, goal_y)
        
        # Convert path from grid to world coordinates and create Path message
        if not path_grid:
            return []
        
        path = []
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"
        
        for grid_x, grid_y in path_grid:
            world_x, world_y = self.grid_to_world(grid_x, grid_y)
            
            pose = PoseStamped()
            pose.header = header
            pose.pose.position.x = world_x
            pose.pose.position.y = world_y
            pose.pose.position.z = 0.0
            
            # Use current orientation for all path points (could be improved)
            pose.pose.orientation = self.current_pose.orientation
            
            path.append(pose)
        
        return path
    
    def plan_astar(self, start_x: int, start_y: int, goal_x: int, goal_y: int) -> List[Tuple[int, int]]:
        """
        A* path planning algorithm
        
        Args:
            start_x: Start x coordinate in grid cells
            start_y: Start y coordinate in grid cells
            goal_x: Goal x coordinate in grid cells
            goal_y: Goal y coordinate in grid cells
            
        Returns:
            List of (x, y) tuples representing the path in grid coordinates
        """
        # Priority queue for A*
        open_set = []
        
        # Initial node
        g_score = {(start_x, start_y): 0}
        f_score = {(start_x, start_y): self.heuristic(start_x, start_y, goal_x, goal_y)}
        heapq.heappush(open_set, (f_score[(start_x, start_y)], (start_x, start_y)))
        
        # For reconstructing the path
        came_from = {}
        
        # Possible movements (8-connected grid)
        directions = [
            (0, 1), (1, 0), (0, -1), (-1, 0),  # 4-connected
            (1, 1), (-1, 1), (1, -1), (-1, -1)  # diagonals
        ]
        
        # A* search
        while open_set:
            _, current = heapq.heappop(open_set)
            current_x, current_y = current
            
            # Check if goal reached
            if abs(current_x - goal_x) <= 1 and abs(current_y - goal_y) <= 1:
                # Reconstruct and return path
                path = self.reconstruct_path(came_from, current)
                path.append((goal_x, goal_y))  # Add goal
                return path
            
            # Explore neighbors
            for dx, dy in directions:
                neighbor_x, neighbor_y = current_x + dx, current_y + dy
                neighbor = (neighbor_x, neighbor_y)
                
                # Check if valid
                if not self.is_valid_cell(neighbor_x, neighbor_y):
                    continue
                
                # Cost to neighbor (diagonal movements cost more)
                movement_cost = 1.4 if abs(dx) + abs(dy) == 2 else 1.0
                
                # Apply cost based on occupancy (higher cost for close-to-obstacle cells)
                if self.occupancy_grid[neighbor_y, neighbor_x] > 0:
                    # Higher cost for cells close to obstacles
                    movement_cost *= 2.0 + self.occupancy_grid[neighbor_y, neighbor_x] / 50.0
                
                # Calculate tentative g score
                tentative_g_score = g_score.get((current_x, current_y), float('inf')) + movement_cost
                
                # Check if this path is better
                if tentative_g_score < g_score.get(neighbor, float('inf')):
                    # Record this path
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor_x, neighbor_y, goal_x, goal_y)
                    
                    # Add to open set if not already there
                    if not any(neighbor == node[1] for node in open_set):
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))
        
        # No path found
        self.get_logger().warn('A* could not find a path')
        return []
    
    def heuristic(self, x1: int, y1: int, x2: int, y2: int) -> float:
        """
        Heuristic function for A* (Euclidean distance)
        """
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    
    def reconstruct_path(self, came_from: Dict, current: Tuple[int, int]) -> List[Tuple[int, int]]:
        """
        Reconstruct path from came_from dictionary
        """
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        
        # Reverse to get path from start to goal
        path.reverse()
        
        # Simplify path (reduce number of points)
        simplified_path = self.simplify_path(path)
        
        return simplified_path
    
    def simplify_path(self, path: List[Tuple[int, int]]) -> List[Tuple[int, int]]:
        """
        Simplify path by removing unnecessary waypoints
        """
        if len(path) <= 2:
            return path
        
        # Keep start point
        simplified = [path[0]]
        
        for i in range(1, len(path) - 1):
            # Check if we can skip this point (line-of-sight check)
            if not self.is_line_free(simplified[-1], path[i+1]):
                simplified.append(path[i])
        
        # Add end point
        simplified.append(path[-1])
        
        return simplified
    
    def is_line_free(self, p1: Tuple[int, int], p2: Tuple[int, int]) -> bool:
        """
        Check if a straight line between two points is free of obstacles
        """
        x1, y1 = p1
        x2, y2 = p2
        
        # Bresenham's line algorithm
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        sx = 1 if x1 < x2 else -1
        sy = 1 if y1 < y2 else -1
        err = dx - dy
        
        while x1 != x2 or y1 != y2:
            if not self.is_valid_cell(x1, y1) or self.occupancy_grid[y1, x1] >= 50:
                return False
            
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x1 += sx
            if e2 < dx:
                err += dx
                y1 += sy
        
        return True
    
    def plan_rrt(self, start_x: int, start_y: int, goal_x: int, goal_y: int) -> List[Tuple[int, int]]:
        """
        RRT path planning algorithm (simplified)
        """
        # For demo purposes, return A* path
        return self.plan_astar(start_x, start_y, goal_x, goal_y)
    
    def plan_rrt_star(self, start_x: int, start_y: int, goal_x: int, goal_y: int) -> List[Tuple[int, int]]:
        """
        RRT* path planning algorithm (simplified)
        """
        # For demo purposes, return A* path
        return self.plan_astar(start_x, start_y, goal_x, goal_y)
    
    def plan_potential_field(self, start_x: int, start_y: int, goal_x: int, goal_y: int) -> List[Tuple[int, int]]:
        """
        Potential field path planning algorithm (simplified)
        """
        # For demo purposes, return A* path
        return self.plan_astar(start_x, start_y, goal_x, goal_y)
    
    def plan_inspection(self, start_x: int, start_y: int, goal_x: int, goal_y: int) -> List[Tuple[int, int]]:
        """
        Plan a path for aircraft inspection
        Generate a pattern that circles around the aircraft
        """
        # First, find a path to the vicinity of the goal
        approach_path = self.plan_astar(start_x, start_y, goal_x, goal_y)
        
        if not approach_path:
            return []
        
        # If inspection patterns are not enabled, just return direct path
        if not self.inspection_patterns:
            return approach_path
        
        # For inspection, generate a circle around the goal
        inspection_path = []
        
        # Circle parameters
        circle_radius = int(3.0 / self.map_resolution)  # 3 meters in grid cells
        num_points = 16  # Number of points in the circle
        
        for i in range(num_points + 1):  # +1 to close the loop
            angle = 2 * math.pi * i / num_points
            x = goal_x + int(circle_radius * math.cos(angle))
            y = goal_y + int(circle_radius * math.sin(angle))
            
            # Ensure point is valid
            if self.is_valid_cell(x, y):
                inspection_path.append((x, y))
        
        # Connect approach path to inspection circle
        # Skip the goal point from the approach path
        connected_path = approach_path[:-1]
        
        # Find best connection point to circle
        if connected_path and inspection_path:
            best_dist = float('inf')
            best_idx = 0
            last_point = connected_path[-1]
            
            for i, point in enumerate(inspection_path):
                dist = math.sqrt((point[0] - last_point[0])**2 + (point[1] - last_point[1])**2)
                if dist < best_dist:
                    best_dist = dist
                    best_idx = i
            
            # Reorder inspection path to start at best connection point
            inspection_path = inspection_path[best_idx:] + inspection_path[:best_idx]
        
        # Combine paths
        combined_path = connected_path + inspection_path
        
        return combined_path
    
    def world_to_grid(self, world_x: float, world_y: float) -> Tuple[int, int]:
        """
        Convert world coordinates to grid coordinates
        
        Args:
            world_x: x coordinate in world frame
            world_y: y coordinate in world frame
            
        Returns:
            (grid_x, grid_y) tuple
        """
        grid_x = int((world_x - self.map_origin_x) / self.map_resolution)
        grid_y = int((world_y - self.map_origin_y) / self.map_resolution)
        
        return grid_x, grid_y
    
    def grid_to_world(self, grid_x: int, grid_y: int) -> Tuple[float, float]:
        """
        Convert grid coordinates to world coordinates
        
        Args:
            grid_x: x coordinate in grid cells
            grid_y: y coordinate in grid cells
            
        Returns:
            (world_x, world_y) tuple
        """
        world_x = grid_x * self.map_resolution + self.map_origin_x
        world_y = grid_y * self.map_resolution + self.map_origin_y
        
        return world_x, world_y
    
    def is_valid_cell(self, x: int, y: int) -> bool:
        """
        Check if a grid cell is valid (in bounds and not an obstacle)
        
        Args:
            x: x coordinate in grid cells
            y: y coordinate in grid cells
            
        Returns:
            True if cell is valid, False otherwise
        """
        # Check bounds
        if (x < 0 or x >= self.map_width_cells or 
            y < 0 or y >= self.map_height_cells):
            return False
        
        # Check if cell is an obstacle (100 is full occupancy)
        if self.occupancy_grid[y, x] >= 80:
            return False
        
        return True
    
    def publish_path(self, path: List[PoseStamped]):
        """
        Publish path as a Path message
        
        Args:
            path: List of PoseStamped representing the path
        """
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"
        path_msg.poses = path
        
        self.path_pub.publish(path_msg)
    
    def publish_map(self):
        """
        Publish occupancy grid map
        """
        map_msg = OccupancyGrid()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = "map"
        
        map_msg.info.resolution = self.map_resolution
        map_msg.info.width = self.map_width_cells
        map_msg.info.height = self.map_height_cells
        
        # Origin is at the bottom-left corner of the grid
        map_msg.info.origin.position.x = self.map_origin_x
        map_msg.info.origin.position.y = self.map_origin_y
        map_msg.info.origin.position.z = 0.0
        
        # Convert numpy array to 1D list
        map_msg.data = self.occupancy_grid.flatten().tolist()
        
        self.map_pub.publish(map_msg)
    
    def publish_path_visualization(self, path: List[PoseStamped]):
        """
        Publish path visualization as MarkerArray
        
        Args:
            path: List of PoseStamped representing the path
        """
        marker_array = MarkerArray()
        
        # Line strip for path
        line_marker = Marker()
        line_marker.header.frame_id = "map"
        line_marker.header.stamp = self.get_clock().now().to_msg()
        line_marker.ns = "path"
        line_marker.id = 0
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        
        line_marker.scale.x = 0.1  # Line width
        line_marker.color.r = 0.0
        line_marker.color.g = 1.0
        line_marker.color.b = 0.0
        line_marker.color.a = 1.0
        
        # Add points to line strip
        for pose in path:
            p = Point()
            p.x = pose.pose.position.x
            p.y = pose.pose.position.y
            p.z = 0.1  # Slightly above ground
            line_marker.points.append(p)
        
        marker_array.markers.append(line_marker)
        
        # Sphere markers for waypoints
        for i, pose in enumerate(path):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "waypoints"
            marker.id = i + 1
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose.position.x = pose.pose.position.x
            marker.pose.position.y = pose.pose.position.y
            marker.pose.position.z = 0.1
            
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            
            # Different color for start and end
            if i == 0:
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            elif i == len(path) - 1:
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            else:
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
            
            marker.color.a = 1.0
            
            marker_array.markers.append(marker)
        
        self.markers_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = PathPlannerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
