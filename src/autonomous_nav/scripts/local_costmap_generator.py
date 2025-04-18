#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError
import tf2_ros
import tf2_geometry_msgs
import numpy as np
import cv2
import threading
import time
import math
import random
from dataclasses import dataclass
from typing import List, Tuple, Optional

@dataclass
class GridPoint:
    """Point in the 2D grid with cost"""
    x: int
    y: int
    cost: int

class LocalCostmapGeneratorNode(Node):
    """
    ROS2 node that generates a dynamic local costmap from ZED 2i depth data
    that grows and updates based on camera movement
    """
    def __init__(self):
        super().__init__('local_costmap_generator')
        
        # Declare parameters
        self.declare_parameter('depth_topic', '/zed2i/zed_node/depth/depth_registered')
        self.declare_parameter('camera_info_topic', '/zed2i/zed_node/rgb/camera_info')
        self.declare_parameter('pointcloud_topic', '/zed2i/zed_node/point_cloud/cloud_registered')
        self.declare_parameter('costmap_topic', '/local_costmap')
        self.declare_parameter('costmap_resolution', 0.05)  # meters/cell
        self.declare_parameter('initial_costmap_width', 20.0)  # initial width in meters
        self.declare_parameter('initial_costmap_height', 20.0)  # initial height in meters
        self.declare_parameter('initial_costmap_origin_x', -10.0)  # initial origin x in meters
        self.declare_parameter('initial_costmap_origin_y', -10.0)  # initial origin y in meters
        self.declare_parameter('min_height', 0.05)  # min height for obstacle detection
        self.declare_parameter('max_height', 2.0)  # max height for obstacle detection
        self.declare_parameter('obstacle_threshold', 0.5)  # meters
        self.declare_parameter('inflation_radius', 0.5)  # meters
        self.declare_parameter('update_rate', 10.0)  # Hz - increased for smoother display
        self.declare_parameter('use_pointcloud', True)  # Whether to use pointcloud or depth image
        self.declare_parameter('map_growth_threshold', 2.0)  # meters from edge to trigger growth
        self.declare_parameter('map_growth_factor', 1.5)  # How much to grow the map by
        self.declare_parameter('edge_tolerance', 4.0)  # How close camera needs to be to edge (m) to resize map
        
        # Get parameters
        self.depth_topic = self.get_parameter('depth_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.pointcloud_topic = self.get_parameter('pointcloud_topic').value
        self.costmap_topic = self.get_parameter('costmap_topic').value
        self.costmap_resolution = self.get_parameter('costmap_resolution').value
        self.initial_costmap_width = self.get_parameter('initial_costmap_width').value
        self.initial_costmap_height = self.get_parameter('initial_costmap_height').value
        self.initial_costmap_origin_x = self.get_parameter('initial_costmap_origin_x').value
        self.initial_costmap_origin_y = self.get_parameter('initial_costmap_origin_y').value
        self.min_height = self.get_parameter('min_height').value
        self.max_height = self.get_parameter('max_height').value
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value
        self.inflation_radius = self.get_parameter('inflation_radius').value
        self.update_rate = self.get_parameter('update_rate').value
        self.use_pointcloud = self.get_parameter('use_pointcloud').value
        self.map_growth_threshold = self.get_parameter('map_growth_threshold').value
        self.map_growth_factor = self.get_parameter('map_growth_factor').value
        self.edge_tolerance = self.get_parameter('edge_tolerance').value
        
        # Dynamic costmap properties
        self.costmap_width = self.initial_costmap_width
        self.costmap_height = self.initial_costmap_height
        self.costmap_origin_x = self.initial_costmap_origin_x
        self.costmap_origin_y = self.initial_costmap_origin_y
        
        # Initialize grid dimensions
        self.grid_width = int(self.costmap_width / self.costmap_resolution)
        self.grid_height = int(self.costmap_height / self.costmap_resolution)
        
        # Initialize occupancy grid - Use -1 (unknown) instead of 0 (free) to avoid initial purple test pattern
        self.occupancy_grid = np.full((self.grid_height, self.grid_width), -1, dtype=np.int8)
        
        # Initialize inflation kernel
        inflation_cells = int(self.inflation_radius / self.costmap_resolution)
        y, x = np.ogrid[-inflation_cells:inflation_cells+1, -inflation_cells:inflation_cells+1]
        self.inflation_kernel = x**2 + y**2 <= inflation_cells**2
        
        # Create QoS profile for sensor data
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Initialize TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Initialize instance variables
        self.depth_image = None
        self.camera_info = None
        self.last_update_time = self.get_clock().now()
        
        # Track camera position for map growth
        self.camera_pos_x = 0.0
        self.camera_pos_y = 0.0
        self.camera_pos_updated = False
        
        # Create publisher for costmap
        self.costmap_pub = self.create_publisher(
            OccupancyGrid,
            self.costmap_topic,
            10
        )
        
        # Create subscribers
        if self.use_pointcloud:
            self.pointcloud_sub = self.create_subscription(
                PointCloud2,
                self.pointcloud_topic,
                self.pointcloud_callback,
                sensor_qos
            )
        else:
            self.depth_sub = self.create_subscription(
                Image,
                self.depth_topic,
                self.depth_callback,
                sensor_qos
            )
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            sensor_qos
        )
        
        # Processing lock to prevent concurrent processing
        self.processing_lock = threading.Lock()
        
        # Create timer for costmap update
        update_period = 1.0 / self.update_rate
        self.update_timer = self.create_timer(update_period, self.update_costmap)
        
        # Create a timer specifically for checking camera position and growing map
        self.map_growth_timer = self.create_timer(0.5, self.check_map_growth)
        
        self.get_logger().info('Dynamic local costmap generator node initialized')
        self.get_logger().info(f'Initial map size: {self.costmap_width}x{self.costmap_height} meters')
    
    def depth_callback(self, msg):
        """Callback for depth images"""
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
            self.depth_image = np.nan_to_num(self.depth_image, nan=0.0)
            if np.max(self.depth_image) > 100:  # likely in mm
                self.depth_image /= 1000.0  # convert to meters
        except CvBridgeError as e:
            self.get_logger().error(f'CV Bridge error: {e}')
    
    def camera_info_callback(self, msg):
        """Callback for camera info"""
        self.camera_info = msg
    
    def pointcloud_callback(self, msg):
        """Callback for point cloud data"""
        self.latest_pointcloud = msg
        self.get_logger().info('Received point cloud message from ZED camera')
    
    def world_to_grid_x(self, world_x):
        """Convert world x coordinate to grid x coordinate"""
        return int((world_x - self.costmap_origin_x) / self.costmap_resolution)
    
    def world_to_grid_y(self, world_y):
        """Convert world y coordinate to grid y coordinate"""
        return int((world_y - self.costmap_origin_y) / self.costmap_resolution)
    
    def grid_to_world_x(self, grid_x):
        """Convert grid x coordinate to world x coordinate"""
        return grid_x * self.costmap_resolution + self.costmap_origin_x
    
    def grid_to_world_y(self, grid_y):
        """Convert grid y coordinate to world y coordinate"""
        return grid_y * self.costmap_resolution + self.costmap_origin_y
    
    def is_valid_grid_coord(self, x, y):
        """Check if grid coordinates are valid"""
        return 0 <= x < self.grid_width and 0 <= y < self.grid_height
    
    def update_camera_position(self):
        """Update the camera position for map growth decisions"""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',  # target frame
                'camera_link',  # source frame (camera position)
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            
            self.camera_pos_x = transform.transform.translation.x
            self.camera_pos_y = transform.transform.translation.y
            self.camera_pos_updated = True
            
            if int(self.get_clock().now().nanoseconds / 1e9) % 10 == 0:
                self.get_logger().debug(f"Camera at ({self.camera_pos_x:.2f}, {self.camera_pos_y:.2f}) in map frame")
            
            return True
        
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.camera_pos_x = 0.0
            self.camera_pos_y = 0.0
            return False
    
    def check_map_growth(self):
        """Check if the map needs to grow based on camera position"""
        if not self.update_camera_position() or not self.camera_pos_updated:
            return
        
        # Calculate distance from camera to map edges
        dist_to_right = (self.costmap_origin_x + self.costmap_width) - self.camera_pos_x
        dist_to_left = self.camera_pos_x - self.costmap_origin_x
        dist_to_top = (self.costmap_origin_y + self.costmap_height) - self.camera_pos_y
        dist_to_bottom = self.camera_pos_y - self.costmap_origin_y
        
        # Check if camera is too close to any edge
        need_growth = False
        growth_direction = []
        
        if dist_to_right < self.edge_tolerance:
            need_growth = True
            growth_direction.append("right")
            self.get_logger().info(f"Camera approaching right edge: {dist_to_right:.2f}m left")
        
        if dist_to_left < self.edge_tolerance:
            need_growth = True
            growth_direction.append("left")
            self.get_logger().info(f"Camera approaching left edge: {dist_to_left:.2f}m left")
        
        if dist_to_top < self.edge_tolerance:
            need_growth = True
            growth_direction.append("top")
            self.get_logger().info(f"Camera approaching top edge: {dist_to_top:.2f}m left")
        
        if dist_to_bottom < self.edge_tolerance:
            need_growth = True
            growth_direction.append("bottom")
            self.get_logger().info(f"Camera approaching bottom edge: {dist_to_bottom:.2f}m left")
        
        # Grow map if needed
        if need_growth:
            self.grow_map(growth_direction)
    
    def grow_map(self, directions):
        """Grow the map in the specified directions"""
        with self.processing_lock:
            old_width = self.grid_width
            old_height = self.grid_height
            old_origin_x = self.costmap_origin_x
            old_origin_y = self.costmap_origin_y
            
            # Calculate growth in meters
            growth_meters = self.costmap_width * (self.map_growth_factor - 1.0)
            
            # Store old occupancy grid
            old_grid = self.occupancy_grid.copy()
            
            # Update map properties based on growth directions
            if "right" in directions:
                self.costmap_width += growth_meters
            
            if "left" in directions:
                self.costmap_width += growth_meters
                self.costmap_origin_x -= growth_meters
            
            if "top" in directions:
                self.costmap_height += growth_meters
            
            if "bottom" in directions:
                self.costmap_height += growth_meters
                self.costmap_origin_y -= growth_meters
            
            # Update grid dimensions
            self.grid_width = int(self.costmap_width / self.costmap_resolution)
            self.grid_height = int(self.costmap_height / self.costmap_resolution)
            
            # Calculate offset for copying old grid into new grid
            offset_x = int((self.costmap_origin_x - old_origin_x) / self.costmap_resolution)
            offset_y = int((self.costmap_origin_y - old_origin_y) / self.costmap_resolution)
            
            # Create new grid with unknown (-1) cells to avoid purple test pattern
            self.occupancy_grid = np.full((self.grid_height, self.grid_width), -1, dtype=np.int8)
            
            # Copy old grid to new grid with offset
            old_start_x = max(0, -offset_x)
            old_start_y = max(0, -offset_y)
            old_end_x = min(old_width, self.grid_width - offset_x)
            old_end_y = min(old_height, self.grid_height - offset_y)
            
            new_start_x = max(0, offset_x)
            new_start_y = max(0, offset_y)
            new_end_x = min(self.grid_width, old_width + offset_x)
            new_end_y = min(self.grid_height, old_height + offset_y)
            
            # Only copy if there's a valid overlap
            if (old_end_x > old_start_x and old_end_y > old_start_y and
                new_end_x > new_start_x and new_end_y > new_start_y):
                
                width_to_copy = min(old_end_x - old_start_x, new_end_x - new_start_x)
                height_to_copy = min(old_end_y - old_start_y, new_end_y - new_start_y)
                
                self.occupancy_grid[new_start_y:new_start_y + height_to_copy,
                                   new_start_x:new_start_x + width_to_copy] = \
                    old_grid[old_start_y:old_start_y + height_to_copy,
                            old_start_x:old_start_x + width_to_copy]
            
            self.get_logger().info(f"Map grown: new size {self.costmap_width:.1f}x{self.costmap_height:.1f}m")
    
    
    def process_pointcloud(self):
        """Process point cloud data from ZED camera to extract obstacle points"""
        try:
            # Print available frames for debugging
            frames = self.tf_buffer.all_frames_as_string()
            self.get_logger().info(f"Available TF frames:\n{frames}")
            
            # Try different possible camera frame names
            camera_frames = ['zed2i_camera_center', 'zed_camera_center', 'zed2i_left_camera_frame', 'camera_link']
            transform = None
            
            for frame in camera_frames:
                try:
                    self.get_logger().info(f"Trying to lookup transform for frame: {frame}")
                    transform = self.tf_buffer.lookup_transform(
                        'map',
                        frame,
                        rclpy.time.Time(),
                        timeout=rclpy.duration.Duration(seconds=0.1)
                    )
                    self.get_logger().info(f"Successfully found transform for frame: {frame}")
                    break
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                    self.get_logger().warning(f"Could not find transform for frame {frame}: {e}")
                    continue
            
            if transform is None:
                self.get_logger().error("Could not find transform for any camera frame")
                return None
            
            # Update camera position for dynamic map growth
            self.camera_pos_x = transform.transform.translation.x
            self.camera_pos_y = transform.transform.translation.y
            self.camera_pos_updated = True
            
            # We need to wait for a pointcloud message to be received
            if not hasattr(self, 'latest_pointcloud') or self.latest_pointcloud is None:
                self.get_logger().warn('No pointcloud data available yet')
                return None
                
            # Process the pointcloud data from ZED
            self.get_logger().debug('Processing real pointcloud data from ZED camera')
            
            # Extract points from the PointCloud2 message
            points = []
            
            # Get pointcloud dimensions and field offsets
            width = self.latest_pointcloud.width
            height = self.latest_pointcloud.height
            point_step = self.latest_pointcloud.point_step
            row_step = self.latest_pointcloud.row_step
            
            # Get field offsets - we need x, y, z coordinates
            x_offset = y_offset = z_offset = None
            for field in self.latest_pointcloud.fields:
                if field.name == 'x':
                    x_offset = field.offset
                elif field.name == 'y':
                    y_offset = field.offset
                elif field.name == 'z':
                    z_offset = field.offset
            
            if x_offset is None or y_offset is None or z_offset is None:
                self.get_logger().warn('Pointcloud missing x, y, or z coordinates')
                return None
            
            # Sample points from pointcloud (don't process every point for efficiency)
            stride = 10  # Process every 10th point
            obstacle_points = []
            
            for y in range(0, height, stride):
                for x in range(0, width, stride):
                    # Calculate index in the data array
                    index = y * row_step + x * point_step
                    
                    # Extract x, y, z coordinates
                    try:
                        # Get the coordinates using struct.unpack
                        import struct
                        
                        x_val = struct.unpack('f', self.latest_pointcloud.data[index + x_offset:index + x_offset + 4])[0]
                        y_val = struct.unpack('f', self.latest_pointcloud.data[index + y_offset:index + y_offset + 4])[0]
                        z_val = struct.unpack('f', self.latest_pointcloud.data[index + z_offset:index + z_offset + 4])[0]
                        
                        # Skip invalid points
                        if math.isnan(x_val) or math.isnan(y_val) or math.isnan(z_val):
                            continue
                        
                        # Skip points too far away or too close
                        if z_val < self.min_height or z_val > self.max_height:
                            continue
                        
                        # Transform the point from camera frame to map frame
                        p = PoseStamped()
                        p.header.frame_id = self.latest_pointcloud.header.frame_id
                        p.header.stamp = self.get_clock().now().to_msg()
                        p.pose.position.x = x_val
                        p.pose.position.y = y_val
                        p.pose.position.z = z_val
                        
                        # Transform point to map frame
                        p_map = tf2_geometry_msgs.do_transform_pose(p, transform)
                        
                        # Convert point to grid coordinates
                        grid_x = self.world_to_grid_x(p_map.pose.position.x)
                        grid_y = self.world_to_grid_y(p_map.pose.position.y)
                        
                        # Check that the point is within the grid
                        if self.is_valid_grid_coord(grid_x, grid_y):
                            obstacle_points.append(GridPoint(x=grid_x, y=grid_y, cost=100))
                    except (IndexError, struct.error) as e:
                        # Handle any errors quietly
                        pass
            
            self.get_logger().info(f'Processed {len(obstacle_points)} obstacle points from pointcloud')
            return obstacle_points
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f'TF error in pointcloud processing: {e}')
            return None
    
    def process_depth_image(self):
        """Process depth image to extract obstacle points"""
        if self.depth_image is None or self.camera_info is None:
            return None
        
        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]
        
        stride = 4
        height, width = self.depth_image.shape
        
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                self.camera_info.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            
            self.camera_pos_x = transform.transform.translation.x
            self.camera_pos_y = transform.transform.translation.y
            self.camera_pos_updated = True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f'TF error: {e}')
            transform = None
        
        obstacle_points = []
        
        for v in range(0, height, stride):
            for u in range(0, width, stride):
                depth = self.depth_image[v, u]
                
                if depth <= 0.01 or depth > self.max_height:
                    continue
                
                x = (u - cx) * depth / fx
                y = (v - cy) * depth / fy
                z = depth
                
                if z < self.min_height or z > self.max_height:
                    continue
                
                if transform is not None:
                    p = PoseStamped()
                    p.header.frame_id = self.camera_info.header.frame_id
                    p.header.stamp = self.get_clock().now().to_msg()
                    p.pose.position.x = z
                    p.pose.position.y = -x
                    p.pose.position.z = -y
                    
                    try:
                        p_map = tf2_geometry_msgs.do_transform_pose(p, transform)
                        
                        x_map = p_map.pose.position.x
                        y_map = p_map.pose.position.y
                        z_map = p_map.pose.position.z
                        
                        if z_map < self.min_height or z_map > self.max_height:
                            continue
                        
                        grid_x = self.world_to_grid_x(x_map)
                        grid_y = self.world_to_grid_y(y_map)
                        
                        if self.is_valid_grid_coord(grid_x, grid_y):
                            obstacle_points.append(GridPoint(x=grid_x, y=grid_y, cost=100))
                    except Exception as e:
                        self.get_logger().warn(f'Transform error: {e}')
                else:
                    # Fallback to simple mapping
                    x_map = z
                    y_map = -x
                    
                    grid_x = self.world_to_grid_x(x_map)
                    grid_y = self.world_to_grid_y(y_map)
                    
                    if self.is_valid_grid_coord(grid_x, grid_y):
                        obstacle_points.append(GridPoint(x=grid_x, y=grid_y, cost=100))
        
        return obstacle_points
    
    def add_obstacles_to_map(self, obstacles):
        """Add obstacle points to the occupancy grid"""
        if not obstacles:
            return
        
        for point in obstacles:
            if self.is_valid_grid_coord(point.x, point.y):
                self.occupancy_grid[point.y, point.x] = point.cost
    
    def update_costmap(self):
        """Update the costmap from sensor data"""
        # Check what data is missing and log it
        if self.use_pointcloud and not hasattr(self, 'latest_pointcloud') or self.latest_pointcloud is None:
            self.get_logger().warning("Missing pointcloud data from ZED camera")
        if not self.use_pointcloud and self.depth_image is None:
            self.get_logger().warning("Missing depth image from ZED camera")
        if self.camera_info is None:
            self.get_logger().warning("Missing camera info from ZED camera")
        
        # Skip processing if essential data is missing
        if (self.depth_image is None and not self.use_pointcloud) or \
           (self.use_pointcloud and (not hasattr(self, 'latest_pointcloud') or self.latest_pointcloud is None)) or \
           self.camera_info is None:
            self.get_logger().warning("Missing required sensor data for costmap generation")
            # Don't add test data, just publish the current empty costmap
            self.publish_costmap()
            return
        
        # Check if enough time has passed since last update
        current_time = self.get_clock().now()
        time_since_last_update = (current_time - self.last_update_time).nanoseconds / 1e9
        
        if time_since_last_update < (1.0 / self.update_rate):
            return
        
        # Acquire lock to prevent concurrent processing
        if not self.processing_lock.acquire(blocking=False):
            return
        
        try:
            # Process sensor data based on available inputs
            if self.use_pointcloud:
                new_obstacles = self.process_pointcloud()
                if new_obstacles:
                    self.add_obstacles_to_map(new_obstacles)
            else:
                new_obstacles = self.process_depth_image()
                if new_obstacles:
                    self.add_obstacles_to_map(new_obstacles)
            
            # Apply inflation to obstacles periodically
            if int(current_time.nanoseconds / 1e9) % 3 == 0:
                self.inflate_obstacles()
            
            # Publish costmap
            self.publish_costmap()
            
            # Update last update time
            self.last_update_time = current_time
        finally:
            self.processing_lock.release()
    
    def inflate_obstacles(self):
        """Apply inflation to obstacles in the occupancy grid"""
        obstacle_cells = np.where(self.occupancy_grid == 100)
        inflated_grid = np.zeros_like(self.occupancy_grid)
        
        for y, x in zip(obstacle_cells[0], obstacle_cells[1]):
            x_min = max(0, x - self.inflation_kernel.shape[1]//2)
            x_max = min(self.grid_width, x + self.inflation_kernel.shape[1]//2 + 1)
            y_min = max(0, y - self.inflation_kernel.shape[0]//2)
            y_max = min(self.grid_height, y + self.inflation_kernel.shape[0]//2 + 1)
            
            kernel_x_min = max(0, -x + self.inflation_kernel.shape[1]//2)
            kernel_x_max = min(self.inflation_kernel.shape[1], self.grid_width - x + self.inflation_kernel.shape[1]//2)
            kernel_y_min = max(0, -y + self.inflation_kernel.shape[0]//2)
            kernel_y_max = min(self.inflation_kernel.shape[0], self.grid_height - y + self.inflation_kernel.shape[0]//2)
            
            kernel_section = self.inflation_kernel[kernel_y_min:kernel_y_max, kernel_x_min:kernel_x_max]
            grid_section = inflated_grid[y_min:y_max, x_min:x_max]
            grid_section[kernel_section] = np.maximum(grid_section[kernel_section], 50)
        
        mask = (self.occupancy_grid != 100) & (inflated_grid > 0)
        self.occupancy_grid[mask] = inflated_grid[mask]
    
    
    def publish_costmap(self):
        """Publish the occupancy grid as a costmap"""
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        
        msg.info.resolution = self.costmap_resolution
        msg.info.width = self.grid_width
        msg.info.height = self.grid_height
        
        msg.info.origin.position.x = self.costmap_origin_x
        msg.info.origin.position.y = self.costmap_origin_y
        msg.info.origin.position.z = 0.0
        
        # Fix blinking in RViz by ensuring map_load_time is consistent with header stamp
        msg.info.map_load_time = msg.header.stamp
        
        # Convert occupancy grid to 1D list for OccupancyGrid message
        flat_grid = self.occupancy_grid.flatten().tolist()
        msg.data = flat_grid
        
        # Publish
        self.costmap_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LocalCostmapGeneratorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
