#!/usr/bin/env python3

import numpy as np
import threading
import time
from nav_msgs.msg import OccupancyGrid

from .grid_utils import GridPoint, GridUtils
from .sensor_processing import PointCloudProcessor, DepthImageProcessor

class LocalCostmapGenerator:
    """Class that generates a dynamic local costmap from sensor data"""
    
    def __init__(self, node, use_pointcloud=True, 
                 costmap_resolution=0.05,
                 costmap_width=20.0,
                 costmap_height=20.0,
                 costmap_origin_x=-10.0,
                 costmap_origin_y=-10.0,
                 min_height=0.05,
                 max_height=2.0,
                 inflation_radius=0.5,
                 map_growth_factor=1.5):
        """Initialize the costmap generator with parameters"""
        self.node = node
        self.logger = node.get_logger()
        
        # Store parameters
        self.use_pointcloud = use_pointcloud
        self.costmap_resolution = costmap_resolution
        self.costmap_width = costmap_width
        self.costmap_height = costmap_height
        self.costmap_origin_x = costmap_origin_x
        self.costmap_origin_y = costmap_origin_y
        self.min_height = min_height
        self.max_height = max_height
        self.inflation_radius = inflation_radius
        self.map_growth_factor = map_growth_factor
        
        # Initialize grid dimensions
        self.grid_width = int(self.costmap_width / self.costmap_resolution)
        self.grid_height = int(self.costmap_height / self.costmap_resolution)
        
        # Initialize occupancy grid with free space (0) instead of unknown (-1)
        # Using 0 (free) makes costmaps more visible in RViz
        self.occupancy_grid = np.full((self.grid_height, self.grid_width), 0, dtype=np.int8)
        
        # Initialize inflation kernel
        self.inflation_kernel = GridUtils.create_inflation_kernel(self.inflation_radius, self.costmap_resolution)
        
        # Track camera position for map growth
        self.camera_pos_x = 0.0
        self.camera_pos_y = 0.0
        self.camera_pos_updated = False
        
        # Processing lock to prevent concurrent processing
        self.processing_lock = threading.Lock()
        
        # Initialize processors
        self.pointcloud_processor = PointCloudProcessor(
            logger=self.logger,
            min_height=self.min_height,
            max_height=self.max_height
        )
        
        self.depth_processor = DepthImageProcessor(
            logger=self.logger,
            min_height=self.min_height,
            max_height=self.max_height
        )
        
        self.logger.info('LocalCostmapGenerator initialized')
        self.logger.info(f'Initial map size: {self.costmap_width}x{self.costmap_height} meters')
    
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
            
            # Create new grid with free space (0) cells for better visibility
            self.occupancy_grid = np.full((self.grid_height, self.grid_width), 0, dtype=np.int8)
            
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
            
            self.logger.info(f"Map grown: new size {self.costmap_width:.1f}x{self.costmap_height:.1f}m")
    
    def check_map_growth(self, camera_pos_x, camera_pos_y, edge_tolerance=4.0):
        """Check if the map needs to grow based on camera position"""
        if camera_pos_x is None or camera_pos_y is None:
            return
        
        # Update camera position for future reference
        self.camera_pos_x = camera_pos_x
        self.camera_pos_y = camera_pos_y
        self.camera_pos_updated = True
        
        # Calculate distance from camera to map edges
        dist_to_right = (self.costmap_origin_x + self.costmap_width) - camera_pos_x
        dist_to_left = camera_pos_x - self.costmap_origin_x
        dist_to_top = (self.costmap_origin_y + self.costmap_height) - camera_pos_y
        dist_to_bottom = camera_pos_y - self.costmap_origin_y
        
        # Check if camera is too close to any edge
        need_growth = False
        growth_direction = []
        
        if dist_to_right < edge_tolerance:
            need_growth = True
            growth_direction.append("right")
            self.logger.info(f"Camera approaching right edge: {dist_to_right:.2f}m left")
        
        if dist_to_left < edge_tolerance:
            need_growth = True
            growth_direction.append("left")
            self.logger.info(f"Camera approaching left edge: {dist_to_left:.2f}m left")
        
        if dist_to_top < edge_tolerance:
            need_growth = True
            growth_direction.append("top")
            self.logger.info(f"Camera approaching top edge: {dist_to_top:.2f}m left")
        
        if dist_to_bottom < edge_tolerance:
            need_growth = True
            growth_direction.append("bottom")
            self.logger.info(f"Camera approaching bottom edge: {dist_to_bottom:.2f}m left")
        
        # Grow map if needed
        if need_growth:
            self.grow_map(growth_direction)
            return True
        return False
    
    def add_obstacles_to_map(self, obstacles):
        """Add obstacle points to the occupancy grid"""
        if not obstacles:
            return
        
        for point in obstacles:
            if GridUtils.is_valid_grid_coord(point.x, point.y, self.grid_width, self.grid_height):
                self.occupancy_grid[point.y, point.x] = point.cost
    
    def process_pointcloud(self, pointcloud, tf_buffer, update_counter):
        """Process point cloud and update the camera position"""
        if pointcloud is None:
            return False
        
        camera_pos_x, camera_pos_y, obstacle_points = self.pointcloud_processor.process_pointcloud(
            pointcloud=pointcloud,
            tf_buffer=tf_buffer,
            costmap_origin_x=self.costmap_origin_x,
            costmap_origin_y=self.costmap_origin_y,
            grid_width=self.grid_width,
            grid_height=self.grid_height,
            costmap_resolution=self.costmap_resolution,
            update_counter=update_counter
        )
        
        # Check if map needs to grow
        if camera_pos_x is not None and camera_pos_y is not None:
            self.check_map_growth(camera_pos_x, camera_pos_y)
        
        # Add obstacle points to the map
        if obstacle_points:
            self.add_obstacles_to_map(obstacle_points)
            return True
        return False
    
    def process_depth_image(self, depth_image, camera_info, tf_buffer, update_counter):
        """Process depth image and update the camera position"""
        if depth_image is None or camera_info is None:
            return False
        
        camera_pos_x, camera_pos_y, obstacle_points = self.depth_processor.process_depth_image(
            depth_image=depth_image,
            camera_info=camera_info,
            tf_buffer=tf_buffer,
            costmap_origin_x=self.costmap_origin_x,
            costmap_origin_y=self.costmap_origin_y,
            grid_width=self.grid_width,
            grid_height=self.grid_height,
            costmap_resolution=self.costmap_resolution,
            update_counter=update_counter
        )
        
        # Check if map needs to grow
        if camera_pos_x is not None and camera_pos_y is not None:
            self.check_map_growth(camera_pos_x, camera_pos_y)
        
        # Add obstacle points to the map
        if obstacle_points:
            self.add_obstacles_to_map(obstacle_points)
            return True
        return False
    
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
            
            # Use the kernel to set inflation values
            grid_section[kernel_section] = np.maximum(grid_section[kernel_section], 50)
        
        # Apply inflation to the occupancy grid (but don't overwrite obstacles)
        mask = (self.occupancy_grid != 100) & (inflated_grid > 0)
        self.occupancy_grid[mask] = inflated_grid[mask]
    
    def create_occupancy_grid_msg(self, stamp, frame_id="map"):
        """Create a ROS OccupancyGrid message from the current grid"""
        msg = OccupancyGrid()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        
        msg.info.resolution = self.costmap_resolution
        msg.info.width = self.grid_width
        msg.info.height = self.grid_height
        
        msg.info.origin.position.x = self.costmap_origin_x
        msg.info.origin.position.y = self.costmap_origin_y
        msg.info.origin.position.z = 0.0
        
        # Fix blinking in RViz by ensuring map_load_time is consistent with header stamp
        msg.info.map_load_time = stamp
        
        # Convert occupancy grid to 1D list for OccupancyGrid message
        # Convert -1 (unknown) to 0 (free) for better visibility in RViz
        # and ensure values are in the range [0, 100]
        flat_grid = []
        for val in self.occupancy_grid.flatten():
            if val < 0:  # Unknown space (-1)
                flat_grid.append(0)  # Convert to free space for better visualization
            elif val > 100:  # Cap at 100
                flat_grid.append(100)
            else:
                flat_grid.append(val)
        
        msg.data = flat_grid
        return msg
