#!/usr/bin/env python3

import math
import numpy as np
import struct
import rclpy
from rclpy.node import Node
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2, CameraInfo, Image
from cv_bridge import CvBridge, CvBridgeError

from .grid_utils import GridPoint, GridUtils

class PointCloudProcessor:
    """Process point cloud data from ZED camera to extract obstacle points"""
    
    def __init__(self, logger, min_height=0.05, max_height=2.0):
        self.logger = logger
        self.min_height = min_height
        self.max_height = max_height
    
    def process_pointcloud(self, pointcloud, tf_buffer, costmap_origin_x, costmap_origin_y,
                        grid_width, grid_height, costmap_resolution, update_counter):
        """Process point cloud data and return obstacle points in grid coordinates"""
        try:
            # Try different possible camera frame names
            camera_frames = ['zed2i_camera_center', 'zed_camera_center', 'zed2i_left_camera_frame', 'camera_link']
            transform = None
            
            for frame in camera_frames:
                try:
                    transform = tf_buffer.lookup_transform(
                        'map',
                        frame,
                        rclpy.time.Time(),
                        timeout=rclpy.duration.Duration(seconds=0.1)
                    )
                    # Only log on first successful transform
                    if update_counter % 100 == 0:
                        self.logger.info(f"Using transform for frame: {frame}")
                    break
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                    # Don't spam the log with these errors
                    if update_counter % 100 == 0:
                        self.logger.debug(f"Could not find transform for frame {frame}")
                    continue
            
            if transform is None:
                # Print all available frames for debugging if nothing is found
                if update_counter % 100 == 0:
                    frames = tf_buffer.all_frames_as_string()
                    self.logger.error(f"Could not find transform for any camera frame. Available frames:\n{frames}")
                return None, None, None
            
            # Extract camera position for map growth
            camera_pos_x = transform.transform.translation.x
            camera_pos_y = transform.transform.translation.y
            camera_pos_updated = True
            
            # Extract points from the PointCloud2 message
            width = pointcloud.width
            height = pointcloud.height
            point_step = pointcloud.point_step
            row_step = pointcloud.row_step
            
            # Get field offsets - we need x, y, z coordinates
            x_offset = y_offset = z_offset = None
            for field in pointcloud.fields:
                if field.name == 'x':
                    x_offset = field.offset
                elif field.name == 'y':
                    y_offset = field.offset
                elif field.name == 'z':
                    z_offset = field.offset
            
            if x_offset is None or y_offset is None or z_offset is None:
                if update_counter % 100 == 0:
                    self.logger.warn('Pointcloud missing x, y, or z coordinates')
                return camera_pos_x, camera_pos_y, None
            
            # Sample points from pointcloud (don't process every point for efficiency)
            stride = 10  # Process every 10th point
            obstacle_points = []
            
            for y in range(0, height, stride):
                for x in range(0, width, stride):
                    # Calculate index in the data array
                    index = y * row_step + x * point_step
                    
                    # Extract x, y, z coordinates
                    try:
                        x_val = struct.unpack('f', pointcloud.data[index + x_offset:index + x_offset + 4])[0]
                        y_val = struct.unpack('f', pointcloud.data[index + y_offset:index + y_offset + 4])[0]
                        z_val = struct.unpack('f', pointcloud.data[index + z_offset:index + z_offset + 4])[0]
                        
                        # Skip invalid points
                        if math.isnan(x_val) or math.isnan(y_val) or math.isnan(z_val):
                            continue
                        
                        # Skip points too far away or too close
                        if z_val < self.min_height or z_val > self.max_height:
                            continue
                        
                        # Transform the point from camera frame to map frame
                        p = PoseStamped()
                        p.header.frame_id = pointcloud.header.frame_id
                        p.header.stamp = tf_buffer.get_clock().now().to_msg()
                        p.pose.position.x = x_val
                        p.pose.position.y = y_val
                        p.pose.position.z = z_val
                        
                        # Transform point to map frame
                        p_map = tf2_geometry_msgs.do_transform_pose(p, transform)
                        
                        # Convert point to grid coordinates
                        grid_x = GridUtils.world_to_grid_x(p_map.pose.position.x, costmap_origin_x, costmap_resolution)
                        grid_y = GridUtils.world_to_grid_y(p_map.pose.position.y, costmap_origin_y, costmap_resolution)
                        
                        # Check that the point is within the grid
                        if GridUtils.is_valid_grid_coord(grid_x, grid_y, grid_width, grid_height):
                            obstacle_points.append(GridPoint(x=grid_x, y=grid_y, cost=100))
                    except (IndexError, struct.error) as e:
                        # Handle any errors quietly
                        pass
            
            if len(obstacle_points) > 0 and update_counter % 30 == 0:
                self.logger.info(f'Processed {len(obstacle_points)} obstacle points from pointcloud')
            return camera_pos_x, camera_pos_y, obstacle_points
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            if update_counter % 100 == 0:
                self.logger.warn(f'TF error in pointcloud processing: {e}')
            return None, None, None

class DepthImageProcessor:
    """Process depth image data from ZED camera to extract obstacle points"""
    
    def __init__(self, logger, min_height=0.05, max_height=2.0):
        self.logger = logger
        self.min_height = min_height
        self.max_height = max_height
        self.bridge = CvBridge()
    
    def process_depth_image(self, depth_image, camera_info, tf_buffer, costmap_origin_x, costmap_origin_y,
                           grid_width, grid_height, costmap_resolution, update_counter):
        """Process depth image and return obstacle points in grid coordinates"""
        
        fx = camera_info.k[0]
        fy = camera_info.k[4]
        cx = camera_info.k[2]
        cy = camera_info.k[5]
        
        stride = 4
        height, width = depth_image.shape
        
        try:
            transform = tf_buffer.lookup_transform(
                'map',
                camera_info.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            
            camera_pos_x = transform.transform.translation.x
            camera_pos_y = transform.transform.translation.y
            camera_pos_updated = True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            if update_counter % 100 == 0:
                self.logger.warn(f'TF error: {e}')
            transform = None
            camera_pos_x = None
            camera_pos_y = None
        
        obstacle_points = []
        
        for v in range(0, height, stride):
            for u in range(0, width, stride):
                depth = depth_image[v, u]
                
                if depth <= 0.01 or depth > self.max_height:
                    continue
                
                x = (u - cx) * depth / fx
                y = (v - cy) * depth / fy
                z = depth
                
                if z < self.min_height or z > self.max_height:
                    continue
                
                if transform is not None:
                    p = PoseStamped()
                    p.header.frame_id = camera_info.header.frame_id
                    p.header.stamp = tf_buffer.get_clock().now().to_msg()
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
                        
                        grid_x = GridUtils.world_to_grid_x(x_map, costmap_origin_x, costmap_resolution)
                        grid_y = GridUtils.world_to_grid_y(y_map, costmap_origin_y, costmap_resolution)
                        
                        if GridUtils.is_valid_grid_coord(grid_x, grid_y, grid_width, grid_height):
                            obstacle_points.append(GridPoint(x=grid_x, y=grid_y, cost=100))
                    except Exception as e:
                        if update_counter % 100 == 0:
                            self.logger.warn(f'Transform error: {e}')
                else:
                    # Fallback to simple mapping without transform
                    x_map = z
                    y_map = -x
                    
                    grid_x = GridUtils.world_to_grid_x(x_map, costmap_origin_x, costmap_resolution)
                    grid_y = GridUtils.world_to_grid_y(y_map, costmap_origin_y, costmap_resolution)
                    
                    if GridUtils.is_valid_grid_coord(grid_x, grid_y, grid_width, grid_height):
                        obstacle_points.append(GridPoint(x=grid_x, y=grid_y, cost=100))
        
        return camera_pos_x, camera_pos_y, obstacle_points
