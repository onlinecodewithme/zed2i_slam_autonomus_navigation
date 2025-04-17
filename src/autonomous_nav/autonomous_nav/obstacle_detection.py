#!/usr/bin/env python3

import numpy as np
import cv2
from enum import Enum
import math
from dataclasses import dataclass

class ObstacleDetectionMethod(Enum):
    """
    Different methods for obstacle detection from depth data
    """
    THRESHOLD = 0       # Simple depth thresholding
    GRADIENT = 1        # Depth gradient (detect edges)
    U_DISPARITY = 2     # U-disparity map
    POINT_CLOUD = 3     # Point cloud clustering


@dataclass
class Obstacle:
    """
    Class representing a detected obstacle
    """
    x: float            # x position in camera frame (meters)
    y: float            # y position in camera frame (meters)
    z: float            # z position in camera frame (meters)
    radius: float       # Approximate radius of obstacle (meters)
    height: float       # Height of obstacle (meters)
    velocity_x: float = 0.0  # Estimated velocity in x (meters/sec)
    velocity_y: float = 0.0  # Estimated velocity in y (meters/sec)
    is_dynamic: bool = False  # Whether this is a dynamic obstacle


class ObstacleDetector:
    """
    Class for detecting obstacles from ZED camera depth data
    """
    
    def __init__(self, 
                 method=ObstacleDetectionMethod.THRESHOLD,
                 min_depth=0.5,      # Minimum depth to consider (meters)
                 max_depth=10.0,     # Maximum depth to consider (meters)
                 min_height=0.1,     # Minimum obstacle height (meters)
                 min_width=0.1,      # Minimum obstacle width (meters)
                 ground_margin=0.2,  # Margin above ground plane (meters)
                 depth_threshold=0.5, # Depth change threshold for obstacle detection (meters)
                 point_stride=4      # Process every Nth pixel for performance
                ):
        """
        Initialize the obstacle detector
        
        Args:
            method: Detection method
            min_depth: Minimum depth to consider (meters)
            max_depth: Maximum depth to consider (meters)
            min_height: Minimum obstacle height (meters)
            min_width: Minimum obstacle width (meters)
            ground_margin: Margin above ground plane (meters)
            depth_threshold: Depth change threshold for obstacle detection (meters)
            point_stride: Process every Nth pixel for performance
        """
        self.method = method
        self.min_depth = min_depth
        self.max_depth = max_depth
        self.min_height = min_height
        self.min_width = min_width
        self.ground_margin = ground_margin
        self.depth_threshold = depth_threshold
        self.point_stride = point_stride
        
        # Track obstacles across frames
        self.tracked_obstacles = []
        
        # Previous frame info for tracking
        self.prev_depth = None
        self.prev_timestamp = None
    
    def detect_obstacles(self, depth_image, camera_info=None, timestamp=None):
        """
        Detect obstacles in depth image
        
        Args:
            depth_image: Depth image (32FC1, in meters)
            camera_info: Camera intrinsic parameters (for 3D projection)
            timestamp: Image timestamp (for tracking)
            
        Returns:
            List of Obstacle objects
        """
        # Choose detection method
        if self.method == ObstacleDetectionMethod.THRESHOLD:
            obstacles = self._detect_obstacles_threshold(depth_image, camera_info)
        elif self.method == ObstacleDetectionMethod.GRADIENT:
            obstacles = self._detect_obstacles_gradient(depth_image, camera_info)
        elif self.method == ObstacleDetectionMethod.U_DISPARITY:
            obstacles = self._detect_obstacles_u_disparity(depth_image, camera_info)
        elif self.method == ObstacleDetectionMethod.POINT_CLOUD:
            obstacles = self._detect_obstacles_point_cloud(depth_image, camera_info)
        else:
            obstacles = self._detect_obstacles_threshold(depth_image, camera_info)
        
        # Track obstacles if timestamp is provided
        if timestamp is not None and self.prev_depth is not None and self.prev_timestamp is not None:
            time_delta = (timestamp - self.prev_timestamp).nanoseconds / 1e9
            obstacles = self._track_obstacles(obstacles, time_delta)
        
        # Update previous frame info
        self.prev_depth = depth_image.copy()
        self.prev_timestamp = timestamp
        
        return obstacles
    
    def _detect_obstacles_threshold(self, depth_image, camera_info):
        """
        Detect obstacles using simple depth thresholding
        
        Args:
            depth_image: Depth image (32FC1, in meters)
            camera_info: Camera intrinsic parameters
            
        Returns:
            List of Obstacle objects
        """
        obstacles = []
        
        # Ensure depth image is valid
        if depth_image is None or depth_image.size == 0:
            return obstacles
        
        # Get camera intrinsics
        fx, fy, cx, cy = self._get_camera_intrinsics(camera_info, depth_image)
        
        # Create a binary mask of potential obstacles
        # Clip depth to min/max range
        depth_clipped = np.clip(depth_image, self.min_depth, self.max_depth)
        
        # Identify potential obstacles as areas with valid depth within range
        obstacle_mask = np.logical_and(
            depth_clipped > self.min_depth,
            depth_clipped < self.max_depth
        ).astype(np.uint8) * 255
        
        # Filter out small regions and noise
        kernel = np.ones((5, 5), np.uint8)
        obstacle_mask = cv2.morphologyEx(obstacle_mask, cv2.MORPH_OPEN, kernel)
        
        # Find contours in the mask
        contours, _ = cv2.findContours(obstacle_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Process each contour to create an obstacle
        for contour in contours:
            # Get bounding rectangle
            x, y, w, h = cv2.boundingRect(contour)
            
            # Skip small contours
            if w < 10 or h < 10:  # Minimum pixel size
                continue
            
            # Get region of depth image
            depth_roi = depth_clipped[y:y+h, x:x+w]
            
            # Skip if not enough valid depth points
            valid_points = depth_roi[depth_roi > 0]
            if len(valid_points) < 10:
                continue
            
            # Calculate median depth of the contour (robust to outliers)
            median_depth = np.median(valid_points)
            
            # Calculate physical size using camera intrinsics
            # Width at median depth
            width_meters = (w * median_depth) / fx
            
            # Skip if too small
            if width_meters < self.min_width:
                continue
            
            # Calculate 3D position (in camera frame)
            center_x = x + w/2
            center_y = y + h/2
            
            # Convert to 3D coordinates
            pos_z = median_depth  # depth
            pos_x = (center_x - cx) * pos_z / fx
            pos_y = (center_y - cy) * pos_z / fy
            
            # Create obstacle
            obstacle = Obstacle(
                x=pos_x,
                y=pos_y,
                z=pos_z,
                radius=width_meters/2,  # Approximate as circle
                height=self.min_height  # Assume minimum height
            )
            
            obstacles.append(obstacle)
        
        return obstacles
    
    def _detect_obstacles_gradient(self, depth_image, camera_info):
        """
        Detect obstacles using depth gradient analysis
        
        Args:
            depth_image: Depth image (32FC1, in meters)
            camera_info: Camera intrinsic parameters
            
        Returns:
            List of Obstacle objects
        """
        obstacles = []
        
        # Ensure depth image is valid
        if depth_image is None or depth_image.size == 0:
            return obstacles
        
        # Get camera intrinsics
        fx, fy, cx, cy = self._get_camera_intrinsics(camera_info, depth_image)
        
        # Clip depth to min/max range
        depth_clipped = np.clip(depth_image, self.min_depth, self.max_depth)
        
        # Replace NaN and inf with 0
        depth_clipped = np.nan_to_num(depth_clipped, nan=0.0, posinf=0.0, neginf=0.0)
        
        # Convert to uint8 for gradient computation
        depth_scaled = cv2.normalize(depth_clipped, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
        
        # Compute gradients
        sobel_x = cv2.Sobel(depth_scaled, cv2.CV_64F, 1, 0, ksize=3)
        sobel_y = cv2.Sobel(depth_scaled, cv2.CV_64F, 0, 1, ksize=3)
        
        # Compute gradient magnitude
        gradient_mag = cv2.magnitude(sobel_x, sobel_y)
        
        # Threshold gradient magnitude to find edges
        _, edge_mask = cv2.threshold(gradient_mag, 50, 255, cv2.THRESH_BINARY)
        edge_mask = edge_mask.astype(np.uint8)
        
        # Dilate edges to connect nearby edges
        kernel = np.ones((5, 5), np.uint8)
        edge_mask = cv2.dilate(edge_mask, kernel, iterations=1)
        
        # Find contours in the edge mask
        contours, _ = cv2.findContours(edge_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Process each contour as potential obstacle
        for contour in contours:
            # Get bounding rectangle
            x, y, w, h = cv2.boundingRect(contour)
            
            # Skip small contours
            if w < 10 or h < 10:
                continue
            
            # Get region of depth image
            depth_roi = depth_clipped[y:y+h, x:x+w]
            
            # Skip if not enough valid depth points
            valid_points = depth_roi[depth_roi > 0]
            if len(valid_points) < 10:
                continue
            
            # Calculate median depth of the contour
            median_depth = np.median(valid_points)
            
            # Calculate physical size using camera intrinsics
            width_meters = (w * median_depth) / fx
            height_meters = (h * median_depth) / fy
            
            # Skip if too small
            if width_meters < self.min_width or height_meters < self.min_height:
                continue
            
            # Calculate 3D position (in camera frame)
            center_x = x + w/2
            center_y = y + h/2
            
            # Convert to 3D coordinates
            pos_z = median_depth  # depth
            pos_x = (center_x - cx) * pos_z / fx
            pos_y = (center_y - cy) * pos_z / fy
            
            # Create obstacle
            obstacle = Obstacle(
                x=pos_x,
                y=pos_y,
                z=pos_z,
                radius=max(width_meters, height_meters)/2,  # Use larger dimension
                height=height_meters
            )
            
            obstacles.append(obstacle)
        
        return obstacles
    
    def _detect_obstacles_u_disparity(self, depth_image, camera_info):
        """
        Detect obstacles using U-disparity map
        This method is especially effective for vertical obstacles
        
        Args:
            depth_image: Depth image (32FC1, in meters)
            camera_info: Camera intrinsic parameters
            
        Returns:
            List of Obstacle objects
        """
        # This is just a placeholder. For a real implementation, we would:
        # 1. Convert depth to disparity
        # 2. Generate U-disparity by summing disparity values along columns
        # 3. Threshold U-disparity to find obstacles
        # 4. Project back to 3D space
        return self._detect_obstacles_threshold(depth_image, camera_info)
    
    def _detect_obstacles_point_cloud(self, depth_image, camera_info):
        """
        Detect obstacles by analyzing a point cloud
        
        Args:
            depth_image: Depth image (32FC1, in meters)
            camera_info: Camera intrinsic parameters
            
        Returns:
            List of Obstacle objects
        """
        # This is just a placeholder. For a real implementation, we would:
        # 1. Convert depth image to point cloud
        # 2. Perform ground plane removal
        # 3. Cluster remaining points
        # 4. Create obstacles from clusters
        return self._detect_obstacles_gradient(depth_image, camera_info)
    
    def _track_obstacles(self, obstacles, time_delta):
        """
        Track obstacles across frames to estimate velocity
        
        Args:
            obstacles: Current obstacles
            time_delta: Time since last frame (seconds)
            
        Returns:
            Tracked obstacles with velocity estimates
        """
        if not self.tracked_obstacles or time_delta <= 0:
            self.tracked_obstacles = obstacles
            return obstacles
        
        new_tracked = []
        
        # For each current obstacle, find closest previous obstacle
        for obs in obstacles:
            closest_prev = None
            min_dist = float('inf')
            
            for prev_obs in self.tracked_obstacles:
                dist = math.sqrt((obs.x - prev_obs.x)**2 + (obs.y - prev_obs.y)**2 + (obs.z - prev_obs.z)**2)
                if dist < min_dist and dist < 1.0:  # Max 1m distance for tracking
                    min_dist = dist
                    closest_prev = prev_obs
            
            # If found a match, update with velocity
            if closest_prev is not None:
                # Calculate velocity
                vx = (obs.x - closest_prev.x) / time_delta
                vy = (obs.y - closest_prev.y) / time_delta
                
                # Update obstacle with velocity
                obs.velocity_x = vx
                obs.velocity_y = vy
                
                # Mark as dynamic if velocity is significant
                speed = math.sqrt(vx**2 + vy**2)
                obs.is_dynamic = speed > 0.2  # 0.2 m/s threshold
            
            new_tracked.append(obs)
        
        self.tracked_obstacles = new_tracked
        return new_tracked
    
    def _get_camera_intrinsics(self, camera_info, depth_image):
        """
        Extract camera intrinsics from camera_info message
        
        Args:
            camera_info: CameraInfo message
            depth_image: Depth image (for fallback dimensions)
            
        Returns:
            Tuple of (fx, fy, cx, cy)
        """
        if camera_info is not None:
            fx = camera_info.k[0]
            fy = camera_info.k[4]
            cx = camera_info.k[2]
            cy = camera_info.k[5]
        else:
            # Fallback to reasonable defaults
            height, width = depth_image.shape
            fx = fy = width / 2  # Approximate focal length
            cx = width / 2
            cy = height / 2
        
        return fx, fy, cx, cy
    
    def visualize_obstacles(self, rgb_image, obstacles, show_velocity=True):
        """
        Visualize detected obstacles
        
        Args:
            rgb_image: RGB image to draw on
            obstacles: List of Obstacle objects
            show_velocity: Whether to show velocity vectors
            
        Returns:
            RGB image with obstacles visualized
        """
        vis_img = rgb_image.copy()
        
        # Get image dimensions
        height, width = vis_img.shape[:2]
        
        for obstacle in obstacles:
            # Calculate 2D position on image
            # This is approximate since we don't have camera intrinsics here
            fx = width / 2  # Approximate focal length
            fy = height / 2
            cx = width / 2
            cy = height / 2
            
            # Project 3D position to 2D
            x_2d = int((obstacle.x * fx / obstacle.z) + cx)
            y_2d = int((obstacle.y * fy / obstacle.z) + cy)
            
            # Calculate radius in pixels
            radius_pixels = int((obstacle.radius * fx) / obstacle.z)
            radius_pixels = max(5, radius_pixels)  # Minimum visual size
            
            # Draw circle for obstacle
            color = (0, 0, 255) if obstacle.is_dynamic else (0, 255, 0)
            cv2.circle(vis_img, (x_2d, y_2d), radius_pixels, color, 2)
            
            # Draw distance
            distance_text = f"{obstacle.z:.1f}m"
            cv2.putText(vis_img, distance_text, (x_2d, y_2d - radius_pixels - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            
            # Draw velocity vector if requested
            if show_velocity and (obstacle.velocity_x != 0 or obstacle.velocity_y != 0):
                # Scale velocity for visibility
                vel_scale = 50.0
                end_x = int(x_2d + obstacle.velocity_x * vel_scale)
                end_y = int(y_2d + obstacle.velocity_y * vel_scale)
                
                # Draw arrow
                cv2.arrowedLine(vis_img, (x_2d, y_2d), (end_x, end_y), (255, 0, 0), 2)
                
                # Draw speed text
                speed = math.sqrt(obstacle.velocity_x**2 + obstacle.velocity_y**2)
                speed_text = f"{speed:.1f}m/s"
                cv2.putText(vis_img, speed_text, (x_2d, y_2d + radius_pixels + 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        
        return vis_img
