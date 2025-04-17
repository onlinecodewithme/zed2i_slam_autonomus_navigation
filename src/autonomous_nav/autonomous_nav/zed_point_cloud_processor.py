#!/usr/bin/env python3

import rclpy
import numpy as np
import cv2
import tf2_ros
import math
from rclpy.node import Node
from geometry_msgs.msg import Point, TransformStamped
from sensor_msgs.msg import PointCloud2, PointField, Image
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge, CvBridgeError
import struct
import array
from typing import List, Tuple, Dict, Optional
from autonomous_nav.obstacle_detection import Obstacle

class ZedPointCloudProcessor:
    """
    Process ZED point cloud data to detect obstacles including dynamic ones
    """
    def __init__(self, 
                 robot_width=0.82,      # Width of robot in meters
                 robot_length=1.07,     # Length of robot in meters
                 robot_height=0.68,     # Height of robot in meters
                 safety_margin=0.5,     # Safety margin around robot
                 ground_threshold=0.15, # Height threshold for ground detection
                 cluster_distance=0.3,  # Distance threshold for clustering
                 min_cluster_size=10,   # Minimum cluster size for obstacle detection
                 voxel_size=0.1):       # Voxel size for downsampling
        
        self.robot_width = robot_width
        self.robot_length = robot_length
        self.robot_height = robot_height
        self.safety_margin = safety_margin
        self.ground_threshold = ground_threshold
        self.cluster_distance = cluster_distance
        self.min_cluster_size = min_cluster_size
        self.voxel_size = voxel_size
        
        # Previous frame data for tracking
        self.prev_obstacles = []
        self.prev_timestamp = None
        
        # Bridge for CV operations
        self.bridge = CvBridge()
        
    def process_point_cloud(self, cloud_msg: PointCloud2, timestamp=None) -> List[Obstacle]:
        """
        Process point cloud data to detect obstacles
        
        Args:
            cloud_msg: PointCloud2 message from ZED
            timestamp: Time for obstacle tracking
            
        Returns:
            List of detected obstacles
        """
        # Convert PointCloud2 to numpy array
        points = self.point_cloud2_to_array(cloud_msg)
        
        if len(points) == 0:
            return []
        
        # Perform ground plane removal
        non_ground_points = self.remove_ground_plane(points)
        
        # Downsample point cloud with voxel grid
        downsampled_points = self.voxel_grid_downsample(non_ground_points)
        
        # Cluster points to find obstacles
        obstacle_clusters = self.cluster_points(downsampled_points)
        
        # Convert clusters to obstacle objects
        obstacles = self.convert_clusters_to_obstacles(obstacle_clusters)
        
        # Track obstacles across frames
        if timestamp is not None and self.prev_timestamp is not None:
            time_delta = (timestamp - self.prev_timestamp).nanoseconds / 1e9
            obstacles = self.track_obstacles(obstacles, time_delta)
        
        # Update previous frame data
        self.prev_obstacles = obstacles
        self.prev_timestamp = timestamp
        
        return obstacles
    
    def point_cloud2_to_array(self, cloud_msg: PointCloud2) -> np.ndarray:
        """
        Convert PointCloud2 message to numpy array of points
        
        Args:
            cloud_msg: PointCloud2 message
            
        Returns:
            Numpy array of shape (N, 3) containing points
        """
        # Check for organized point cloud (height > 1)
        if cloud_msg.height > 1:
            return self.process_organized_point_cloud(cloud_msg)
        
        # Get field offsets
        field_names = [field.name for field in cloud_msg.fields]
        cloud_data = list(struct.iter_unpack('fff', cloud_msg.data))
        
        # Convert to numpy array
        points = np.array(cloud_data)
        
        # Remove invalid points (NaN, Inf)
        valid_mask = np.all(np.isfinite(points), axis=1)
        return points[valid_mask]
    
    def process_organized_point_cloud(self, cloud_msg: PointCloud2) -> np.ndarray:
        """
        Process organized point cloud (each point maps to an image pixel)
        
        Args:
            cloud_msg: PointCloud2 message
            
        Returns:
            Numpy array of shape (N, 3) containing points
        """
        width = cloud_msg.width
        height = cloud_msg.height
        field_names = [field.name for field in cloud_msg.fields]
        
        point_step = cloud_msg.point_step
        row_step = cloud_msg.row_step
        
        # Get indices for x, y, z
        x_idx = field_names.index('x') if 'x' in field_names else 0
        y_idx = field_names.index('y') if 'y' in field_names else 1
        z_idx = field_names.index('z') if 'z' in field_names else 2
        
        # Initialize point array
        points = np.zeros((height * width, 3), dtype=np.float32)
        
        # Iterate through points
        point_count = 0
        for i in range(height):
            for j in range(width):
                idx = i * row_step + j * point_step
                
                # Extract x, y, z using point_step
                x = struct.unpack_from('f', cloud_msg.data, idx + x_idx * 4)[0]
                y = struct.unpack_from('f', cloud_msg.data, idx + y_idx * 4)[0]
                z = struct.unpack_from('f', cloud_msg.data, idx + z_idx * 4)[0]
                
                # Only add valid points
                if np.isfinite(x) and np.isfinite(y) and np.isfinite(z):
                    points[point_count] = [x, y, z]
                    point_count += 1
        
        # Return only valid points
        return points[:point_count]
    
    def remove_ground_plane(self, points: np.ndarray) -> np.ndarray:
        """
        Remove ground plane from point cloud using height threshold
        
        Args:
            points: Numpy array of shape (N, 3) containing points
            
        Returns:
            Numpy array of points with ground removed
        """
        # Simple height-based filtering (Z-axis is up)
        non_ground_mask = points[:, 2] > self.ground_threshold
        
        return points[non_ground_mask]
    
    def voxel_grid_downsample(self, points: np.ndarray) -> np.ndarray:
        """
        Downsample point cloud using voxel grid
        
        Args:
            points: Numpy array of shape (N, 3) containing points
            
        Returns:
            Downsampled point cloud
        """
        # Early return if no points
        if len(points) == 0:
            return np.array([])
        
        # Calculate voxel indices
        voxel_indices = np.floor(points / self.voxel_size).astype(int)
        
        # Create dictionary to store voxels
        voxel_dict = {}
        
        # Assign points to voxels
        for i, (point, voxel_index) in enumerate(zip(points, voxel_indices)):
            voxel_key = tuple(voxel_index)
            if voxel_key in voxel_dict:
                voxel_dict[voxel_key].append(i)
            else:
                voxel_dict[voxel_key] = [i]
        
        # Calculate centroids of voxels
        downsampled_points = np.zeros((len(voxel_dict), 3), dtype=np.float32)
        
        for i, indices in enumerate(voxel_dict.values()):
            downsampled_points[i] = np.mean(points[indices], axis=0)
        
        return downsampled_points
    
    def cluster_points(self, points: np.ndarray) -> List[np.ndarray]:
        """
        Cluster points to find obstacles using DBSCAN-like approach
        
        Args:
            points: Numpy array of shape (N, 3) containing points
            
        Returns:
            List of clusters, each a numpy array of points
        """
        # Early return if no points
        if len(points) == 0:
            return []
            
        # Initialize clusters
        clusters = []
        processed = np.zeros(len(points), dtype=bool)
        
        # Check each point
        for i in range(len(points)):
            if processed[i]:
                continue
            
            # Mark point as processed
            processed[i] = True
            
            # Find neighbors
            current_cluster = [i]
            self.region_growing(points, i, processed, current_cluster)
            
            # Add cluster if large enough
            if len(current_cluster) >= self.min_cluster_size:
                cluster_points = points[current_cluster]
                clusters.append(cluster_points)
        
        return clusters
    
    def region_growing(self, points: np.ndarray, index: int, processed: np.ndarray, 
                      current_cluster: List[int]):
        """
        Region growing algorithm to cluster nearby points - Iterative implementation
        to avoid stack overflow
        
        Args:
            points: Point cloud array
            index: Current point index
            processed: Boolean array indicating processed points
            current_cluster: Current cluster indices
        """
        # Use a queue for processing instead of recursion
        queue = [index]
        
        # Process points until queue is empty
        while queue:
            current_index = queue.pop(0)
            current_point = points[current_index]
            
            # Calculate distances to all points
            distances = np.linalg.norm(points - current_point, axis=1)
            
            # Find neighbors within threshold
            neighbors = np.where((distances < self.cluster_distance) & (~processed))[0]
            
            # Add neighbors to cluster and queue
            for neighbor in neighbors:
                processed[neighbor] = True
                current_cluster.append(neighbor)
                queue.append(neighbor)
    
    def convert_clusters_to_obstacles(self, clusters: List[np.ndarray]) -> List[Obstacle]:
        """
        Convert point clusters to obstacle objects
        
        Args:
            clusters: List of point clusters
            
        Returns:
            List of Obstacle objects
        """
        obstacles = []
        
        for cluster in clusters:
            # Calculate cluster centroid
            centroid = np.mean(cluster, axis=0)
            
            # Calculate approximate size using bounding box
            min_bounds = np.min(cluster, axis=0)
            max_bounds = np.max(cluster, axis=0)
            dimensions = max_bounds - min_bounds
            
            # Create obstacle object
            # X forward, Y left, Z up in camera frame
            obstacle = Obstacle(
                x=centroid[0],    # X position in camera frame
                y=centroid[1],    # Y position in camera frame
                z=centroid[2],    # Z position in camera frame
                radius=max(dimensions[0], dimensions[1]) / 2.0,  # Approximate radius
                height=dimensions[2]  # Height
            )
            
            obstacles.append(obstacle)
        
        return obstacles
    
    def track_obstacles(self, current_obstacles: List[Obstacle], time_delta: float) -> List[Obstacle]:
        """
        Track obstacles between frames to detect dynamic obstacles
        
        Args:
            current_obstacles: Current frame obstacles
            time_delta: Time since last frame in seconds
            
        Returns:
            Updated obstacles with tracking info
        """
        if not self.prev_obstacles or time_delta <= 0:
            return current_obstacles
        
        # For each current obstacle, find closest previous obstacle
        for current in current_obstacles:
            closest_prev = None
            min_dist = float('inf')
            
            for prev in self.prev_obstacles:
                dist = math.sqrt((current.x - prev.x)**2 + 
                                (current.y - prev.y)**2 + 
                                (current.z - prev.z)**2)
                
                if dist < min_dist and dist < 2.0 * current.radius:  # Match threshold
                    min_dist = dist
                    closest_prev = prev
            
            # If found matching obstacle, calculate velocity
            if closest_prev is not None:
                # Calculate velocity
                vx = (current.x - closest_prev.x) / time_delta
                vy = (current.y - closest_prev.y) / time_delta
                
                # Update obstacle with velocity
                current.velocity_x = vx
                current.velocity_y = vy
                
                # Mark as dynamic if velocity is significant
                speed = math.sqrt(vx**2 + vy**2)
                current.is_dynamic = speed > 0.2  # 0.2 m/s threshold
        
        return current_obstacles
    
    def create_obstacle_markers(self, obstacles: List[Obstacle], frame_id: str) -> MarkerArray:
        """
        Create markers for visualization
        
        Args:
            obstacles: List of obstacles
            frame_id: Frame ID for markers
            
        Returns:
            MarkerArray for visualization
        """
        marker_array = MarkerArray()
        
        for i, obstacle in enumerate(obstacles):
            # Create marker for obstacle
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            
            # Set position - explicitly convert to float to prevent type errors
            marker.pose.position.x = float(obstacle.x)
            marker.pose.position.y = float(obstacle.y)
            marker.pose.position.z = float(obstacle.z)
            
            # Set orientation (identity)
            marker.pose.orientation.w = 1.0
            
            # Set scale - explicitly convert to float to prevent type errors
            marker.scale.x = float(obstacle.radius * 2.0)
            marker.scale.y = float(obstacle.radius * 2.0)
            marker.scale.z = float(obstacle.height)
            
            # Set color based on dynamic/static
            if obstacle.is_dynamic:
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            else:
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            
            marker.color.a = 0.7  # Semi-transparent
            
            marker_array.markers.append(marker)
            
            # Add velocity arrow for dynamic obstacles
            if obstacle.is_dynamic:
                arrow = Marker()
                arrow.header.frame_id = frame_id
                arrow.id = i + 1000  # Different ID space
                arrow.type = Marker.ARROW
                arrow.action = Marker.ADD
                
                # Start point at obstacle - explicit float conversions
                arrow.points.append(Point(x=float(obstacle.x), y=float(obstacle.y), z=float(obstacle.z)))
                
                # End point showing velocity (scaled)
                vel_scale = 1.0  # Scale factor for visualization
                arrow.points.append(Point(
                    x=float(obstacle.x + obstacle.velocity_x * vel_scale),
                    y=float(obstacle.y + obstacle.velocity_y * vel_scale),
                    z=float(obstacle.z)
                ))
                
                # Set scale (arrow dimensions)
                arrow.scale.x = 0.1  # Shaft diameter
                arrow.scale.y = 0.2  # Head diameter
                arrow.scale.z = 0.3  # Head length
                
                # Set color (blue for velocity)
                arrow.color.r = 0.0
                arrow.color.g = 0.0
                arrow.color.b = 1.0
                arrow.color.a = 1.0
                
                marker_array.markers.append(arrow)
        
        return marker_array

# Main pointcloud processor node
class ZedPointCloudProcessorNode(Node):
    """
    ROS2 node for processing ZED point cloud data
    """
    def __init__(self):
        super().__init__('zed_point_cloud_processor')
        
        # Declare parameters
        self.declare_parameter('point_cloud_topic', '/zed2i/zed_node/point_cloud/cloud_registered')
        self.declare_parameter('markers_topic', '/obstacles/markers')
        self.declare_parameter('obstacles_topic', '/obstacles')
        self.declare_parameter('robot_width', 0.82)
        self.declare_parameter('robot_length', 1.07)
        self.declare_parameter('robot_height', 0.68)
        self.declare_parameter('safety_margin', 0.5)
        self.declare_parameter('ground_threshold', 0.15)
        self.declare_parameter('cluster_distance', 0.3)
        self.declare_parameter('min_cluster_size', 10)
        self.declare_parameter('voxel_size', 0.1)
        
        # Get parameters
        self.point_cloud_topic = self.get_parameter('point_cloud_topic').value
        self.markers_topic = self.get_parameter('markers_topic').value
        self.obstacles_topic = self.get_parameter('obstacles_topic').value
        robot_width = self.get_parameter('robot_width').value
        robot_length = self.get_parameter('robot_length').value
        robot_height = self.get_parameter('robot_height').value
        safety_margin = self.get_parameter('safety_margin').value
        ground_threshold = self.get_parameter('ground_threshold').value
        cluster_distance = self.get_parameter('cluster_distance').value
        min_cluster_size = self.get_parameter('min_cluster_size').value
        voxel_size = self.get_parameter('voxel_size').value
        
        # Initialize point cloud processor
        self.processor = ZedPointCloudProcessor(
            robot_width=robot_width,
            robot_length=robot_length,
            robot_height=robot_height,
            safety_margin=safety_margin,
            ground_threshold=ground_threshold,
            cluster_distance=cluster_distance,
            min_cluster_size=min_cluster_size,
            voxel_size=voxel_size
        )
        
        # Create publishers
        self.markers_publisher = self.create_publisher(
            MarkerArray,
            self.markers_topic,
            10
        )
        
        # Create subscribers
        self.point_cloud_subscription = self.create_subscription(
            PointCloud2,
            self.point_cloud_topic,
            self.point_cloud_callback,
            10
        )
        
        self.get_logger().info('ZED Point Cloud Processor node initialized')
    
    def point_cloud_callback(self, msg: PointCloud2):
        """
        Callback for point cloud messages
        
        Args:
            msg: PointCloud2 message
        """
        # Process point cloud to detect obstacles
        timestamp = self.get_clock().now()
        obstacles = self.processor.process_point_cloud(msg, timestamp)
        
        # Create and publish visualization markers
        markers = self.processor.create_obstacle_markers(obstacles, msg.header.frame_id)
        self.markers_publisher.publish(markers)
        
        self.get_logger().debug(f'Detected {len(obstacles)} obstacles')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ZedPointCloudProcessorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
