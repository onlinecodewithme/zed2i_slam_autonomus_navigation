#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from cv_bridge import CvBridge, CvBridgeError
import tf2_ros
import numpy as np
import threading
import time

# Import our modular costmap components
from autonomous_nav.costmap.costmap_generator import LocalCostmapGenerator

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
        
        # Initialize TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Initialize instance variables
        self.depth_image = None
        self.camera_info = None
        self.latest_pointcloud = None
        self.last_update_time = self.get_clock().now()
        
        # Initialize update counter for periodic logging
        self.update_counter = 0
        
        # Processing lock to prevent concurrent processing
        self.processing_lock = threading.Lock()
        
        # Create costmap generator instance
        self.costmap_generator = LocalCostmapGenerator(
            node=self,
            use_pointcloud=self.use_pointcloud,
            costmap_resolution=self.costmap_resolution,
            costmap_width=self.initial_costmap_width,
            costmap_height=self.initial_costmap_height,
            costmap_origin_x=self.initial_costmap_origin_x,
            costmap_origin_y=self.initial_costmap_origin_y,
            min_height=self.min_height,
            max_height=self.max_height,
            inflation_radius=self.inflation_radius,
            map_growth_factor=self.map_growth_factor
        )
        
        # Create QoS profile for sensor data
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
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
        
        # Create timer for costmap update
        update_period = 1.0 / self.update_rate
        self.update_timer = self.create_timer(update_period, self.update_costmap)
        
        self.get_logger().info('Dynamic local costmap generator node initialized')
    
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
        self.update_counter += 1
        # Only log occasionally to avoid spamming
        if self.update_counter % 100 == 0:
            self.get_logger().info('Received point cloud message from ZED camera')
    
    def update_costmap(self):
        """Update the costmap from sensor data"""
        # Increment update counter for logging
        self.update_counter += 1
        
        # Only log issues occasionally to prevent log spam
        log_this_update = (self.update_counter % 100 == 0)
        
        # Check what data is missing and log it
        if self.use_pointcloud and self.latest_pointcloud is None:
            if log_this_update:
                self.get_logger().warning("Missing pointcloud data from ZED camera")
        if not self.use_pointcloud and self.depth_image is None:
            if log_this_update:
                self.get_logger().warning("Missing depth image from ZED camera")
        if self.camera_info is None:
            if log_this_update:
                self.get_logger().warning("Missing camera info from ZED camera")
        
        # Always publish the costmap, even if empty, to ensure RViz can display it
        self.publish_costmap()
        
        # Skip processing if essential data is missing
        if (self.depth_image is None and not self.use_pointcloud) or \
           (self.use_pointcloud and self.latest_pointcloud is None) or \
           self.camera_info is None:
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
                self.costmap_generator.process_pointcloud(
                    pointcloud=self.latest_pointcloud,
                    tf_buffer=self.tf_buffer,
                    update_counter=self.update_counter
                )
            else:
                self.costmap_generator.process_depth_image(
                    depth_image=self.depth_image,
                    camera_info=self.camera_info,
                    tf_buffer=self.tf_buffer,
                    update_counter=self.update_counter
                )
            
            # Apply inflation to obstacles periodically
            if int(current_time.nanoseconds / 1e9) % 3 == 0:
                self.costmap_generator.inflate_obstacles()
            
            # Update last update time
            self.last_update_time = current_time
        finally:
            self.processing_lock.release()
    
    def publish_costmap(self):
        """Publish the occupancy grid as a costmap"""
        # Create a ROS OccupancyGrid message
        msg = self.costmap_generator.create_occupancy_grid_msg(
            stamp=self.get_clock().now().to_msg()
        )
        
        # Publish the costmap
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
