#!/usr/bin/env python3

import os
import sys
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point, Quaternion
import numpy as np
import cv2
from cv_bridge import CvBridge
import tf2_ros
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.time import Time
import time

class MapPublisher(Node):
    """
    A node that publishes a dynamic map to ensure RViz can display it properly.
    This helps debug TF and map display issues and supports dynamic resizing.
    """
    
    def __init__(self):
        super().__init__('map_publisher')
        
        # Declare parameters
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('initial_width', 150)  # cells
        self.declare_parameter('initial_height', 150) # cells
        self.declare_parameter('initial_origin_x', -5.0)
        self.declare_parameter('initial_origin_y', -5.0)
        self.declare_parameter('publish_rate', 1.0)  # Hz
        self.declare_parameter('publish_map_topic', '/local_costmap')
        self.declare_parameter('publish_global_topic', '/global_costmap/costmap')
        self.declare_parameter('dynamic_resize', True)
        self.declare_parameter('resize_check_interval', 5) # iterations
        self.declare_parameter('min_free_border', 15) # cells
        self.declare_parameter('resize_step', 50) # cells to add when resizing
        
        # Get parameters
        self.resolution = self.get_parameter('resolution').value
        self.width = self.get_parameter('initial_width').value
        self.height = self.get_parameter('initial_height').value
        self.origin_x = self.get_parameter('initial_origin_x').value
        self.origin_y = self.get_parameter('initial_origin_y').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.map_topic = self.get_parameter('publish_map_topic').value
        self.global_topic = self.get_parameter('publish_global_topic').value
        self.dynamic_resize = self.get_parameter('dynamic_resize').value
        self.resize_check_interval = self.get_parameter('resize_check_interval').value
        self.min_free_border = self.get_parameter('min_free_border').value
        self.resize_step = self.get_parameter('resize_step').value
        
        # Dynamic map data
        self.data = np.zeros((self.height, self.width), dtype=np.int8)
        self.last_robot_position = (0.0, 0.0)
        self.occupied_min_x = float('inf')
        self.occupied_min_y = float('inf')
        self.occupied_max_x = float('-inf')
        self.occupied_max_y = float('-inf')
        
        # Create publishers for both maps and their updates
        self.map_publisher = self.create_publisher(OccupancyGrid, self.map_topic, 10)
        self.global_publisher = self.create_publisher(OccupancyGrid, self.global_topic, 10)
        
        # Also create publishers for map updates
        self.map_update_publisher = self.create_publisher(OccupancyGrid, self.map_topic + '_updates', 10)
        self.global_update_publisher = self.create_publisher(OccupancyGrid, self.global_topic + '_updates', 10)
        
        # Initialize TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Create a timer for publishing
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_maps)
        
        # Track publish count for logging
        self.publish_count = 0
        
        # Log node start
        self.get_logger().info(
            f"Map Publisher started. Publishing to {self.map_topic} and {self.global_topic} " +
            f"at {self.publish_rate} Hz with resolution {self.resolution}m"
        )
        
        # Ensure map frame exists by publishing a static transform if needed
        self.tf_static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.publish_static_transforms()
    
    def publish_static_transforms(self):
        """Publish static transforms to ensure all required frames exist"""
        from geometry_msgs.msg import TransformStamped
        
        # Ensure map->odom transform exists
        map_to_odom = TransformStamped()
        map_to_odom.header.stamp = self.get_clock().now().to_msg()
        map_to_odom.header.frame_id = 'map'
        map_to_odom.child_frame_id = 'odom'
        map_to_odom.transform.translation.x = 0.0
        map_to_odom.transform.translation.y = 0.0
        map_to_odom.transform.translation.z = 0.0
        map_to_odom.transform.rotation.x = 0.0
        map_to_odom.transform.rotation.y = 0.0
        map_to_odom.transform.rotation.z = 0.0
        map_to_odom.transform.rotation.w = 1.0
        
        # Publish static transform
        self.tf_static_broadcaster.sendTransform(map_to_odom)
        self.get_logger().info("Published static transform: map -> odom")
    
    def update_map_data(self):
        """Update the map with dynamic data based on robot position and environment"""
        try:
            # Try to get robot position from TF
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                Time(seconds=0),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y
            
            # Convert robot position to grid coordinates
            grid_x = int((robot_x - self.origin_x) / self.resolution)
            grid_y = int((robot_y - self.origin_y) / self.resolution)
            
            # Check if robot is within grid bounds
            if 0 <= grid_x < self.width and 0 <= grid_y < self.height:
                # Mark area around robot as free
                clear_radius = 10  # cells
                for i in range(max(0, grid_y - clear_radius), min(self.height, grid_y + clear_radius)):
                    for j in range(max(0, grid_x - clear_radius), min(self.width, grid_x + clear_radius)):
                        dist = np.sqrt((i - grid_y)**2 + (j - grid_x)**2)
                        if dist < clear_radius:
                            self.data[i, j] = 0  # Free
                
                # In real mode, we don't need to create test obstacles
                # Just update the occupied bounds for any existing obstacles
                for y in range(self.height):
                    for x in range(self.width):
                        if self.data[y, x] == 100:  # occupied cell
                            self.occupied_min_x = min(self.occupied_min_x, x)
                            self.occupied_min_y = min(self.occupied_min_y, y)
                            self.occupied_max_x = max(self.occupied_max_x, x)
                            self.occupied_max_y = max(self.occupied_max_y, y)
                
                # Store robot position for move detection
                self.last_robot_position = (robot_x, robot_y)
                
                # Check if map needs resizing
                if self.dynamic_resize and self.publish_count % self.resize_check_interval == 0:
                    self.check_and_resize_map(grid_x, grid_y)
            
        except TransformException as e:
            self.get_logger().warning(f"Could not get transform from map to base_link: {e}")
            # If we can't get the transform, just generate random obstacles
            self.generate_random_obstacles()
    
    def check_and_resize_map(self, robot_grid_x, robot_grid_y):
        """Check if map needs resizing and resize if necessary"""
        # Skip if we haven't detected any obstacles yet
        if (self.occupied_min_x == float('inf') or 
            self.occupied_min_y == float('inf') or 
            self.occupied_max_x == float('-inf') or 
            self.occupied_max_y == float('-inf')):
            return False
        
        # Check free borders around obstacles and robot
        need_resize = False
        new_width = self.width
        new_height = self.height
        new_origin_x = self.origin_x
        new_origin_y = self.origin_y
        
        # Check if we need to expand left
        left_border = self.occupied_min_x
        if left_border < self.min_free_border:
            need_resize = True
            new_width += self.resize_step
            new_origin_x -= self.resize_step * self.resolution
        
        # Check if we need to expand right
        right_border = self.width - self.occupied_max_x
        if right_border < self.min_free_border:
            need_resize = True
            new_width += self.resize_step
        
        # Check if we need to expand top
        top_border = self.occupied_min_y
        if top_border < self.min_free_border:
            need_resize = True
            new_height += self.resize_step
            new_origin_y -= self.resize_step * self.resolution
        
        # Check if we need to expand bottom
        bottom_border = self.height - self.occupied_max_y
        if bottom_border < self.min_free_border:
            need_resize = True
            new_height += self.resize_step
        
        # Also check if robot is getting close to the edge
        if robot_grid_x < self.min_free_border:
            need_resize = True
            new_width += self.resize_step
            new_origin_x -= self.resize_step * self.resolution
        elif robot_grid_x > self.width - self.min_free_border:
            need_resize = True
            new_width += self.resize_step
        
        if robot_grid_y < self.min_free_border:
            need_resize = True
            new_height += self.resize_step
            new_origin_y -= self.resize_step * self.resolution
        elif robot_grid_y > self.height - self.min_free_border:
            need_resize = True
            new_height += self.resize_step
        
        # If we need to resize, create new data array and copy old data
        if need_resize:
            self.get_logger().info(f"Resizing map from {self.width}x{self.height} to {new_width}x{new_height}")
            
            # Create new data array
            new_data = np.zeros((new_height, new_width), dtype=np.int8)
            
            # Calculate offsets for copying data
            offset_x = 0
            offset_y = 0
            
            # If origin shifted, calculate copy offset
            if new_origin_x < self.origin_x:
                offset_x = int((self.origin_x - new_origin_x) / self.resolution)
            
            if new_origin_y < self.origin_y:
                offset_y = int((self.origin_y - new_origin_y) / self.resolution)
            
            # Copy old data to new array
            for y in range(self.height):
                for x in range(self.width):
                    if 0 <= y+offset_y < new_height and 0 <= x+offset_x < new_width:
                        new_data[y+offset_y, x+offset_x] = self.data[y, x]
            
            # Update map properties
            self.data = new_data
            self.width = new_width
            self.height = new_height
            self.origin_x = new_origin_x
            self.origin_y = new_origin_y
            
            # Update obstacle bounds after resize
            if offset_x > 0:
                self.occupied_min_x += offset_x
                self.occupied_max_x += offset_x
            
            if offset_y > 0:
                self.occupied_min_y += offset_y
                self.occupied_max_y += offset_y
            
            return True
        
        return False
    
    def generate_random_obstacles(self):
        """No random obstacles in real mode"""
        # In real mode, we don't generate random obstacles
        # This is just kept as a placeholder for the test mode
        pass
    
    def create_map_message(self):
        """Create a ROS map message from the current data"""
        # Create costmap message
        map_msg = OccupancyGrid()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = 'map'
        map_msg.info.resolution = self.resolution
        map_msg.info.width = self.width
        map_msg.info.height = self.height
        map_msg.info.origin.position.x = self.origin_x
        map_msg.info.origin.position.y = self.origin_y
        map_msg.info.origin.position.z = 0.0
        map_msg.info.origin.orientation.w = 1.0
        
        # Convert to 1D array in row-major order
        map_msg.data = self.data.flatten().tolist()
        
        return map_msg
    
    def publish_maps(self):
        """Update and publish the maps"""
        # Update map data based on robot position and environment
        self.update_map_data()
        
        # Create map message
        map_msg = self.create_map_message()
        
        # Create a separate copy for updates (identical to main map in this case)
        update_msg = OccupancyGrid()
        update_msg.header = map_msg.header
        update_msg.info = map_msg.info
        update_msg.data = map_msg.data
        
        # Publish to all topics - main maps and updates
        self.map_publisher.publish(map_msg)
        self.global_publisher.publish(map_msg)
        self.map_update_publisher.publish(update_msg)
        self.global_update_publisher.publish(update_msg)
        
        # Log periodically
        self.publish_count += 1
        if self.publish_count % 10 == 0:
            self.get_logger().info(f"Published map #{self.publish_count} size: {self.width}x{self.height}")
        
        # Also save map to a file periodically
        if self.publish_count % 30 == 0:
            self.save_map_to_file(map_msg)
    
    def save_map_to_file(self, map_msg):
        """Save the current map to a PGM file"""
        try:
            # Create maps directory if it doesn't exist
            maps_dir = os.path.expanduser('~/maps')
            os.makedirs(maps_dir, exist_ok=True)
            
            # Convert the 1D data back to 2D grid
            grid_data = np.array(map_msg.data).reshape(map_msg.info.height, map_msg.info.width)
            
            # Convert the data to an image (0-255)
            # -1 (unknown) -> 205 (gray), 0 (free) -> 254 (near white), 100 (occupied) -> 0 (black)
            img = np.zeros((map_msg.info.height, map_msg.info.width), dtype=np.uint8)
            img[grid_data == -1] = 205  # Unknown
            img[grid_data == 0] = 254   # Free
            img[grid_data == 100] = 0   # Occupied
            
            # Save the image
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            filename = os.path.join(maps_dir, f"rviz_debug_map_{timestamp}.pgm")
            cv2.imwrite(filename, img)
            
            self.get_logger().info(f"Saved map to {filename}")
            
        except Exception as e:
            self.get_logger().error(f"Error saving map to file: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MapPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Map Publisher shutting down")
    except Exception as e:
        node.get_logger().error(f"Unexpected error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
