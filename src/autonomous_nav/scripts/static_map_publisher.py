#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
import yaml
import cv2
import numpy as np
import argparse

class StaticMapPublisherNode(Node):
    """
    Node to load a costmap from file and publish it as a static map
    This enables saved costmaps to be used for autonomous navigation
    """
    def __init__(self, map_yaml_path):
        super().__init__('static_map_publisher')
        
        # Declare parameters
        self.declare_parameter('map_yaml_path', map_yaml_path)
        self.declare_parameter('publish_rate', 5.0)  # Hz
        self.declare_parameter('map_topic', '/map')
        
        # Get parameters
        self.map_yaml_path = self.get_parameter('map_yaml_path').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.map_topic = self.get_parameter('map_topic').value
        
        # Load map from file
        self.map_msg = self.load_map_from_yaml(self.map_yaml_path)
        
        if self.map_msg is None:
            self.get_logger().error(f'Failed to load map from {self.map_yaml_path}')
            return
        
        # Create publisher for map
        self.map_publisher = self.create_publisher(
            OccupancyGrid,
            self.map_topic,
            10  # QoS profile depth
        )
        
        # Create timer for regular publishing
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.publish_map)
        
        # Create service to provide map on request
        self.map_service = self.create_service(
            GetMap,
            'static_map',
            self.map_service_callback
        )
        
        self.get_logger().info(f'Static map publisher initialized with map: {self.map_yaml_path}')
        self.get_logger().info(f'Publishing on topic: {self.map_topic}')
    
    def load_map_from_yaml(self, yaml_path):
        """
        Load map data from a YAML file with accompanying PGM image
        """
        try:
            # Parse YAML file
            with open(yaml_path, 'r') as yaml_file:
                map_data = yaml.safe_load(yaml_file)
            
            # Get PGM path relative to YAML file
            pgm_path = os.path.join(os.path.dirname(yaml_path), map_data['image'])
            
            # Read PGM image
            image = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
            if image is None:
                self.get_logger().error(f'Failed to load image from {pgm_path}')
                return None
            
            # Flip image vertically to match ROS convention
            image = np.flipud(image)
            
            # Convert image to occupancy data:
            # 205 (gray/unknown) -> -1 (unknown)
            # 254 (white/free) -> 0 (free)
            # 0-254 (black to white) -> 100-0 (occupied to free)
            height, width = image.shape
            occupancy_data = np.zeros(width * height, dtype=np.int8)
            
            # Convert to flat array for OccupancyGrid message
            flat_image = image.flatten()
            
            # Set unknown cells (gray = 205) to -1
            unknown_mask = (flat_image == 205)
            occupancy_data[unknown_mask] = -1
            
            # Set free cells (white = 254) to 0
            free_mask = (flat_image == 254)
            occupancy_data[free_mask] = 0
            
            # Map other values from image grayscale to occupancy values
            # 0 (black) -> 100 (fully occupied)
            # 253 (almost white) -> 1 (barely occupied)
            for i in range(254):
                if i != 205 and i != 254:  # Skip unknown and free cells
                    mask = (flat_image == i)
                    # Linear mapping from 0-253 to 100-1
                    value = max(1, min(100, 100 - int(99 * i / 253.0)))
                    occupancy_data[mask] = value
            
            # Create OccupancyGrid message
            map_msg = OccupancyGrid()
            
            # Set header
            map_msg.header.frame_id = 'map'
            map_msg.header.stamp = self.get_clock().now().to_msg()
            
            # Set map info
            map_msg.info.resolution = map_data['resolution']
            map_msg.info.width = width
            map_msg.info.height = height
            map_msg.info.origin.position.x = map_data['origin'][0]
            map_msg.info.origin.position.y = map_data['origin'][1]
            map_msg.info.origin.position.z = map_data['origin'][2] if len(map_data['origin']) > 2 else 0.0
            
            # Set map data
            map_msg.data = occupancy_data.tolist()
            
            self.get_logger().info(f'Loaded map {pgm_path} with dimensions {width}x{height}')
            
            return map_msg
        
        except Exception as e:
            self.get_logger().error(f'Error loading map: {e}')
            return None
    
    def publish_map(self):
        """
        Publish the static map
        """
        if self.map_msg is not None:
            # Update timestamp
            self.map_msg.header.stamp = self.get_clock().now().to_msg()
            
            # Publish
            self.map_publisher.publish(self.map_msg)
            self.get_logger().debug('Published static map')
    
    def map_service_callback(self, request, response):
        """
        Service callback to provide the map on request
        """
        if self.map_msg is not None:
            response.map = self.map_msg
            return response
        else:
            self.get_logger().error('Map not available for service request')
            return None


def main(args=None):
    parser = argparse.ArgumentParser(description='Static Map Publisher')
    parser.add_argument('map_yaml_path', type=str, 
                        help='Path to the YAML file describing the map')
    parser.add_argument('--rate', type=float, default=5.0,
                        help='Rate at which to publish the map (Hz)')
    
    # Parse command line arguments first, then ROS arguments
    parser_args, remaining_args = parser.parse_known_args(args)
    
    rclpy.init(args=remaining_args)
    
    node = StaticMapPublisherNode(parser_args.map_yaml_path)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
