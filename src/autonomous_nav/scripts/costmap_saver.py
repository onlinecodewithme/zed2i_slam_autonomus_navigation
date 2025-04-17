#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.srv import SaveMap
import yaml
import cv2
import numpy as np
from datetime import datetime

class CostmapSaverNode(Node):
    """
    Node to save the 2D costmap from the /local_costmap topic to a file
    """
    def __init__(self):
        super().__init__('costmap_saver')
        
        # Declare parameters
        self.declare_parameter('costmap_topic', '/local_costmap')
        self.declare_parameter('output_dir', os.path.expanduser('~/maps'))
        self.declare_parameter('save_frequency', 0.0)  # Hz, 0 means save once only
        self.declare_parameter('map_name', 'costmap')
        
        # Get parameters
        self.costmap_topic = self.get_parameter('costmap_topic').value
        self.output_dir = self.get_parameter('output_dir').value
        self.save_frequency = self.get_parameter('save_frequency').value
        self.map_name = self.get_parameter('map_name').value
        
        # Ensure output directory exists
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Initialize variables
        self.latest_costmap = None
        self.save_count = 0
        self.last_save_time = self.get_clock().now()
        
        # Create subscription to costmap
        self.costmap_subscription = self.create_subscription(
            OccupancyGrid,
            self.costmap_topic,
            self.costmap_callback,
            10
        )
        
        # If save_frequency is > 0, create a timer
        if self.save_frequency > 0.0:
            period = 1.0 / self.save_frequency
            self.timer = self.create_timer(period, self.timer_callback)
            self.get_logger().info(f'Will save costmap every {period:.2f} seconds')
        else:
            self.get_logger().info('Will save costmap once when received')
        
        self.get_logger().info('Costmap saver node initialized')
    
    def costmap_callback(self, msg):
        """
        Callback for costmap messages
        """
        self.latest_costmap = msg
        
        # If save_frequency is 0, save immediately when received
        if self.save_frequency <= 0.0 and self.save_count == 0:
            self.save_costmap()
    
    def timer_callback(self):
        """
        Timer callback to periodically save the costmap
        """
        if self.latest_costmap is not None:
            self.save_costmap()
    
    def save_costmap(self):
        """
        Save the latest costmap to a file
        """
        if self.latest_costmap is None:
            self.get_logger().warn('No costmap received yet')
            return
        
        # Create filename with timestamp
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        if self.save_frequency > 0.0:
            # Include count in filename if saving periodically
            filename_base = f'{self.map_name}_{timestamp}_{self.save_count:03d}'
        else:
            filename_base = f'{self.map_name}_{timestamp}'
        
        pgm_path = os.path.join(self.output_dir, f'{filename_base}.pgm')
        yaml_path = os.path.join(self.output_dir, f'{filename_base}.yaml')
        
        # Convert OccupancyGrid to image
        self.occupancy_grid_to_image(self.latest_costmap, pgm_path)
        
        # Create YAML file
        self.create_map_yaml(self.latest_costmap, pgm_path, yaml_path)
        
        self.get_logger().info(f'Saved costmap to: {yaml_path}')
        self.save_count += 1
    
    def occupancy_grid_to_image(self, occupancy_grid, pgm_path):
        """
        Convert OccupancyGrid to PGM image file
        """
        # Get dimensions
        width = occupancy_grid.info.width
        height = occupancy_grid.info.height
        
        # Create empty array for the image (0-255)
        # In PGM: 0 = black (occupied), 255 = white (free)
        grid_data = np.array(occupancy_grid.data, dtype=np.int8).reshape(height, width)
        
        # Convert from costmap values to image values:
        # -1 (unknown) -> 205 (gray)
        # 0 (free) -> 254 (white)
        # 1-100 (increasingly occupied) -> 253-0 (nearly white to black)
        image_data = np.zeros((height, width), dtype=np.uint8)
        
        # Set free cells (0) to white (254)
        free_mask = (grid_data == 0)
        image_data[free_mask] = 254
        
        # Set occupied cells (1-100) to black with gradient (253-0)
        for i in range(1, 101):
            mask = (grid_data == i)
            value = max(0, 254 - int(254 * i / 100.0))  # Linear mapping from 1-100 to 253-0
            image_data[mask] = value
        
        # Set unknown cells (-1) to gray (205)
        unknown_mask = (grid_data == -1)
        image_data[unknown_mask] = 205
        
        # Flip the image vertically to match ROS convention
        image_data = np.flipud(image_data)
        
        # Save as PGM
        # PGM P5 format (binary)
        with open(pgm_path, 'wb') as f:
            f.write(f'P5\n{width} {height}\n255\n'.encode())
            f.write(image_data.tobytes())
    
    def create_map_yaml(self, occupancy_grid, pgm_path, yaml_path):
        """
        Create YAML file for the map
        """
        # Get map info
        resolution = occupancy_grid.info.resolution
        origin_x = occupancy_grid.info.origin.position.x
        origin_y = occupancy_grid.info.origin.position.y
        
        # Create YAML data
        yaml_data = {
            'image': os.path.basename(pgm_path),
            'resolution': resolution,
            'origin': [origin_x, origin_y, 0.0],
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.196,
            'mode': 'scale'
        }
        
        # Write YAML file
        with open(yaml_path, 'w') as f:
            yaml.dump(yaml_data, f, default_flow_style=False)


def main(args=None):
    rclpy.init(args=args)
    
    node = CostmapSaverNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
