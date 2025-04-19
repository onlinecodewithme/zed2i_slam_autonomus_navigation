#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
import matplotlib.pyplot as plt
import datetime

class CostmapDebugNode(Node):
    """
    Utility node to monitor and debug costmap messages
    """
    def __init__(self):
        super().__init__('costmap_debug_node')
        
        # Declare parameters
        self.declare_parameter('costmap_topics', ['/local_costmap', '/global_costmap'])
        self.declare_parameter('save_images', False)
        self.declare_parameter('output_dir', '/tmp/costmap_debug')
        
        # Get parameters
        self.costmap_topics = self.get_parameter('costmap_topics').value
        self.save_images = self.get_parameter('save_images').value
        self.output_dir = self.get_parameter('output_dir').value
        
        # Create subscribers
        self.subscribers = {}
        for topic in self.costmap_topics:
            self.subscribers[topic] = self.create_subscription(
                OccupancyGrid,
                topic,
                lambda msg, topic=topic: self.costmap_callback(msg, topic),
                10
            )
        
        self.get_logger().info(f"Costmap debug node started. Monitoring topics: {self.costmap_topics}")
        
        # Stats for each topic
        self.stats = {topic: {'count': 0, 'last_size': None, 'occupied_cells': 0} for topic in self.costmap_topics}
    
    def costmap_callback(self, msg, topic):
        """Process a costmap message"""
        # Get statistics
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y
        
        # Convert to 2D array
        costmap_data = np.array(msg.data).reshape(height, width)
        
        # Calculate statistics
        occupied_cells = np.sum(costmap_data > 0)
        free_cells = np.sum(costmap_data == 0)
        unknown_cells = np.sum(costmap_data < 0)
        total_cells = width * height
        
        # Update stats
        self.stats[topic]['count'] += 1
        self.stats[topic]['last_size'] = (width, height)
        self.stats[topic]['occupied_cells'] = occupied_cells

        # Only log every 10th message to reduce spam
        if self.stats[topic]['count'] % 10 == 0:
            self.get_logger().info(f"Topic {topic} - Message #{self.stats[topic]['count']}:")
            self.get_logger().info(f"  Size: {width}x{height} cells ({width*resolution:.1f}x{height*resolution:.1f} meters)")
            self.get_logger().info(f"  Origin: ({origin_x:.2f}, {origin_y:.2f})")
            self.get_logger().info(f"  Occupied cells: {occupied_cells}/{total_cells} ({occupied_cells/total_cells*100:.1f}%)")
            self.get_logger().info(f"  Free cells: {free_cells}/{total_cells} ({free_cells/total_cells*100:.1f}%)")
            self.get_logger().info(f"  Unknown cells: {unknown_cells}/{total_cells} ({unknown_cells/total_cells*100:.1f}%)")
        
        # Save image if requested
        if self.save_images and self.stats[topic]['count'] % 30 == 0:
            self.save_costmap_image(costmap_data, topic)
    
    def save_costmap_image(self, costmap_data, topic):
        """Save the costmap as an image"""
        import os
        from matplotlib import colors
        
        # Create output directory if it doesn't exist
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Create colormap: white for free space, black for unknown, red for occupied
        cmap = colors.ListedColormap(['white', 'red'])
        norm = colors.BoundaryNorm([0, 1, 101], cmap.N)
        
        # Create the figure
        plt.figure(figsize=(10, 10))
        plt.imshow(costmap_data, cmap=cmap, norm=norm, interpolation='none')
        plt.colorbar(label='Cost')
        plt.title(f'Costmap from {topic} - {datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")}')
        
        # Save the figure
        topic_name = topic.replace('/', '_')
        filename = f"{self.output_dir}/{topic_name}_{self.stats[topic]['count']}.png"
        plt.savefig(filename)
        plt.close()
        
        self.get_logger().info(f"Saved costmap image to {filename}")

def main(args=None):
    rclpy.init(args=args)
    node = CostmapDebugNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
