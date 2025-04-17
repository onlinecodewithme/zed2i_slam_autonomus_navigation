#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import threading

class GlobalCostmapRelay(Node):
    """
    Node that relays local costmap messages to the global costmap topic
    This ensures NAV2 can use the same costmap for both local and global planning
    """
    
    def __init__(self):
        super().__init__('global_costmap_relay')
        
        # Declare parameters
        self.declare_parameter('local_costmap_topic', '/local_costmap')
        self.declare_parameter('global_costmap_topic', '/global_costmap/costmap')
        self.declare_parameter('update_rate', 5.0)  # Hz
        
        # Get parameters
        self.local_costmap_topic = self.get_parameter('local_costmap_topic').value
        self.global_costmap_topic = self.get_parameter('global_costmap_topic').value
        self.update_rate = self.get_parameter('update_rate').value
        
        # Create publishers
        self.global_costmap_pub = self.create_publisher(
            OccupancyGrid,
            self.global_costmap_topic,
            10
        )
        
        # Also create a publisher for the updates topic
        self.global_costmap_updates_pub = self.create_publisher(
            OccupancyGrid,
            self.global_costmap_topic + '_updates',
            10
        )
        
        # Create subscription to local costmap
        self.local_costmap_sub = self.create_subscription(
            OccupancyGrid,
            self.local_costmap_topic,
            self.local_costmap_callback,
            10
        )
        
        # Initialize variables
        self.latest_costmap = None
        self.costmap_lock = threading.Lock()
        
        # Create timer for publishing global costmap at a regular rate
        timer_period = 1.0 / self.update_rate
        self.timer = self.create_timer(timer_period, self.publish_global_costmap)
        
        self.get_logger().info(
            f"Global costmap relay started. Relaying from {self.local_costmap_topic} to {self.global_costmap_topic}"
        )
    
    def local_costmap_callback(self, msg):
        """
        Callback for local costmap messages
        """
        with self.costmap_lock:
            self.latest_costmap = msg
    
    def publish_global_costmap(self):
        """
        Publish the latest costmap as global costmap
        """
        with self.costmap_lock:
            if self.latest_costmap is not None:
                # Create a proper global costmap message
                global_costmap = OccupancyGrid()
                
                # Set header with fresh timestamp
                global_costmap.header = self.latest_costmap.header
                global_costmap.header.stamp = self.get_clock().now().to_msg()
                global_costmap.header.frame_id = 'map'  # Ensure frame_id is 'map'
                
                # Copy map info but ensure frames are correct
                global_costmap.info = self.latest_costmap.info
                global_costmap.info.map_load_time = self.get_clock().now().to_msg()
                
                # Make a clean copy of the data
                global_costmap.data = list(self.latest_costmap.data)
                
                # Publish to global costmap topic
                self.global_costmap_pub.publish(global_costmap)
                
                # Create separate updates message with incrementing timestamp
                updates_msg = OccupancyGrid()
                updates_msg.header = global_costmap.header
                updates_msg.header.stamp = self.get_clock().now().to_msg()
                updates_msg.info = global_costmap.info
                updates_msg.data = global_costmap.data
                
                # Publish to updates topic
                self.global_costmap_updates_pub.publish(updates_msg)
                
                self.get_logger().debug("Published global costmap and updates")


def main(args=None):
    rclpy.init(args=args)
    
    node = GlobalCostmapRelay()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Global costmap relay node shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
