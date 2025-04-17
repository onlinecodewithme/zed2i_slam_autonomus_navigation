#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import threading

class MapUpdatesPublisher(Node):
    """
    Node that subscribes to a map topic and publishes the updates to another topic
    This ensures RViz can properly display the map with updates.
    """
    
    def __init__(self):
        super().__init__('map_updates_publisher')
        
        # Declare parameters
        self.declare_parameter('map_topic', '/local_costmap')
        self.declare_parameter('update_topic', '/local_costmap_updates')
        self.declare_parameter('update_rate', 5.0)  # Hz
        
        # Get parameters
        self.map_topic = self.get_parameter('map_topic').value
        self.update_topic = self.get_parameter('update_topic').value
        self.update_rate = self.get_parameter('update_rate').value
        
        # Create updates publisher
        self.updates_publisher = self.create_publisher(
            OccupancyGrid,
            self.update_topic,
            10
        )
        
        # Create subscription to map
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            10
        )
        
        # Initialize variables
        self.latest_map = None
        self.map_lock = threading.Lock()
        
        # Create timer for publishing updates
        timer_period = 1.0 / self.update_rate
        self.timer = self.create_timer(timer_period, self.publish_updates)
        
        self.get_logger().info(
            f"Map updates publisher started. Republishing maps from {self.map_topic} as updates to {self.update_topic}"
        )
    
    def map_callback(self, msg):
        """
        Callback for map messages
        """
        with self.map_lock:
            self.latest_map = msg
    
    def publish_updates(self):
        """
        Publish the latest map as updates
        """
        with self.map_lock:
            if self.latest_map is not None:
                # Create a proper updates message
                update_msg = OccupancyGrid()
                
                # Copy header and set correct timestamp
                update_msg.header = self.latest_map.header
                update_msg.header.stamp = self.get_clock().now().to_msg()
                
                # Ensure the frame_id is set correctly
                if not update_msg.header.frame_id:
                    update_msg.header.frame_id = "map"
                
                # Copy map info
                update_msg.info = self.latest_map.info
                
                # Copy data
                update_msg.data = list(self.latest_map.data)
                
                # Publish to updates topic
                self.updates_publisher.publish(update_msg)
                self.get_logger().debug(f"Published map update with size {update_msg.info.width}x{update_msg.info.height}")

def main(args=None):
    rclpy.init(args=args)
    
    node = MapUpdatesPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Map updates publisher node shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
