#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

class GlobalCostmapRelay(Node):
    """
    Relay node that republishes the local costmap as the global costmap
    This enables compatibility with the autonomous airplane inspector
    """
    def __init__(self):
        super().__init__('global_costmap_relay')
        
        # Declare parameters
        self.declare_parameter(
            'input_topic', 
            '/local_costmap',
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description='Topic to subscribe to for costmap data'
            )
        )
        
        self.declare_parameter(
            'output_topic', 
            '/global_costmap/costmap',
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description='Topic to republish the costmap data to'
            )
        )
        
        # Get parameters
        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        
        # Create publisher and subscriber
        self.publisher = self.create_publisher(
            OccupancyGrid, 
            self.output_topic, 
            10
        )
        
        self.subscription = self.create_subscription(
            OccupancyGrid,
            self.input_topic,
            self.costmap_callback,
            10
        )
        
        self.get_logger().info(f'Global costmap relay initialized')
        self.get_logger().info(f'Relaying messages from {self.input_topic} to {self.output_topic}')
    
    def costmap_callback(self, msg):
        """Republish the received costmap message"""
        # Update the frame_id for compatibility
        msg.header.frame_id = 'map'
        
        # Debug output occasionally
        if (msg.header.stamp.sec % 10) == 0:
            self.get_logger().info(f'Relaying costmap: {msg.info.width}x{msg.info.height} cells')
        
        # Republish the message
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = GlobalCostmapRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
