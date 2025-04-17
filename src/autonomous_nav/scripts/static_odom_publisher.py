#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
import math

class StaticOdomPublisher(Node):
    def __init__(self):
        super().__init__('static_odom_publisher')
        self.get_logger().info('Static Odom Publisher started')
        
        # Create a static transform broadcaster
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        
        # Publish transform from odom to base_link
        self.publish_static_transform()
        
    def publish_static_transform(self):
        """
        Publish a static transform from odom to base_link
        """
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        
        # Initialize at origin with no rotation
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        
        # Set rotation (quaternion for identity rotation)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        
        # Send the transform
        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info('Published static transform from odom to base_link')

def main(args=None):
    rclpy.init(args=args)
    node = StaticOdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
