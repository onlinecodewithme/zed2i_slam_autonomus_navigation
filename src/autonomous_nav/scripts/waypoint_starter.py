#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
import threading
import time

class WaypointStarter(Node):
    """
    Simple node to automatically start waypoint navigation after a delay.
    Used for automated testing and demonstrations.
    """
    def __init__(self):
        super().__init__('waypoint_starter')
        
        # Declare parameters
        self.declare_parameter('start_delay', 10.0)  # seconds
        
        # Get parameters
        self.start_delay = self.get_parameter('start_delay').value
        
        # Publishers
        self.start_pub = self.create_publisher(Empty, '/waypoints/start', 10)
        
        # Start timer
        self.get_logger().info(f'Will start waypoint navigation in {self.start_delay} seconds')
        self.start_timer = threading.Timer(self.start_delay, self.start_waypoints)
        self.start_timer.daemon = True
        self.start_timer.start()
        
    def start_waypoints(self):
        """
        Publish a message to start waypoint navigation
        """
        start_msg = Empty()
        self.start_pub.publish(start_msg)
        self.get_logger().info('Started waypoint navigation')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        starter = WaypointStarter()
        rclpy.spin(starter)
    except KeyboardInterrupt:
        pass
    finally:
        if 'starter' in locals():
            starter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
