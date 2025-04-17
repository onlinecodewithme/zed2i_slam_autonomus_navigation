#!/usr/bin/env python3

import rclpy
from autonomous_nav.zed_point_cloud_processor import ZedPointCloudProcessorNode

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ZedPointCloudProcessorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
