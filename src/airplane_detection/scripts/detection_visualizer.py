#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import time

from robot_interfaces.msg import AirplaneDetection

class DetectionVisualizerNode(Node):
    """
    ROS2 node for visualizing airplane detections
    """
    
    def __init__(self):
        super().__init__('detection_visualizer')
        
        # Declare parameters
        self.declare_parameter('image_topic', '/zed2i/zed_node/rgb/image_rect_color')
        self.declare_parameter('detection_topic', '/airplane_detection/detections')
        self.declare_parameter('display_window', True)
        self.declare_parameter('publish_visualization', True)
        
        # Get parameters
        self.image_topic = self.get_parameter('image_topic').value
        self.detection_topic = self.get_parameter('detection_topic').value
        self.display_window = self.get_parameter('display_window').value
        self.publish_visualization = self.get_parameter('publish_visualization').value
        
        # Create QoS profile
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Create subscribers
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            qos_profile
        )
        
        self.detection_sub = self.create_subscription(
            AirplaneDetection,
            self.detection_topic,
            self.detection_callback,
            10
        )
        
        # Publisher for visualization
        if self.publish_visualization:
            self.vis_pub = self.create_publisher(
                Image,
                '/airplane_detection/visualization',
                10
            )
        
        # Initialize instance variables
        self.current_image = None
        self.current_detections = []
        self.detection_timestamps = []
        self.detection_timeout = 1.0  # Remove detections after 1 second
        
        # Create timer for visualization
        self.vis_timer = self.create_timer(0.1, self.visualize_detections)
        
        self.get_logger().info('Detection visualizer node initialized')
    
    def image_callback(self, msg):
        """
        Callback for RGB images
        """
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f'CV Bridge error: {e}')
    
    def detection_callback(self, msg):
        """
        Callback for airplane detections
        """
        # Add timestamp to detection
        timestamp = time.time()
        
        # Store detection and timestamp
        self.current_detections.append(msg)
        self.detection_timestamps.append(timestamp)
    
    def visualize_detections(self):
        """
        Visualize all current detections on the image
        """
        if self.current_image is None:
            return
        
        # Create a copy of the image for visualization
        vis_image = self.current_image.copy()
        
        # Current time for timeout checks
        current_time = time.time()
        
        # Lists for updated detections and timestamps
        updated_detections = []
        updated_timestamps = []
        
        # Process each detection
        for detection, timestamp in zip(self.current_detections, self.detection_timestamps):
            # Check if detection is still valid (not timed out)
            if current_time - timestamp <= self.detection_timeout:
                # Keep this detection
                updated_detections.append(detection)
                updated_timestamps.append(timestamp)
                
                # Draw the detection
                self.draw_detection(vis_image, detection)
        
        # Update the lists
        self.current_detections = updated_detections
        self.detection_timestamps = updated_timestamps
        
        # Display the visualization if requested
        if self.display_window:
            cv2.imshow('Airplane Detections', vis_image)
            cv2.waitKey(1)
        
        # Publish visualization if requested
        if self.publish_visualization:
            try:
                vis_msg = self.bridge.cv2_to_imgmsg(vis_image, "bgr8")
                self.vis_pub.publish(vis_msg)
            except CvBridgeError as e:
                self.get_logger().error(f'CV Bridge error: {e}')
    
    def draw_detection(self, image, detection):
        """
        Draw an airplane detection on the image
        
        Args:
            image: OpenCV image to draw on
            detection: AirplaneDetection message
        """
        # Extract detection information
        x = detection.x
        y = detection.y
        width = detection.width
        height = detection.height
        confidence = detection.confidence
        distance = detection.distance
        class_id = detection.class_id
        
        # Draw bounding box
        cv2.rectangle(image, (x, y), (x + width, y + height), (0, 255, 0), 2)
        
        # Prepare label text
        label = f"{class_id}: {confidence:.2f} ({distance:.2f}m)"
        
        # Draw label background
        label_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
        cv2.rectangle(image, (x, y - 25), (x + label_size[0], y), (0, 255, 0), -1)
        
        # Draw label text
        cv2.putText(image, label, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
        
        # Draw 3D position indicator
        pos = detection.position
        center_x = x + width // 2
        center_y = y + height // 2
        
        # Draw a circle at the center
        cv2.circle(image, (center_x, center_y), 5, (0, 0, 255), -1)
        
        # Draw 3D position text
        pos_label = f"X:{pos.x:.2f} Y:{pos.y:.2f} Z:{pos.z:.2f}"
        cv2.putText(image, pos_label, (x, y + height + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = DetectionVisualizerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node and close any OpenCV windows
        if 'node' in locals():
            node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
