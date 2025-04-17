#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
import threading
import time
import os
from pathlib import Path

from airplane_detection.detector import AirplaneDetector
from robot_interfaces.msg import AirplaneDetection

class AirplaneDetectorNode(Node):
    """
    ROS2 node for airplane detection using ZED 2i camera
    """
    
    def __init__(self):
        super().__init__('airplane_detector')
        
        # Declare parameters
        self.declare_parameter('model_path', '')
        self.declare_parameter('conf_threshold', 0.5)
        self.declare_parameter('nms_threshold', 0.45)
        self.declare_parameter('detection_rate', 5.0)  # Hz
        self.declare_parameter('publish_visualization', True)
        
        # Get parameters
        model_path = self.get_parameter('model_path').value
        conf_threshold = self.get_parameter('conf_threshold').value
        nms_threshold = self.get_parameter('nms_threshold').value
        self.detection_rate = self.get_parameter('detection_rate').value
        self.publish_visualization = self.get_parameter('publish_visualization').value
        
        # Default model path if not specified
        if not model_path:
            # Use YOLO v5 pretrained model
            model_path = None
            
        # Create QoS profile for sensor data
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Initialize detector
        self.get_logger().info('Initializing airplane detector...')
        self.detector = AirplaneDetector(
            model_path=model_path,
            conf_threshold=conf_threshold,
            nms_threshold=nms_threshold
        )
        
        # Create subscribers for ZED camera
        self.rgb_sub = self.create_subscription(
            Image,
            '/zed2i/zed_node/rgb/image_rect_color',
            self.rgb_callback,
            sensor_qos
        )
        
        self.depth_sub = self.create_subscription(
            Image,
            '/zed2i/zed_node/depth/depth_registered',
            self.depth_callback,
            sensor_qos
        )
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/zed2i/zed_node/rgb/camera_info',
            self.camera_info_callback,
            sensor_qos
        )
        
        # Publisher for detections
        self.detection_pub = self.create_publisher(
            AirplaneDetection,
            '/airplane_detection/detections',
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
        self.rgb_image = None
        self.depth_image = None
        self.camera_info = None
        
        # Processing lock to prevent concurrent processing
        self.processing_lock = threading.Lock()
        
        # Create timer for detection
        detection_period = 1.0 / self.detection_rate
        self.detection_timer = self.create_timer(detection_period, self.detect_airplanes)
        
        self.get_logger().info('Airplane detector node initialized')
    
    def rgb_callback(self, msg):
        """
        Callback for RGB images
        """
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        except CvBridgeError as e:
            self.get_logger().error(f'CV Bridge error: {e}')
    
    def depth_callback(self, msg):
        """
        Callback for depth images
        """
        try:
            # Convert depth image (32FC1) to mm
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
            # Convert NaN values to zero
            self.depth_image = np.nan_to_num(self.depth_image, nan=0.0)
            # Convert to mm (if not already)
            if np.max(self.depth_image) < 100:  # likely meters
                self.depth_image *= 1000.0  # convert to mm
        except CvBridgeError as e:
            self.get_logger().error(f'CV Bridge error: {e}')
    
    def camera_info_callback(self, msg):
        """
        Callback for camera info
        """
        self.camera_info = msg
    
    def detect_airplanes(self):
        """
        Detect airplanes in the current RGB and depth images
        """
        if self.rgb_image is None or self.depth_image is None:
            return
        
        # Acquire lock to prevent concurrent processing
        if not self.processing_lock.acquire(blocking=False):
            return
        
        try:
            # Process the images
            start_time = time.time()
            
            # Perform airplane detection
            detections = self.detector.detect(self.rgb_image)
            
            if detections:
                self.get_logger().info(f'Detected {len(detections)} airplanes')
                
                # Process each detection
                distances = []
                for det in detections:
                    # Estimate distance
                    distance = self.detector.estimate_distance(det, self.depth_image)
                    distances.append(distance)
                    
                    # Create and publish detection message
                    detection_msg = self.create_detection_msg(det, distance)
                    self.detection_pub.publish(detection_msg)
                
                # Visualize detections if requested
                if self.publish_visualization:
                    vis_img = self.detector.draw_detections(self.rgb_image, detections, distances)
                    try:
                        vis_msg = self.bridge.cv2_to_imgmsg(vis_img, "rgb8")
                        self.vis_pub.publish(vis_msg)
                    except CvBridgeError as e:
                        self.get_logger().error(f'CV Bridge error: {e}')
            
            # Calculate processing time
            elapsed = time.time() - start_time
            self.get_logger().debug(f'Detection processing time: {elapsed:.3f} seconds')
            
        finally:
            # Release lock
            self.processing_lock.release()
    
    def create_detection_msg(self, detection, distance):
        """
        Create AirplaneDetection message from detection data
        
        Args:
            detection: [x, y, w, h, confidence, class_id]
            distance: Estimated distance in meters
            
        Returns:
            AirplaneDetection message
        """
        x, y, w, h, confidence, class_id = detection
        
        # Create header
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "zed2i_left_camera_optical_frame"  # ZED left camera frame
        
        # Create detection message
        msg = AirplaneDetection()
        msg.header = header
        msg.x = int(x)
        msg.y = int(y)
        msg.width = int(w)
        msg.height = int(h)
        msg.confidence = float(confidence)
        msg.distance = float(distance)
        
        # Set position (using depth data - in camera frame)
        # Note: For accurate 3D position, we would need to use camera intrinsics to convert to 3D
        if self.camera_info is not None:
            # Calculate 3D position using depth and camera intrinsics
            center_x = x + w/2
            center_y = y + h/2
            
            # Camera intrinsics
            fx = self.camera_info.k[0]  # focal length x
            fy = self.camera_info.k[4]  # focal length y
            cx = self.camera_info.k[2]  # optical center x
            cy = self.camera_info.k[5]  # optical center y
            
            # Convert pixel to 3D point
            z = distance  # meters
            x_3d = (center_x - cx) * z / fx
            y_3d = (center_y - cy) * z / fy
            
            # Add 3D position to message
            msg.position = Point(x=x_3d, y=y_3d, z=z)
        else:
            # No camera info yet, use dummy position
            msg.position = Point(x=0.0, y=0.0, z=distance)
        
        # Set class ID (YOLO class name if available)
        if hasattr(self.detector, 'class_names') and int(class_id) < len(self.detector.class_names):
            msg.class_id = self.detector.class_names[int(class_id)]
        else:
            msg.class_id = "airplane"
        
        return msg

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = AirplaneDetectorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
