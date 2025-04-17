#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PoseStamped, Pose
from nav_msgs.msg import Path
from std_msgs.msg import ColorRGBA, Header

# Import our custom interfaces
from robot_interfaces.msg import InspectionStatus, AirplaneDetection

import math
import numpy as np
from enum import Enum

class InspectionVisualizerNode(Node):
    """
    ROS2 node for visualizing inspection plans and progress
    """
    
    def __init__(self):
        super().__init__('inspection_visualizer')
        
        # Declare parameters
        self.declare_parameter('status_topic', '/inspection_status')
        self.declare_parameter('path_topic', '/path')
        self.declare_parameter('detection_topic', '/airplane_detection/detections')
        self.declare_parameter('visualization_topic', '/inspection_visualization')
        self.declare_parameter('marker_scale', 1.0)
        self.declare_parameter('marker_lifetime', 0.0)  # 0 = persistent
        self.declare_parameter('use_unique_namespaces', True)
        
        # Get parameters
        self.status_topic = self.get_parameter('status_topic').value
        self.path_topic = self.get_parameter('path_topic').value
        self.detection_topic = self.get_parameter('detection_topic').value
        self.visualization_topic = self.get_parameter('visualization_topic').value
        self.marker_scale = self.get_parameter('marker_scale').value
        self.marker_lifetime = self.get_parameter('marker_lifetime').value
        self.use_unique_namespaces = self.get_parameter('use_unique_namespaces').value
        
        # Create publishers
        self.marker_pub = self.create_publisher(
            MarkerArray,
            self.visualization_topic,
            10
        )
        
        # Create subscribers
        self.status_sub = self.create_subscription(
            InspectionStatus,
            self.status_topic,
            self.status_callback,
            10
        )
        
        self.path_sub = self.create_subscription(
            Path,
            self.path_topic,
            self.path_callback,
            10
        )
        
        # QoS profile for sensor data
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.detection_sub = self.create_subscription(
            AirplaneDetection,
            self.detection_topic,
            self.detection_callback,
            sensor_qos
        )
        
        # Initialize state variables
        self.current_status = None
        self.current_path = None
        self.latest_detection = None
        self.marker_id_counter = 0
        
        # Last timestamp used for namespace (if use_unique_namespaces is True)
        self.last_timestamp = self.get_clock().now()
        
        # Buffer of marker arrays for different types of visualization
        self.status_markers = MarkerArray()
        self.path_markers = MarkerArray()
        self.detection_markers = MarkerArray()
        
        # Create timer for visualization updates
        self.update_timer = self.create_timer(0.1, self.update_visualization)
        
        self.get_logger().info('Inspection visualizer node initialized')
    
    def status_callback(self, msg):
        """
        Callback for inspection status updates
        """
        self.current_status = msg
        self.update_status_markers()
    
    def path_callback(self, msg):
        """
        Callback for path updates
        """
        self.current_path = msg
        self.update_path_markers()
    
    def detection_callback(self, msg):
        """
        Callback for airplane detections
        """
        self.latest_detection = msg
        self.update_detection_markers()
    
    def get_unique_namespace(self, base_name):
        """
        Generate a unique namespace based on timestamp
        """
        if not self.use_unique_namespaces:
            return base_name
        
        timestamp = self.get_clock().now()
        # Only update last_timestamp if it's significantly different (avoid spam)
        if (timestamp.nanoseconds - self.last_timestamp.nanoseconds) > 1e9:  # 1 second
            self.last_timestamp = timestamp
        
        return f"{base_name}_{self.last_timestamp.nanoseconds}"
    
    def update_status_markers(self):
        """
        Update markers for inspection status visualization
        """
        if not self.current_status:
            return
        
        # Create a new marker array
        markers = MarkerArray()
        
        # Generate a unique namespace
        ns = self.get_unique_namespace("inspection_status")
        
        # Target marker
        if self.current_status.target_pose.position.x != 0 or \
           self.current_status.target_pose.position.y != 0 or \
           self.current_status.target_pose.position.z != 0:
            
            target_marker = Marker()
            target_marker.header.frame_id = self.current_status.header.frame_id
            target_marker.header.stamp = self.get_clock().now().to_msg()
            target_marker.ns = ns
            target_marker.id = self.marker_id_counter
            self.marker_id_counter += 1
            
            target_marker.type = Marker.SPHERE
            target_marker.action = Marker.ADD
            
            # Set pose
            target_marker.pose = self.current_status.target_pose
            
            # Set scale
            scale = self.marker_scale
            target_marker.scale.x = scale
            target_marker.scale.y = scale
            target_marker.scale.z = scale
            
            # Set color (red for target)
            target_marker.color.r = 1.0
            target_marker.color.g = 0.0
            target_marker.color.b = 0.0
            target_marker.color.a = 0.7
            
            # Set lifetime
            if self.marker_lifetime > 0:
                target_marker.lifetime.sec = int(self.marker_lifetime)
                target_marker.lifetime.nanosec = int((self.marker_lifetime % 1) * 1e9)
            
            markers.markers.append(target_marker)
            
            # Add text marker for status
            text_marker = Marker()
            text_marker.header.frame_id = self.current_status.header.frame_id
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = ns
            text_marker.id = self.marker_id_counter
            self.marker_id_counter += 1
            
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            # Position text above target
            text_marker.pose = self.current_status.target_pose
            text_marker.pose.position.z += scale + 0.5
            
            # Set scale (text height)
            text_marker.scale.z = 0.5 * scale
            
            # Set color (white text)
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            
            # Set text
            text_marker.text = self.current_status.status_message
            
            # Set lifetime
            if self.marker_lifetime > 0:
                text_marker.lifetime.sec = int(self.marker_lifetime)
                text_marker.lifetime.nanosec = int((self.marker_lifetime % 1) * 1e9)
            
            markers.markers.append(text_marker)
            
            # Add progress bar if applicable
            if self.current_status.state == InspectionStatus.INSPECTING:
                progress_marker = Marker()
                progress_marker.header.frame_id = self.current_status.header.frame_id
                progress_marker.header.stamp = self.get_clock().now().to_msg()
                progress_marker.ns = ns
                progress_marker.id = self.marker_id_counter
                self.marker_id_counter += 1
                
                progress_marker.type = Marker.CUBE
                progress_marker.action = Marker.ADD
                
                # Position progress bar above text
                progress_marker.pose = self.current_status.target_pose
                progress_marker.pose.position.z += scale + 1.0
                
                # Set scale (width based on progress)
                progress = max(0.0, min(1.0, self.current_status.progress))
                progress_marker.scale.x = 2.0 * scale * progress
                progress_marker.scale.y = 0.2 * scale
                progress_marker.scale.z = 0.2 * scale
                
                # Set color (green for progress)
                progress_marker.color.r = 0.0
                progress_marker.color.g = 1.0
                progress_marker.color.b = 0.0
                progress_marker.color.a = 0.7
                
                # Set lifetime
                if self.marker_lifetime > 0:
                    progress_marker.lifetime.sec = int(self.marker_lifetime)
                    progress_marker.lifetime.nanosec = int((self.marker_lifetime % 1) * 1e9)
                
                markers.markers.append(progress_marker)
        
        # Update status markers
        self.status_markers = markers
    
    def update_path_markers(self):
        """
        Update markers for path visualization
        """
        if not self.current_path or not self.current_path.poses:
            return
        
        # Create a new marker array
        markers = MarkerArray()
        
        # Generate a unique namespace
        ns = self.get_unique_namespace("inspection_path")
        
        # Line strip marker for path
        line_marker = Marker()
        line_marker.header.frame_id = self.current_path.header.frame_id
        line_marker.header.stamp = self.get_clock().now().to_msg()
        line_marker.ns = ns
        line_marker.id = self.marker_id_counter
        self.marker_id_counter += 1
        
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        
        # Set scale
        line_marker.scale.x = 0.1 * self.marker_scale  # Line width
        
        # Set color (green for path)
        line_marker.color.r = 0.0
        line_marker.color.g = 1.0
        line_marker.color.b = 0.0
        line_marker.color.a = 0.7
        
        # Add points
        for pose in self.current_path.poses:
            line_marker.points.append(pose.pose.position)
        
        # Close the loop if needed (if first and last points are close)
        if len(self.current_path.poses) > 2:
            first_pos = self.current_path.poses[0].pose.position
            last_pos = self.current_path.poses[-1].pose.position
            dx = first_pos.x - last_pos.x
            dy = first_pos.y - last_pos.y
            dz = first_pos.z - last_pos.z
            distance = math.sqrt(dx*dx + dy*dy + dz*dz)
            if distance < 1.0:  # If points are within 1 meter
                line_marker.points.append(first_pos)  # Close the loop
        
        # Set lifetime
        if self.marker_lifetime > 0:
            line_marker.lifetime.sec = int(self.marker_lifetime)
            line_marker.lifetime.nanosec = int((self.marker_lifetime % 1) * 1e9)
        
        markers.markers.append(line_marker)
        
        # Add markers for each waypoint
        for i, pose in enumerate(self.current_path.poses):
            # Skip some waypoints if there are too many
            if len(self.current_path.poses) > 20 and i % 4 != 0 and i != len(self.current_path.poses) - 1:
                continue
                
            waypoint_marker = Marker()
            waypoint_marker.header.frame_id = self.current_path.header.frame_id
            waypoint_marker.header.stamp = self.get_clock().now().to_msg()
            waypoint_marker.ns = ns
            waypoint_marker.id = self.marker_id_counter
            self.marker_id_counter += 1
            
            waypoint_marker.type = Marker.ARROW
            waypoint_marker.action = Marker.ADD
            
            # Set pose
            waypoint_marker.pose = pose.pose
            
            # Set scale
            waypoint_marker.scale.x = 0.5 * self.marker_scale  # Arrow length
            waypoint_marker.scale.y = 0.1 * self.marker_scale  # Arrow width
            waypoint_marker.scale.z = 0.1 * self.marker_scale  # Arrow height
            
            # Set color (blue for waypoints)
            waypoint_marker.color.r = 0.0
            waypoint_marker.color.g = 0.0
            waypoint_marker.color.b = 1.0
            waypoint_marker.color.a = 0.7
            
            # Special colors for first and last waypoint
            if i == 0:
                waypoint_marker.color.r = 0.0
                waypoint_marker.color.g = 1.0
                waypoint_marker.color.b = 0.0
            elif i == len(self.current_path.poses) - 1:
                waypoint_marker.color.r = 1.0
                waypoint_marker.color.g = 0.0
                waypoint_marker.color.b = 0.0
            
            # Set lifetime
            if self.marker_lifetime > 0:
                waypoint_marker.lifetime.sec = int(self.marker_lifetime)
                waypoint_marker.lifetime.nanosec = int((self.marker_lifetime % 1) * 1e9)
            
            markers.markers.append(waypoint_marker)
        
        # Update path markers
        self.path_markers = markers
    
    def update_detection_markers(self):
        """
        Update markers for detection visualization
        """
        if not self.latest_detection:
            return
        
        # Create a new marker array
        markers = MarkerArray()
        
        # Generate a unique namespace
        ns = self.get_unique_namespace("airplane_detection")
        
        # Detection marker
        detection_marker = Marker()
        detection_marker.header.frame_id = self.latest_detection.header.frame_id
        detection_marker.header.stamp = self.get_clock().now().to_msg()
        detection_marker.ns = ns
        detection_marker.id = self.marker_id_counter
        self.marker_id_counter += 1
        
        detection_marker.type = Marker.CUBE
        detection_marker.action = Marker.ADD
        
        # Set pose
        detection_marker.pose.position = self.latest_detection.position
        detection_marker.pose.orientation.w = 1.0
        
        # Set scale (based on detection size)
        # Convert image pixel dimensions to 3D using rough approximation
        # For a more accurate representation, use the actual 3D dimensions if available
        scale_factor = self.latest_detection.distance / 5.0  # Rough conversion
        detection_marker.scale.x = scale_factor * self.latest_detection.width * self.marker_scale
        detection_marker.scale.y = scale_factor * self.latest_detection.width * self.marker_scale  # Use width as a proxy for depth
        detection_marker.scale.z = scale_factor * self.latest_detection.height * self.marker_scale
        
        # Set color (purple for detections)
        detection_marker.color.r = 0.8
        detection_marker.color.g = 0.1
        detection_marker.color.b = 0.8
        detection_marker.color.a = 0.5
        
        # Set lifetime (shorter for detections to avoid cluttering)
        detection_marker.lifetime.sec = 0
        detection_marker.lifetime.nanosec = 500000000  # 0.5 seconds
        
        markers.markers.append(detection_marker)
        
        # Add text marker for detection class
        text_marker = Marker()
        text_marker.header.frame_id = self.latest_detection.header.frame_id
        text_marker.header.stamp = self.get_clock().now().to_msg()
        text_marker.ns = ns
        text_marker.id = self.marker_id_counter
        self.marker_id_counter += 1
        
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        
        # Position text above detection
        text_marker.pose.position = self.latest_detection.position
        text_marker.pose.position.z += detection_marker.scale.z / 2 + 0.5
        text_marker.pose.orientation.w = 1.0
        
        # Set scale (text height)
        text_marker.scale.z = 0.5 * self.marker_scale
        
        # Set color (white text)
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0
        
        # Set text
        confidence_pct = int(self.latest_detection.confidence * 100)
        text_marker.text = f"{self.latest_detection.class_id} ({confidence_pct}%)"
        
        # Set lifetime
        text_marker.lifetime.sec = 0
        text_marker.lifetime.nanosec = 500000000  # 0.5 seconds
        
        markers.markers.append(text_marker)
        
        # Update detection markers
        self.detection_markers = markers
    
    def update_visualization(self):
        """
        Update and publish all visualization markers
        """
        # Combine all marker arrays
        combined_markers = MarkerArray()
        
        # Add all markers from each category
        for marker in self.status_markers.markers:
            combined_markers.markers.append(marker)
        
        for marker in self.path_markers.markers:
            combined_markers.markers.append(marker)
        
        for marker in self.detection_markers.markers:
            combined_markers.markers.append(marker)
        
        # Reset marker ID counter if it gets too large
        if self.marker_id_counter > 10000:
            self.marker_id_counter = 0
        
        # Publish combined markers if not empty
        if combined_markers.markers:
            self.marker_pub.publish(combined_markers)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = InspectionVisualizerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
