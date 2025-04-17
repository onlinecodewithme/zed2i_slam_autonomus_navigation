#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from nav_msgs.msg import Path
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
from tf2_geometry_msgs import do_transform_pose

import math
import numpy as np
from enum import Enum
import threading

# Import ROS interfaces
from robot_interfaces.msg import AirplaneDetection, InspectionStatus
from robot_interfaces.srv import SetInspectionTarget

# Import inspection pattern generator
from inspection_planner.inspection_patterns import InspectionPatternGenerator, InspectionPattern

class InspectionState(Enum):
    """State machine for inspection planning"""
    IDLE = 0
    WAITING_FOR_TARGET = 1
    PLANNING = 2
    INSPECTING = 3
    COMPLETED = 4
    ERROR = 5

class InspectionPlannerNode(Node):
    """
    ROS2 node for planning and executing aircraft inspections
    """
    
    def __init__(self):
        super().__init__('inspection_planner')
        
        # Declare parameters
        self.declare_parameter('path_topic', '/path')
        self.declare_parameter('detection_topic', '/airplane_detection/detections')
        self.declare_parameter('status_topic', '/inspection_status')
        self.declare_parameter('inspection_radius', 3.0)  # Default inspection distance (meters)
        self.declare_parameter('inspection_height', 1.0)  # Default height offset (meters)
        self.declare_parameter('inspection_points', 24)  # Number of waypoints in inspection patterns
        self.declare_parameter('inspection_pattern', 'circle')  # Default pattern type
        self.declare_parameter('inspection_timeout', 300.0)  # Timeout for inspection (seconds)
        self.declare_parameter('enable_visualization', True)
        
        # Get parameters
        self.path_topic = self.get_parameter('path_topic').value
        self.detection_topic = self.get_parameter('detection_topic').value
        self.status_topic = self.get_parameter('status_topic').value
        self.inspection_radius = self.get_parameter('inspection_radius').value
        self.inspection_height = self.get_parameter('inspection_height').value
        self.inspection_points = self.get_parameter('inspection_points').value
        self.inspection_pattern_str = self.get_parameter('inspection_pattern').value
        self.inspection_timeout = self.get_parameter('inspection_timeout').value
        self.enable_visualization = self.get_parameter('enable_visualization').value
        
        # Map pattern string to enum
        pattern_map = {
            'circle': InspectionPattern.CIRCLE,
            'square': InspectionPattern.SQUARE,
            'zigzag': InspectionPattern.ZIGZAG,
            'spiral': InspectionPattern.SPIRAL,
            'vertical_scan': InspectionPattern.VERTICAL_SCAN,
            'horizontal_scan': InspectionPattern.HORIZONTAL_SCAN
        }
        self.inspection_pattern = pattern_map.get(
            self.inspection_pattern_str.lower(), 
            InspectionPattern.CIRCLE
        )
        
        # Initialize TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Create publishers
        self.path_pub = self.create_publisher(
            Path,
            self.path_topic,
            10
        )
        
        self.status_pub = self.create_publisher(
            InspectionStatus,
            self.status_topic,
            10
        )
        
        # Create visualization publisher if enabled
        if self.enable_visualization:
            self.markers_pub = self.create_publisher(
                MarkerArray,
                '/inspection_visualization',
                10
            )
        
        # Create subscribers
        # Quality of service profile for sensor data
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
        
        # Create service server
        self.inspection_service = self.create_service(
            SetInspectionTarget,
            'set_inspection_target',
            self.handle_set_inspection_target
        )
        
        # Initialize inspection state
        self.state = InspectionState.IDLE
        self.target_pose = None
        self.target_airplane_class = ""
        self.current_path = None
        self.current_pose = None
        self.inspection_radius_override = None
        self.inspection_pattern_override = None
        
        # Pattern generator
        self.pattern_generator = InspectionPatternGenerator()
        
        # Latest airplane detection
        self.latest_detection = None
        
        # Processing lock to prevent concurrent processing
        self.processing_lock = threading.Lock()
        
        # Create timer for inspection state updates
        self.state_timer = self.create_timer(1.0, self.update_inspection_state)
        
        self.get_logger().info(f'Inspection planner node initialized with {self.inspection_pattern_str} pattern')
    
    def detection_callback(self, msg):
        """
        Callback for airplane detections
        """
        # Store latest detection
        self.latest_detection = msg
        
        # If we're waiting for a specific airplane class, check if this matches
        if (self.state == InspectionState.WAITING_FOR_TARGET and 
            self.target_airplane_class and 
            msg.class_id == self.target_airplane_class):
            
            self.get_logger().info(f'Found target airplane of class {msg.class_id}')
            
            # Calculate target pose from detection
            # Convert from camera frame to map frame if possible
            try:
                # Create a pose in camera frame
                detection_pose = PoseStamped()
                detection_pose.header = msg.header
                detection_pose.pose.position = msg.position
                
                # Set a default orientation (facing the camera)
                detection_pose.pose.orientation.w = 1.0
                
                # Transform to map frame
                target_pose = self.tf_buffer.transform(
                    detection_pose,
                    'map',
                    rclpy.duration.Duration(seconds=1.0)
                )
                
                # Store as target pose
                self.target_pose = target_pose.pose
                
                # Transition to planning state
                self.state = InspectionState.PLANNING
                
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                    tf2_ros.ExtrapolationException) as e:
                self.get_logger().error(f'TF Error: {e}')
                # Continue in WAITING_FOR_TARGET state
    
    def handle_set_inspection_target(self, request, response):
        """
        Service handler for setting inspection target
        """
        self.get_logger().info('Received inspection target request')
        
        # Check target type
        if request.target_type == request.POINT:
            # Direct pose target
            self.target_pose = request.target_pose
            self.target_airplane_class = ""
            self.state = InspectionState.PLANNING
            
            self.get_logger().info(f'Set target to position ({self.target_pose.position.x:.2f}, '
                                   f'{self.target_pose.position.y:.2f}, {self.target_pose.position.z:.2f})')
            
            response.success = True
            response.message = "Target set successfully"
            
        elif request.target_type == request.AIRPLANE:
            # Airplane detection-based target
            self.target_airplane_class = request.airplane_class_id
            self.target_pose = None
            self.state = InspectionState.WAITING_FOR_TARGET
            
            self.get_logger().info(f'Waiting for airplane detection of class {self.target_airplane_class}')
            
            response.success = True
            response.message = f"Waiting for airplane detection of class {self.target_airplane_class}"
            
        else:
            # Invalid target type
            response.success = False
            response.message = f"Invalid target type: {request.target_type}"
            return response
        
        # Store inspection parameters from request
        if request.inspection_radius > 0:
            self.inspection_radius_override = request.inspection_radius
            self.get_logger().info(f'Using custom inspection radius: {self.inspection_radius_override}')
        else:
            self.inspection_radius_override = None
        
        # Set pattern type
        if request.pattern == request.CIRCLE:
            self.inspection_pattern_override = InspectionPattern.CIRCLE
        elif request.pattern == request.SQUARE:
            self.inspection_pattern_override = InspectionPattern.SQUARE
        elif request.pattern == request.ZIGZAG:
            self.inspection_pattern_override = InspectionPattern.ZIGZAG
        else:
            self.inspection_pattern_override = None
        
        if self.inspection_pattern_override:
            self.get_logger().info(f'Using custom inspection pattern: {self.inspection_pattern_override.name}')
        
        return response
    
    def update_inspection_state(self):
        """
        Update the inspection state and take actions based on current state
        """
        # Acquire lock to prevent concurrent processing
        if not self.processing_lock.acquire(blocking=False):
            return
        
        try:
            if self.state == InspectionState.IDLE:
                # Nothing to do in idle state
                pass
                
            elif self.state == InspectionState.WAITING_FOR_TARGET:
                # Keep publishing status
                self.publish_status()
                
            elif self.state == InspectionState.PLANNING:
                # Generate inspection path
                self.plan_inspection()
                
                # Publish the planned path
                if self.current_path:
                    self.path_pub.publish(self.current_path)
                    
                    # Transition to INSPECTING state
                    self.state = InspectionState.INSPECTING
                    self.get_logger().info('Inspection plan generated, executing')
                else:
                    self.get_logger().error('Failed to generate inspection plan')
                    self.state = InspectionState.ERROR
                
            elif self.state == InspectionState.INSPECTING:
                # Keep publishing status
                self.publish_status()
                
                # In a real implementation, we would monitor progress
                # For now, just stay in this state until a new target is set
                
            elif self.state == InspectionState.COMPLETED:
                # Inspection completed
                self.publish_status()
                
                # Transition back to IDLE
                self.state = InspectionState.IDLE
                
            elif self.state == InspectionState.ERROR:
                # Error during inspection
                self.publish_status()
                
                # For now, just transition back to IDLE
                self.state = InspectionState.IDLE
                
        finally:
            # Release lock
            self.processing_lock.release()
    
    def plan_inspection(self):
        """
        Generate an inspection plan based on target and parameters
        """
        if self.target_pose is None:
            self.get_logger().error('Cannot plan inspection: No target pose')
            self.state = InspectionState.ERROR
            return
        
        # Determine pattern type
        pattern_type = self.inspection_pattern_override if self.inspection_pattern_override else self.inspection_pattern
        
        # Determine inspection radius
        radius = self.inspection_radius_override if self.inspection_radius_override else self.inspection_radius
        
        # Create center point
        center = Point()
        center.x = self.target_pose.position.x
        center.y = self.target_pose.position.y
        center.z = self.target_pose.position.z
        
        # Generate pattern
        self.current_path = self.pattern_generator.generate_pattern(
            pattern_type=pattern_type,
            center=center,
            size=radius,
            num_points=self.inspection_points,
            height=self.inspection_height,
            orientation_towards_center=True,
            frame_id="map"
        )
        
        # Publish visualization
        if self.enable_visualization:
            self.publish_visualization()
    
    def publish_status(self):
        """
        Publish inspection status
        """
        status = InspectionStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.header.frame_id = "map"
        
        # Map state
        if self.state == InspectionState.IDLE:
            status.state = status.IDLE
            status.status_message = "Idle"
        elif self.state == InspectionState.WAITING_FOR_TARGET:
            status.state = status.IDLE
            status.status_message = f"Waiting for airplane of class {self.target_airplane_class}"
        elif self.state == InspectionState.PLANNING:
            status.state = status.IDLE
            status.status_message = "Planning inspection path"
        elif self.state == InspectionState.INSPECTING:
            status.state = status.INSPECTING
            status.status_message = "Executing inspection"
            status.progress = 0.5  # TODO: Calculate actual progress
        elif self.state == InspectionState.COMPLETED:
            status.state = status.COMPLETED
            status.status_message = "Inspection completed"
            status.progress = 1.0
        elif self.state == InspectionState.ERROR:
            status.state = status.ERROR
            status.status_message = "Error during inspection"
        
        # Set poses
        if self.target_pose:
            status.target_pose = self.target_pose
        
        if self.current_pose:
            status.current_pose = self.current_pose
        
        # Publish status
        self.status_pub.publish(status)
    
    def publish_visualization(self):
        """
        Publish visualization of inspection plan
        """
        if not self.enable_visualization or not self.current_path:
            return
        
        marker_array = MarkerArray()
        
        # Target marker
        target_marker = Marker()
        target_marker.header.frame_id = "map"
        target_marker.header.stamp = self.get_clock().now().to_msg()
        target_marker.ns = "inspection_target"
        target_marker.id = 0
        target_marker.type = Marker.SPHERE
        target_marker.action = Marker.ADD
        
        # Set target position
        target_marker.pose = self.target_pose
        
        # Set scale and color
        target_marker.scale.x = 1.0
        target_marker.scale.y = 1.0
        target_marker.scale.z = 1.0
        target_marker.color.r = 1.0
        target_marker.color.g = 0.0
        target_marker.color.b = 0.0
        target_marker.color.a = 0.8
        
        marker_array.markers.append(target_marker)
        
        # Path markers
        path_line_marker = Marker()
        path_line_marker.header.frame_id = "map"
        path_line_marker.header.stamp = self.get_clock().now().to_msg()
        path_line_marker.ns = "inspection_path"
        path_line_marker.id = 1
        path_line_marker.type = Marker.LINE_STRIP
        path_line_marker.action = Marker.ADD
        
        # Set line properties
        path_line_marker.scale.x = 0.1  # Line width
        path_line_marker.color.r = 0.0
        path_line_marker.color.g = 1.0
        path_line_marker.color.b = 0.0
        path_line_marker.color.a = 0.8
        
        # Add points from path
        for pose in self.current_path.poses:
            path_line_marker.points.append(pose.pose.position)
        
        marker_array.markers.append(path_line_marker)
        
        # Waypoint markers
        for i, pose in enumerate(self.current_path.poses):
            waypoint_marker = Marker()
            waypoint_marker.header.frame_id = "map"
            waypoint_marker.header.stamp = self.get_clock().now().to_msg()
            waypoint_marker.ns = "inspection_waypoints"
            waypoint_marker.id = i + 10  # Offset IDs to avoid collision
            waypoint_marker.type = Marker.ARROW
            waypoint_marker.action = Marker.ADD
            
            # Set pose
            waypoint_marker.pose = pose.pose
            
            # Set scale
            waypoint_marker.scale.x = 0.5  # Arrow length
            waypoint_marker.scale.y = 0.1  # Arrow width
            waypoint_marker.scale.z = 0.1  # Arrow height
            
            # Set color
            waypoint_marker.color.r = 0.0
            waypoint_marker.color.g = 0.0
            waypoint_marker.color.b = 1.0
            waypoint_marker.color.a = 0.8
            
            marker_array.markers.append(waypoint_marker)
        
        # Publish marker array
        self.markers_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = InspectionPlannerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
