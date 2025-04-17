#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Twist, Point, PoseStamped
from std_msgs.msg import Header
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry, Path
from cv_bridge import CvBridge, CvBridgeError
import tf2_ros
from tf2_geometry_msgs import do_transform_pose

import numpy as np
import cv2
import math
import time
from enum import Enum
import threading

# Import obstacle detection
from autonomous_nav.obstacle_detection import ObstacleDetector, ObstacleDetectionMethod, Obstacle

class NavigationState(Enum):
    """State machine for navigation"""
    IDLE = 0
    MOVING_TO_GOAL = 1
    AVOIDING_OBSTACLE = 2
    ROTATING = 3
    BACKING_UP = 4
    GOAL_REACHED = 5
    ERROR = 6

class ObstacleAvoidanceNode(Node):
    """
    ROS2 node for autonomous navigation with obstacle avoidance using the ZED 2i camera
    """
    def __init__(self):
        super().__init__('obstacle_avoidance')
        
        # Declare parameters
        self.declare_parameter('velocity_topic', '/cmd_vel')
        self.declare_parameter('depth_topic', '/zed2i/zed_node/depth/depth_registered')
        self.declare_parameter('camera_info_topic', '/zed2i/zed_node/rgb/camera_info')
        self.declare_parameter('rgb_topic', '/zed2i/zed_node/rgb/image_rect_color')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('local_costmap_topic', '/local_costmap')
        self.declare_parameter('path_topic', '/path')

        self.declare_parameter('detection_method', 'threshold')
        self.declare_parameter('min_depth', 0.5)
        self.declare_parameter('max_depth', 10.0)
        self.declare_parameter('safety_distance', 1.0)
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('goal_tolerance', 0.5)
        self.declare_parameter('update_rate', 10.0)  # Hz
        self.declare_parameter('publish_visualization', True)
        
        # Get parameters
        self.velocity_topic = self.get_parameter('velocity_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.rgb_topic = self.get_parameter('rgb_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.goal_topic = self.get_parameter('goal_topic').value
        self.local_costmap_topic = self.get_parameter('local_costmap_topic').value
        self.path_topic = self.get_parameter('path_topic').value

        detection_method_str = self.get_parameter('detection_method').value
        self.min_depth = self.get_parameter('min_depth').value
        self.max_depth = self.get_parameter('max_depth').value
        self.safety_distance = self.get_parameter('safety_distance').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.update_rate = self.get_parameter('update_rate').value
        self.publish_visualization = self.get_parameter('publish_visualization').value
        
        # Convert detection method string to enum
        detection_method = ObstacleDetectionMethod.THRESHOLD
        if detection_method_str == 'gradient':
            detection_method = ObstacleDetectionMethod.GRADIENT
        elif detection_method_str == 'u_disparity':
            detection_method = ObstacleDetectionMethod.U_DISPARITY
        elif detection_method_str == 'point_cloud':
            detection_method = ObstacleDetectionMethod.POINT_CLOUD
        
        # Create QoS profile for sensor data
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Initialize obstacle detector
        self.detector = ObstacleDetector(
            method=detection_method,
            min_depth=self.min_depth,
            max_depth=self.max_depth
        )
        
        # Initialize navigation state
        self.state = NavigationState.IDLE
        self.goal_pose = None
        self.current_pose = None
        self.current_path = None
        self.obstacle_detected = False
        self.obstacle_direction = 0.0  # Positive: obstacle on left, Negative: obstacle on right
        
        # Initialize TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Create velocity publisher
        self.vel_pub = self.create_publisher(
            Twist,
            self.velocity_topic,
            10
        )
        
        # Create path publisher
        self.path_pub = self.create_publisher(
            Path,
            self.path_topic,
            10
        )
        
        # Create visualization publisher
        if self.publish_visualization:
            self.vis_pub = self.create_publisher(
                Image,
                '/obstacle_avoidance/visualization',
                10
            )
        
        # Create subscribers
        self.depth_sub = self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_callback,
            sensor_qos
        )
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            sensor_qos
        )
        
        self.rgb_sub = self.create_subscription(
            Image,
            self.rgb_topic,
            self.rgb_callback,
            sensor_qos
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10
        )
        
        self.goal_sub = self.create_subscription(
            PoseStamped,
            self.goal_topic,
            self.goal_callback,
            10
        )
        
        # Initialize instance variables
        self.depth_image = None
        self.rgb_image = None
        self.camera_info = None
        self.obstacles = []
        self.last_cmd_vel = Twist()
        
        # Processing lock to prevent concurrent processing
        self.processing_lock = threading.Lock()
        
        # Create timer for navigation update
        update_period = 1.0 / self.update_rate
        self.nav_timer = self.create_timer(update_period, self.navigation_callback)
        
        self.get_logger().info('Obstacle avoidance node initialized')
    
    def depth_callback(self, msg):
        """
        Callback for depth images
        """
        try:
            # Convert depth image to meters
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
            
            # Convert NaN values to zero
            self.depth_image = np.nan_to_num(self.depth_image, nan=0.0)
            
            # Scale to meters if needed
            if np.max(self.depth_image) > 100:  # likely in mm
                self.depth_image /= 1000.0  # convert to meters
        except CvBridgeError as e:
            self.get_logger().error(f'CV Bridge error: {e}')
    
    def camera_info_callback(self, msg):
        """
        Callback for camera info
        """
        self.camera_info = msg
    
    def rgb_callback(self, msg):
        """
        Callback for RGB images
        """
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        except CvBridgeError as e:
            self.get_logger().error(f'CV Bridge error: {e}')
    
    def odom_callback(self, msg):
        """
        Callback for odometry data
        """
        self.current_pose = msg.pose.pose
    
    def goal_callback(self, msg):
        """
        Callback for goal pose
        """
        self.goal_pose = msg.pose
        self.state = NavigationState.MOVING_TO_GOAL
        self.get_logger().info(f'New goal received: ({self.goal_pose.position.x:.2f}, {self.goal_pose.position.y:.2f})')
        
        # Generate path to goal (simple straight line for now)
        self.generate_path_to_goal()
    
    def generate_path_to_goal(self):
        """
        Generate a path to the goal
        Currently a simple straight line, could be replaced with a path planner
        """
        if self.goal_pose is None or self.current_pose is None:
            return
        
        # Create a path message
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = "map"
        
        # Add current position as first point
        current_pose_stamped = PoseStamped()
        current_pose_stamped.header = path.header
        current_pose_stamped.pose = self.current_pose
        path.poses.append(current_pose_stamped)
        
        # Add goal as final point
        goal_pose_stamped = PoseStamped()
        goal_pose_stamped.header = path.header
        goal_pose_stamped.pose = self.goal_pose
        path.poses.append(goal_pose_stamped)
        
        # Store and publish path
        self.current_path = path
        self.path_pub.publish(path)
    
    def navigation_callback(self):
        """
        Main navigation callback that runs at regular intervals
        """
        if self.depth_image is None or self.current_pose is None:
            return
        
        # Acquire lock to prevent concurrent processing
        if not self.processing_lock.acquire(blocking=False):
            return
        
        try:
            # Process the depth image to detect obstacles
            self.process_depth_image()
            
            # Update robot control based on obstacles and goal
            self.update_robot_control()
            
            # Publish visualization if requested
            self.publish_obstacle_visualization()
        finally:
            # Release lock
            self.processing_lock.release()
    
    def process_depth_image(self):
        """
        Process depth image to detect obstacles
        """
        if self.depth_image is None or self.camera_info is None:
            return
        
        # Detect obstacles
        timestamp = self.get_clock().now()
        self.obstacles = self.detector.detect_obstacles(self.depth_image, self.camera_info, timestamp)
        
        # Check if there are obstacles in the safety zone
        self.check_obstacles_in_safety_zone()
    
    def check_obstacles_in_safety_zone(self):
        """
        Check if there are obstacles in the safety zone
        """
        self.obstacle_detected = False
        self.obstacle_direction = 0.0
        
        for obstacle in self.obstacles:
            # Convert obstacle from camera frame to base frame
            # For simplicity, assuming camera aligned with robot:
            # x forward, y left, z up
            
            # Check if obstacle is in front of the robot within safety distance
            if (obstacle.z < self.safety_distance and 
                abs(obstacle.x) < self.safety_distance/2):
                
                self.obstacle_detected = True
                
                # Determine direction (positive: obstacle on left, negative: obstacle on right)
                self.obstacle_direction += obstacle.x
                
                self.get_logger().debug(f'Obstacle detected at ({obstacle.x:.2f}, {obstacle.y:.2f}, {obstacle.z:.2f})')
    
    def update_robot_control(self):
        """
        Update robot control based on obstacles and goal
        """
        cmd_vel = Twist()
        
        # State machine for navigation
        if self.state == NavigationState.IDLE:
            # Do nothing
            pass
            
        elif self.state == NavigationState.MOVING_TO_GOAL:
            if self.goal_pose is None:
                self.state = NavigationState.IDLE
                return
            
            if self.obstacle_detected:
                # Transition to obstacle avoidance
                self.state = NavigationState.AVOIDING_OBSTACLE
                self.get_logger().info('Obstacle detected, switching to avoidance')
            else:
                # Move toward goal
                cmd_vel = self.calculate_goal_velocity()
                
                # Check if goal reached
                distance_to_goal = self.calculate_distance_to_goal()
                if distance_to_goal < self.goal_tolerance:
                    self.state = NavigationState.GOAL_REACHED
                    self.get_logger().info('Goal reached!')
            
        elif self.state == NavigationState.AVOIDING_OBSTACLE:
            # Calculate avoidance velocity
            cmd_vel = self.calculate_avoidance_velocity()
            
            # If no obstacles detected, return to moving to goal
            if not self.obstacle_detected:
                self.state = NavigationState.MOVING_TO_GOAL
                self.get_logger().info('Path clear, resuming navigation to goal')
            
        elif self.state == NavigationState.ROTATING:
            # Rotate to align with goal
            cmd_vel = self.calculate_rotation_velocity()
            
            # If aligned, switch to moving to goal
            if abs(self.calculate_heading_error()) < 0.1:  # ~5 degrees
                self.state = NavigationState.MOVING_TO_GOAL
            
        elif self.state == NavigationState.BACKING_UP:
            # Back up from obstacle
            cmd_vel.linear.x = -0.2  # Slow reverse
            
            # After backing up for a short time, switch to rotating
            # (This would normally be handled with a timer)
            self.state = NavigationState.ROTATING
            
        elif self.state == NavigationState.GOAL_REACHED:
            # Stop and notify
            self.goal_pose = None
            self.state = NavigationState.IDLE
            
        elif self.state == NavigationState.ERROR:
            # Stop and log error
            self.get_logger().error('Navigation error occurred')
            self.state = NavigationState.IDLE
        
        # Publish command velocity
        self.vel_pub.publish(cmd_vel)
        self.last_cmd_vel = cmd_vel
    
    def calculate_goal_velocity(self):
        """
        Calculate velocity to move toward goal
        """
        cmd_vel = Twist()
        
        if self.goal_pose is None or self.current_pose is None:
            return cmd_vel
        
        # Calculate distance and heading to goal
        distance = self.calculate_distance_to_goal()
        heading_error = self.calculate_heading_error()
        
        # Set angular velocity to correct heading
        cmd_vel.angular.z = self.clamp(heading_error * 1.0, -self.max_angular_speed, self.max_angular_speed)
        
        # Set forward velocity (reduce speed when turning or close to goal)
        heading_factor = max(0.0, 1.0 - abs(heading_error) / math.pi)
        distance_factor = min(1.0, distance / 2.0)  # Slow down when close to goal
        
        cmd_vel.linear.x = self.max_linear_speed * heading_factor * distance_factor
        
        return cmd_vel
    
    def calculate_avoidance_velocity(self):
        """
        Calculate velocity to avoid obstacles
        """
        cmd_vel = Twist()
        
        if not self.obstacle_detected:
            return self.calculate_goal_velocity()
        
        # Determine which way to turn based on obstacle direction
        turn_direction = -1.0 if self.obstacle_direction >= 0.0 else 1.0
        
        # Set angular velocity to turn away from obstacle
        cmd_vel.angular.z = turn_direction * self.max_angular_speed * 0.8
        
        # Set forward velocity (slower when avoiding obstacles)
        cmd_vel.linear.x = self.max_linear_speed * 0.3
        
        return cmd_vel
    
    def calculate_rotation_velocity(self):
        """
        Calculate velocity to rotate to align with goal
        """
        cmd_vel = Twist()
        
        if self.goal_pose is None or self.current_pose is None:
            return cmd_vel
        
        # Calculate heading error
        heading_error = self.calculate_heading_error()
        
        # Set angular velocity to align with goal
        cmd_vel.angular.z = self.clamp(heading_error * 2.0, -self.max_angular_speed, self.max_angular_speed)
        
        return cmd_vel
    
    def calculate_distance_to_goal(self):
        """
        Calculate Euclidean distance to goal
        """
        if self.goal_pose is None or self.current_pose is None:
            return float('inf')
        
        dx = self.goal_pose.position.x - self.current_pose.position.x
        dy = self.goal_pose.position.y - self.current_pose.position.y
        
        return math.sqrt(dx*dx + dy*dy)
    
    def calculate_heading_error(self):
        """
        Calculate heading error to goal
        """
        if self.goal_pose is None or self.current_pose is None:
            return 0.0
        
        # Calculate target heading based on vector to goal
        dx = self.goal_pose.position.x - self.current_pose.position.x
        dy = self.goal_pose.position.y - self.current_pose.position.y
        target_heading = math.atan2(dy, dx)
        
        # Calculate current heading from quaternion
        # Note: This is a simplified version assuming 2D robot
        qx = self.current_pose.orientation.x
        qy = self.current_pose.orientation.y
        qz = self.current_pose.orientation.z
        qw = self.current_pose.orientation.w
        
        current_heading = math.atan2(2.0 * (qw * qz + qx * qy), qw * qw + qx * qx - qy * qy - qz * qz)
        
        # Calculate heading error
        error = target_heading - current_heading
        
        # Normalize to [-pi, pi]
        while error > math.pi:
            error -= 2.0 * math.pi
        while error < -math.pi:
            error += 2.0 * math.pi
        
        return error
    
    def clamp(self, value, min_value, max_value):
        """
        Clamp a value between min and max
        """
        return max(min_value, min(max_value, value))
    
    def publish_obstacle_visualization(self):
        """
        Publish visualization of obstacles and navigation info
        """
        if not self.publish_visualization or self.rgb_image is None:
            return
        
        try:
            # Create a copy of the RGB image
            vis_image = self.rgb_image.copy()
            
            # Draw detected obstacles
            if self.obstacles:
                vis_image = self.detector.visualize_obstacles(vis_image, self.obstacles)
            
            # Draw command velocity
            self.draw_command_velocity(vis_image)
            
            # Draw navigation state
            self.draw_navigation_state(vis_image)
            
            # Publish visualization image
            vis_msg = self.bridge.cv2_to_imgmsg(vis_image, "rgb8")
            self.vis_pub.publish(vis_msg)
        except Exception as e:
            self.get_logger().error(f'Visualization error: {e}')
    
    def draw_command_velocity(self, image):
        """
        Draw command velocity vector
        """
        height, width = image.shape[:2]
        center_x = width // 2
        center_y = height - 50
        
        # Scale velocities for visualization
        linear_scale = 50.0
        angular_scale = 50.0
        
        # Draw linear velocity
        end_x = center_x
        end_y = center_y - int(self.last_cmd_vel.linear.x * linear_scale)
        cv2.arrowedLine(image, (center_x, center_y), (end_x, end_y), (0, 255, 0), 2)
        
        # Draw angular velocity
        angular_end_x = center_x + int(self.last_cmd_vel.angular.z * angular_scale)
        cv2.arrowedLine(image, (center_x, center_y), (angular_end_x, center_y), (255, 0, 0), 2)
        
        # Draw velocity text
        vel_text = f"Lin: {self.last_cmd_vel.linear.x:.2f} m/s, Ang: {self.last_cmd_vel.angular.z:.2f} rad/s"
        cv2.putText(image, vel_text, (10, height - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    
    def draw_navigation_state(self, image):
        """
        Draw navigation state info
        """
        height, width = image.shape[:2]
        
        # Draw state
        state_text = f"State: {self.state.name}"
        cv2.putText(image, state_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
        
        # Draw obstacle info
        if self.obstacle_detected:
            obstacle_text = "OBSTACLE DETECTED"
            cv2.putText(image, obstacle_text, (width//2 - 100, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
        
        # Draw goal info
        if self.goal_pose is not None and self.current_pose is not None:
            dist = self.calculate_distance_to_goal()
            heading_error = self.calculate_heading_error()
            goal_text = f"Goal: {dist:.2f}m, Heading error: {math.degrees(heading_error):.1f}°"
            cv2.putText(image, goal_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ObstacleAvoidanceNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node
        if 'node' in locals():
            # Send zero velocity command
            zero_vel = Twist()
            node.vel_pub.publish(zero_vel)
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
