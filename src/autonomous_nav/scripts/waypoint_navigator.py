#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Twist, Point
from std_msgs.msg import Header
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs

import math
from enum import Enum
import numpy as np
import threading
from typing import List, Optional, Tuple

class NavigationState(Enum):
    """State machine for waypoint navigation"""
    IDLE = 0
    NAVIGATING = 1
    ROTATING = 2
    GOAL_REACHED = 3
    ERROR = 4

class WaypointNavigatorNode(Node):
    """
    ROS2 node for navigating through waypoints in a path
    """
    def __init__(self):
        super().__init__('waypoint_navigator')
        
        # Declare parameters
        self.declare_parameter('path_topic', '/path')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('goal_reached_topic', '/goal_reached')
        self.declare_parameter('max_linear_speed', 0.5)  # m/s
        self.declare_parameter('max_angular_speed', 1.0)  # rad/s
        self.declare_parameter('goal_tolerance', 0.2)  # meters
        self.declare_parameter('heading_tolerance', 0.1)  # radians
        self.declare_parameter('lookahead_distance', 1.0)  # meters
        self.declare_parameter('update_rate', 10.0)  # Hz
        
        # Get parameters
        self.path_topic = self.get_parameter('path_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.goal_reached_topic = self.get_parameter('goal_reached_topic').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.heading_tolerance = self.get_parameter('heading_tolerance').value
        self.lookahead_distance = self.get_parameter('lookahead_distance').value
        self.update_rate = self.get_parameter('update_rate').value
        
        # Initialize TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Initialize publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            10
        )
        
        self.goal_reached_pub = self.create_publisher(
            PoseStamped,
            self.goal_reached_topic,
            10
        )
        
        # Initialize subscribers
        self.path_sub = self.create_subscription(
            Path,
            self.path_topic,
            self.path_callback,
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10
        )
        
        # Initialize navigation state
        self.state = NavigationState.IDLE
        self.current_path = None
        self.current_waypoint_idx = 0
        self.current_pose = None
        self.current_velocity = Twist()
        
        # Processing lock to prevent concurrent processing
        self.processing_lock = threading.Lock()
        
        # Create timer for navigation update
        update_period = 1.0 / self.update_rate
        self.nav_timer = self.create_timer(update_period, self.navigation_callback)
        
        self.get_logger().info('Waypoint navigator node initialized')
    
    def path_callback(self, msg):
        """
        Callback for new path
        """
        if not msg.poses:
            self.get_logger().warn('Received empty path')
            return
        
        self.get_logger().info(f'Received new path with {len(msg.poses)} waypoints')
        self.current_path = msg
        self.current_waypoint_idx = 0
        self.state = NavigationState.NAVIGATING
    
    def odom_callback(self, msg):
        """
        Callback for odometry data
        """
        self.current_pose = msg.pose.pose
        self.current_velocity = msg.twist.twist
    
    def navigation_callback(self):
        """
        Main navigation callback that runs at regular intervals
        """
        if self.current_path is None or self.current_pose is None:
            return
        
        # Acquire lock to prevent concurrent processing
        if not self.processing_lock.acquire(blocking=False):
            return
        
        try:
            # Update navigation based on current state
            if self.state == NavigationState.NAVIGATING:
                self.navigate_to_waypoint()
            elif self.state == NavigationState.ROTATING:
                self.rotate_to_target()
            elif self.state == NavigationState.GOAL_REACHED:
                self.handle_goal_reached()
            elif self.state == NavigationState.ERROR:
                self.handle_error()
        finally:
            # Release lock
            self.processing_lock.release()
    
    def navigate_to_waypoint(self):
        """
        Navigate to the current waypoint
        """
        # Get current waypoint
        current_waypoint = self.current_path.poses[self.current_waypoint_idx]
        
        # Calculate distance to waypoint
        distance = self.calculate_distance(
            self.current_pose.position, 
            current_waypoint.pose.position
        )
        
        # If waypoint reached, move to next waypoint
        if distance < self.goal_tolerance:
            self.get_logger().debug(f'Reached waypoint {self.current_waypoint_idx}')
            
            # Move to next waypoint if available
            if self.current_waypoint_idx < len(self.current_path.poses) - 1:
                self.current_waypoint_idx += 1
                return
            else:
                # Final goal reached
                self.state = NavigationState.GOAL_REACHED
                self.get_logger().info('Final goal reached')
                
                # Stop robot
                self.send_zero_velocity()
                return
        
        # Find target point along the path using lookahead distance
        target_pose = self.find_lookahead_point()
        
        if target_pose is None:
            # Use current waypoint if no lookahead point found
            target_pose = current_waypoint.pose
        
        # Calculate heading error
        heading_error = self.calculate_heading_error(target_pose.position)
        
        # If heading error is too large, switch to rotation mode
        if abs(heading_error) > self.heading_tolerance * 3:  # 3x threshold for switching modes
            self.state = NavigationState.ROTATING
            return
        
        # Calculate velocity commands using pure pursuit controller
        cmd_vel = self.calculate_pure_pursuit_velocity(target_pose, distance, heading_error)
        
        # Send velocity command
        self.cmd_vel_pub.publish(cmd_vel)
    
    def rotate_to_target(self):
        """
        Rotate to align with the target waypoint
        """
        # Get current waypoint
        current_waypoint = self.current_path.poses[self.current_waypoint_idx]
        
        # Calculate heading error
        heading_error = self.calculate_heading_error(current_waypoint.pose.position)
        
        # If heading aligned, switch back to navigation
        if abs(heading_error) < self.heading_tolerance:
            self.state = NavigationState.NAVIGATING
            return
        
        # Calculate rotation velocity
        cmd_vel = Twist()
        
        # Proportional controller for rotation
        # Scale down angular velocity as we get closer to the target heading
        rotation_scale = min(1.0, abs(heading_error) / (self.heading_tolerance * 5))
        angular_velocity = self.max_angular_speed * rotation_scale
        
        # Set direction based on sign of heading error
        cmd_vel.angular.z = angular_velocity if heading_error > 0 else -angular_velocity
        
        # Send velocity command
        self.cmd_vel_pub.publish(cmd_vel)
    
    def handle_goal_reached(self):
        """
        Handle goal reached state
        """
        # Stop robot
        self.send_zero_velocity()
        
        # Publish goal reached message
        if self.current_path and self.current_path.poses:
            self.goal_reached_pub.publish(self.current_path.poses[-1])
            
        # Reset path
        self.current_path = None
        self.state = NavigationState.IDLE
    
    def handle_error(self):
        """
        Handle error state
        """
        # Stop robot
        self.send_zero_velocity()
        
        # Log error
        self.get_logger().error('Navigation error occurred')
        
        # Reset state
        self.state = NavigationState.IDLE
        self.current_path = None
    
    def calculate_distance(self, point1: Point, point2: Point) -> float:
        """
        Calculate Euclidean distance between two points
        """
        return math.sqrt(
            (point2.x - point1.x) ** 2 +
            (point2.y - point1.y) ** 2
        )
    
    def calculate_heading_error(self, target_position: Point) -> float:
        """
        Calculate heading error to target position
        """
        # Calculate target heading
        dx = target_position.x - self.current_pose.position.x
        dy = target_position.y - self.current_pose.position.y
        target_heading = math.atan2(dy, dx)
        
        # Get current heading from quaternion
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
    
    def find_lookahead_point(self) -> Optional[PoseStamped]:
        """
        Find a point along the path at lookahead distance
        
        Returns:
            Target pose or None if not found
        """
        if self.current_waypoint_idx >= len(self.current_path.poses):
            return None
        
        # Start with current waypoint
        current_waypoint = self.current_path.poses[self.current_waypoint_idx]
        
        # If this is the last waypoint, just return it
        if self.current_waypoint_idx == len(self.current_path.poses) - 1:
            return current_waypoint
        
        # Find a point along the path that is approximately lookahead_distance away
        current_distance = self.calculate_distance(
            self.current_pose.position,
            current_waypoint.pose.position
        )
        
        # If current waypoint is beyond lookahead distance, use it
        if current_distance >= self.lookahead_distance:
            return current_waypoint
        
        # Search along remaining waypoints for a point close to lookahead distance
        remaining_distance = self.lookahead_distance - current_distance
        
        for i in range(self.current_waypoint_idx, len(self.current_path.poses) - 1):
            # Calculate distance between consecutive waypoints
            wp_distance = self.calculate_distance(
                self.current_path.poses[i].pose.position,
                self.current_path.poses[i+1].pose.position
            )
            
            # If this segment contains the lookahead point
            if remaining_distance <= wp_distance:
                # Interpolate between waypoints
                alpha = remaining_distance / wp_distance
                
                # Create interpolated pose
                target_pose = PoseStamped()
                target_pose.header = self.current_path.header
                
                # Interpolate position
                p1 = self.current_path.poses[i].pose.position
                p2 = self.current_path.poses[i+1].pose.position
                
                target_pose.pose.position.x = p1.x + alpha * (p2.x - p1.x)
                target_pose.pose.position.y = p1.y + alpha * (p2.y - p1.y)
                target_pose.pose.position.z = p1.z + alpha * (p2.z - p1.z)
                
                # Use orientation of next waypoint
                target_pose.pose.orientation = self.current_path.poses[i+1].pose.orientation
                
                return target_pose
            
            # Update remaining distance
            remaining_distance -= wp_distance
        
        # If no point found, use the last waypoint
        return self.current_path.poses[-1]
    
    def calculate_pure_pursuit_velocity(self, target_pose, distance, heading_error) -> Twist:
        """
        Calculate velocity command using pure pursuit controller
        
        Args:
            target_pose: Target pose to pursue
            distance: Distance to the current waypoint
            heading_error: Error in heading angle
            
        Returns:
            Twist message with velocity commands
        """
        cmd_vel = Twist()
        
        # Scale linear velocity based on distance and heading error
        # Slow down when approaching waypoint or when heading error is large
        distance_factor = min(1.0, distance / self.lookahead_distance)
        heading_factor = max(0.0, 1.0 - abs(heading_error) / math.pi)
        
        # Linear velocity
        cmd_vel.linear.x = self.max_linear_speed * distance_factor * heading_factor
        
        # Angular velocity proportional to heading error
        # Use a gain to determine how aggressively to turn
        gain = 1.0
        cmd_vel.angular.z = self.clamp(
            gain * heading_error,
            -self.max_angular_speed,
            self.max_angular_speed
        )
        
        return cmd_vel
    
    def clamp(self, value, min_val, max_val):
        """
        Clamp a value between min and max
        """
        return max(min_val, min(max_val, value))
    
    def send_zero_velocity(self):
        """
        Send zero velocity command to stop the robot
        """
        cmd_vel = Twist()
        self.cmd_vel_pub.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = WaypointNavigatorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            # Send zero velocity for safety
            node.send_zero_velocity()
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
