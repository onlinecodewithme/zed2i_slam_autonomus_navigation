#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from nav2_msgs.action import NavigateToPose, FollowWaypoints
from std_msgs.msg import Empty
from visualization_msgs.msg import MarkerArray, Marker
import tf_transformations
import math
import yaml
import threading
import time
from typing import List, Dict, Any

class WaypointManager(Node):
    """
    Manages navigation waypoints for autonomous inspection missions.
    Supports waypoint loading, sending to Nav2, and monitoring execution.
    Integrates with obstacle avoidance and intelligent path planning.
    """
    def __init__(self):
        super().__init__('waypoint_manager')
        
        # Declare parameters
        self.declare_parameter('waypoint_file', '')
        self.declare_parameter('loop_waypoints', False)
        self.declare_parameter('wait_time_at_waypoint', 1.0)  # seconds
        self.declare_parameter('obstacle_check_radius', 1.0)  # meters
        self.declare_parameter('small_obstacle_threshold', 0.3)  # meters (height)
        
        # Get parameters
        self.waypoint_file = self.get_parameter('waypoint_file').value
        self.loop_waypoints = self.get_parameter('loop_waypoints').value
        self.wait_time = self.get_parameter('wait_time_at_waypoint').value
        self.obstacle_check_radius = self.get_parameter('obstacle_check_radius').value
        self.small_obstacle_threshold = self.get_parameter('small_obstacle_threshold').value
        
        # Initialize variables
        self.waypoints = []
        self.current_waypoint_idx = 0
        self.is_navigating = False
        self.is_inspection_paused = False
        self.obstacles = []
        
        # Create action clients for navigation
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.follow_waypoints_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        
        # Publishers for visualization
        self.waypoint_markers_pub = self.create_publisher(MarkerArray, '/waypoints/markers', 10)
        self.path_markers_pub = self.create_publisher(Marker, '/waypoints/path', 10)
        
        # Subscribers
        self.obstacles_sub = self.create_subscription(
            MarkerArray, 
            '/obstacles/markers', 
            self.obstacles_callback, 
            10
        )
        
        # Services
        self.pause_srv = self.create_subscription(Empty, '/waypoints/pause', self.pause_callback, 10)
        self.resume_srv = self.create_subscription(Empty, '/waypoints/resume', self.resume_callback, 10)
        self.cancel_srv = self.create_subscription(Empty, '/waypoints/cancel', self.cancel_callback, 10)
        
        # Timers
        self.create_timer(1.0, self.publish_visualization)
        
        # If waypoint file provided, load it
        if self.waypoint_file:
            self.load_waypoints(self.waypoint_file)
            
        self.get_logger().info('Waypoint Manager initialized')
            
    def load_waypoints(self, file_path: str) -> bool:
        """
        Load waypoints from a YAML file
        
        Args:
            file_path: Path to the YAML file
            
        Returns:
            bool: True if successful, False otherwise
        """
        try:
            with open(file_path, 'r') as f:
                data = yaml.safe_load(f)
                
            if 'waypoints' not in data:
                self.get_logger().error('Invalid waypoint file: "waypoints" key not found')
                return False
                
            self.waypoints = []
            for wp in data['waypoints']:
                if 'position' not in wp or 'x' not in wp['position'] or 'y' not in wp['position']:
                    self.get_logger().error(f'Invalid waypoint: {wp}')
                    continue
                    
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = wp['position']['x']
                pose.pose.position.y = wp['position']['y']
                pose.pose.position.z = wp.get('position', {}).get('z', 0.0)
                
                # Handle orientation
                if 'orientation' in wp and all(k in wp['orientation'] for k in ['x', 'y', 'z', 'w']):
                    pose.pose.orientation.x = wp['orientation']['x']
                    pose.pose.orientation.y = wp['orientation']['y']
                    pose.pose.orientation.z = wp['orientation']['z']
                    pose.pose.orientation.w = wp['orientation']['w']
                else:
                    # Default orientation (forward)
                    pose.pose.orientation.w = 1.0
                
                self.waypoints.append(pose)
                
            self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints')
            self.publish_visualization()
            return True
            
        except Exception as e:
            self.get_logger().error(f'Error loading waypoints: {str(e)}')
            return False
            
    def start_navigation(self) -> bool:
        """
        Start navigating through all waypoints
        
        Returns:
            bool: True if navigation started, False otherwise
        """
        if not self.waypoints:
            self.get_logger().error('No waypoints to navigate to')
            return False
            
        if self.is_navigating:
            self.get_logger().info('Already navigating')
            return False
            
        self.get_logger().info('Starting waypoint navigation')
        self.is_navigating = True
        self.current_waypoint_idx = 0
        
        # Start navigation thread to avoid blocking
        self.nav_thread = threading.Thread(target=self.navigation_worker)
        self.nav_thread.daemon = True
        self.nav_thread.start()
        
        return True
        
    def navigation_worker(self):
        """
        Worker thread for navigation to handle waypoints sequentially
        """
        while (self.is_navigating and 
               self.current_waypoint_idx < len(self.waypoints)):
            
            # Check if navigation is paused
            if self.is_inspection_paused:
                time.sleep(0.5)
                continue
                
            # Get current waypoint
            current_wp = self.waypoints[self.current_waypoint_idx]
            self.get_logger().info(f'Navigating to waypoint {self.current_waypoint_idx+1}/{len(self.waypoints)}: '
                           f'({current_wp.pose.position.x}, {current_wp.pose.position.y})')
            
            # Check for obstacles near the waypoint
            if self.check_waypoint_obstacles(current_wp):
                self.get_logger().info('Waypoint has obstacles, checking if reroute needed')
                
            # Create goal
            current_wp.header.stamp = self.get_clock().now().to_msg()
            goal = NavigateToPose.Goal()
            goal.pose = current_wp
            
            # Send goal and wait for result
            self.nav_to_pose_client.wait_for_server()
            result_future = self.nav_to_pose_client.send_goal_async(goal)
            
            # Wait for goal to be accepted
            goal_handle_future = result_future.result()
            if not goal_handle_future.accepted:
                self.get_logger().error(f'Goal to waypoint {self.current_waypoint_idx+1} was rejected')
                # Try next waypoint if this one fails
                self.current_waypoint_idx += 1
                continue
                
            self.get_logger().info(f'Goal to waypoint {self.current_waypoint_idx+1} accepted')
            
            # Wait for navigation result
            result_future = goal_handle_future.get_result_async()
            result_future.add_done_callback(self.navigation_result_callback)
            
            # Wait until navigation is complete to continue to next waypoint
            while not result_future.done() and self.is_navigating and not self.is_inspection_paused:
                time.sleep(0.1)
                
            # Wait at waypoint if not paused
            if self.is_navigating and not self.is_inspection_paused:
                self.get_logger().info(f'Waiting at waypoint {self.current_waypoint_idx+1} for {self.wait_time} seconds')
                time.sleep(self.wait_time)
                
            # Move to next waypoint
            self.current_waypoint_idx += 1
            
        # If we completed the waypoints and loop is enabled, start over
        if self.is_navigating and self.loop_waypoints and self.current_waypoint_idx >= len(self.waypoints):
            self.get_logger().info('Completed all waypoints, looping back to start')
            self.current_waypoint_idx = 0
            self.navigation_worker()  # Recursive call to continue loop
        elif self.is_navigating:
            self.get_logger().info('Completed all waypoints')
            self.is_navigating = False
            
    def navigation_result_callback(self, future):
        """
        Callback for navigation action result
        
        Args:
            future: Future containing the action result
        """
        result = future.result().result
        if result:
            self.get_logger().info(f'Reached waypoint {self.current_waypoint_idx+1}')
        else:
            self.get_logger().warn(f'Failed to reach waypoint {self.current_waypoint_idx+1}')
            
    def check_waypoint_obstacles(self, waypoint: PoseStamped) -> bool:
        """
        Check if there are obstacles near the waypoint
        
        Args:
            waypoint: The waypoint to check
            
        Returns:
            bool: True if obstacles found, False otherwise
        """
        has_obstacles = False
        
        for obstacle in self.obstacles:
            # Calculate distance to obstacle
            dx = obstacle.x - waypoint.pose.position.x
            dy = obstacle.y - waypoint.pose.position.y
            distance = math.sqrt(dx*dx + dy*dy)
            
            # Check if obstacle is within range
            if distance <= self.obstacle_check_radius:
                # Check obstacle height to determine if it's a significant obstacle
                if obstacle.height > self.small_obstacle_threshold:
                    self.get_logger().info(f'Large obstacle detected near waypoint {self.current_waypoint_idx+1}')
                    has_obstacles = True
                else:
                    self.get_logger().info(f'Small obstacle detected near waypoint {self.current_waypoint_idx+1}, can traverse')
                    
        return has_obstacles
        
    def obstacles_callback(self, msg):
        """
        Callback for obstacle markers
        
        Args:
            msg: MarkerArray message with obstacles
        """
        self.obstacles = []
        
        for marker in msg.markers:
            if marker.type == Marker.CYLINDER:
                # Extract obstacle properties (position, dimensions)
                obstacle = type('Obstacle', (), {
                    'x': marker.pose.position.x,
                    'y': marker.pose.position.y,
                    'z': marker.pose.position.z,
                    'radius': marker.scale.x / 2.0,  # Scale x is diameter
                    'height': marker.scale.z,
                    'is_dynamic': marker.color.r > 0.5  # Red color indicates dynamic obstacle
                })
                self.obstacles.append(obstacle)
                
    def pause_callback(self, msg):
        """
        Pause the waypoint navigation
        """
        if self.is_navigating and not self.is_inspection_paused:
            self.get_logger().info('Pausing waypoint navigation')
            self.is_inspection_paused = True
            
    def resume_callback(self, msg):
        """
        Resume the waypoint navigation
        """
        if self.is_navigating and self.is_inspection_paused:
            self.get_logger().info('Resuming waypoint navigation')
            self.is_inspection_paused = False
            
    def cancel_callback(self, msg):
        """
        Cancel the current navigation
        """
        if self.is_navigating:
            self.get_logger().info('Canceling waypoint navigation')
            self.is_navigating = False
            self.is_inspection_paused = False
            
            # Cancel any active goals
            self.nav_to_pose_client.wait_for_server()
            cancel_future = self.nav_to_pose_client._cancel_goal_async()
            
    def publish_visualization(self):
        """
        Publish visualization markers for waypoints and path
        """
        if not self.waypoints:
            return
            
        # Create waypoint markers
        marker_array = MarkerArray()
        
        for i, waypoint in enumerate(self.waypoints):
            # Waypoint marker
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'waypoints'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose = waypoint.pose
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            
            # Current waypoint is red, others are blue
            if i == self.current_waypoint_idx and self.is_navigating:
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            else:
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
            marker.color.a = 1.0
            
            marker_array.markers.append(marker)
            
            # Waypoint label
            text_marker = Marker()
            text_marker.header.frame_id = 'map'
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = 'waypoint_labels'
            text_marker.id = i + 1000  # Different ID range for labels
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            text_marker.pose = waypoint.pose
            text_marker.pose.position.z += 0.5  # Place text above waypoint
            
            text_marker.text = str(i + 1)  # Waypoint number
            
            text_marker.scale.z = 0.3  # Text height
            
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            
            marker_array.markers.append(text_marker)
            
        # Publish waypoint markers
        self.waypoint_markers_pub.publish(marker_array)
        
        # Create path marker
        path_marker = Marker()
        path_marker.header.frame_id = 'map'
        path_marker.header.stamp = self.get_clock().now().to_msg()
        path_marker.ns = 'waypoint_path'
        path_marker.id = 0
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        
        path_marker.scale.x = 0.05  # Line width
        
        path_marker.color.r = 0.0
        path_marker.color.g = 1.0
        path_marker.color.b = 0.0
        path_marker.color.a = 0.7
        
        # Add points to path
        for waypoint in self.waypoints:
            path_marker.points.append(waypoint.pose.position)
            
        # Add the first point again to close the loop if looping
        if self.loop_waypoints and self.waypoints:
            path_marker.points.append(self.waypoints[0].pose.position)
            
        # Publish path marker
        self.path_markers_pub.publish(path_marker)
        
    def save_waypoints(self, file_path: str) -> bool:
        """
        Save current waypoints to a YAML file
        
        Args:
            file_path: Path to save the YAML file
            
        Returns:
            bool: True if successful, False otherwise
        """
        try:
            data = {'waypoints': []}
            
            for wp in self.waypoints:
                wp_dict = {
                    'position': {
                        'x': wp.pose.position.x,
                        'y': wp.pose.position.y,
                        'z': wp.pose.position.z
                    },
                    'orientation': {
                        'x': wp.pose.orientation.x,
                        'y': wp.pose.orientation.y,
                        'z': wp.pose.orientation.z,
                        'w': wp.pose.orientation.w
                    }
                }
                data['waypoints'].append(wp_dict)
                
            with open(file_path, 'w') as f:
                yaml.dump(data, f, default_flow_style=False)
                
            self.get_logger().info(f'Saved {len(self.waypoints)} waypoints to {file_path}')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Error saving waypoints: {str(e)}')
            return False
            
    def add_waypoint(self, pose: PoseStamped):
        """
        Add a new waypoint to the list
        
        Args:
            pose: PoseStamped for the new waypoint
        """
        self.waypoints.append(pose)
        self.get_logger().info(f'Added waypoint {len(self.waypoints)} at '
                      f'({pose.pose.position.x}, {pose.pose.position.y})')
        self.publish_visualization()
        
    def clear_waypoints(self):
        """
        Clear all waypoints
        """
        self.waypoints = []
        self.get_logger().info('Cleared all waypoints')
        self.publish_visualization()
        
    def remove_waypoint(self, index: int):
        """
        Remove a waypoint by index
        
        Args:
            index: Index of waypoint to remove (0-based)
        """
        if 0 <= index < len(self.waypoints):
            self.waypoints.pop(index)
            self.get_logger().info(f'Removed waypoint {index+1}')
            self.publish_visualization()
        else:
            self.get_logger().error(f'Invalid waypoint index: {index}')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        waypoint_manager = WaypointManager()
        rclpy.spin(waypoint_manager)
    except KeyboardInterrupt:
        pass
    finally:
        if 'waypoint_manager' in locals():
            waypoint_manager.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
