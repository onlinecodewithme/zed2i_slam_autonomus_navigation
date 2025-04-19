#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from nav2_msgs.action import NavigateToPose, ComputePathToPose, FollowPath
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from visualization_msgs.msg import MarkerArray
from robot_interfaces.msg import AirplaneDetection
from robot_interfaces.msg import InspectionStatus
from std_msgs.msg import Header

import numpy as np
import math
import tf2_ros
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose
import time
import threading
import copy

class AutonomousAirplaneInspector(Node):
    def __init__(self):
        super().__init__('autonomous_airplane_inspector')
        self.get_logger().info('Starting Autonomous Airplane Inspector')
        
        # Create callback group for threading
        self.callback_group = ReentrantCallbackGroup()
        
        # Initialize robot state variables
        self.current_pose = None
        self.goal_pose = None
        self.obstacles = None
        self.airplane_detection = None
        self.inspection_status = InspectionStatus()
        self.inspection_status.header = Header()
        self.inspection_status.state = InspectionStatus.IDLE
        self.inspection_status.progress = 0.0
        self.inspection_status.current_pose = Pose()
        self.inspection_status.target_pose = Pose()
        self.inspection_status.status_message = "System initialized"
        
        # Additional tracking variables not in message
        self.current_waypoint = 0
        self.total_waypoints = 0
        self.costmap = None
        self.path = None
        self.inspection_path = []
        self.airplane_detected = False
        self.is_navigating = False
        
        # TF Buffer for coordinate transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Create action clients
        self.nav_to_pose_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',
            callback_group=self.callback_group
        )
        
        self.compute_path_client = ActionClient(
            self,
            ComputePathToPose,
            'compute_path_to_pose',
            callback_group=self.callback_group
        )
        
        self.follow_path_client = ActionClient(
            self,
            FollowPath,
            'follow_path',
            callback_group=self.callback_group
        )
        
        # Create subscribers
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom', 
            self.odom_callback,
            10,
            callback_group=self.callback_group
        )
        
        self.obstacles_subscription = self.create_subscription(
            MarkerArray,
            '/obstacles/markers',
            self.obstacles_callback,
            10,
            callback_group=self.callback_group
        )
        
        # Subscribe to costmap - can use either local or global costmap topic
        costmap_topic = self.declare_parameter('costmap_topic', '/global_costmap/costmap').value
        self.get_logger().info(f'Subscribing to costmap topic: {costmap_topic}')
        
        self.costmap_subscription = self.create_subscription(
            OccupancyGrid,
            costmap_topic,
            self.costmap_callback,
            10,
            callback_group=self.callback_group
        )
        
        self.airplane_detection_subscription = self.create_subscription(
            AirplaneDetection,
            '/airplane_detection',
            self.airplane_detection_callback,
            10,
            callback_group=self.callback_group
        )
        
        # Create publishers
        self.inspection_status_publisher = self.create_publisher(
            InspectionStatus,
            '/inspection_status',
            10
        )
        
        self.path_publisher = self.create_publisher(
            Path,
            '/inspection_path',
            10
        )
        
        # Timer for status updates
        self.status_timer = self.create_timer(
            1.0, 
            self.publish_status,
            callback_group=self.callback_group
        )
        
        # Wait for the action servers
        self.wait_for_action_servers()
        
        # Initialize navigation planner
        self.inspection_planner = AirplaneInspectionPlanner()
        
    def wait_for_action_servers(self):
        """Wait for the Nav2 action servers to become available"""
        self.get_logger().info('Waiting for Nav2 action servers...')
        self.nav_to_pose_client.wait_for_server()
        self.compute_path_client.wait_for_server()
        self.follow_path_client.wait_for_server()
        self.get_logger().info('Nav2 action servers are available!')
        
    def odom_callback(self, msg):
        """Update current robot pose from odometry messages"""
        self.current_pose = msg.pose.pose
        
    def obstacles_callback(self, msg):
        """Process obstacle marker information"""
        self.obstacles = msg
        
    def costmap_callback(self, msg):
        """Update the current costmap"""
        self.costmap = msg
        
        # Log costmap receipt periodically to verify data flow
        if hasattr(self, 'costmap_log_counter'):
            self.costmap_log_counter += 1
        else:
            self.costmap_log_counter = 0
            
        # Only log occasionally to avoid flooding
        if self.costmap_log_counter % 30 == 0:
            width = msg.info.width
            height = msg.info.height
            resolution = msg.info.resolution
            
            # Count cells by type
            free_cells = sum(1 for cell in msg.data if cell == 0)
            occupied_cells = sum(1 for cell in msg.data if cell > 0)
            unknown_cells = sum(1 for cell in msg.data if cell < 0)
            
            total_cells = width * height
            self.get_logger().info(f"Received costmap: {width}x{height} cells ({width*resolution:.1f}x{height*resolution:.1f}m)")
            self.get_logger().info(f"  Free: {free_cells}/{total_cells} ({free_cells/total_cells*100:.1f}%)")
            self.get_logger().info(f"  Occupied: {occupied_cells}/{total_cells} ({occupied_cells/total_cells*100:.1f}%)")
            self.get_logger().info(f"  Unknown: {unknown_cells}/{total_cells} ({unknown_cells/total_cells*100:.1f}%)")
        
    def airplane_detection_callback(self, msg):
        """Process airplane detection information"""
        self.airplane_detection = msg
        if not self.airplane_detected and msg.confidence > 0.7:
            self.airplane_detected = True
            self.get_logger().info(f'Airplane detected at position: ({msg.pose.position.x}, {msg.pose.position.y}, {msg.pose.position.z})')
            self.plan_inspection_path(msg.pose)
        
    def plan_inspection_path(self, airplane_pose):
        """Generate a path to inspect the airplane"""
        self.get_logger().info('Planning inspection path around airplane...')
        
        # Check if we have a valid costmap
        if self.costmap is None:
            self.get_logger().warn('No costmap available for planning. Using default path generation.')
        
        # Use the inspection planner to generate waypoints
        self.inspection_path = self.inspection_planner.generate_inspection_path(
            airplane_pose,
            self.current_pose,
            robot_size={'length': 1.07, 'width': 0.82, 'height': 0.68},
            safety_margin=1.0
        )
        
        self.get_logger().info(f'Generated {len(self.inspection_path)} inspection waypoints')
        
        # Update status
        self.inspection_status.state = InspectionStatus.NAVIGATING
        self.total_waypoints = len(self.inspection_path)
        self.current_waypoint = 0
        self.inspection_status.status_message = f"Planning inspection path with {self.total_waypoints} waypoints"
        
        # Publish the planned path for visualization
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for waypoint in self.inspection_path:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose = waypoint
            path_msg.poses.append(pose)
            
        self.path_publisher.publish(path_msg)
        self.path = path_msg
        
        # Start navigation
        self.execute_inspection()
        
    def execute_inspection(self):
        """Execute the inspection by navigating through the waypoints"""
        if len(self.inspection_path) == 0:
            self.get_logger().warn('No inspection path available')
            return
            
        self.inspection_status.state = InspectionStatus.INSPECTING
        self.inspection_status.status_message = "Executing inspection plan"
        
        # Start navigation in a separate thread to avoid blocking
        navigation_thread = threading.Thread(target=self.navigate_through_waypoints)
        navigation_thread.daemon = True
        navigation_thread.start()
        
    def navigate_through_waypoints(self):
        """Navigate through all waypoints in the inspection path"""
        self.is_navigating = True
        
        for i, waypoint_pose in enumerate(self.inspection_path):
            if not rclpy.ok():
                break
                
            self.current_waypoint = i + 1
            self.inspection_status.status_message = f"Navigating to waypoint {i+1}/{len(self.inspection_path)}"
            self.inspection_status.target_pose = waypoint_pose
            self.inspection_status.progress = float(i + 1) / len(self.inspection_path)
            
            self.get_logger().info(f'Navigating to waypoint {i+1}/{len(self.inspection_path)}')
            
            # Create a PoseStamped message for the goal
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.pose = waypoint_pose
            
            # Send the goal to Nav2
            self.send_goal(goal_pose)
            
            # Brief pause between waypoints
            time.sleep(1.0)
            
        self.inspection_status.state = InspectionStatus.COMPLETED
        self.inspection_status.progress = 1.0
        self.inspection_status.status_message = "Inspection completed successfully"
        self.is_navigating = False
        self.get_logger().info('Inspection completed!')
        
    def send_goal(self, goal_pose):
        """Send a navigation goal to Nav2 and wait for completion"""
        self.get_logger().info(f'Navigating to: ({goal_pose.pose.position.x:.2f}, {goal_pose.pose.position.y:.2f})')
        
        # Verify we have a costmap for navigation
        if self.costmap is None:
            self.get_logger().warn('No costmap available for navigation. This may cause navigation to fail.')
        
        # Create the NavigateToPose goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        
        # Send the goal
        send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        
        # Wait for acceptance
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Goal was rejected!')
            return False
            
        self.get_logger().info('Goal accepted!')
        
        # Get the result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        status = result_future.result().status
        if status == 4:  # SUCCEEDED
            self.get_logger().info('Goal reached successfully!')
            return True
        else:
            self.get_logger().warn(f'Goal failed with status: {status}')
            return False
            
    def feedback_callback(self, feedback_msg):
        """Process feedback from Nav2 during navigation"""
        feedback = feedback_msg.feedback
        # Update with current navigation progress if needed
        
    def publish_status(self):
        """Publish the current inspection status"""
        self.inspection_status.header.stamp = self.get_clock().now().to_msg()
        self.inspection_status_publisher.publish(self.inspection_status)


class AirplaneInspectionPlanner:
    """Generate inspection paths around an airplane"""
    
    def __init__(self):
        self.safety_distance = 1.5  # meters
        
    def generate_inspection_path(self, airplane_pose, robot_pose, robot_size, safety_margin=1.0):
        """
        Generate a path to inspect the airplane
        
        Parameters:
            airplane_pose: Pose of the detected airplane
            robot_pose: Current robot pose
            robot_size: Dictionary with robot dimensions (length, width, height)
            safety_margin: Additional safety margin in meters
        
        Returns:
            List of Pose objects representing the inspection path
        """
        # For this initial implementation, we'll create a simplified 
        # inspection path that circles the airplane
        
        # Estimate airplane dimensions based on a typical commercial aircraft
        # In a real implementation, this would come from the detection model
        airplane_length = 40.0  # meters
        airplane_width = 35.0   # meters (wingspan)
        
        # Calculate safe distance from airplane
        min_distance = max(robot_size['length'], robot_size['width']) + safety_margin
        inspection_radius = max(airplane_length, airplane_width) / 2.0 + min_distance
        
        # Create a circular path around the airplane
        waypoints = []
        center_x = airplane_pose.position.x
        center_y = airplane_pose.position.y
        
        # Generate waypoints along a circle at 45-degree intervals
        num_points = 8
        for i in range(num_points):
            angle = 2.0 * math.pi * i / num_points
            
            # Calculate position on the circle
            x = center_x + inspection_radius * math.cos(angle)
            y = center_y + inspection_radius * math.sin(angle)
            
            # Create pose oriented toward the airplane
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = 0.0
            
            # Calculate quaternion to face the airplane
            angle_to_center = math.atan2(center_y - y, center_x - x)
            pose.orientation.z = math.sin(angle_to_center / 2.0)
            pose.orientation.w = math.cos(angle_to_center / 2.0)
            
            waypoints.append(pose)
        
        # Add a final waypoint to return to the starting position
        waypoints.append(copy.deepcopy(waypoints[0]))
        
        return waypoints


def main(args=None):
    rclpy.init(args=args)
    
    inspector_node = AutonomousAirplaneInspector()
    
    # Use a multi-threaded executor for concurrent callbacks
    executor = MultiThreadedExecutor()
    executor.add_node(inspector_node)
    
    try:
        inspector_node.get_logger().info('Autonomous Airplane Inspector Running')
        executor.spin()
    except KeyboardInterrupt:
        inspector_node.get_logger().info('Keyboard interrupt, shutting down')
    finally:
        executor.shutdown()
        inspector_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
