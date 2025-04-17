#!/usr/bin/env python3

import math
import numpy as np
from enum import Enum
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from nav_msgs.msg import Path

class InspectionPattern(Enum):
    """Enumeration of inspection pattern types"""
    CIRCLE = 0
    SQUARE = 1
    ZIGZAG = 2
    SPIRAL = 3
    VERTICAL_SCAN = 4
    HORIZONTAL_SCAN = 5
    CUSTOM = 6

class InspectionPatternGenerator:
    """
    Class to generate inspection patterns for aircraft inspection
    """
    
    def __init__(self):
        """Initialize the pattern generator"""
        pass
    
    def generate_pattern(self, 
                         pattern_type: InspectionPattern, 
                         center: Point, 
                         size: float, 
                         num_points: int = 16,
                         height: float = 1.0,
                         orientation_towards_center: bool = True,
                         frame_id: str = "map") -> Path:
        """
        Generate an inspection pattern around a target
        
        Args:
            pattern_type: The type of pattern to generate
            center: The center point of the inspection target
            size: The size of the pattern (radius or side length)
            num_points: Number of waypoints to generate
            height: Height offset for the pattern (meters)
            orientation_towards_center: Whether poses should be oriented towards the center
            frame_id: Frame ID for the path
            
        Returns:
            Path message containing the generated pattern
        """
        if pattern_type == InspectionPattern.CIRCLE:
            return self._generate_circle(center, size, num_points, height, orientation_towards_center, frame_id)
        elif pattern_type == InspectionPattern.SQUARE:
            return self._generate_square(center, size, num_points, height, orientation_towards_center, frame_id)
        elif pattern_type == InspectionPattern.ZIGZAG:
            return self._generate_zigzag(center, size, num_points, height, orientation_towards_center, frame_id)
        elif pattern_type == InspectionPattern.SPIRAL:
            return self._generate_spiral(center, size, num_points, height, orientation_towards_center, frame_id)
        elif pattern_type == InspectionPattern.VERTICAL_SCAN:
            return self._generate_vertical_scan(center, size, num_points, height, orientation_towards_center, frame_id)
        elif pattern_type == InspectionPattern.HORIZONTAL_SCAN:
            return self._generate_horizontal_scan(center, size, num_points, height, orientation_towards_center, frame_id)
        else:
            # Default to circle pattern
            return self._generate_circle(center, size, num_points, height, orientation_towards_center, frame_id)
    
    def _generate_circle(self, 
                        center: Point, 
                        radius: float, 
                        num_points: int = 16,
                        height: float = 1.0,
                        orientation_towards_center: bool = True,
                        frame_id: str = "map") -> Path:
        """
        Generate a circular inspection pattern around a target
        
        Args:
            center: The center point of the inspection target
            radius: The radius of the circle
            num_points: Number of waypoints to generate
            height: Height offset for the pattern (meters)
            orientation_towards_center: Whether poses should be oriented towards the center
            frame_id: Frame ID for the path
            
        Returns:
            Path message containing the circular pattern
        """
        # Create a path message
        path = Path()
        path.header.frame_id = frame_id
        
        # Generate points along the circle
        for i in range(num_points + 1):  # +1 to close the loop
            # Calculate angle
            angle = 2.0 * math.pi * i / num_points
            
            # Calculate position
            x = center.x + radius * math.cos(angle)
            y = center.y + radius * math.sin(angle)
            z = center.z + height
            
            # Create pose
            pose = PoseStamped()
            pose.header.frame_id = frame_id
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            
            # Set orientation (towards center or along path)
            if orientation_towards_center:
                # Calculate quaternion to look at center
                dx = center.x - x
                dy = center.y - y
                yaw = math.atan2(dy, dx)
                
                # Create quaternion from yaw (simple 2D case)
                pose.pose.orientation.z = math.sin(yaw / 2.0)
                pose.pose.orientation.w = math.cos(yaw / 2.0)
            else:
                # Orientation along the path (tangent to circle)
                yaw = angle + math.pi / 2.0  # Tangent direction
                
                # Create quaternion from yaw (simple 2D case)
                pose.pose.orientation.z = math.sin(yaw / 2.0)
                pose.pose.orientation.w = math.cos(yaw / 2.0)
            
            # Add pose to path
            path.poses.append(pose)
        
        return path
    
    def _generate_square(self, 
                        center: Point, 
                        side_length: float, 
                        num_points: int = 16,
                        height: float = 1.0,
                        orientation_towards_center: bool = True,
                        frame_id: str = "map") -> Path:
        """
        Generate a square inspection pattern around a target
        
        Args:
            center: The center point of the inspection target
            side_length: The side length of the square
            num_points: Number of waypoints to generate (distributed among sides)
            height: Height offset for the pattern (meters)
            orientation_towards_center: Whether poses should be oriented towards the center
            frame_id: Frame ID for the path
            
        Returns:
            Path message containing the square pattern
        """
        # Create a path message
        path = Path()
        path.header.frame_id = frame_id
        
        # Calculate half side length
        half_side = side_length / 2.0
        
        # Points per side (ensure at least 2 points per side)
        points_per_side = max(2, num_points // 4)
        
        # Generate points for each side of the square
        sides = [
            # Bottom side (left to right)
            [(center.x - half_side + i * side_length / (points_per_side - 1), 
              center.y - half_side, 
              0.0) for i in range(points_per_side)],
            
            # Right side (bottom to top)
            [(center.x + half_side, 
              center.y - half_side + i * side_length / (points_per_side - 1), 
              0.0) for i in range(points_per_side)],
            
            # Top side (right to left)
            [(center.x + half_side - i * side_length / (points_per_side - 1), 
              center.y + half_side, 
              0.0) for i in range(points_per_side)],
            
            # Left side (top to bottom)
            [(center.x - half_side, 
              center.y + half_side - i * side_length / (points_per_side - 1), 
              0.0) for i in range(points_per_side)]
        ]
        
        # Directions for each side (for orientation along path)
        directions = [
            (1.0, 0.0),    # Bottom: facing right
            (0.0, 1.0),    # Right: facing up
            (-1.0, 0.0),   # Top: facing left
            (0.0, -1.0)    # Left: facing down
        ]
        
        # Create poses for each side
        for side_idx, side in enumerate(sides):
            for point in side:
                x, y, _ = point
                
                # Create pose
                pose = PoseStamped()
                pose.header.frame_id = frame_id
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.position.z = center.z + height
                
                # Set orientation
                if orientation_towards_center:
                    # Calculate quaternion to look at center
                    dx = center.x - x
                    dy = center.y - y
                    yaw = math.atan2(dy, dx)
                else:
                    # Orientation along the path
                    dx, dy = directions[side_idx]
                    yaw = math.atan2(dy, dx)
                
                # Create quaternion from yaw (simple 2D case)
                pose.pose.orientation.z = math.sin(yaw / 2.0)
                pose.pose.orientation.w = math.cos(yaw / 2.0)
                
                # Add pose to path
                path.poses.append(pose)
        
        return path
    
    def _generate_zigzag(self, 
                         center: Point, 
                         size: float, 
                         num_points: int = 16,
                         height: float = 1.0,
                         orientation_towards_center: bool = True,
                         frame_id: str = "map") -> Path:
        """
        Generate a zigzag inspection pattern across a target
        
        Args:
            center: The center point of the inspection target
            size: The size of the area to zigzag across
            num_points: Number of waypoints to generate
            height: Height offset for the pattern (meters)
            orientation_towards_center: Whether poses should be oriented towards the center
            frame_id: Frame ID for the path
            
        Returns:
            Path message containing the zigzag pattern
        """
        # Create a path message
        path = Path()
        path.header.frame_id = frame_id
        
        # Calculate half size
        half_size = size / 2.0
        
        # Number of zigzag lines
        num_lines = max(2, int(math.sqrt(num_points)))
        
        # Points per line
        points_per_line = max(2, num_points // num_lines)
        
        # Spacing between lines
        line_spacing = size / (num_lines - 1) if num_lines > 1 else size
        
        # Generate zigzag pattern
        for i in range(num_lines):
            # Calculate y position of this line
            y = center.y - half_size + i * line_spacing
            
            # Determine direction of this line (alternate left-to-right and right-to-left)
            start_x = center.x - half_size if i % 2 == 0 else center.x + half_size
            end_x = center.x + half_size if i % 2 == 0 else center.x - half_size
            
            # Generate points along this line
            for j in range(points_per_line):
                # Calculate x position along the line
                t = j / (points_per_line - 1) if points_per_line > 1 else 0.5
                x = start_x + t * (end_x - start_x)
                
                # Create pose
                pose = PoseStamped()
                pose.header.frame_id = frame_id
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.position.z = center.z + height
                
                # Set orientation
                if orientation_towards_center:
                    # Calculate quaternion to look at center
                    dx = center.x - x
                    dy = center.y - y
                    yaw = math.atan2(dy, dx)
                else:
                    # Orientation along the path
                    dx = 1.0 if i % 2 == 0 else -1.0
                    yaw = math.atan2(0.0, dx)
                
                # Create quaternion from yaw (simple 2D case)
                pose.pose.orientation.z = math.sin(yaw / 2.0)
                pose.pose.orientation.w = math.cos(yaw / 2.0)
                
                # Add pose to path
                path.poses.append(pose)
        
        return path
    
    def _generate_spiral(self, 
                         center: Point, 
                         max_radius: float, 
                         num_points: int = 16,
                         height: float = 1.0,
                         orientation_towards_center: bool = True,
                         frame_id: str = "map") -> Path:
        """
        Generate a spiral inspection pattern around a target
        
        Args:
            center: The center point of the inspection target
            max_radius: The maximum radius of the spiral
            num_points: Number of waypoints to generate
            height: Height offset for the pattern (meters)
            orientation_towards_center: Whether poses should be oriented towards the center
            frame_id: Frame ID for the path
            
        Returns:
            Path message containing the spiral pattern
        """
        # Create a path message
        path = Path()
        path.header.frame_id = frame_id
        
        # Number of full rotations in the spiral
        num_rotations = 2.0
        
        # Generate points along the spiral
        for i in range(num_points):
            # Calculate angle and radius
            t = i / (num_points - 1) if num_points > 1 else 0
            angle = 2.0 * math.pi * num_rotations * t
            radius = max_radius * t
            
            # Calculate position
            x = center.x + radius * math.cos(angle)
            y = center.y + radius * math.sin(angle)
            z = center.z + height
            
            # Create pose
            pose = PoseStamped()
            pose.header.frame_id = frame_id
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            
            # Set orientation
            if orientation_towards_center:
                # Calculate quaternion to look at center
                dx = center.x - x
                dy = center.y - y
                yaw = math.atan2(dy, dx)
            else:
                # Orientation along the path (tangent to spiral)
                # The tangent direction is perpendicular to the radial direction
                yaw = angle + math.pi / 2.0
            
            # Create quaternion from yaw (simple 2D case)
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            
            # Add pose to path
            path.poses.append(pose)
        
        return path
    
    def _generate_vertical_scan(self, 
                              center: Point, 
                              size: float, 
                              num_points: int = 16,
                              height: float = 1.0,
                              orientation_towards_center: bool = True,
                              frame_id: str = "map") -> Path:
        """
        Generate a vertical scan pattern
        
        Args:
            center: The center point of the inspection target
            size: The size of the area to scan
            num_points: Number of waypoints to generate
            height: Height offset for the pattern (meters)
            orientation_towards_center: Whether poses should be oriented towards the center
            frame_id: Frame ID for the path
            
        Returns:
            Path message containing the vertical scan pattern
        """
        # Create a path message
        path = Path()
        path.header.frame_id = frame_id
        
        # Calculate half size
        half_size = size / 2.0
        
        # Vertical scan is similar to zigzag but with vertical lines
        # Number of vertical lines
        num_lines = max(2, int(math.sqrt(num_points)))
        
        # Points per line
        points_per_line = max(2, num_points // num_lines)
        
        # Spacing between lines
        line_spacing = size / (num_lines - 1) if num_lines > 1 else size
        
        # Generate vertical scan pattern
        for i in range(num_lines):
            # Calculate x position of this line
            x = center.x - half_size + i * line_spacing
            
            # Determine direction of this line (alternate bottom-to-top and top-to-bottom)
            start_y = center.y - half_size if i % 2 == 0 else center.y + half_size
            end_y = center.y + half_size if i % 2 == 0 else center.y - half_size
            
            # Generate points along this line
            for j in range(points_per_line):
                # Calculate y position along the line
                t = j / (points_per_line - 1) if points_per_line > 1 else 0.5
                y = start_y + t * (end_y - start_y)
                
                # Create pose
                pose = PoseStamped()
                pose.header.frame_id = frame_id
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.position.z = center.z + height
                
                # Set orientation
                if orientation_towards_center:
                    # Calculate quaternion to look at center
                    dx = center.x - x
                    dy = center.y - y
                    yaw = math.atan2(dy, dx)
                else:
                    # Orientation along the path
                    dy = 1.0 if i % 2 == 0 else -1.0
                    yaw = math.atan2(dy, 0.0)
                
                # Create quaternion from yaw (simple 2D case)
                pose.pose.orientation.z = math.sin(yaw / 2.0)
                pose.pose.orientation.w = math.cos(yaw / 2.0)
                
                # Add pose to path
                path.poses.append(pose)
        
        return path
    
    def _generate_horizontal_scan(self, 
                                center: Point, 
                                size: float, 
                                num_points: int = 16,
                                height: float = 1.0,
                                orientation_towards_center: bool = True,
                                frame_id: str = "map") -> Path:
        """
        Generate a horizontal scan pattern (same as zigzag but with different orientation)
        
        Args:
            center: The center point of the inspection target
            size: The size of the area to scan
            num_points: Number of waypoints to generate
            height: Height offset for the pattern (meters)
            orientation_towards_center: Whether poses should be oriented towards the center
            frame_id: Frame ID for the path
            
        Returns:
            Path message containing the horizontal scan pattern
        """
        # This is essentially the same as zigzag, but we force orientation_towards_center=True
        # to ensure the robot is always facing the target
        return self._generate_zigzag(center, size, num_points, height, True, frame_id)
    
    def generate_custom_pattern(self, waypoints, frame_id: str = "map") -> Path:
        """
        Generate a path from a list of custom waypoints
        
        Args:
            waypoints: List of (x, y, z, yaw) tuples
            frame_id: Frame ID for the path
            
        Returns:
            Path message containing the custom pattern
        """
        # Create a path message
        path = Path()
        path.header.frame_id = frame_id
        
        # Generate poses from waypoints
        for wp in waypoints:
            x, y, z, yaw = wp
            
            # Create pose
            pose = PoseStamped()
            pose.header.frame_id = frame_id
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            
            # Convert yaw to quaternion
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            
            # Add pose to path
            path.poses.append(pose)
        
        return path
