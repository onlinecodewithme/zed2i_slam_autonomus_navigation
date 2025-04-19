#!/usr/bin/env python3

from dataclasses import dataclass
from typing import List, Tuple, Optional
import numpy as np

@dataclass
class GridPoint:
    """Point in the 2D grid with cost"""
    x: int
    y: int
    cost: int

class GridUtils:
    """Utility functions for costmap grid operations"""
    
    @staticmethod
    def world_to_grid_x(world_x: float, origin_x: float, resolution: float) -> int:
        """Convert world x coordinate to grid x coordinate"""
        return int((world_x - origin_x) / resolution)
    
    @staticmethod
    def world_to_grid_y(world_y: float, origin_y: float, resolution: float) -> int:
        """Convert world y coordinate to grid y coordinate"""
        return int((world_y - origin_y) / resolution)
    
    @staticmethod
    def grid_to_world_x(grid_x: int, origin_x: float, resolution: float) -> float:
        """Convert grid x coordinate to world x coordinate"""
        return grid_x * resolution + origin_x
    
    @staticmethod
    def grid_to_world_y(grid_y: int, origin_y: float, resolution: float) -> float:
        """Convert grid y coordinate to world y coordinate"""
        return grid_y * resolution + origin_y
    
    @staticmethod
    def is_valid_grid_coord(x: int, y: int, grid_width: int, grid_height: int) -> bool:
        """Check if grid coordinates are valid"""
        return 0 <= x < grid_width and 0 <= y < grid_height
    
    @staticmethod
    def create_inflation_kernel(inflation_radius: float, resolution: float):
        """Create a circular kernel for obstacle inflation"""
        inflation_cells = int(inflation_radius / resolution)
        y, x = np.ogrid[-inflation_cells:inflation_cells+1, -inflation_cells:inflation_cells+1]
        return x**2 + y**2 <= inflation_cells**2
