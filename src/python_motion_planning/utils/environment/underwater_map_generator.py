"""
@file: underwater_map_generator.py
@brief: Generator for underwater ROV inspection environments
@author: Generated for IN5060 Assignment
@update: 2025.9.11
"""
import numpy as np
import random
from typing import List, Tuple, Set, Optional
from .env import Grid

class UnderwaterMapGenerator:
    """
    Generator for creating realistic underwater infrastructure inspection maps for ROVs.
    
    This class creates 3D environments that simulate underwater scenarios including:
    - Pipelines and infrastructure
    - Underwater terrain (sea floor, rocks, structures)
    - Marine life obstacles
    - Current zones with different costs
    """
    
    def __init__(self, x_range: int, y_range: int, z_range: int, seed: Optional[int] = None):
        self.x_range = x_range
        self.y_range = y_range
        self.z_range = z_range
        
        if seed is not None:
            random.seed(seed)
            np.random.seed(seed)
    
    def generate_pipeline_inspection_map(self, 
                                       pipeline_config: dict,
                                       terrain_density: float = 0.1,
                                       marine_life_density: float = 0.05) -> Grid:
        """
        Generate a complete underwater inspection map with pipelines.
        
        Args:
            pipeline_config: Configuration for pipeline generation
            terrain_density: Density of terrain obstacles (0.0 to 1.0)
            marine_life_density: Density of marine life obstacles (0.0 to 1.0)
        
        Returns:
            Grid: Configured 3D grid environment
        """
        env = Grid(self.x_range, self.y_range, self.z_range)
        
        # Start with basic boundary obstacles
        obstacles = env.obstacles.copy()
        
        # Add pipelines
        pipeline_obstacles = self._generate_pipelines(pipeline_config)
        obstacles.update(pipeline_obstacles)
        
        # Add terrain
        terrain_obstacles = self._generate_underwater_terrain(terrain_density)
        obstacles.update(terrain_obstacles)
        
        # Add marine life obstacles
        marine_obstacles = self._generate_marine_life_obstacles(marine_life_density)
        obstacles.update(marine_obstacles)
        
        # Update environment
        env.update(obstacles)
        
        # Add inner obstacles for inspection points
        env.inner_obstacles = self._generate_inspection_points(pipeline_config)
        
        return env
    
    def _generate_pipelines(self, config: dict) -> Set[Tuple[int, int, int]]:
        """Generate pipeline structures based on configuration."""
        obstacles = set()
        
        pipeline_type = config.get('type', 'straight')
        pipeline_width = config.get('width', 1)
        start_point = config.get('start', (2, 2, 2))
        end_point = config.get('end', (self.x_range-3, self.y_range-3, 2))
        
        if pipeline_type == 'straight':
            obstacles.update(self._create_straight_pipeline(start_point, end_point, pipeline_width))
        elif pipeline_type == 'l_shaped':
            corner_point = config.get('corner', (self.x_range//2, self.y_range//2, 2))
            obstacles.update(self._create_l_shaped_pipeline(start_point, corner_point, end_point, pipeline_width))
        elif pipeline_type == 'curved':
            obstacles.update(self._create_curved_pipeline(start_point, end_point, pipeline_width))
        elif pipeline_type == 'network':
            network_points = config.get('network_points', [])
            obstacles.update(self._create_pipeline_network(network_points, pipeline_width))
        
        return obstacles
    
    def _create_straight_pipeline(self, start: Tuple[int, int, int], 
                                end: Tuple[int, int, int], width: int) -> Set[Tuple[int, int, int]]:
        """Create a straight pipeline between two points."""
        obstacles = set()
        
        # Linear interpolation between start and end points
        steps = max(abs(end[0] - start[0]), abs(end[1] - start[1]), abs(end[2] - start[2]))
        
        if steps == 0:
            return obstacles
        
        for i in range(steps + 1):
            t = i / steps
            x = int(start[0] + t * (end[0] - start[0]))
            y = int(start[1] + t * (end[1] - start[1]))
            z = int(start[2] + t * (end[2] - start[2]))
            
            # Add pipeline width
            for dx in range(-width//2, width//2 + 1):
                for dy in range(-width//2, width//2 + 1):
                    px, py = x + dx, y + dy
                    if 0 <= px < self.x_range and 0 <= py < self.y_range and 0 <= z < self.z_range:
                        obstacles.add((px, py, z))
        
        return obstacles
    
    def _create_l_shaped_pipeline(self, start: Tuple[int, int, int], 
                                corner: Tuple[int, int, int], 
                                end: Tuple[int, int, int], width: int) -> Set[Tuple[int, int, int]]:
        """Create an L-shaped pipeline."""
        obstacles = set()
        obstacles.update(self._create_straight_pipeline(start, corner, width))
        obstacles.update(self._create_straight_pipeline(corner, end, width))
        return obstacles
    
    def _create_curved_pipeline(self, start: Tuple[int, int, int], 
                              end: Tuple[int, int, int], width: int) -> Set[Tuple[int, int, int]]:
        """Create a curved pipeline using Bezier curve approximation."""
        obstacles = set()
        
        # Control points for Bezier curve
        mid_x = (start[0] + end[0]) // 2
        mid_y = (start[1] + end[1]) // 2
        control_point = (mid_x, mid_y + (end[1] - start[1]) // 3, (start[2] + end[2]) // 2)
        
        # Generate curved path
        for t in np.linspace(0, 1, 50):
            # Quadratic Bezier curve
            x = int((1-t)**2 * start[0] + 2*(1-t)*t * control_point[0] + t**2 * end[0])
            y = int((1-t)**2 * start[1] + 2*(1-t)*t * control_point[1] + t**2 * end[1])
            z = int((1-t)**2 * start[2] + 2*(1-t)*t * control_point[2] + t**2 * end[2])
            
            # Add pipeline width
            for dx in range(-width//2, width//2 + 1):
                for dy in range(-width//2, width//2 + 1):
                    px, py = x + dx, y + dy
                    if 0 <= px < self.x_range and 0 <= py < self.y_range and 0 <= z < self.z_range:
                        obstacles.add((px, py, z))
        
        return obstacles
    
    def _create_pipeline_network(self, network_points: List[Tuple[int, int, int]], 
                               width: int) -> Set[Tuple[int, int, int]]:
        """Create a network of connected pipelines."""
        obstacles = set()
        
        # Connect each point to the next
        for i in range(len(network_points) - 1):
            obstacles.update(self._create_straight_pipeline(
                network_points[i], network_points[i + 1], width))
        
        return obstacles
    
    def _generate_underwater_terrain(self, density: float) -> Set[Tuple[int, int, int]]:
        """Generate underwater terrain obstacles like rocks, coral, etc."""
        obstacles = set()
        
        # Sea floor terrain (bottom layers)
        for x in range(self.x_range):
            for y in range(self.y_range):
                if random.random() < density:
                    # Random height terrain
                    height = random.randint(1, min(3, self.z_range - 2))
                    for z in range(1, height + 1):
                        obstacles.add((x, y, z))
        
        # Scattered rocks and structures
        num_structures = int(density * self.x_range * self.y_range * 0.1)
        for _ in range(num_structures):
            x = random.randint(2, self.x_range - 3)
            y = random.randint(2, self.y_range - 3)
            z = random.randint(2, self.z_range - 3)
            
            # Create rock/structure cluster
            size = random.randint(1, 3)
            for dx in range(-size, size + 1):
                for dy in range(-size, size + 1):
                    for dz in range(-size//2, size//2 + 1):
                        px, py, pz = x + dx, y + dy, z + dz
                        if (0 < px < self.x_range - 1 and 
                            0 < py < self.y_range - 1 and 
                            0 < pz < self.z_range - 1):
                            if random.random() < 0.7:  # Not all cells in cluster
                                obstacles.add((px, py, pz))
        
        return obstacles
    
    def _generate_marine_life_obstacles(self, density: float) -> Set[Tuple[int, int, int]]:
        """Generate marine life obstacles (schools of fish, kelp, etc.)."""
        obstacles = set()
        
        # Kelp forests (vertical structures)
        num_kelp = int(density * self.x_range * self.y_range * 0.05)
        for _ in range(num_kelp):
            x = random.randint(1, self.x_range - 2)
            y = random.randint(1, self.y_range - 2)
            
            # Kelp grows from bottom up
            height = random.randint(3, min(self.z_range - 2, 8))
            for z in range(1, height):
                # Kelp sways, not perfectly straight
                sway_x = x + random.randint(-1, 1) if z > 2 else x
                sway_y = y + random.randint(-1, 1) if z > 2 else y
                
                if (0 < sway_x < self.x_range - 1 and 
                    0 < sway_y < self.y_range - 1):
                    obstacles.add((sway_x, sway_y, z))
        
        # Fish schools (temporary obstacles - could be dynamic)
        num_schools = int(density * self.x_range * self.y_range * 0.02)
        for _ in range(num_schools):
            center_x = random.randint(3, self.x_range - 4)
            center_y = random.randint(3, self.y_range - 4)
            center_z = random.randint(2, self.z_range - 3)
            
            # School size
            school_size = random.randint(2, 4)
            for dx in range(-school_size, school_size + 1):
                for dy in range(-school_size, school_size + 1):
                    for dz in range(-1, 2):
                        if random.random() < 0.3:  # Sparse school
                            px = center_x + dx
                            py = center_y + dy
                            pz = center_z + dz
                            if (0 < px < self.x_range - 1 and 
                                0 < py < self.y_range - 1 and 
                                0 < pz < self.z_range - 1):
                                obstacles.add((px, py, pz))
        
        return obstacles
    
    def _generate_inspection_points(self, pipeline_config: dict) -> Set[Tuple[int, int, int]]:
        """Generate inspection points along pipelines."""
        inspection_points = set()
        
        pipeline_type = pipeline_config.get('type', 'straight')
        start_point = pipeline_config.get('start', (2, 2, 2))
        end_point = pipeline_config.get('end', (self.x_range-3, self.y_range-3, 2))
        
        if pipeline_type == 'straight':
            # Add inspection points along the pipeline
            steps = max(abs(end_point[0] - start_point[0]), 
                       abs(end_point[1] - start_point[1]), 
                       abs(end_point[2] - start_point[2]))
            
            # Place inspection points every 5-10 steps
            for i in range(0, steps, random.randint(5, 10)):
                t = i / steps if steps > 0 else 0
                x = int(start_point[0] + t * (end_point[0] - start_point[0]))
                y = int(start_point[1] + t * (end_point[1] - start_point[1]))
                z = int(start_point[2] + t * (end_point[2] - start_point[2]))
                
                # Offset inspection point slightly from pipeline
                inspection_points.add((x, y, z + 1))
        
        return inspection_points
    
    def create_scenario_configs(self) -> List[dict]:
        """Create predefined scenario configurations for testing."""
        scenarios = []
        
        # Scenario 1: Simple straight pipeline inspection
        scenarios.append({
            'name': 'straight_pipeline',
            'description': 'Simple straight pipeline inspection',
            'pipeline_config': {
                'type': 'straight',
                'width': 1,
                'start': (2, 2, 2),
                'end': (self.x_range-3, self.y_range-3, 2)
            },
            'terrain_density': 0.1,
            'marine_life_density': 0.05
        })
        
        # Scenario 2: L-shaped pipeline with obstacles
        scenarios.append({
            'name': 'l_shaped_complex',
            'description': 'L-shaped pipeline with heavy obstacles',
            'pipeline_config': {
                'type': 'l_shaped',
                'width': 2,
                'start': (2, 2, 2),
                'corner': (self.x_range//2, self.y_range//2, 3),
                'end': (self.x_range-3, self.y_range-3, 4)
            },
            'terrain_density': 0.2,
            'marine_life_density': 0.1
        })
        
        # Scenario 3: Curved pipeline in kelp forest
        scenarios.append({
            'name': 'curved_kelp_forest',
            'description': 'Curved pipeline through kelp forest',
            'pipeline_config': {
                'type': 'curved',
                'width': 1,
                'start': (2, 2, 3),
                'end': (self.x_range-3, self.y_range-3, 5)
            },
            'terrain_density': 0.05,
            'marine_life_density': 0.15
        })
        
        # Scenario 4: Pipeline network
        scenarios.append({
            'name': 'pipeline_network',
            'description': 'Complex pipeline network inspection',
            'pipeline_config': {
                'type': 'network',
                'width': 1,
                'network_points': [
                    (2, 2, 2),
                    (self.x_range//3, self.y_range//3, 2),
                    (2*self.x_range//3, self.y_range//2, 3),
                    (self.x_range-3, 2*self.y_range//3, 2),
                    (self.x_range-3, self.y_range-3, 4)
                ]
            },
            'terrain_density': 0.15,
            'marine_life_density': 0.08
        })
        
        return scenarios