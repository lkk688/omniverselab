#!/usr/bin/env python3
"""
Autonomous Driving Scenario Setup for NVIDIA Isaac Sim

A comprehensive Python module for setting up autonomous driving scenarios in Isaac Sim,
including map loading, vehicle configuration with sensors, driving modes, and dataset
collection with ground truth annotations.

Note: This script is designed to run within Isaac Sim environment.
Some imports will fail if run outside of Isaac Sim.

Author: Generated from autonomous_driving_tutorial.md
Date: 2024
"""

import numpy as np
import asyncio
import os
import json
from datetime import datetime
from typing import Dict, List, Tuple, Optional

# Isaac Sim imports - with error handling for non-Isaac Sim environments
try:
    import omni
    import omni.client
    from omni.isaac.core import World
    from omni.isaac.core.robots import Robot
    from omni.isaac.core.utils.stage import add_reference_to_stage
    from omni.isaac.sensor import Camera, LidarRtx
    from omni.isaac.wheeled_robots.controllers import DifferentialController
    from omni.isaac.core.utils.prims import create_prim, get_prim_at_path
    from omni.isaac.core.utils.rotations import euler_angles_to_quat
    import carb.input
    ISAAC_SIM_AVAILABLE = True
    OMNI_CLIENT_AVAILABLE = True
except ImportError as e:
    print(f"Isaac Sim modules not available: {e}")
    print("This script must be run within Isaac Sim environment")
    ISAAC_SIM_AVAILABLE = False
    OMNI_CLIENT_AVAILABLE = False
    # Create dummy classes for type checking
    class World: pass
    class Robot: pass
    class Camera: pass
    class LidarRtx: pass
    class DifferentialController: pass
    # Create dummy omni module
    class omni:
        class client:
            class Result:
                OK = 0
            @staticmethod
            def list(path):
                return (1, [])

# Additional imports
try:
    import cv2
    # Test cv2 attributes to ensure they're available
    _ = cv2.cvtColor
    _ = cv2.COLOR_RGBA2BGR
    _ = cv2.imwrite
    CV2_AVAILABLE = True
except (ImportError, AttributeError):
    print("OpenCV not available or missing required functions, image processing will be limited")
    cv2 = None
    CV2_AVAILABLE = False


class IsaacSimSetup:
    """Setup and configuration for Isaac Sim environment"""
    
    @staticmethod
    def enable_extensions():
        """Enable required Isaac Sim extensions"""
        if not ISAAC_SIM_AVAILABLE:
            print("Isaac Sim not available - cannot enable extensions")
            return False
            
        try:
            manager = omni.kit.app.get_app().get_extension_manager()
            
            extensions = [
                "omni.isaac.sensor",
                "omni.isaac.wheeled_robots", 
                "omni.isaac.synthetic_utils",
                "omni.replicator.core",
                "omni.isaac.core"
            ]
            
            for ext in extensions:
                manager.set_extension_enabled_immediate(ext, True)
            
            print(f"Enabled {len(extensions)} Isaac Sim extensions")
            return True
        except Exception as e:
            print(f"Error enabling extensions: {e}")
            return False


class EnvironmentLoader:
    """Load and manage different environments and maps"""
    
    def __init__(self, world: World):
        self.world = world
    
    def load_warehouse_environment(self):
        """Load simple warehouse for testing"""
        warehouse_usd = "omniverse://localhost/NVIDIA/Assets/Isaac/2023.1.1/Isaac/Environments/Simple_Warehouse/warehouse.usd"
        add_reference_to_stage(usd_path=warehouse_usd, prim_path="/World/Warehouse")
        self.world.scene.add_default_ground_plane()
        print("Loaded warehouse environment")
    
    def load_hospital_environment(self):
        """Load hospital environment with more complex geometry"""
        hospital_usd = "omniverse://localhost/NVIDIA/Assets/Isaac/2023.1.1/Isaac/Environments/Hospital/hospital.usd"
        add_reference_to_stage(usd_path=hospital_usd, prim_path="/World/Hospital")
        print("Loaded hospital environment")
    
    def load_custom_city_map(self, city_map_path: str):
        """Load custom city map from USD file"""
        add_reference_to_stage(usd_path=city_map_path, prim_path="/World/CityMap")
        print(f"Loaded custom city map: {city_map_path}")
    
    def load_carla_map(self, town_name: str = "Town01"):
        """Load CARLA map if available"""
        carla_maps = {
            "Town01": "omniverse://localhost/NVIDIA/Assets/Isaac/2023.1.1/Isaac/Environments/Carla/Town01.usd",
            "Town02": "omniverse://localhost/NVIDIA/Assets/Isaac/2023.1.1/Isaac/Environments/Carla/Town02.usd",
            "Town03": "omniverse://localhost/NVIDIA/Assets/Isaac/2023.1.1/Isaac/Environments/Carla/Town03.usd",
            "Town10HD": "omniverse://localhost/NVIDIA/Assets/Isaac/2023.1.1/Isaac/Environments/Carla/Town10HD.usd"
        }
        
        if town_name in carla_maps:
            carla_map_path = carla_maps[town_name]
            add_reference_to_stage(usd_path=carla_map_path, prim_path="/World/CarlaMap")
            print(f"Loaded CARLA map: {town_name}")
        else:
            print(f"CARLA map {town_name} not available")
    
    def load_carla_map_custom(self, custom_path: str):
        """Load CARLA map from custom exported location"""
        add_reference_to_stage(usd_path=custom_path, prim_path="/World/CarlaMap")
        print(f"Loaded custom CARLA map from {custom_path}")
    
    def load_multiple_carla_towns(self):
        """Load multiple CARLA towns for comparison"""
        carla_maps = {
            "Town01": "omniverse://localhost/NVIDIA/Assets/Isaac/2023.1.1/Isaac/Environments/Carla/Town01.usd",
            "Town02": "omniverse://localhost/NVIDIA/Assets/Isaac/2023.1.1/Isaac/Environments/Carla/Town02.usd",
            "Town03": "omniverse://localhost/NVIDIA/Assets/Isaac/2023.1.1/Isaac/Environments/Carla/Town03.usd",
            "Town10HD": "omniverse://localhost/NVIDIA/Assets/Isaac/2023.1.1/Isaac/Environments/Carla/Town10HD.usd"
        }
        
        for i, (town_name, town_path) in enumerate(carla_maps.items()):
            prim_path = f"/World/CARLA_{town_name}"
            add_reference_to_stage(usd_path=town_path, prim_path=prim_path)
            
            # Offset each town to avoid overlap
            offset_x = i * 1000  # 1km spacing
            if ISAAC_SIM_AVAILABLE:
                omni.kit.commands.execute(
                    "ChangeProperty",
                    prop_path=f"{prim_path}.xformOp:translate",
                    value=(offset_x, 0, 0),
                    prev=None
                )
        print(f"Loaded {len(carla_maps)} CARLA towns with spacing")
    
    def configure_carla_environment(self):
        """Configure Isaac Sim for CARLA map compatibility"""
        if not ISAAC_SIM_AVAILABLE:
            return
            
        # Set appropriate physics timestep for CARLA compatibility
        self.world.set_physics_dt(1.0/50.0)  # 50 FPS (CARLA standard)
        
        # Configure lighting for outdoor scenarios
        omni.kit.commands.execute(
            "ChangeProperty",
            prop_path="/Environment/sky.intensity",
            value=1.0,
            prev=None
        )
        
        # Set appropriate camera exposure for outdoor lighting
        omni.kit.commands.execute(
            "ChangeProperty",
            prop_path="/Render/PostProcess/toneCurveAmount",
            value=1.2,
            prev=None
        )
        print("Configured CARLA environment settings")
    
    def check_carla_map_availability(self):
        """Check if CARLA maps are available in Omniverse"""
        if not ISAAC_SIM_AVAILABLE:
            return []
            
        if not OMNI_CLIENT_AVAILABLE:
            print("Omni client not available - cannot check CARLA maps")
            return []
            
        try:
            carla_base_path = "omniverse://localhost/NVIDIA/Assets/Isaac/2023.1.1/Isaac/Environments/Carla/"
            
            result = omni.client.list(carla_base_path)
            if result[0] == omni.client.Result.OK:
                available_maps = [entry.relative_path for entry in result[1]]
                print(f"Available CARLA maps: {available_maps}")
                return available_maps
            else:
                print("CARLA maps not found. Check Omniverse installation.")
                return []
        except Exception as e:
            print(f"Error checking CARLA maps: {e}")
            return []
    
    def optimize_carla_performance(self):
        """Optimize Isaac Sim settings for large CARLA environments"""
        if not ISAAC_SIM_AVAILABLE:
            return
            
        # Reduce rendering quality for better performance
        omni.kit.commands.execute(
            "ChangeProperty",
            prop_path="/Render/RayTracing/enabled",
            value=False,
            prev=None
        )
        
        # Enable level-of-detail (LOD) for distant objects
        omni.kit.commands.execute(
            "ChangeProperty",
            prop_path="/Render/LOD/enabled",
            value=True,
            prev=None
        )
        
        # Limit simulation area to reduce computational load
        simulation_bounds = {"min": [-500, -500, -10], "max": [500, 500, 100]}
        print(f"Simulation bounded to: {simulation_bounds}")
        print("CARLA performance optimization applied")
    
    def create_carla_style_roads(self):
        """Create road networks inspired by CARLA towns"""
        if not ISAAC_SIM_AVAILABLE:
            return
            
        # Town01-style: Simple grid with roundabout
        self._create_grid_roads(size=400, spacing=100, lane_width=3.5)
        self._create_roundabout(center=[0, 0], radius=30, lanes=2)
        
        # Add traffic infrastructure
        self._add_traffic_lights_at_intersections()
        self._create_road_markings()
        self._add_sidewalks(width=2.0)
        
        print("CARLA-style road network created")
    
    def _create_grid_roads(self, size: float, spacing: float, lane_width: float):
        """Create a grid of roads"""
        if not ISAAC_SIM_AVAILABLE:
            return
            
        # Create horizontal roads
        for i in range(-int(size//spacing), int(size//spacing) + 1):
            y_pos = i * spacing
            road_prim = create_prim(
                prim_path=f"/World/Roads/HorizontalRoad_{i}",
                prim_type="Cube",
                position=[0, y_pos, -0.1],
                scale=[size, lane_width * 2, 0.2]
            )
            
        # Create vertical roads
        for i in range(-int(size//spacing), int(size//spacing) + 1):
            x_pos = i * spacing
            road_prim = create_prim(
                prim_path=f"/World/Roads/VerticalRoad_{i}",
                prim_type="Cube",
                position=[x_pos, 0, -0.1],
                scale=[lane_width * 2, size, 0.2]
            )
            
        print(f"Created grid road network: {size}x{size}m with {spacing}m spacing")
    
    def _create_roundabout(self, center: List[float], radius: float, lanes: int):
        """Create a roundabout"""
        if not ISAAC_SIM_AVAILABLE:
            return
            
        # Create outer ring
        outer_radius = radius + (lanes * 3.5) / 2
        outer_ring = create_prim(
            prim_path="/World/Roads/RoundaboutOuter",
            prim_type="Torus",
            position=[center[0], center[1], -0.1],
            scale=[outer_radius, outer_radius, 0.2]
        )
        
        # Create inner ring (island)
        inner_radius = radius - (lanes * 3.5) / 2
        if inner_radius > 0:
            inner_ring = create_prim(
                prim_path="/World/Roads/RoundaboutInner",
                prim_type="Cylinder",
                position=[center[0], center[1], 0.1],
                scale=[inner_radius, inner_radius, 0.4]
            )
            
        # Create connecting roads (4 directions)
        connection_length = 50.0
        lane_width = 3.5 * lanes
        
        directions = [
            ([center[0] + radius + connection_length/2, center[1], -0.1], [connection_length, lane_width, 0.2]),  # East
            ([center[0] - radius - connection_length/2, center[1], -0.1], [connection_length, lane_width, 0.2]),  # West
            ([center[0], center[1] + radius + connection_length/2, -0.1], [lane_width, connection_length, 0.2]),  # North
            ([center[0], center[1] - radius - connection_length/2, -0.1], [lane_width, connection_length, 0.2])   # South
        ]
        
        for i, (position, scale) in enumerate(directions):
            create_prim(
                prim_path=f"/World/Roads/RoundaboutConnection_{i}",
                prim_type="Cube",
                position=position,
                scale=scale
            )
            
        print(f"Created roundabout at {center} with radius {radius}m and {lanes} lanes")
    
    def _add_traffic_lights_at_intersections(self):
        """Add traffic lights at intersections"""
        if not ISAAC_SIM_AVAILABLE:
            return
            
        # Define intersection positions (assuming grid layout)
        intersections = [
            [0, 0, 0],      # Center intersection
            [100, 0, 0],    # East intersection
            [-100, 0, 0],   # West intersection
            [0, 100, 0],    # North intersection
            [0, -100, 0]    # South intersection
        ]
        
        for i, intersection in enumerate(intersections):
            # Create traffic light poles (4 per intersection)
            pole_positions = [
                [intersection[0] + 8, intersection[1] + 8, 0],   # NE corner
                [intersection[0] - 8, intersection[1] + 8, 0],   # NW corner
                [intersection[0] - 8, intersection[1] - 8, 0],   # SW corner
                [intersection[0] + 8, intersection[1] - 8, 0]    # SE corner
            ]
            
            for j, pole_pos in enumerate(pole_positions):
                # Create pole
                pole = create_prim(
                    prim_path=f"/World/TrafficLights/Intersection_{i}_Pole_{j}",
                    prim_type="Cylinder",
                    position=pole_pos,
                    scale=[0.2, 0.2, 4.0]
                )
                
                # Create traffic light housing
                light_housing = create_prim(
                    prim_path=f"/World/TrafficLights/Intersection_{i}_Light_{j}",
                    prim_type="Cube",
                    position=[pole_pos[0], pole_pos[1], pole_pos[2] + 3.5],
                    scale=[0.8, 0.3, 1.2]
                )
                
        print(f"Added traffic lights at {len(intersections)} intersections")
    
    def _add_sidewalks(self, width: float):
        """Add sidewalks"""
        if not ISAAC_SIM_AVAILABLE:
            return
            
        # Add sidewalks along main roads
        road_length = 200.0
        road_width = 8.0
        sidewalk_height = 0.15
        
        # Main road sidewalks (East-West)
        for side in [-1, 1]:  # Both sides of the road
            sidewalk_y = side * (road_width/2 + width/2)
            
            sidewalk = create_prim(
                prim_path=f"/World/Sidewalks/MainRoad_Side_{side}",
                prim_type="Cube",
                position=[0, sidewalk_y, sidewalk_height/2],
                scale=[road_length, width, sidewalk_height]
            )
            
            # Set sidewalk material (lighter color)
            omni.kit.commands.execute(
                "ChangeProperty",
                prop_path=f"/World/Sidewalks/MainRoad_Side_{side}.material:inputs:diffuse_color_constant",
                value=(0.8, 0.8, 0.8),  # Light gray
                prev=None
            )
        
        # Cross road sidewalks (North-South)
        for side in [-1, 1]:  # Both sides of the road
            sidewalk_x = side * (road_width/2 + width/2)
            
            sidewalk = create_prim(
                prim_path=f"/World/Sidewalks/CrossRoad_Side_{side}",
                prim_type="Cube",
                position=[sidewalk_x, 0, sidewalk_height/2],
                scale=[width, road_length, sidewalk_height]
            )
            
            # Set sidewalk material (lighter color)
            omni.kit.commands.execute(
                "ChangeProperty",
                prop_path=f"/World/Sidewalks/CrossRoad_Side_{side}.material:inputs:diffuse_color_constant",
                value=(0.8, 0.8, 0.8),  # Light gray
                prev=None
            )
            
        print(f"Added sidewalks with width {width}m along main roads")
    
    def create_simple_road_network_advanced(self):
        """Create a simple road network with intersections (from markdown)"""
        if not ISAAC_SIM_AVAILABLE:
            return None, None
            
        # Create road segments
        road_width = 8.0
        road_length = 100.0
        
        # Main road (East-West)
        main_road = create_prim(
            prim_path="/World/Roads/MainRoad",
            prim_type="Cube",
            position=[0, 0, -0.1],
            scale=[road_length, road_width, 0.2]
        )
        
        # Cross road (North-South)
        cross_road = create_prim(
            prim_path="/World/Roads/CrossRoad",
            prim_type="Cube", 
            position=[0, 0, -0.1],
            scale=[road_width, road_length, 0.2]
        )
        
        # Add road markings
        self.create_road_markings_advanced()
        
        return main_road, cross_road
    
    def create_road_markings_advanced(self):
        """Add lane markings and traffic signs (from markdown)"""
        if not ISAAC_SIM_AVAILABLE:
            return
            
        # Lane dividers
        for i in range(-40, 41, 10):
            marking = create_prim(
                prim_path=f"/World/RoadMarkings/Divider_{i}",
                prim_type="Cube",
                position=[i, 0, 0.01],
                scale=[2.0, 0.2, 0.02]
            )
            
            # Set yellow color for lane markings
            omni.kit.commands.execute(
                "ChangeProperty",
                prop_path=f"/World/RoadMarkings/Divider_{i}.material:inputs:diffuse_color_constant",
                value=(1.0, 1.0, 0.0),
                prev=None
            )
        
        print("Added advanced road markings with lane dividers")
    
    def create_simple_road_network(self):
        """Create a simple road network with intersections"""
        road_width = 8.0
        road_length = 100.0
        
        # Main road (East-West)
        main_road = create_prim(
            prim_path="/World/Roads/MainRoad",
            prim_type="Cube",
            position=[0, 0, -0.1],
            scale=[road_length, road_width, 0.2]
        )
        
        # Cross road (North-South)
        cross_road = create_prim(
            prim_path="/World/Roads/CrossRoad",
            prim_type="Cube", 
            position=[0, 0, -0.1],
            scale=[road_width, road_length, 0.2]
        )
        
        self._create_road_markings()
        print("Created simple road network")
        
        return main_road, cross_road
    
    def _create_road_markings(self):
        """Add lane markings and traffic signs"""
        for i in range(-40, 41, 10):
            marking = create_prim(
                prim_path=f"/World/RoadMarkings/Divider_{i}",
                prim_type="Cube",
                position=[i, 0, 0.01],
                scale=[2.0, 0.2, 0.02]
            )
            
            # Set yellow color for lane markings
            omni.kit.commands.execute(
                "ChangeProperty",
                prop_path=f"/World/RoadMarkings/Divider_{i}.material:inputs:diffuse_color_constant",
                value=(1.0, 1.0, 0.0),
                prev=None
            )


class AutonomousVehicle:
    """Autonomous vehicle with sensors and control systems"""
    
    def __init__(self, world: World, vehicle_path: str = "/World/Vehicle"):
        self.world = world
        self.vehicle_path = vehicle_path
        self.cameras = {}
        self.lidars = {}
        self.robot = None
        self.controller = None
        
        self.setup_vehicle()
        self.setup_sensors()
    
    def setup_vehicle(self):
        """Setup the autonomous vehicle"""
        # Load vehicle USD
        vehicle_usd = "omniverse://localhost/NVIDIA/Assets/Isaac/2023.1.1/Isaac/Robots/Carter/carter_v1.usd"
        
        add_reference_to_stage(
            usd_path=vehicle_usd, 
            prim_path=self.vehicle_path
        )
        
        # Create robot instance
        self.robot = Robot(
            prim_path=self.vehicle_path,
            name="autonomous_vehicle",
            position=[0, 0, 0.5],
            orientation=euler_angles_to_quat([0, 0, 0])
        )
        
        # Add to world
        self.world.scene.add(self.robot)
        
        # Setup controller
        self.controller = DifferentialController(
            name="vehicle_controller",
            wheel_radius=0.125,
            wheel_base=0.4132
        )
        
        print("Vehicle setup completed")
    
    def setup_sensors(self):
        """Setup cameras and LiDAR sensors"""
        # Front camera
        self.cameras['front'] = Camera(
            prim_path=f"{self.vehicle_path}/FrontCamera",
            position=[0.8, 0, 0.7],
            orientation=euler_angles_to_quat([0, 0, 0]),
            frequency=30,
            resolution=(1920, 1080)
        )
        
        # Rear camera
        self.cameras['rear'] = Camera(
            prim_path=f"{self.vehicle_path}/RearCamera",
            position=[-0.8, 0, 0.7],
            orientation=euler_angles_to_quat([0, 0, np.pi]),
            frequency=30,
            resolution=(1920, 1080)
        )
        
        # Left camera
        self.cameras['left'] = Camera(
            prim_path=f"{self.vehicle_path}/LeftCamera",
            position=[0, 0.8, 0.7],
            orientation=euler_angles_to_quat([0, 0, np.pi/2]),
            frequency=30,
            resolution=(1920, 1080)
        )
        
        # Right camera
        self.cameras['right'] = Camera(
            prim_path=f"{self.vehicle_path}/RightCamera",
            position=[0, -0.8, 0.7],
            orientation=euler_angles_to_quat([0, 0, -np.pi/2]),
            frequency=30,
            resolution=(1920, 1080)
        )
        
        # 3D LiDAR (roof-mounted)
        self.lidars['main'] = LidarRtx(
            prim_path=f"{self.vehicle_path}/LiDAR",
            position=[0, 0, 1.2],
            orientation=euler_angles_to_quat([0, 0, 0]),
            config="Velodyne_VLS128"
        )
        
        # Add sensors to world
        for camera in self.cameras.values():
            self.world.scene.add(camera)
            
        for lidar in self.lidars.values():
            self.world.scene.add(lidar)
        
        print(f"Setup {len(self.cameras)} cameras and {len(self.lidars)} LiDAR sensors")
    
    def configure_vehicle_physics(self):
        """Configure realistic vehicle physics"""
        if not ISAAC_SIM_AVAILABLE:
            return
            
        # Set vehicle mass and inertia
        vehicle_prim = get_prim_at_path(self.vehicle_path)
        
        # Add rigid body properties
        omni.kit.commands.execute(
            "AddPhysicsComponent",
            usd_prim=vehicle_prim,
            component="PhysicsRigidBodyAPI"
        )
        
        # Set mass properties
        omni.kit.commands.execute(
            "ChangeProperty",
            prop_path=f"{self.vehicle_path}.physics:mass",
            value=1500.0,  # 1500 kg
            prev=None
        )
        
        # Configure wheel physics
        wheel_paths = [
            f"{self.vehicle_path}/front_left_wheel",
            f"{self.vehicle_path}/front_right_wheel", 
            f"{self.vehicle_path}/rear_left_wheel",
            f"{self.vehicle_path}/rear_right_wheel"
        ]
        
        for wheel_path in wheel_paths:
            if get_prim_at_path(wheel_path):
                # Add wheel collider
                omni.kit.commands.execute(
                    "AddPhysicsComponent",
                    usd_prim=get_prim_at_path(wheel_path),
                    component="PhysicsCollisionAPI"
                )
        
        print("Configured vehicle physics with realistic mass and wheel properties")
    
    def configure_cameras(self):
        """Configure cameras with realistic parameters"""
        camera_configs = {
            'front': {
                'focal_length': 24.0,
                'focus_distance': 400.0,
                'f_stop': 2.8,
                'horizontal_aperture': 36.0,
                'clipping_range': (0.1, 1000.0)
            },
            'rear': {
                'focal_length': 18.0,
                'focus_distance': 400.0, 
                'f_stop': 2.8,
                'horizontal_aperture': 36.0,
                'clipping_range': (0.1, 1000.0)
            }
        }
        
        for camera_name, camera in self.cameras.items():
            if camera_name in camera_configs:
                config = camera_configs[camera_name]
                
                # Apply camera settings
                camera.set_focal_length(config['focal_length'])
                camera.set_focus_distance(config['focus_distance'])
                camera.set_f_stop(config['f_stop'])
                camera.set_horizontal_aperture(config['horizontal_aperture'])
                camera.set_clipping_range(*config['clipping_range'])
                
                # Enable semantic segmentation
                camera.add_motion_vectors()
                camera.add_distance_to_image_plane()
                camera.add_semantic_segmentation()
                camera.add_instance_segmentation()
        
        print("Camera configuration completed")
    
    def configure_lidar(self):
        """Configure LiDAR with realistic parameters"""
        lidar = self.lidars['main']
        
        lidar_config = {
            'min_range': 0.4,
            'max_range': 120.0,
            'draw_points': True,
            'draw_lines': False,
            'horizontal_fov': 360.0,
            'vertical_fov': 40.0,
            'horizontal_resolution': 0.4,
            'vertical_resolution': 0.4,
            'rotation_frequency': 10.0,
            'high_lod': True,
            'yaw_offset': 0.0,
            'enable_semantics': True
        }
        
        # Apply LiDAR configuration
        for param, value in lidar_config.items():
            if hasattr(lidar, f'set_{param}'):
                getattr(lidar, f'set_{param}')(value)
        
        print("LiDAR configuration completed")


class AutonomousDriving:
    """Autonomous driving controller"""
    
    def __init__(self, vehicle: AutonomousVehicle, world: World):
        self.vehicle = vehicle
        self.world = world
        self.waypoints = []
        self.current_waypoint = 0
        self.driving_speed = 5.0  # m/s
    
    def set_waypoints(self, waypoints: List[List[float]]):
        """Set waypoints for autonomous navigation"""
        self.waypoints = waypoints
        self.current_waypoint = 0
        print(f"Set {len(waypoints)} waypoints for autonomous driving")
    
    def create_circular_path(self, center: List[float] = [0, 0], radius: float = 20, num_points: int = 20):
        """Create a circular driving path"""
        waypoints = []
        for i in range(num_points):
            angle = 2 * np.pi * i / num_points
            x = center[0] + radius * np.cos(angle)
            y = center[1] + radius * np.sin(angle)
            waypoints.append([x, y, 0.5])
        
        self.set_waypoints(waypoints)
        return waypoints
    
    def update_autonomous_driving(self):
        """Update autonomous driving logic"""
        if not self.waypoints:
            return
            
        # Get current vehicle position
        current_pos = self.vehicle.robot.get_world_pose()[0]
        
        # Get target waypoint
        if self.current_waypoint < len(self.waypoints):
            target = np.array(self.waypoints[self.current_waypoint])
            
            # Calculate distance to target
            distance = np.linalg.norm(target[:2] - current_pos[:2])
            
            # Move to next waypoint if close enough
            if distance < 2.0:
                self.current_waypoint = (self.current_waypoint + 1) % len(self.waypoints)
                target = np.array(self.waypoints[self.current_waypoint])
            
            # Calculate steering angle
            direction = target[:2] - current_pos[:2]
            target_angle = np.arctan2(direction[1], direction[0])
            
            # Get current vehicle orientation
            current_quat = self.vehicle.robot.get_world_pose()[1]
            current_angle = np.arctan2(2*(current_quat[3]*current_quat[2] + current_quat[0]*current_quat[1]),
                                     1 - 2*(current_quat[1]**2 + current_quat[2]**2))
            
            # Calculate steering command
            angle_diff = target_angle - current_angle
            angle_diff = np.arctan2(np.sin(angle_diff), np.cos(angle_diff))  # Normalize to [-pi, pi]
            
            # Simple PID control
            steering = np.clip(angle_diff * 2.0, -1.0, 1.0)
            
            # Apply control commands
            self.vehicle.controller.apply_wheel_actions(
                self.vehicle.robot,
                [self.driving_speed, steering]
            )


class ManualDriving:
    """Manual driving controller with keyboard input"""
    
    def __init__(self, vehicle: AutonomousVehicle, world: World):
        self.vehicle = vehicle
        self.world = world
        self.keyboard = carb.input.acquire_input_interface()
        self.max_speed = 10.0
        self.max_steering = 1.0
        
        # Key mappings
        self.key_mappings = {
            carb.input.KeyboardInput.W: 'forward',
            carb.input.KeyboardInput.S: 'backward',
            carb.input.KeyboardInput.A: 'left',
            carb.input.KeyboardInput.D: 'right',
            carb.input.KeyboardInput.SPACE: 'brake'
        }
    
    def update_manual_driving(self):
        """Update manual driving based on keyboard input"""
        speed = 0.0
        steering = 0.0
        
        # Check keyboard inputs
        if self.keyboard.is_key_pressed(carb.input.KeyboardInput.W):
            speed = self.max_speed
        elif self.keyboard.is_key_pressed(carb.input.KeyboardInput.S):
            speed = -self.max_speed
        
        if self.keyboard.is_key_pressed(carb.input.KeyboardInput.A):
            steering = self.max_steering
        elif self.keyboard.is_key_pressed(carb.input.KeyboardInput.D):
            steering = -self.max_steering
        
        if self.keyboard.is_key_pressed(carb.input.KeyboardInput.SPACE):
            speed = 0.0
            steering = 0.0
        
        # Apply control commands
        self.vehicle.controller.apply_wheel_actions(
            self.vehicle.robot,
            [speed, steering]
        )


class DataCollector:
    """Collect sensor data and ground truth annotations"""
    
    def __init__(self, vehicle: AutonomousVehicle, output_dir: str = "./dataset"):
        self.vehicle = vehicle
        self.output_dir = output_dir
        self.frame_count = 0
        self.collected_data = []
        
        # Create output directories
        os.makedirs(output_dir, exist_ok=True)
        os.makedirs(os.path.join(output_dir, "images"), exist_ok=True)
        os.makedirs(os.path.join(output_dir, "lidar"), exist_ok=True)
        os.makedirs(os.path.join(output_dir, "annotations"), exist_ok=True)
        
        print(f"Data collector initialized with output directory: {output_dir}")
    
    def collect_frame_data(self):
        """Collect data for current frame"""
        timestamp = datetime.now().isoformat()
        frame_data = {
            'frame_id': self.frame_count,
            'timestamp': timestamp,
            'vehicle_pose': self._get_vehicle_pose(),
            'camera_data': {},
            'lidar_data': {},
            'annotations': self._get_ground_truth_annotations()
        }
        
        # Collect camera data
        for camera_name, camera in self.vehicle.cameras.items():
            image_data = camera.get_rgba()
            if image_data is not None:
                image_path = os.path.join(self.output_dir, "images", f"{self.frame_count:06d}_{camera_name}.png")
                if CV2_AVAILABLE and cv2 is not None:
                    # Convert RGBA to BGR for OpenCV
                    bgr_data = cv2.cvtColor(image_data, cv2.COLOR_RGBA2BGR)
                    cv2.imwrite(image_path, bgr_data)
                else:
                    # Fallback: save as numpy array
                    np.save(image_path.replace('.png', '.npy'), image_data)
                
                frame_data['camera_data'][camera_name] = {
                    'image_path': image_path,
                    'resolution': image_data.shape[:2],
                    'intrinsics': self._get_camera_intrinsics(camera)
                }
        
        # Collect LiDAR data
        for lidar_name, lidar in self.vehicle.lidars.items():
            point_cloud = lidar.get_point_cloud_data()
            if point_cloud is not None:
                lidar_path = os.path.join(self.output_dir, "lidar", f"{self.frame_count:06d}_{lidar_name}.npy")
                np.save(lidar_path, point_cloud)
                
                frame_data['lidar_data'][lidar_name] = {
                    'point_cloud_path': lidar_path,
                    'num_points': len(point_cloud)
                }
        
        # Save frame annotations
        annotation_path = os.path.join(self.output_dir, "annotations", f"{self.frame_count:06d}.json")
        with open(annotation_path, 'w') as f:
            json.dump(frame_data, f, indent=2)
        
        self.collected_data.append(frame_data)
        self.frame_count += 1
        
        return frame_data
    
    def _get_vehicle_pose(self):
        """Get current vehicle pose"""
        position, orientation = self.vehicle.robot.get_world_pose()
        return {
            'position': position.tolist(),
            'orientation': orientation.tolist()
        }
    
    def _get_camera_intrinsics(self, camera):
        """Get camera intrinsic parameters"""
        # Placeholder - implement based on camera API
        return {
            'focal_length': camera.get_focal_length() if hasattr(camera, 'get_focal_length') else 24.0,
            'horizontal_aperture': camera.get_horizontal_aperture() if hasattr(camera, 'get_horizontal_aperture') else 36.0
        }
    
    def _get_ground_truth_annotations(self):
        """Get ground truth annotations for objects in scene"""
        # Placeholder - implement based on semantic segmentation data
        return {
            'objects': [],
            'semantic_classes': [],
            'bounding_boxes_3d': []
        }
    
    def export_kitti_format(self):
        """Export collected data in KITTI format"""
        kitti_dir = os.path.join(self.output_dir, "kitti_format")
        os.makedirs(kitti_dir, exist_ok=True)
        
        # Create KITTI directory structure
        for subdir in ['image_2', 'velodyne', 'calib', 'label_2']:
            os.makedirs(os.path.join(kitti_dir, subdir), exist_ok=True)
        
        print(f"Exported {len(self.collected_data)} frames in KITTI format to {kitti_dir}")
    
    def save_dataset_summary(self):
        """Save dataset summary and statistics"""
        summary = {
            'total_frames': len(self.collected_data),
            'cameras': list(self.vehicle.cameras.keys()),
            'lidars': list(self.vehicle.lidars.keys()),
            'output_directory': self.output_dir,
            'creation_time': datetime.now().isoformat()
        }
        
        summary_path = os.path.join(self.output_dir, "dataset_summary.json")
        with open(summary_path, 'w') as f:
            json.dump(summary, f, indent=2)
        
        print(f"Dataset summary saved to {summary_path}")


class CustomDatasetExporter:
    """Export dataset to custom JSON-based format"""
    
    def __init__(self, dataset_dir, output_dir="./custom_dataset"):
        self.dataset_dir = dataset_dir
        self.output_dir = output_dir
        
    def export_to_custom_format(self):
        """Export to custom JSON-based format"""
        
        dataset_info = {
            'name': 'Isaac Sim Autonomous Driving Dataset',
            'version': '1.0',
            'description': 'Synthetic autonomous driving dataset generated with Isaac Sim',
            'created': datetime.now().isoformat(),
            'sensors': {
                'cameras': ['front', 'rear', 'left', 'right'],
                'lidars': ['main']
            },
            'annotations': {
                'types': ['3d_bbox', 'semantic_segmentation', 'instance_segmentation'],
                'classes': self.get_semantic_classes()
            },
            'frames': []
        }
        
        # Process each frame
        frame_files = sorted([f for f in os.listdir(f"{self.dataset_dir}/images") 
                            if f.endswith('_front_rgb.png')])
        
        for image_file in frame_files:
            frame_id = image_file.split('_')[0]
            frame_info = self.process_frame(frame_id)
            dataset_info['frames'].append(frame_info)
            
        # Save dataset info
        os.makedirs(self.output_dir, exist_ok=True)
        with open(f"{self.output_dir}/dataset_info.json", 'w') as f:
            json.dump(dataset_info, f, indent=2)
            
        print(f"Custom dataset exported with {len(dataset_info['frames'])} frames")
        
    def get_semantic_classes(self):
        """Get list of semantic classes in the dataset"""
        # This would be populated based on your scene setup
        return [
            'car', 'truck', 'bus', 'motorcycle', 'bicycle',
            'person', 'road', 'sidewalk', 'building', 'vegetation',
            'sky', 'pole', 'traffic_sign', 'traffic_light'
        ]
        
    def process_frame(self, frame_id):
        """Process individual frame data"""
        
        frame_info = {
            'frame_id': frame_id,
            'timestamp': None,
            'sensors': {
                'cameras': {},
                'lidars': {}
            },
            'annotations': {
                '3d_bboxes': [],
                'semantic_masks': {},
                'instance_masks': {}
            },
            'vehicle_pose': None
        }
        
        # Load pose information
        pose_file = f"{self.dataset_dir}/annotations/{frame_id}_pose.json"
        if os.path.exists(pose_file):
            with open(pose_file, 'r') as f:
                pose_data = json.load(f)
                frame_info['timestamp'] = pose_data['timestamp']
                frame_info['vehicle_pose'] = {
                    'position': pose_data['position'],
                    'orientation': pose_data['orientation']
                }
        
        # Process camera data
        for camera_name in ['front', 'rear', 'left', 'right']:
            rgb_file = f"{self.dataset_dir}/images/{frame_id}_{camera_name}_rgb.png"
            if os.path.exists(rgb_file):
                frame_info['sensors']['cameras'][camera_name] = {
                    'rgb': f"images/{frame_id}_{camera_name}_rgb.png",
                    'depth': f"images/{frame_id}_{camera_name}_depth.npy",
                    'semantic': f"annotations/{frame_id}_{camera_name}_semantic.png",
                    'instance': f"annotations/{frame_id}_{camera_name}_instance.png"
                }
        
        # Process LiDAR data
        lidar_file = f"{self.dataset_dir}/lidar/{frame_id}_main_points.bin"
        if os.path.exists(lidar_file):
            frame_info['sensors']['lidars']['main'] = {
                'points': f"lidar/{frame_id}_main_points.bin",
                'semantic': f"lidar/{frame_id}_main_semantic.npy"
            }
        
        return frame_info


class AutonomousDrivingScenario:
    """Main class to orchestrate autonomous driving scenario"""
    
    def __init__(self):
        if not ISAAC_SIM_AVAILABLE:
            raise RuntimeError("Isaac Sim is required to run this scenario")
            
        self.world = None
        self.environment_loader = None
        self.vehicle = None
        self.autonomous_driver = None
        self.manual_driver = None
        self.data_collector = None
        
        self.setup_isaac_sim()
    
    def setup_isaac_sim(self):
        """Initialize Isaac Sim environment"""
        # Enable extensions
        if not IsaacSimSetup.enable_extensions():
            raise RuntimeError("Failed to enable required Isaac Sim extensions")
        
        # Create world
        try:
            self.world = World(stage_units_in_meters=1.0)
        except Exception as e:
            print(f"Error initializing world: {e}")
            raise
        
        # Initialize components
        self.environment_loader = EnvironmentLoader(self.world)
        
        print("Isaac Sim setup completed")
    
    def setup_environment(self, env_type: str = "warehouse", **kwargs):
        """Setup the driving environment"""
        if env_type == "warehouse":
            self.environment_loader.load_warehouse_environment()
        elif env_type == "hospital":
            self.environment_loader.load_hospital_environment()
        elif env_type == "carla":
            town_name = kwargs.get("town_name", "Town01")
            self.environment_loader.load_carla_map(town_name)
        elif env_type == "custom":
            map_path = kwargs.get("map_path")
            if map_path:
                self.environment_loader.load_custom_city_map(map_path)
        elif env_type == "road_network":
            self.environment_loader.create_simple_road_network()
        else:
            print(f"Unknown environment type: {env_type}")
    
    def setup_vehicle(self):
        """Setup the autonomous vehicle"""
        self.vehicle = AutonomousVehicle(self.world)
        self.vehicle.configure_cameras()
        self.vehicle.configure_lidar()
        
        # Initialize driving controllers
        self.autonomous_driver = AutonomousDriving(self.vehicle, self.world)
        self.manual_driver = ManualDriving(self.vehicle, self.world)
        
        print("Vehicle setup completed")
    
    def setup_data_collection(self, output_dir: str = "./dataset"):
        """Setup data collection system"""
        self.data_collector = DataCollector(self.vehicle, output_dir)
        print("Data collection setup completed")
    
    def run_autonomous_scenario(self, waypoints: Optional[List[List[float]]] = None):
        """Run autonomous driving scenario"""
        if waypoints:
            self.autonomous_driver.set_waypoints(waypoints)
        else:
            # Create default circular path
            self.autonomous_driver.create_circular_path()
        
        print("Starting autonomous driving scenario...")
        
        # Reset world
        self.world.reset()
        
        # Main simulation loop
        while True:
            # Update autonomous driving
            self.autonomous_driver.update_autonomous_driving()
            
            # Collect data if enabled
            if self.data_collector:
                self.data_collector.collect_frame_data()
            
            # Step simulation
            self.world.step(render=True)
    
    def run_manual_scenario(self):
        """Run manual driving scenario"""
        print("Starting manual driving scenario...")
        print("Controls: W/S - Forward/Backward, A/D - Left/Right, Space - Brake")
        
        # Reset world
        self.world.reset()
        
        # Main simulation loop
        while True:
            # Update manual driving
            self.manual_driver.update_manual_driving()
            
            # Collect data if enabled
            if self.data_collector:
                self.data_collector.collect_frame_data()
            
            # Step simulation
            self.world.step(render=True)


def main():
    """Main function to run autonomous driving scenario"""
    if not ISAAC_SIM_AVAILABLE:
        print("Error: Isaac Sim is not available. This script must be run within Isaac Sim environment.")
        return
    
    try:
        # Create scenario
        scenario = AutonomousDrivingScenario()
        
        # Setup environment (change as needed)
        scenario.setup_environment(env_type="warehouse")
        
        # Setup vehicle
        scenario.setup_vehicle()
        
        # Setup data collection
        scenario.setup_data_collection(output_dir="./autonomous_driving_dataset")
        
        # Run scenario (change to run_manual_scenario() for manual control)
        scenario.run_autonomous_scenario()
    except Exception as e:
        print(f"Error running autonomous driving scenario: {e}")
        raise


def validate_environment():
    """Validate that all required dependencies are available"""
    issues = []
    
    if not ISAAC_SIM_AVAILABLE:
        issues.append("Isaac Sim modules not available - script must run within Isaac Sim")
    
    if not CV2_AVAILABLE:
        issues.append("OpenCV not available - image processing will use numpy fallback")
    
    # Check numpy availability
    try:
        import numpy as np
    except ImportError:
        issues.append("NumPy is required but not available")
    
    if issues:
        print("Environment validation issues:")
        for issue in issues:
            print(f"  - {issue}")
        return False
    
    print("Environment validation passed")
    return True


if __name__ == "__main__":
    # Validate environment before running
    if validate_environment():
        main()
    else:
        print("Environment validation failed. Please check dependencies.")
        exit(1)