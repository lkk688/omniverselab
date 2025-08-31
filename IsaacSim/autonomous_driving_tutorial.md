# Autonomous Driving Scenario Setup in NVIDIA Isaac Sim

A comprehensive tutorial for setting up autonomous driving scenarios in Isaac Sim, including map loading, vehicle configuration with sensors, driving modes, and dataset collection with ground truth annotations.

## Table of Contents

1. [Prerequisites](#prerequisites)
2. [Environment Setup](#environment-setup)
3. [Loading Maps and Environments](#loading-maps-and-environments)
4. [Vehicle Setup](#vehicle-setup)
5. [Sensor Configuration](#sensor-configuration)
6. [Driving Modes](#driving-modes)
7. [Data Collection](#data-collection)
8. [Dataset Export](#dataset-export)
9. [Advanced Features](#advanced-features)
10. [Troubleshooting](#troubleshooting)

## Prerequisites

### System Requirements
- NVIDIA Isaac Sim 2023.1.1 or later
- NVIDIA GPU with RTX capabilities (RTX 2070 or higher recommended)
- Ubuntu 20.04/22.04 or Windows 10/11
- 32GB RAM minimum (64GB recommended)
- 100GB+ free disk space for datasets

### Software Dependencies
- ROS2 Humble (optional, for ROS integration)
- Python 3.8+
- OpenCV (for image processing)
- NumPy, SciPy

### Isaac Sim Extensions
Ensure these extensions are enabled:
- `omni.isaac.sensor` - Camera and LiDAR sensors
- `omni.isaac.wheeled_robots` - Vehicle dynamics
- `omni.isaac.synthetic_utils` - Data generation utilities
- `omni.isaac.core` - Core Isaac Sim functionality
- `omni.replicator.core` - Synthetic data generation

## Environment Setup

### 1. Launch Isaac Sim

```bash
# From Isaac Sim installation directory
./isaac_sim.sh

# Or with specific GPU
CUDA_VISIBLE_DEVICES=0 ./isaac_sim.sh
```

### 2. Enable Required Extensions

Use the <mcsymbol name="enable_extensions" filename="autonomous_driving_isaac_sim.py" path="/Users/kaikailiu/Documents/MyRepo/omniverselab/IsaacSim/autonomous_driving_isaac_sim.py" startline="25" type="function"></mcsymbol> method from the `IsaacSimSetup` class:

```python
from autonomous_driving_isaac_sim import IsaacSimSetup

# Enable all required extensions
IsaacSimSetup.enable_extensions()
```

### 3. Import Required Libraries

For a complete implementation, see the modularized Python script: <mcfile name="autonomous_driving_isaac_sim.py" path="/Users/kaikailiu/Documents/MyRepo/omniverselab/IsaacSim/autonomous_driving_isaac_sim.py"></mcfile>

The script includes all necessary imports and is organized into the following classes:
- `IsaacSimSetup` - Extension management
- `EnvironmentLoader` - Map and environment loading
- `AutonomousVehicle` - Vehicle setup and sensor configuration
- `AutonomousDriving` - Autonomous driving controller
- `ManualDriving` - Manual driving controller
- `DataCollector` - Data collection and export
- `AutonomousDrivingScenario` - Main orchestration class

## Loading Maps and Environments

### 1. Using Pre-built Environments

#### Simple Warehouse Environment
Use the <mcsymbol name="load_warehouse_environment" filename="autonomous_driving_isaac_sim.py" path="/Users/kaikailiu/Documents/MyRepo/omniverselab/IsaacSim/autonomous_driving_isaac_sim.py" startline="48" type="function"></mcsymbol> method:

```python
from autonomous_driving_isaac_sim import EnvironmentLoader

world = World(stage_units_in_meters=1.0)
env_loader = EnvironmentLoader(world)
env_loader.load_warehouse_environment()
```

#### Hospital Environment
Use the <mcsymbol name="load_hospital_environment" filename="autonomous_driving_isaac_sim.py" path="/Users/kaikailiu/Documents/MyRepo/omniverselab/IsaacSim/autonomous_driving_isaac_sim.py" startline="53" type="function"></mcsymbol> method:

```python
env_loader.load_hospital_environment()
```

### 2. Loading Custom City Maps

#### From USD Files
Use the <mcsymbol name="load_custom_city_map" filename="autonomous_driving_isaac_sim.py" path="/Users/kaikailiu/Documents/MyRepo/omniverselab/IsaacSim/autonomous_driving_isaac_sim.py" startline="57" type="function"></mcsymbol> method:

```python
env_loader.load_custom_city_map("/path/to/your/city_map.usd")
```

#### From CARLA Maps (if available)

**Prerequisites for CARLA Integration**:
- Windows 10/11 (Omniverse Unreal Engine Connector is Windows-only)
- NVIDIA Omniverse installed and configured
- CARLA Simulator with Omniverse integration
- Unreal Engine 4.26 with Omniverse Connector

**Setup CARLA-Omniverse Integration**:

1. **Install Omniverse Unreal Engine Connector**:
   ```bash
   # Launch NVIDIA Omniverse Launcher
   # Navigate to Exchange tab
   # Install Epic Games Unreal Engine 4.26 Omniverse Connector (Release 105.1.578)
   ```

2. **Setup Local Omniverse Server**:
   ```bash
   # In Omniverse Launcher:
   # 1. Navigate to Nucleus tab
   # 2. Click "Create Local Server"
   # 3. Create administrator credentials
   # 4. Note the server URL (typically omniverse://localhost)
   ```

3. **Connect CARLA to Omniverse**:
   ```bash
   # Launch CARLA
   make launch
   
   # In Unreal Engine:
   # 1. Select Omniverse icon → Add Server
   # 2. Connect to your local server
   # 3. Authenticate with admin credentials
   ```

4. **Export CARLA Maps to USD**:
   ```python
   # In CARLA Unreal Engine project
   # Navigate to CarlaTools/Content/USDImporter
   # Use UW_USDVehicleImporterEditorWidget for USD export
   
   # Export Town maps to USD format
   town_maps = [
       "Town01", "Town02", "Town03", "Town04", "Town05",
       "Town06", "Town07", "Town10HD"
   ]
   
   for town in town_maps:
       export_path = f"omniverse://localhost/CARLA_Maps/{town}.usd"
       # Use CARLA's USD export functionality
   ```

**Loading CARLA Maps in Isaac Sim**:

```python
# Method 1: Direct USD reference (if maps are already exported)
carla_maps = {
    "Town01": "omniverse://localhost/NVIDIA/Assets/Isaac/2023.1.1/Isaac/Environments/Carla/Town01.usd",
    "Town02": "omniverse://localhost/NVIDIA/Assets/Isaac/2023.1.1/Isaac/Environments/Carla/Town02.usd",
    "Town03": "omniverse://localhost/NVIDIA/Assets/Isaac/2023.1.1/Isaac/Environments/Carla/Town03.usd",
    "Town10HD": "omniverse://localhost/NVIDIA/Assets/Isaac/2023.1.1/Isaac/Environments/Carla/Town10HD.usd"
}

# Load specific CARLA town
selected_town = "Town01"  # Change as needed
carla_map_path = carla_maps[selected_town]
add_reference_to_stage(usd_path=carla_map_path, prim_path="/World/CarlaMap")

# Method 2: Load from custom exported location
custom_carla_path = "omniverse://localhost/CARLA_Maps/Town01.usd"
add_reference_to_stage(usd_path=custom_carla_path, prim_path="/World/CarlaMap")

# Method 3: Load multiple towns for comparison
for i, (town_name, town_path) in enumerate(carla_maps.items()):
    prim_path = f"/World/CARLA_{town_name}"
    add_reference_to_stage(usd_path=town_path, prim_path=prim_path)
    
    # Offset each town to avoid overlap
    offset_x = i * 1000  # 1km spacing
    omni.kit.commands.execute(
        "ChangeProperty",
        prop_path=f"{prim_path}.xformOp:translate",
        value=(offset_x, 0, 0),
        prev=None
    )
```

**CARLA Map Features**:

```python
# CARLA maps include:
# - Realistic road networks with lanes, intersections, traffic lights
# - Urban environments with buildings, sidewalks, parking areas
# - Highway sections with on/off ramps
# - Weather and lighting variations
# - Traffic signs and road markings

# Configure CARLA-specific settings
def configure_carla_environment():
    """Configure Isaac Sim for CARLA map compatibility"""
    
    # Set appropriate physics timestep for CARLA compatibility
    world.set_physics_dt(1.0/50.0)  # 50 FPS (CARLA standard)
    
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
```

**Troubleshooting CARLA Integration**:

```python
# Common issues and solutions:

# Issue 1: Map not loading
def check_carla_map_availability():
    """Check if CARLA maps are available in Omniverse"""
    import omni.client
    
    carla_base_path = "omniverse://localhost/NVIDIA/Assets/Isaac/2023.1.1/Isaac/Environments/Carla/"
    
    try:
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

# Issue 2: Performance optimization for large CARLA maps
def optimize_carla_performance():
    """Optimize Isaac Sim settings for large CARLA environments"""
    
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

# Check availability before loading
available_maps = check_carla_map_availability()
if available_maps:
    print("CARLA integration ready")
else:
    print("Using alternative environments or custom road networks")
```

**Alternative: Custom CARLA-Style Roads**:

If CARLA maps are not available, create CARLA-inspired road networks:

```python
def create_carla_style_roads():
    """Create road networks inspired by CARLA towns"""
    
    # Town01-style: Simple grid with roundabout
    create_grid_roads(size=400, spacing=100, lane_width=3.5)
    create_roundabout(center=[0, 0], radius=30, lanes=2)
    
    # Add traffic infrastructure
    add_traffic_lights_at_intersections()
    add_road_markings()
    add_sidewalks(width=2.0)
    
    print("CARLA-style road network created")
```

### 3. Creating Road Networks Programmatically

```python
def create_simple_road_network():
    """Create a simple road network with intersections"""
    
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
    create_road_markings()
    
    return main_road, cross_road

def create_road_markings():
    """Add lane markings and traffic signs"""
    
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
```

## Vehicle Setup

### 1. Load Vehicle Model

The complete <mcsymbol name="AutonomousVehicle" filename="autonomous_driving_isaac_sim.py" path="/Users/kaikailiu/Documents/MyRepo/omniverselab/IsaacSim/autonomous_driving_isaac_sim.py" startline="108" type="class"></mcsymbol> class handles vehicle setup and sensor configuration:

```python
from autonomous_driving_isaac_sim import AutonomousVehicle

# Create autonomous vehicle with sensors
vehicle = AutonomousVehicle(world, vehicle_path="/World/Vehicle")

# Configure cameras and LiDAR
vehicle.configure_cameras()
vehicle.configure_lidar()
```

The class includes:
- <mcsymbol name="setup_vehicle" filename="autonomous_driving_isaac_sim.py" path="/Users/kaikailiu/Documents/MyRepo/omniverselab/IsaacSim/autonomous_driving_isaac_sim.py" startline="117" type="function"></mcsymbol> - Vehicle loading and controller setup
- <mcsymbol name="setup_sensors" filename="autonomous_driving_isaac_sim.py" path="/Users/kaikailiu/Documents/MyRepo/omniverselab/IsaacSim/autonomous_driving_isaac_sim.py" startline="148" type="function"></mcsymbol> - Multi-camera and LiDAR sensor setup

### 2. Advanced Vehicle Configuration

```python
def configure_vehicle_physics(vehicle_path):
    """Configure realistic vehicle physics"""
    
    # Set vehicle mass and inertia
    vehicle_prim = get_prim_at_path(vehicle_path)
    
    # Add rigid body properties
    omni.kit.commands.execute(
        "AddPhysicsComponent",
        usd_prim=vehicle_prim,
        component="PhysicsRigidBodyAPI"
    )
    
    # Set mass properties
    omni.kit.commands.execute(
        "ChangeProperty",
        prop_path=f"{vehicle_path}.physics:mass",
        value=1500.0,  # 1500 kg
        prev=None
    )
    
    # Configure wheel physics
    wheel_paths = [
        f"{vehicle_path}/front_left_wheel",
        f"{vehicle_path}/front_right_wheel", 
        f"{vehicle_path}/rear_left_wheel",
        f"{vehicle_path}/rear_right_wheel"
    ]
    
    for wheel_path in wheel_paths:
        if get_prim_at_path(wheel_path):
            # Add wheel collider
            omni.kit.commands.execute(
                "AddPhysicsComponent",
                usd_prim=get_prim_at_path(wheel_path),
                component="PhysicsCollisionAPI"
            )
```

## Sensor Configuration

### 1. Camera Configuration

Use the <mcsymbol name="configure_cameras" filename="autonomous_driving_isaac_sim.py" path="/Users/kaikailiu/Documents/MyRepo/omniverselab/IsaacSim/autonomous_driving_isaac_sim.py" startline="207" type="function"></mcsymbol> method from the `AutonomousVehicle` class:

```python
# Configure cameras with realistic parameters and enable segmentation
vehicle.configure_cameras()
```

This method automatically:
- Sets realistic focal lengths, aperture, and clipping ranges
- Enables motion vectors, distance mapping, and semantic/instance segmentation
- Configures different parameters for front/rear cameras

### 2. LiDAR Configuration

Use the <mcsymbol name="configure_lidar" filename="autonomous_driving_isaac_sim.py" path="/Users/kaikailiu/Documents/MyRepo/omniverselab/IsaacSim/autonomous_driving_isaac_sim.py" startline="240" type="function"></mcsymbol> method from the `AutonomousVehicle` class:

```python
# Configure LiDAR with realistic Velodyne VLS128 parameters
vehicle.configure_lidar()
```

This method configures:
- Range: 0.4m to 120m
- 360° horizontal FOV, 40° vertical FOV
- High resolution (0.4° resolution)
- 10Hz rotation frequency
- Semantic segmentation enabled

## Driving Modes

### 1. Autonomous Driving

Use the <mcsymbol name="AutonomousDriving" filename="autonomous_driving_isaac_sim.py" path="/Users/kaikailiu/Documents/MyRepo/omniverselab/IsaacSim/autonomous_driving_isaac_sim.py" startline="264" type="class"></mcsymbol> class for waypoint-based navigation:

```python
from autonomous_driving_isaac_sim import AutonomousDriving

# Setup autonomous driving
autonomous_driver = AutonomousDriving(vehicle)

# Define waypoints (x, y coordinates)
waypoints = [
    [10, 0], [10, 10], [0, 10], [-10, 10], 
    [-10, 0], [-10, -10], [0, -10], [10, -10]
]
autonomous_driver.set_waypoints(waypoints)
autonomous_driver.setup_navigation()

# In simulation loop
autonomous_driver.navigate_to_waypoint()
```

The class provides:
- <mcsymbol name="set_waypoints" filename="autonomous_driving_isaac_sim.py" path="/Users/kaikailiu/Documents/MyRepo/omniverselab/IsaacSim/autonomous_driving_isaac_sim.py" startline="271" type="function"></mcsymbol> - Define navigation path
- <mcsymbol name="navigate_to_waypoint" filename="autonomous_driving_isaac_sim.py" path="/Users/kaikailiu/Documents/MyRepo/omniverselab/IsaacSim/autonomous_driving_isaac_sim.py" startline="283" type="function"></mcsymbol> - Proportional control navigation

### 2. Manual Driving (Keyboard)

Use the <mcsymbol name="ManualDriving" filename="autonomous_driving_isaac_sim.py" path="/Users/kaikailiu/Documents/MyRepo/omniverselab/IsaacSim/autonomous_driving_isaac_sim.py" startline="330" type="class"></mcsymbol> class for keyboard-controlled driving:

```python
from autonomous_driving_isaac_sim import ManualDriving

# Setup manual driving
manual_driver = ManualDriving(vehicle)

# In simulation loop
key_states = manual_driver.get_keyboard_state()
manual_driver.handle_keyboard_input(key_states)
```

Key mappings:
- **W/S**: Forward/Backward movement
- **A/D**: Left/Right steering  
- **Space**: Emergency brake

The class includes:
- <mcsymbol name="setup_keyboard_control" filename="autonomous_driving_isaac_sim.py" path="/Users/kaikailiu/Documents/MyRepo/omniverselab/IsaacSim/autonomous_driving_isaac_sim.py" startline="336" type="function"></mcsymbol> - Initialize input handling
- <mcsymbol name="handle_keyboard_input" filename="autonomous_driving_isaac_sim.py" path="/Users/kaikailiu/Documents/MyRepo/omniverselab/IsaacSim/autonomous_driving_isaac_sim.py" startline="348" type="function"></mcsymbol> - Process keyboard commands

### 3. Game Controller Support

Gamepad control is integrated into the <mcsymbol name="ManualDriving" filename="autonomous_driving_isaac_sim.py" path="/Users/kaikailiu/Documents/MyRepo/omniverselab/IsaacSim/autonomous_driving_isaac_sim.py" startline="330" type="class"></mcsymbol> class:

```python
# Setup gamepad control
manual_driver.setup_gamepad_control()

# In simulation loop
gamepad_state = manual_driver.get_gamepad_state()
manual_driver.handle_gamepad_input(gamepad_state)
```

Gamepad mappings:
- **Left Stick**: Movement and steering
- **Right/Left Triggers**: Fine speed control
- **Deadzone**: 0.1 threshold for stick inputs

The gamepad controller provides:
- Analog stick control with deadzone handling
- Trigger-based acceleration/deceleration
- Smooth velocity transitions

## Data Collection

### 1. Setup Data Collection

Use the <mcsymbol name="DataCollector" filename="autonomous_driving_isaac_sim.py" path="/Users/kaikailiu/Documents/MyRepo/omniverselab/IsaacSim/autonomous_driving_isaac_sim.py" startline="395" type="class"></mcsymbol> class for comprehensive sensor data collection:

```python
from autonomous_driving_isaac_sim import DataCollector

# Setup data collection
data_collector = DataCollector(vehicle, output_dir="./autonomous_driving_dataset")

# In simulation loop
frame_data = data_collector.collect_frame_data()
```

The class automatically collects:
- **Multi-camera data**: RGB, depth, semantic/instance segmentation
- **LiDAR point clouds**: 3D points with semantic labels
- **Vehicle pose**: Position and orientation
- **Ground truth annotations**: 3D bounding boxes and object labels

Key methods:
- <mcsymbol name="collect_frame_data" filename="autonomous_driving_isaac_sim.py" path="/Users/kaikailiu/Documents/MyRepo/omniverselab/IsaacSim/autonomous_driving_isaac_sim.py" startline="415" type="function"></mcsymbol> - Main data collection function
- <mcsymbol name="collect_camera_data" filename="autonomous_driving_isaac_sim.py" path="/Users/kaikailiu/Documents/MyRepo/omniverselab/IsaacSim/autonomous_driving_isaac_sim.py" startline="436" type="function"></mcsymbol> - Multi-modal camera data
- <mcsymbol name="collect_lidar_data" filename="autonomous_driving_isaac_sim.py" path="/Users/kaikailiu/Documents/MyRepo/omniverselab/IsaacSim/autonomous_driving_isaac_sim.py" startline="463" type="function"></mcsymbol> - Point cloud with semantics

### 2. Ground Truth Annotation

Ground truth collection is integrated into the <mcsymbol name="DataCollector" filename="autonomous_driving_isaac_sim.py" path="/Users/kaikailiu/Documents/MyRepo/omniverselab/IsaacSim/autonomous_driving_isaac_sim.py" startline="395" type="class"></mcsymbol> class:

```python
# Ground truth is automatically collected with frame data
frame_data = data_collector.collect_frame_data()
ground_truth = frame_data['ground_truth']

# Access specific annotation types
objects_3d = ground_truth['objects']
scene_info = ground_truth['scene_info']
```

The <mcsymbol name="collect_ground_truth" filename="autonomous_driving_isaac_sim.py" path="/Users/kaikailiu/Documents/MyRepo/omniverselab/IsaacSim/autonomous_driving_isaac_sim.py" startline="481" type="function"></mcsymbol> method provides:
- **3D bounding boxes**: Object positions and dimensions
- **Object classes**: Semantic labels (car, pedestrian, etc.)
- **Camera projections**: 2D bounding boxes for each camera view
- **Scene metadata**: Lighting, weather, traffic density

Supported annotation formats:
- **3D Object Detection**: Position, rotation, dimensions
- **2D Object Detection**: Projected bounding boxes
- **Semantic Segmentation**: Per-pixel class labels
- **Instance Segmentation**: Per-pixel instance IDs

## Dataset Export

### 1. KITTI Format Export

Use the <mcsymbol name="export_to_kitti" filename="autonomous_driving_isaac_sim.py" path="/Users/kaikailiu/Documents/MyRepo/omniverselab/IsaacSim/autonomous_driving_isaac_sim.py" startline="520" type="function"></mcsymbol> method from the `DataCollector` class:

```python
# Export collected dataset to KITTI format
data_collector.export_to_kitti("./kitti_dataset")
```

The export creates standard KITTI directory structure:
```
kitti_dataset/
├── image_2/          # Left camera images
├── image_3/          # Right camera images
├── velodyne/         # LiDAR point clouds (.bin)
├── label_2/          # 2D object labels (.txt)
└── calib/            # Camera calibration (.txt)
```

KITTI format features:
- **Images**: PNG format, stereo camera setup
- **LiDAR**: Binary point clouds (x,y,z,intensity)
- **Labels**: Object detection annotations
- **Calibration**: Camera intrinsics and extrinsics

The method includes:
- <mcsymbol name="export_kitti_images" filename="autonomous_driving_isaac_sim.py" path="/Users/kaikailiu/Documents/MyRepo/omniverselab/IsaacSim/autonomous_driving_isaac_sim.py" startline="540" type="function"></mcsymbol> - Image format conversion
- <mcsymbol name="export_kitti_lidar" filename="autonomous_driving_isaac_sim.py" path="/Users/kaikailiu/Documents/MyRepo/omniverselab/IsaacSim/autonomous_driving_isaac_sim.py" startline="555" type="function"></mcsymbol> - Point cloud binary export
- <mcsymbol name="export_kitti_labels" filename="autonomous_driving_isaac_sim.py" path="/Users/kaikailiu/Documents/MyRepo/omniverselab/IsaacSim/autonomous_driving_isaac_sim.py" startline="572" type="function"></mcsymbol> - Annotation format conversion

### 2. Custom Dataset Format

```python
class CustomDatasetExporter:
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
```

## Advanced Features

### 1. Weather and Lighting Variations

Weather and lighting control is integrated into the main scenario class:

```python
# Setup weather variations
scenario.setup_weather_variations()

# Apply different weather presets
scenario.apply_weather_preset("sunny")
scenario.apply_weather_preset("cloudy")
scenario.apply_weather_preset("rainy")
scenario.apply_weather_preset("night")
```

Available weather presets:
- **Sunny**: Clear sky, high intensity lighting
- **Cloudy**: Overcast conditions, medium lighting
- **Rainy**: Storm conditions, low visibility
- **Night**: Dark environment, minimal lighting

Each preset automatically adjusts:
- Dome light intensity and HDR textures
- Fog density and atmospheric effects
- Camera exposure and sensor parameters

### 2. Traffic and Pedestrian Simulation

Traffic simulation is handled by the <mcsymbol name="TrafficSimulator" filename="autonomous_driving_isaac_sim.py" path="/Users/kaikailiu/Documents/MyRepo/omniverselab/IsaacSim/autonomous_driving_isaac_sim.py" startline="720" type="class"></mcsymbol> class:

```python
# Setup traffic simulation
scenario.setup_traffic_simulation(num_vehicles=5, num_pedestrians=10)

# Update traffic in simulation loop
scenario.update_traffic_behavior()
```

Traffic features:
- **AI Vehicles**: Autonomous traffic with path following
- **Pedestrians**: Animated characters with walking behaviors
- **Collision Avoidance**: Basic obstacle detection and avoidance
- **Random Behaviors**: Varied speeds and movement patterns

The traffic simulator provides:
- <mcsymbol name="spawn_traffic_vehicles" filename="autonomous_driving_isaac_sim.py" path="/Users/kaikailiu/Documents/MyRepo/omniverselab/IsaacSim/autonomous_driving_isaac_sim.py" startline="730" type="function"></mcsymbol> - Create AI-controlled vehicles
- <mcsymbol name="spawn_pedestrians" filename="autonomous_driving_isaac_sim.py" path="/Users/kaikailiu/Documents/MyRepo/omniverselab/IsaacSim/autonomous_driving_isaac_sim.py" startline="750" type="function"></mcsymbol> - Add animated pedestrians
- <mcsymbol name="update_traffic_behavior" filename="autonomous_driving_isaac_sim.py" path="/Users/kaikailiu/Documents/MyRepo/omniverselab/IsaacSim/autonomous_driving_isaac_sim.py" startline="770" type="function"></mcsymbol> - AI behavior updates

## Complete Example Script

The complete autonomous driving scenario is implemented in <mcfile name="autonomous_driving_isaac_sim.py" path="/Users/kaikailiu/Documents/MyRepo/omniverselab/IsaacSim/autonomous_driving_isaac_sim.py"></mcfile>. Here's how to use it:

```python
#!/usr/bin/env python3
"""
Complete Autonomous Driving Scenario in Isaac Sim
Using the modularized autonomous_driving_isaac_sim.py
"""

from autonomous_driving_isaac_sim import AutonomousDrivingScenario

def main():
    """Main function demonstrating autonomous driving scenario"""
    try:
        # Create and setup scenario
        scenario = AutonomousDrivingScenario()
        scenario.setup_environment(environment_type="warehouse")
        scenario.setup_vehicle()
        scenario.setup_driving_modes()
        scenario.setup_data_collection()
        
        # Run autonomous driving simulation
        scenario.run_simulation(duration=30.0, mode="autonomous")
        
        # Export dataset in KITTI format
        scenario.export_dataset(format="kitti")
        
    except Exception as e:
        print(f"Error in simulation: {e}")
    finally:
        scenario.cleanup()

if __name__ == "__main__":
    main()
```

The <mcsymbol name="AutonomousDrivingScenario" filename="autonomous_driving_isaac_sim.py" path="/Users/kaikailiu/Documents/MyRepo/omniverselab/IsaacSim/autonomous_driving_isaac_sim.py" startline="600" type="class"></mcsymbol> class orchestrates all components:

- **Environment Setup**: <mcsymbol name="setup_environment" filename="autonomous_driving_isaac_sim.py" path="/Users/kaikailiu/Documents/MyRepo/omniverselab/IsaacSim/autonomous_driving_isaac_sim.py" startline="615" type="function"></mcsymbol> - Load maps and environments
- **Vehicle Configuration**: <mcsymbol name="setup_vehicle" filename="autonomous_driving_isaac_sim.py" path="/Users/kaikailiu/Documents/MyRepo/omniverselab/IsaacSim/autonomous_driving_isaac_sim.py" startline="625" type="function"></mcsymbol> - Initialize vehicle and sensors
- **Driving Modes**: <mcsymbol name="setup_driving_modes" filename="autonomous_driving_isaac_sim.py" path="/Users/kaikailiu/Documents/MyRepo/omniverselab/IsaacSim/autonomous_driving_isaac_sim.py" startline="635" type="function"></mcsymbol> - Configure autonomous/manual control
- **Data Collection**: <mcsymbol name="setup_data_collection" filename="autonomous_driving_isaac_sim.py" path="/Users/kaikailiu/Documents/MyRepo/omniverselab/IsaacSim/autonomous_driving_isaac_sim.py" startline="650" type="function"></mcsymbol> - Initialize sensor data recording
- **Simulation Loop**: <mcsymbol name="run_simulation" filename="autonomous_driving_isaac_sim.py" path="/Users/kaikailiu/Documents/MyRepo/omniverselab/IsaacSim/autonomous_driving_isaac_sim.py" startline="660" type="function"></mcsymbol> - Execute driving scenario
- **Dataset Export**: <mcsymbol name="export_dataset" filename="autonomous_driving_isaac_sim.py" path="/Users/kaikailiu/Documents/MyRepo/omniverselab/IsaacSim/autonomous_driving_isaac_sim.py" startline="690" type="function"></mcsymbol> - Export in KITTI/custom formats

## Troubleshooting

### Common Issues

1. **Isaac Sim Extensions Not Loading**
   
   Use the <mcsymbol name="IsaacSimSetup" filename="autonomous_driving_isaac_sim.py" path="/Users/kaikailiu/Documents/MyRepo/omniverselab/IsaacSim/autonomous_driving_isaac_sim.py" startline="10" type="class"></mcsymbol> class to ensure proper extension loading:
   
   ```python
   from autonomous_driving_isaac_sim import IsaacSimSetup
   
   # Check and enable extensions
   setup = IsaacSimSetup()
   setup.enable_extensions()
   setup.check_extension_status()
   ```

2. **Vehicle Physics Issues**
   
   The <mcsymbol name="AutonomousVehicle" filename="autonomous_driving_isaac_sim.py" path="/Users/kaikailiu/Documents/MyRepo/omniverselab/IsaacSim/autonomous_driving_isaac_sim.py" startline="108" type="class"></mcsymbol> class includes physics debugging:
   
   ```python
   # Debug vehicle physics
   vehicle = AutonomousVehicle(world, "/World/Vehicle")
   vehicle.debug_physics_configuration()
   vehicle.validate_rigid_body_setup()
   ```

3. **Sensor Data Collection Issues**
   
   Use the <mcsymbol name="DataCollector" filename="autonomous_driving_isaac_sim.py" path="/Users/kaikailiu/Documents/MyRepo/omniverselab/IsaacSim/autonomous_driving_isaac_sim.py" startline="395" type="class"></mcsymbol> debugging methods:
   
   ```python
   # Debug sensor data
   data_collector = DataCollector(vehicle, "./dataset")
   data_collector.debug_sensor_data()
   data_collector.validate_data_integrity()
   ```

4. **Performance Issues**
   
   Performance settings are handled by the main scenario:
   
   ```python
   # Apply performance optimizations
   scenario = AutonomousDrivingScenario()
   scenario.optimize_performance()
   scenario.configure_physics_settings(timestep=1/60)
   ```

5. **Memory Management**
   
   Built-in memory management for long simulations:
   
   ```python
   # Configure memory management
   scenario.setup_data_collection(max_frames=1000, auto_export=True)
   scenario.enable_memory_optimization()
   ```

### Debug Mode

Enable debug mode for detailed logging:

```python
scenario = AutonomousDrivingScenario(debug=True)
scenario.enable_verbose_logging()
```

### Performance Tips

- **Reduce sensor resolution** for faster processing
- **Limit traffic vehicles** to improve performance
- **Use simplified physics** for non-critical objects
- **Enable batch processing** for data export
- **Monitor memory usage** during long simulations

### Performance Optimization

Performance optimization is handled automatically by the <mcsymbol name="AutonomousDrivingScenario" filename="autonomous_driving_isaac_sim.py" path="/Users/kaikailiu/Documents/MyRepo/omniverselab/IsaacSim/autonomous_driving_isaac_sim.py" startline="600" type="class"></mcsymbol> class:

```python
# Apply performance optimizations
scenario = AutonomousDrivingScenario()
scenario.optimize_rendering_settings()
scenario.configure_physics_timestep(fps=60)
scenario.enable_multi_gpu_rendering()
```

Optimization features:
- **GPU acceleration**: RTX sensor GPU and multi-GPU support
- **Physics optimization**: Configurable timestep and solver settings
- **Memory management**: Automatic cleanup and garbage collection
- **Sensor optimization**: Adaptive resolution and frame rate control

## Conclusion

This tutorial demonstrates how to create a comprehensive autonomous driving simulation in Isaac Sim using the modularized <mcfile name="autonomous_driving_isaac_sim.py" path="/Users/kaikailiu/Documents/MyRepo/omniverselab/IsaacSim/autonomous_driving_isaac_sim.py"></mcfile> script. The modular architecture provides:

### Key Benefits
- **Maintainable code**: Separated concerns with dedicated classes
- **Easy customization**: Configurable vehicle, sensor, and environment setups
- **Scalable design**: Support for multiple vehicles and complex scenarios
- **Standard compliance**: KITTI-compatible dataset export
- **Performance optimization**: Built-in memory and processing management

### Integration Opportunities
The modular design enables seamless integration with:
- **ROS2**: Real-time robotics communication and control
- **Machine Learning**: PyTorch, TensorFlow for AI model training
- **Cloud Platforms**: AWS, Azure for distributed simulation
- **Custom Environments**: Import your own USD scenes and assets
- **Hardware Systems**: Connect to real vehicle ECUs and sensors

### Research Applications
- **Algorithm Development**: Test perception, planning, and control algorithms
- **Dataset Generation**: Create large-scale annotated datasets
- **Scenario Testing**: Validate autonomous systems in diverse conditions
- **Multi-Agent Studies**: Investigate vehicle-to-vehicle interactions

## Additional Resources

- [Isaac Sim Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/)
- [Isaac ROS Documentation](https://nvidia-isaac-ros.github.io/)
- [KITTI Dataset Format](http://www.cvlibs.net/datasets/kitti/)
- [OpenUSD Documentation](https://openusd.org/release/index.html)
- [Omniverse Replicator](https://docs.omniverse.nvidia.com/extensions/latest/ext_replicator/)