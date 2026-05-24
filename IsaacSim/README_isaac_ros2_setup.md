# NVIDIA Isaac ROS2 Installation and System Check

A comprehensive Python script for installing and verifying NVIDIA Isaac ROS2 packages on Ubuntu systems.

## Features

### System Checks
- ✅ Ubuntu version compatibility (20.04/22.04)
- ✅ ROS2 installation and distribution verification
- ✅ NVIDIA GPU and CUDA detection
- ✅ Docker and NVIDIA Docker runtime
- ✅ Isaac ROS workspace status
- ✅ Isaac ROS packages verification
- ✅ Jetson platform detection

### Installation Capabilities
- 🔧 Automated dependency installation
- 🔧 Isaac ROS workspace setup
- 🔧 Key Isaac ROS packages cloning and building
- 🔧 Environment setup script generation

## Requirements

- **Operating System**: Ubuntu 20.04 or 22.04
- **ROS2**: Humble or Foxy distribution
- **Hardware**: NVIDIA GPU (recommended)
- **Python**: 3.8+

## Usage

### 1. System Check Only
```bash
python3 isaac_ros2_setup_check.py --check-only
```

### 2. Full Installation
```bash
python3 isaac_ros2_setup_check.py --install
```

### 3. Help
```bash
python3 isaac_ros2_setup_check.py --help
```

## What Gets Installed

### Core Dependencies
- Python development tools
- ROS2 build tools (colcon)
- System libraries for Isaac ROS
- Git and development utilities

### Isaac ROS Packages
- `isaac_ros_common` - Core Isaac ROS functionality
- `isaac_ros_visual_slam` - Visual SLAM capabilities
- `isaac_ros_nvblox` - 3D reconstruction
- `isaac_ros_object_detection` - Object detection
- `isaac_ros_apriltag` - AprilTag detection

### Workspace Structure
```
~/workspaces/isaac_ros-dev/
├── src/
│   ├── isaac_ros_common/
│   ├── isaac_ros_visual_slam/
│   ├── isaac_ros_nvblox/
│   ├── isaac_ros_object_detection/
│   └── isaac_ros_apriltag/
├── build/
├── install/
└── setup_isaac_ros.sh
```

## Post-Installation

After successful installation, source the environment:

```bash
source ~/workspaces/isaac_ros-dev/setup_isaac_ros.sh
```

Or add to your `~/.bashrc`:
```bash
echo "source ~/workspaces/isaac_ros-dev/setup_isaac_ros.sh" >> ~/.bashrc
```

## Verification

Test Isaac ROS installation:

```bash
# Check available Isaac ROS packages
ros2 pkg list | grep isaac_ros

# Test a simple Isaac ROS node
ros2 run isaac_ros_visual_slam isaac_ros_visual_slam_node
```

## Troubleshooting

### Common Issues

1. **ROS2 not found**
   ```bash
   sudo apt update
   sudo apt install software-properties-common
   sudo add-apt-repository universe
   sudo apt update && sudo apt install curl gnupg lsb-release
   # Follow ROS2 installation guide
   ```

2. **NVIDIA drivers not detected**
   ```bash
   sudo apt install nvidia-driver-525
   sudo reboot
   ```

3. **Docker permission denied**
   ```bash
   sudo usermod -aG docker $USER
   newgrp docker
   ```

4. **Build failures**
   ```bash
   cd ~/workspaces/isaac_ros-dev
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```

### Getting Help

- [Isaac ROS Documentation](https://nvidia-isaac-ros.github.io/)
- [Isaac ROS GitHub](https://github.com/NVIDIA-ISAAC-ROS)
- [ROS2 Documentation](https://docs.ros.org/en/humble/)
- [NVIDIA Developer Forums](https://forums.developer.nvidia.com/)

## Script Output Example

```
======================================================================
                    NVIDIA ISAAC ROS2 SYSTEM CHECK                    
======================================================================

Running Ubuntu Version check...

======================================================================
                        UBUNTU VERSION CHECK                         
======================================================================

Ubuntu Version: 22.04
Compatibility: ✅ Supported for Isaac ROS

Running ROS2 Installation check...

======================================================================
                       ROS2 INSTALLATION CHECK                       
======================================================================

ROS2 Version: ros2 cli version: 0.18.5
ROS2 Distro: humble
Isaac ROS Compatibility: ✅ Supported

...

======================================================================
                       SYSTEM CHECK SUMMARY                         
======================================================================

Ubuntu Version: ✅ PASS
ROS2 Installation: ✅ PASS
NVIDIA GPU & CUDA: ✅ PASS
Docker Installation: ✅ PASS
Isaac ROS Workspace: ✅ PASS
Isaac ROS Packages: ✅ PASS
Jetson Platform: ✅ PASS
Overall: 7/7 checks passed
```

## New

```bash
# On the RTX5090 box:
conda activate isaac_lerobot
cd /Developer/IsaacLab
cp /Developer/omniverselab/IsaacSim/isaac_auto_collector_v6.py .
cp /Developer/omniverselab/IsaacSim/isaac_multicam_addons.py .

# First — smoke test that the cameras actually got registered into the scene
./isaaclab.sh -p isaac_auto_collector_v6.py --enable_cameras \
    --env lift-ik-rel --list_cams
# Should print env.scene.sensors keys including top_camera, cam_left,
# cam_right, cam_front, cam_wrist — then exit.

# If that looks right, kick off real collection (~50 demos, ~10 min)
./isaaclab.sh -p isaac_auto_collector_v6.py --enable_cameras --autorun \
    --env lift-ik-rel --max_demos 50 \
    --cams top,left,right,front,wrist --cam_hw 480,640 \
    --save_dir logs/demos_multicam_lift

```

## License

This script is provided as-is for educational and development purposes.