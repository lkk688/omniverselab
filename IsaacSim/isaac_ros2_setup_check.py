#!/usr/bin/env python3
"""
NVIDIA Isaac ROS2 Installation and System Check

A comprehensive script to install and verify NVIDIA Isaac ROS2 packages including:
- System requirements verification (Ubuntu, ROS2, CUDA, Docker)
- Isaac ROS workspace setup
- Isaac ROS packages installation
- Hardware compatibility checks (NVIDIA GPU, Jetson platforms)
- Development environment validation

Requirements: Ubuntu 20.04/22.04, ROS2 Humble/Foxy, NVIDIA GPU
Usage: python3 isaac_ros2_setup_check.py [--install] [--check-only]
"""

import os
import sys
import subprocess
import platform
import json
import argparse
import urllib.request
import urllib.error
from pathlib import Path
import time
import shutil

class Colors:
    """ANSI color codes for terminal output"""
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

def print_header(text):
    """Print a formatted header"""
    print(f"\n{Colors.HEADER}{Colors.BOLD}{'='*70}{Colors.ENDC}")
    print(f"{Colors.HEADER}{Colors.BOLD}{text.center(70)}{Colors.ENDC}")
    print(f"{Colors.HEADER}{Colors.BOLD}{'='*70}{Colors.ENDC}\n")

def print_info(label, value, status="info"):
    """Print formatted information"""
    color = (
        Colors.OKGREEN if status == "success" else 
        Colors.WARNING if status == "warning" else 
        Colors.FAIL if status == "error" else 
        Colors.OKCYAN
    )
    print(f"{Colors.BOLD}{label}:{Colors.ENDC} {color}{value}{Colors.ENDC}")

def print_step(step_num, total_steps, description):
    """Print installation step"""
    print(f"\n{Colors.OKBLUE}{Colors.BOLD}[{step_num}/{total_steps}] {description}{Colors.ENDC}")

def run_command(command, shell=False, capture_output=True, check=False):
    """Run a command and return output"""
    try:
        if shell:
            result = subprocess.run(command, shell=True, capture_output=capture_output, 
                                 text=True, timeout=30)
        else:
            result = subprocess.run(command.split(), capture_output=capture_output, 
                                 text=True, timeout=30)
        
        if check and result.returncode != 0:
            print_info("Command failed", f"{command}: {result.stderr}", "error")
            return None
            
        return result.stdout if capture_output else result.returncode == 0
    except subprocess.TimeoutExpired:
        print_info("Command timeout", command, "error")
        return None
    except Exception as e:
        print_info("Command error", f"{command}: {str(e)}", "error")
        return None

def check_ubuntu_version():
    """Check Ubuntu version compatibility"""
    print_header("UBUNTU VERSION CHECK")
    
    try:
        with open('/etc/os-release', 'r') as f:
            os_info = f.read()
        
        version_line = [line for line in os_info.split('\n') if 'VERSION_ID' in line]
        if version_line:
            version = version_line[0].split('=')[1].strip('"')
            print_info("Ubuntu Version", version)
            
            if version in ['20.04', '22.04']:
                print_info("Compatibility", "✅ Supported for Isaac ROS", "success")
                return True
            else:
                print_info("Compatibility", "⚠️  Isaac ROS officially supports Ubuntu 20.04/22.04", "warning")
                return False
    except Exception as e:
        print_info("Ubuntu Check", f"Error: {str(e)}", "error")
        return False

def check_ros2_installation():
    """Check ROS2 installation and version"""
    print_header("ROS2 INSTALLATION CHECK")
    
    # Check ROS2 version
    ros2_version = run_command("ros2 --version")
    if ros2_version:
        print_info("ROS2 Version", ros2_version.strip(), "success")
        
        # Check ROS2 distro
        if 'ROS_DISTRO' in os.environ:
            distro = os.environ['ROS_DISTRO']
            print_info("ROS2 Distro", distro, "success")
            
            if distro in ['humble', 'foxy']:
                print_info("Isaac ROS Compatibility", "✅ Supported", "success")
                return True
            else:
                print_info("Isaac ROS Compatibility", "⚠️  Prefer Humble or Foxy", "warning")
                return False
        else:
            print_info("ROS2 Environment", "Not sourced", "warning")
            return False
    else:
        print_info("ROS2", "Not installed", "error")
        return False

def check_nvidia_gpu():
    """Check NVIDIA GPU and CUDA installation"""
    print_header("NVIDIA GPU & CUDA CHECK")
    
    # Check nvidia-smi
    nvidia_smi = run_command("nvidia-smi")
    if nvidia_smi:
        lines = nvidia_smi.strip().split('\n')
        for line in lines:
            if 'CUDA Version' in line:
                cuda_version = line.split('CUDA Version: ')[1].split()[0]
                print_info("CUDA Driver Version", cuda_version, "success")
            elif 'GeForce' in line or 'Quadro' in line or 'Tesla' in line or 'RTX' in line:
                gpu_info = ' '.join(line.split()[1:4])
                print_info("GPU Model", gpu_info, "success")
        
        # Check CUDA toolkit
        nvcc_version = run_command("nvcc --version")
        if nvcc_version:
            for line in nvcc_version.split('\n'):
                if 'release' in line:
                    cuda_toolkit = line.split('release ')[1].split(',')[0]
                    print_info("CUDA Toolkit", cuda_toolkit, "success")
                    break
        else:
            print_info("CUDA Toolkit", "Not installed", "warning")
        
        return True
    else:
        print_info("NVIDIA GPU", "Not detected or drivers not installed", "error")
        return False

def check_docker_installation():
    """Check Docker installation"""
    print_header("DOCKER INSTALLATION CHECK")
    
    docker_version = run_command("docker --version")
    if docker_version:
        print_info("Docker Version", docker_version.strip(), "success")
        
        # Check Docker daemon
        docker_info = run_command("docker info")
        if docker_info:
            print_info("Docker Daemon", "Running", "success")
        else:
            print_info("Docker Daemon", "Not running", "warning")
        
        # Check NVIDIA Docker runtime
        nvidia_docker = run_command("docker run --rm --gpus all nvidia/cuda:11.0-base nvidia-smi")
        if nvidia_docker:
            print_info("NVIDIA Docker Runtime", "Available", "success")
            return True
        else:
            print_info("NVIDIA Docker Runtime", "Not available", "warning")
            return False
    else:
        print_info("Docker", "Not installed", "error")
        return False

def check_isaac_ros_workspace():
    """Check Isaac ROS workspace"""
    print_header("ISAAC ROS WORKSPACE CHECK")
    
    workspace_path = Path.home() / "workspaces" / "isaac_ros-dev"
    
    if workspace_path.exists():
        print_info("Workspace Path", str(workspace_path), "success")
        
        # Check src directory
        src_path = workspace_path / "src"
        if src_path.exists():
            isaac_packages = list(src_path.glob("isaac_ros_*"))
            if isaac_packages:
                print_info("Isaac ROS Packages", f"{len(isaac_packages)} packages found", "success")
                for pkg in isaac_packages[:5]:  # Show first 5 packages
                    print_info("  Package", pkg.name)
                if len(isaac_packages) > 5:
                    print_info("  ...", f"and {len(isaac_packages) - 5} more")
            else:
                print_info("Isaac ROS Packages", "No packages found", "warning")
        
        # Check if workspace is built
        build_path = workspace_path / "build"
        install_path = workspace_path / "install"
        
        if build_path.exists() and install_path.exists():
            print_info("Workspace Status", "Built", "success")
            return True
        else:
            print_info("Workspace Status", "Not built", "warning")
            return False
    else:
        print_info("Isaac ROS Workspace", "Not found", "error")
        return False

def check_isaac_ros_packages():
    """Check installed Isaac ROS packages"""
    print_header("ISAAC ROS PACKAGES CHECK")
    
    # Key Isaac ROS packages to check
    isaac_packages = [
        'isaac_ros_common',
        'isaac_ros_visual_slam',
        'isaac_ros_nvblox',
        'isaac_ros_object_detection',
        'isaac_ros_apriltag',
        'isaac_ros_depth_segmentation',
        'isaac_ros_image_segmentation',
        'isaac_ros_stereo_image_proc'
    ]
    
    installed_packages = []
    
    for package in isaac_packages:
        # Check if package is available in ROS2
        result = run_command(f"ros2 pkg list | grep {package}")
        if result and result.strip():
            installed_packages.append(package)
            print_info(f"  {package}", "✅ Installed", "success")
        else:
            print_info(f"  {package}", "❌ Not found", "error")
    
    if installed_packages:
        print_info("Total Installed", f"{len(installed_packages)}/{len(isaac_packages)} packages", 
                  "success" if len(installed_packages) == len(isaac_packages) else "warning")
        return len(installed_packages) > 0
    else:
        print_info("Isaac ROS Packages", "None installed", "error")
        return False

def install_isaac_ros_dependencies():
    """Install Isaac ROS dependencies"""
    print_header("INSTALLING ISAAC ROS DEPENDENCIES")
    
    dependencies = [
        "sudo apt update",
        "sudo apt install -y python3-pip python3-dev",
        "sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential",
        "sudo apt install -y python3-colcon-common-extensions",
        "sudo apt install -y git curl wget",
        "sudo apt install -y libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev",
        "sudo apt install -y libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev"
    ]
    
    for i, cmd in enumerate(dependencies, 1):
        print_step(i, len(dependencies), f"Installing: {cmd.split()[-1] if 'install' in cmd else 'System update'}")
        result = run_command(cmd, shell=True, capture_output=False)
        if not result:
            print_info("Installation", f"Failed: {cmd}", "error")
            return False
    
    print_info("Dependencies", "✅ All installed successfully", "success")
    return True

def setup_isaac_ros_workspace():
    """Setup Isaac ROS workspace"""
    print_header("SETTING UP ISAAC ROS WORKSPACE")
    
    workspace_path = Path.home() / "workspaces" / "isaac_ros-dev"
    
    print_step(1, 4, "Creating workspace directory")
    workspace_path.mkdir(parents=True, exist_ok=True)
    src_path = workspace_path / "src"
    src_path.mkdir(exist_ok=True)
    
    print_step(2, 4, "Cloning Isaac ROS Common")
    os.chdir(src_path)
    result = run_command("git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git", shell=True)
    if not result:
        print_info("Clone", "Failed to clone isaac_ros_common", "error")
        return False
    
    print_step(3, 4, "Installing additional Isaac ROS packages")
    isaac_repos = [
        "https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git",
        "https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox.git",
        "https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_object_detection.git",
        "https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_apriltag.git"
    ]
    
    for repo in isaac_repos:
        repo_name = repo.split('/')[-1].replace('.git', '')
        print(f"  Cloning {repo_name}...")
        result = run_command(f"git clone {repo}", shell=True)
        if not result:
            print_info("Clone", f"Failed to clone {repo_name}", "warning")
    
    print_step(4, 4, "Building workspace")
    os.chdir(workspace_path)
    
    # Source ROS2 and build
    build_cmd = f"source /opt/ros/{os.environ.get('ROS_DISTRO', 'humble')}/setup.bash && colcon build --symlink-install"
    result = run_command(build_cmd, shell=True, capture_output=False)
    
    if result:
        print_info("Workspace Build", "✅ Successfully built", "success")
        return True
    else:
        print_info("Workspace Build", "❌ Build failed", "error")
        return False

def check_jetson_platform():
    """Check if running on NVIDIA Jetson platform"""
    print_header("JETSON PLATFORM CHECK")
    
    # Check for Jetson-specific files
    jetson_files = [
        "/etc/nv_tegra_release",
        "/sys/firmware/devicetree/base/model"
    ]
    
    is_jetson = False
    
    for file_path in jetson_files:
        if os.path.exists(file_path):
            try:
                with open(file_path, 'r') as f:
                    content = f.read().strip()
                    if 'tegra' in content.lower() or 'jetson' in content.lower():
                        print_info("Platform", f"NVIDIA Jetson detected: {content}", "success")
                        is_jetson = True
                        break
            except:
                continue
    
    if not is_jetson:
        # Check CPU info for Jetson
        cpu_info = run_command("cat /proc/cpuinfo")
        if cpu_info and 'tegra' in cpu_info.lower():
            print_info("Platform", "NVIDIA Jetson (detected via CPU info)", "success")
            is_jetson = True
    
    if not is_jetson:
        print_info("Platform", "Standard x86_64 system", "info")
    
    return is_jetson

def generate_setup_script():
    """Generate setup script for Isaac ROS environment"""
    print_header("GENERATING SETUP SCRIPT")
    
    workspace_path = Path.home() / "workspaces" / "isaac_ros-dev"
    script_path = workspace_path / "setup_isaac_ros.sh"
    
    setup_script = f"""#!/bin/bash
# Isaac ROS Environment Setup Script
# Generated by isaac_ros2_setup_check.py

# Source ROS2
source /opt/ros/{os.environ.get('ROS_DISTRO', 'humble')}/setup.bash

# Source Isaac ROS workspace
if [ -f "{workspace_path}/install/setup.bash" ]; then
    source {workspace_path}/install/setup.bash
    echo "✅ Isaac ROS environment loaded"
else
    echo "❌ Isaac ROS workspace not built. Run: colcon build --symlink-install"
fi

# Set useful environment variables
export ISAAC_ROS_WS={workspace_path}
export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp

# Display environment info
echo "Isaac ROS Workspace: $ISAAC_ROS_WS"
echo "ROS Distro: $ROS_DISTRO"
echo "RMW Implementation: $RMW_IMPLEMENTATION"
"""
    
    try:
        with open(script_path, 'w') as f:
            f.write(setup_script)
        
        # Make executable
        os.chmod(script_path, 0o755)
        
        print_info("Setup Script", f"Created: {script_path}", "success")
        print_info("Usage", f"source {script_path}", "info")
        return True
    except Exception as e:
        print_info("Setup Script", f"Failed to create: {str(e)}", "error")
        return False

def run_system_checks():
    """Run all system checks"""
    print_header("NVIDIA ISAAC ROS2 SYSTEM CHECK")
    
    checks = [
        ("Ubuntu Version", check_ubuntu_version),
        ("ROS2 Installation", check_ros2_installation),
        ("NVIDIA GPU & CUDA", check_nvidia_gpu),
        ("Docker Installation", check_docker_installation),
        ("Isaac ROS Workspace", check_isaac_ros_workspace),
        ("Isaac ROS Packages", check_isaac_ros_packages),
        ("Jetson Platform", check_jetson_platform)
    ]
    
    results = {}
    
    for check_name, check_func in checks:
        print(f"\n{Colors.OKCYAN}Running {check_name} check...{Colors.ENDC}")
        try:
            results[check_name] = check_func()
        except Exception as e:
            print_info(check_name, f"Error: {str(e)}", "error")
            results[check_name] = False
    
    # Summary
    print_header("SYSTEM CHECK SUMMARY")
    
    passed = sum(1 for result in results.values() if result)
    total = len(results)
    
    for check_name, result in results.items():
        status = "✅ PASS" if result else "❌ FAIL"
        color = "success" if result else "error"
        print_info(check_name, status, color)
    
    print_info("Overall", f"{passed}/{total} checks passed", 
              "success" if passed == total else "warning")
    
    return results

def run_installation():
    """Run Isaac ROS installation"""
    print_header("NVIDIA ISAAC ROS2 INSTALLATION")
    
    # Pre-installation checks
    if not check_ubuntu_version():
        print_info("Installation", "Ubuntu version not optimal for Isaac ROS", "warning")
    
    if not check_ros2_installation():
        print_info("Installation", "ROS2 must be installed first", "error")
        return False
    
    # Install dependencies
    if not install_isaac_ros_dependencies():
        print_info("Installation", "Failed to install dependencies", "error")
        return False
    
    # Setup workspace
    if not setup_isaac_ros_workspace():
        print_info("Installation", "Failed to setup workspace", "error")
        return False
    
    # Generate setup script
    generate_setup_script()
    
    print_header("INSTALLATION COMPLETE")
    print_info("Status", "✅ Isaac ROS installation completed successfully", "success")
    print_info("Next Steps", "Source the setup script: source ~/workspaces/isaac_ros-dev/setup_isaac_ros.sh", "info")
    
    return True

def main():
    """Main function"""
    parser = argparse.ArgumentParser(description="NVIDIA Isaac ROS2 Installation and System Check")
    parser.add_argument('--install', action='store_true', help='Install Isaac ROS packages')
    parser.add_argument('--check-only', action='store_true', help='Only run system checks')
    
    args = parser.parse_args()
    
    if args.install:
        success = run_installation()
        if success:
            print("\n" + "="*70)
            print("🎉 Isaac ROS installation completed successfully!")
            print("Run system checks with: python3 isaac_ros2_setup_check.py --check-only")
        else:
            print("\n" + "="*70)
            print("❌ Isaac ROS installation failed. Check the errors above.")
            sys.exit(1)
    else:
        results = run_system_checks()
        
        # Provide recommendations
        print_header("RECOMMENDATIONS")
        
        if not results.get("ROS2 Installation", False):
            print_info("Install ROS2", "Follow: https://docs.ros.org/en/humble/Installation.html", "info")
        
        if not results.get("NVIDIA GPU & CUDA", False):
            print_info("Install NVIDIA Drivers", "sudo apt install nvidia-driver-525 nvidia-cuda-toolkit", "info")
        
        if not results.get("Docker Installation", False):
            print_info("Install Docker", "Follow: https://docs.docker.com/engine/install/ubuntu/", "info")
        
        if not results.get("Isaac ROS Workspace", False):
            print_info("Setup Isaac ROS", "Run: python3 isaac_ros2_setup_check.py --install", "info")

if __name__ == "__main__":
    main()