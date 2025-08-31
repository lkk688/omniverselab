# Cloud VM System Information Checker

A comprehensive toolkit for checking and setting up cloud VM environments. These scripts require **no sudo privileges** and provide automated setup for remote development access.

## 🚀 Quick Start

```bash
# Comprehensive check with automated setup options
python3 vm_system_check.py

# Lightweight basic check
bash vm_system_check.sh

# Interactive launcher
bash run_system_check.sh
```

**Key Features:**
- **Automated SSH Setup** - One-click SSH server installation and configuration
- **VS Code Web Setup** - Automatic code-server installation and startup
- **No Sudo Required** - Works in restricted cloud environments
- **Interactive Prompts** - Offers to fix issues when detected

## 🚀 Quick Start

## 🔍 System Analysis

**Hardware & System:**
- CPU, GPU (NVIDIA), Memory, Disk usage
- Operating system and architecture
- Network connectivity and IP addresses

**Development Environment:**
- Python version, virtual environments, and packages
- Conda environments and installation
- ROS/ROS2 installation and active nodes
- System services and open development ports

**Remote Development Access:**
- SSH server status and configuration
- VS Code Web (code-server) availability
- Web IDE processes and ports

**Automated Setup Features:**
- Interactive SSH setup when not available
- One-click VS Code Web installation
- Automatic service configuration and startup

## 🛠️ Automated Setup Functions

### SSH Access Setup (`setup_ssh_access`)

When SSH is not available, the script offers to set it up automatically:

**What it does:**
1. **Detects OS** - Supports Linux (Ubuntu/Debian, RHEL/CentOS/Fedora) and macOS
2. **Installs SSH Server** - Uses appropriate package manager (apt/yum/dnf)
3. **Generates Host Keys** - Creates SSH server host keys if missing
4. **Starts Service** - Enables and starts SSH daemon (systemctl/launchctl)
5. **Creates User Keys** - Generates ED25519 key pair for current user
6. **Sets Permissions** - Configures proper SSH directory and file permissions
7. **Adds to Authorized Keys** - Automatically adds public key for passwordless access

**Process:**
```
SSH is not currently available on this system.
Would you like to set up SSH access? (y/N): y

[HEADER] SSH SETUP
Setup Status: Starting SSH setup process...
Installing: OpenSSH Server (Debian/Ubuntu)
Configuring: Generating SSH host keys
Starting: SSH service
Generating: SSH key pair for current user
Key Setup: SSH key added to authorized_keys
Setup Status: SSH setup completed successfully
```

### VS Code Web Setup (`setup_vscode_web`)

When VS Code Web is not available, the script offers automated installation:

**What it does:**
1. **Checks Installation** - Verifies if code-server is already installed
2. **Downloads & Installs** - Uses official code-server install script
3. **Creates Config** - Sets up configuration directory (~/.config/code-server)
4. **Starts Service** - Launches code-server on port 8080 with password auth
5. **Shows Access Info** - Displays connection URLs and passwords
6. **Network Detection** - Provides local, network, and remote access instructions

**Process:**
```
VS Code web access is not fully set up.
Would you like to set up VS Code web access now? (y/n): y

[HEADER] Setting up VS Code Web Access...
Installing code-server...
Running: curl -fsSL https://code-server.dev/install.sh | sh
code-server installed successfully!
Starting code-server...
code-server started successfully!

[HEADER] VS Code Web Access Instructions:
1. Local Access:
   URL: http://localhost:8080
   Password: [generated-password]

2. Network Access:
   URL: http://192.168.1.100:8080
   Password: [generated-password]

3. Remote Access (if firewall allows):
   URL: http://your-public-ip:8080
   Password: [generated-password]
```

## 🚀 Usage

### Python Version (Recommended)
```bash
chmod +x vm_system_check.py
python3 vm_system_check.py
```

### Shell Version (Lightweight)
```bash
chmod +x vm_system_check.sh
bash vm_system_check.sh
```

### Interactive Launcher
```bash
chmod +x run_system_check.sh
./run_system_check.sh
```

## 📊 Sample Output

```
[HEADER] VM SYSTEM INFORMATION CHECKER

[HEADER] SYSTEM INFORMATION
Operating System: Ubuntu 20.04.6 LTS
Hostname: nvidia-dli-lab | User: dli-user

[HEADER] HARDWARE
CPU: Intel Xeon E5-2686 v4 @ 2.30GHz (4 cores)
GPU: NVIDIA Tesla V100-SXM2-16GB (16GB VRAM)
Memory: 12.3 GB / 15.6 GB available (79%)
Disk: 45.2 GB / 100 GB used (45%)

[HEADER] PYTHON ENVIRONMENT
Python Version: 3.11.0
Environment: Conda environment: myenv
Installed Packages: 245 packages
Key Packages: numpy, pandas, tensorflow, torch, jupyter

[HEADER] ROS/ROS2 INSTALLATION
ROS 2 Version: ros2 doctor 0.10.4
ROS 2 Distro: humble
Running ROS 2 Nodes: 3 nodes

[HEADER] SYSTEM SERVICES & PORTS
Open Port: Port 8888 (Jupyter Notebook)
Open Port: Port 11434 (Ollama API server)
Running Service: ollama (Ollama AI service)
Docker Containers: 2 running

[HEADER] NETWORK
Local IP: 192.168.1.100
Public IP: 203.0.113.45
Location: San Francisco, CA, US
Internet: ✅ Connected

[HEADER] REMOTE ACCESS
SSH: ✅ Ready (port 22)
VS Code Web: ✅ Ready (http://192.168.1.100:8080)
```

## 🛠️ Requirements

### Python Version
- **Python 3.6+** (usually pre-installed on cloud VMs)
- **No additional packages required** (uses only standard library)
- **No sudo privileges needed**

### Shell Version
- **Bash shell** (available on all Linux systems)
- **Basic Unix tools** (`df`, `free`, `lspci`, etc.)
- **No sudo privileges needed**

## 🔧 Installation Options

### Option 1: Direct Download
```bash
# Download Python version
wget https://raw.githubusercontent.com/your-repo/vm_system_check.py
chmod +x vm_system_check.py

# Download Shell version
wget https://raw.githubusercontent.com/your-repo/vm_system_check.sh
chmod +x vm_system_check.sh
```

### Option 2: Clone Repository
```bash
git clone https://github.com/your-repo/omniverselab.git
cd omniverselab
chmod +x vm_system_check.py vm_system_check.sh
```

### Option 3: Copy and Paste
Simply copy the script content and save it to a file on your VM.

## 🎨 Features

### Color-Coded Output
- 🟢 **Green**: Success/Good status
- 🟡 **Yellow**: Warning/Attention needed
- 🔴 **Red**: Error/Problem detected
- 🔵 **Blue**: Information/Neutral

### Status Indicators
- **Memory Usage**: < 80% (Good), 80-90% (Warning), > 90% (Critical)
- **Disk Usage**: < 80% (Good), 80-90% (Warning), > 90% (Critical)
- **Service Status**: Running (Good), Not Running (Warning)

### Comprehensive Analysis
- **Hardware Detection**: Automatic CPU and GPU identification
- **Network Analysis**: Both local and public connectivity
- **Remote Access**: SSH and web-based IDE readiness
- **Performance Metrics**: Real-time system resource usage

## 🚨 Troubleshooting

### Common Issues

**"Permission denied" error:**
```bash
chmod +x vm_system_check.py
# or
chmod +x vm_system_check.sh
```

**Python not found:**
```bash
# Try different Python commands
python3 vm_system_check.py
# or
/usr/bin/python3 vm_system_check.py
```

**Missing tools in shell version:**
- The script will gracefully handle missing tools
- Use the Python version for more comprehensive checks

### Limited Output
If some checks show "Unable to determine" or "Not available":
- This is normal on restricted cloud environments
- The scripts work without sudo and respect security limitations
- Core functionality will still work

## 🎯 Use Cases

### Development Environment Setup
1. **Initial VM Assessment**: Run after launching a new cloud instance
2. **Remote Access Setup**: Verify SSH and web IDE configuration
3. **Resource Planning**: Check available CPU, GPU, and memory
4. **Network Troubleshooting**: Diagnose connectivity issues

### Educational Environments
1. **NVIDIA DLI Courses**: Verify GPU availability and drivers
2. **Machine Learning Workshops**: Check system resources
3. **Cloud Computing Labs**: Understand VM specifications

### Production Monitoring
1. **Health Checks**: Regular system status monitoring
2. **Capacity Planning**: Resource utilization tracking
3. **Access Verification**: Remote connectivity validation

## 📝 Output Interpretation

### GPU Status
- **"NVIDIA GPU(s) found"**: GPU is available and drivers are working
- **"nvidia-smi failed"**: GPU present but drivers may need attention
- **"No GPU detected"**: No GPU or GPU not accessible

### Network Status
- **Public IP detected**: VM is internet-accessible
- **"Unable to determine"**: May be behind NAT or firewall
- **Connectivity tests**: Verify internet access for downloads

### SSH Access
- **"SSH Daemon Running"**: Remote SSH access is possible
- **"Port 22 Open"**: SSH port is accessible
- **"SSH Keys Found"**: Key-based authentication is configured

### VS Code Web
- **"code-server Installed"**: Web-based VS Code is available
- **"Process Running"**: Web IDE is currently active
- **"Port X Open"**: Web interface is accessible



## 🎯 Perfect for Cloud Environments

- **NVIDIA DLI** - GPU detection and CUDA environment checking
- **AWS EC2, Google Cloud, Azure** - General cloud VM analysis
- **DigitalOcean, Linode** - VPS and cloud instance monitoring
- **Container Environments** - Docker, Kubernetes, and sandboxed systems

## 🔧 Technical Features

- **No Root Required** - All checks run with user privileges
- **Cross-Platform** - Linux, macOS, WSL support
- **Minimal Dependencies** - Uses standard Unix utilities
- **Safe & Secure** - No sudo commands, container-friendly
