#!/usr/bin/env python3
"""
Cloud VM System Information Checker

A comprehensive script to check VM environment details including:
- CPU information
- GPU models (if available)
- Memory and disk usage
- Network configuration and accessibility
- SSH and VS Code web access capabilities

Requirements: Python 3.6+ (no sudo required)
Usage: python3 vm_system_check.py
"""

import os
import sys
import subprocess
import socket
import platform
import json
import urllib.request
import urllib.error
from pathlib import Path
import time

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
    print(f"\n{Colors.HEADER}{Colors.BOLD}{'='*60}{Colors.ENDC}")
    print(f"{Colors.HEADER}{Colors.BOLD}{text.center(60)}{Colors.ENDC}")
    print(f"{Colors.HEADER}{Colors.BOLD}{'='*60}{Colors.ENDC}\n")

def print_info(label, value, status="info"):
    """Print formatted information"""
    color = Colors.OKGREEN if status == "success" else Colors.WARNING if status == "warning" else Colors.FAIL if status == "error" else Colors.OKCYAN
    print(f"{Colors.BOLD}{label}:{Colors.ENDC} {color}{value}{Colors.ENDC}")

def run_command(command, shell=False):
    """Run a command and return output"""
    try:
        if shell:
            result = subprocess.run(command, shell=True, capture_output=True, text=True, timeout=10)
        else:
            result = subprocess.run(command.split(), capture_output=True, text=True, timeout=10)
        return result.stdout.strip() if result.returncode == 0 else None
    except (subprocess.TimeoutExpired, subprocess.SubprocessError, FileNotFoundError):
        return None

def get_cpu_info():
    """Get CPU information"""
    print_header("CPU INFORMATION")
    
    # CPU model
    cpu_model = None
    if os.path.exists('/proc/cpuinfo'):
        try:
            with open('/proc/cpuinfo', 'r') as f:
                for line in f:
                    if 'model name' in line:
                        cpu_model = line.split(':')[1].strip()
                        break
        except:
            pass
    
    if not cpu_model:
        cpu_model = platform.processor() or "Unknown"
    
    print_info("CPU Model", cpu_model)
    
    # CPU cores
    cpu_count = os.cpu_count()
    print_info("CPU Cores", f"{cpu_count} cores")
    
    # CPU usage (if possible)
    try:
        load_avg = os.getloadavg()
        print_info("Load Average", f"{load_avg[0]:.2f}, {load_avg[1]:.2f}, {load_avg[2]:.2f} (1m, 5m, 15m)")
    except:
        print_info("Load Average", "Not available")

def get_gpu_info():
    """Get GPU information"""
    print_header("GPU INFORMATION")
    
    # Try nvidia-smi first
    nvidia_output = run_command("nvidia-smi --query-gpu=name,memory.total,driver_version --format=csv,noheader,nounits")
    if nvidia_output:
        print_info("GPU Detection", "NVIDIA GPU(s) found", "success")
        for i, line in enumerate(nvidia_output.split('\n')):
            if line.strip():
                parts = [part.strip() for part in line.split(',')]
                if len(parts) >= 3:
                    print_info(f"GPU {i+1}", f"{parts[0]} ({parts[1]} MB VRAM, Driver: {parts[2]})")
        return
    
    # Try lspci for other GPUs
    lspci_output = run_command("lspci | grep -i vga")
    if lspci_output:
        print_info("GPU Detection", "GPU(s) found via lspci", "warning")
        for line in lspci_output.split('\n'):
            if line.strip():
                print_info("GPU", line.split(': ')[1] if ': ' in line else line)
    else:
        print_info("GPU Detection", "No GPU detected or lspci not available", "error")

def get_memory_info():
    """Get memory information"""
    print_header("MEMORY INFORMATION")
    
    if os.path.exists('/proc/meminfo'):
        try:
            with open('/proc/meminfo', 'r') as f:
                meminfo = f.read()
            
            mem_total = None
            mem_available = None
            mem_free = None
            
            for line in meminfo.split('\n'):
                if 'MemTotal:' in line:
                    mem_total = int(line.split()[1]) * 1024  # Convert KB to bytes
                elif 'MemAvailable:' in line:
                    mem_available = int(line.split()[1]) * 1024
                elif 'MemFree:' in line:
                    mem_free = int(line.split()[1]) * 1024
            
            if mem_total:
                print_info("Total RAM", f"{mem_total / (1024**3):.2f} GB")
            if mem_available:
                print_info("Available RAM", f"{mem_available / (1024**3):.2f} GB")
                usage_percent = ((mem_total - mem_available) / mem_total) * 100 if mem_total else 0
                status = "success" if usage_percent < 80 else "warning" if usage_percent < 90 else "error"
                print_info("Memory Usage", f"{usage_percent:.1f}%", status)
            elif mem_free:
                print_info("Free RAM", f"{mem_free / (1024**3):.2f} GB")
                
        except Exception as e:
            print_info("Memory Info", f"Error reading /proc/meminfo: {e}", "error")
    else:
        print_info("Memory Info", "Memory information not available", "error")

def get_disk_info():
    """Get disk information"""
    print_header("DISK INFORMATION")
    
    try:
        import shutil
        
        # Get disk usage for current directory and root
        paths_to_check = ['/', os.getcwd()]
        
        for path in paths_to_check:
            try:
                total, used, free = shutil.disk_usage(path)
                usage_percent = (used / total) * 100
                status = "success" if usage_percent < 80 else "warning" if usage_percent < 90 else "error"
                
                print_info(f"Disk Usage ({path})", 
                          f"{used / (1024**3):.2f} GB / {total / (1024**3):.2f} GB ({usage_percent:.1f}%)", 
                          status)
            except:
                continue
                
    except ImportError:
        print_info("Disk Info", "shutil module not available", "error")
    
    # Try df command as fallback
    df_output = run_command("df -h /")
    if df_output:
        lines = df_output.split('\n')
        if len(lines) > 1:
            parts = lines[1].split()
            if len(parts) >= 5:
                print_info("Root Filesystem (df)", f"{parts[2]} used / {parts[1]} total ({parts[4]} used)")

def get_network_info():
    """Get network and connectivity information"""
    print_header("NETWORK INFORMATION")
    
    network_data = {}
    
    # Get hostname
    hostname = socket.gethostname()
    print_info("Hostname", hostname)
    network_data['hostname'] = hostname
    
    # Get local IP addresses
    local_ip = None
    try:
        # Get primary IP (the one used to connect to internet)
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        local_ip = s.getsockname()[0]
        s.close()
        print_info("Local IP", local_ip)
    except:
        print_info("Local IP", "Unable to determine", "error")
    
    network_data['local_ip'] = local_ip
    
    # Try to get public IP
    public_ip = None
    ip_services = [
        "https://api.ipify.org",
        "https://ipinfo.io/ip",
        "https://icanhazip.com"
    ]
    
    for service in ip_services:
        try:
            with urllib.request.urlopen(service, timeout=5) as response:
                public_ip = response.read().decode().strip()
                break
        except:
            continue
    
    network_data['public_ip'] = public_ip
    
    if public_ip:
        print_info("Public IP", public_ip, "success")
        
        # Check if it's globally accessible (basic check)
        try:
            with urllib.request.urlopen(f"https://ipinfo.io/{public_ip}/json", timeout=5) as response:
                ip_info = json.loads(response.read().decode())
                print_info("IP Location", f"{ip_info.get('city', 'Unknown')}, {ip_info.get('country', 'Unknown')}")
                print_info("ISP", ip_info.get('org', 'Unknown'))
                network_data['location'] = f"{ip_info.get('city', 'Unknown')}, {ip_info.get('country', 'Unknown')}"
                network_data['isp'] = ip_info.get('org', 'Unknown')
        except:
            pass
    else:
        print_info("Public IP", "Unable to determine", "error")
    
    # Test internet connectivity
    test_sites = ["8.8.8.8", "1.1.1.1", "google.com"]
    connectivity_ok = False
    
    for site in test_sites:
        try:
            if site.replace('.', '').isdigit():  # IP address
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(3)
                result = sock.connect_ex((site, 53))
                sock.close()
                if result == 0:
                    connectivity_ok = True
                    break
            else:  # Domain name
                socket.gethostbyname(site)
                connectivity_ok = True
                break
        except:
            continue
    
    network_data['connectivity'] = connectivity_ok
    
    print_info("Internet Connectivity", "Available" if connectivity_ok else "Limited or unavailable", 
              "success" if connectivity_ok else "error")
    
    return network_data

def setup_ssh_access():
    """Setup SSH access on the system"""
    print_header("SSH SETUP")
    
    try:
        # Check if we're on a supported system
        system = platform.system().lower()
        if system not in ['linux', 'darwin']:
            print_info("Setup Status", "SSH setup not supported on this system", "error")
            return False
        
        print_info("Setup Status", "Starting SSH setup process...", "info")
        
        # Install SSH server if not present
        if system == 'linux':
            # Try to install openssh-server
            distro_info = run_command("cat /etc/os-release")
            if distro_info and ('ubuntu' in distro_info.lower() or 'debian' in distro_info.lower()):
                print_info("Installing", "OpenSSH Server (Debian/Ubuntu)", "info")
                result = run_command("sudo apt update && sudo apt install -y openssh-server")
                if not result:
                    print_info("Installation", "Failed - may need manual installation", "error")
                    return False
            elif distro_info and ('centos' in distro_info.lower() or 'rhel' in distro_info.lower() or 'fedora' in distro_info.lower()):
                print_info("Installing", "OpenSSH Server (RHEL/CentOS/Fedora)", "info")
                result = run_command("sudo yum install -y openssh-server || sudo dnf install -y openssh-server")
                if not result:
                    print_info("Installation", "Failed - may need manual installation", "error")
                    return False
        
        # Generate SSH host keys if they don't exist
        print_info("Configuring", "Generating SSH host keys", "info")
        run_command("sudo ssh-keygen -A")
        
        # Start and enable SSH service
        print_info("Starting", "SSH service", "info")
        if system == 'linux':
            run_command("sudo systemctl start ssh || sudo systemctl start sshd")
            run_command("sudo systemctl enable ssh || sudo systemctl enable sshd")
        elif system == 'darwin':
            run_command("sudo launchctl load -w /System/Library/LaunchDaemons/ssh.plist")
        
        # Create SSH directory for current user
        ssh_dir = Path.home() / '.ssh'
        ssh_dir.mkdir(mode=0o700, exist_ok=True)
        
        # Generate SSH key pair if none exists
        key_files = ['id_rsa', 'id_ed25519', 'id_ecdsa']
        has_key = any((ssh_dir / f"{key}.pub").exists() for key in key_files)
        
        if not has_key:
            print_info("Generating", "SSH key pair for current user", "info")
            key_path = ssh_dir / 'id_ed25519'
            run_command(f"ssh-keygen -t ed25519 -f {key_path} -N ''")
            
            # Add public key to authorized_keys
            pub_key_path = ssh_dir / 'id_ed25519.pub'
            auth_keys_path = ssh_dir / 'authorized_keys'
            
            if pub_key_path.exists():
                with open(pub_key_path, 'r') as pub_file:
                    pub_key = pub_file.read().strip()
                
                with open(auth_keys_path, 'a') as auth_file:
                    auth_file.write(f"{pub_key}\n")
                
                auth_keys_path.chmod(0o600)
                print_info("Key Setup", "SSH key added to authorized_keys", "success")
        
        print_info("Setup Status", "SSH setup completed successfully", "success")
        return True
        
    except Exception as e:
        print_info("Setup Error", f"Failed to setup SSH: {e}", "error")
        return False

def show_ssh_instructions(public_ip, local_ip, username):
    """Show SSH connection instructions"""
    print(f"\n{Colors.BOLD}{Colors.OKGREEN}SSH CONNECTION INSTRUCTIONS:{Colors.ENDC}")
    print(f"{Colors.OKCYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━{Colors.ENDC}")
    
    if public_ip and public_ip != "Unable to determine":
        print(f"{Colors.BOLD}External SSH Access:{Colors.ENDC}")
        print(f"  ssh {username}@{public_ip}")
        print(f"  ssh -i ~/.ssh/id_ed25519 {username}@{public_ip}")
        print()
    
    if local_ip and local_ip != "Unable to determine":
        print(f"{Colors.BOLD}Local Network SSH Access:{Colors.ENDC}")
        print(f"  ssh {username}@{local_ip}")
        print(f"  ssh -i ~/.ssh/id_ed25519 {username}@{local_ip}")
        print()
    
    print(f"{Colors.BOLD}SSH Key-based Authentication:{Colors.ENDC}")
    print(f"  1. Copy your public key to the server:")
    print(f"     ssh-copy-id {username}@{public_ip or local_ip}")
    print(f"  2. Or manually add your public key to ~/.ssh/authorized_keys")
    print()
    
    print(f"{Colors.BOLD}SSH Tunneling Examples:{Colors.ENDC}")
    print(f"  # Forward local port 8080 to remote port 8080")
    print(f"  ssh -L 8080:localhost:8080 {username}@{public_ip or local_ip}")
    print(f"  # Forward local port 8443 to remote port 8443 (VS Code)")
    print(f"  ssh -L 8443:localhost:8443 {username}@{public_ip or local_ip}")
    print()
    
    print(f"{Colors.WARNING}Security Tips:{Colors.ENDC}")
    print(f"  • Use key-based authentication instead of passwords")
    print(f"  • Consider changing the default SSH port (22)")
    print(f"  • Use SSH tunneling for secure access to web services")
    print(f"{Colors.OKCYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━{Colors.ENDC}")

def check_ssh_access():
    """Check SSH accessibility"""
    print_header("SSH ACCESS CHECK")
    
    ssh_available = False
    ssh_port_open = False
    
    # Check if SSH daemon is running
    ssh_processes = run_command("pgrep -f sshd")
    if ssh_processes:
        print_info("SSH Daemon", "Running", "success")
        ssh_available = True
        
        # Check SSH port
        ssh_port = 22
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(1)
            result = sock.connect_ex(('localhost', ssh_port))
            sock.close()
            if result == 0:
                print_info(f"SSH Port {ssh_port}", "Open", "success")
                ssh_port_open = True
            else:
                print_info(f"SSH Port {ssh_port}", "Closed or filtered", "warning")
        except:
            print_info(f"SSH Port {ssh_port}", "Unable to check", "error")
            
    else:
        print_info("SSH Daemon", "Not running or not accessible", "error")
    
    # Check SSH configuration (if readable)
    ssh_config_paths = ['/etc/ssh/sshd_config', '/etc/ssh/ssh_config']
    for config_path in ssh_config_paths:
        if os.path.exists(config_path) and os.access(config_path, os.R_OK):
            try:
                with open(config_path, 'r') as f:
                    config_content = f.read()
                    if 'PasswordAuthentication' in config_content:
                        print_info(f"SSH Config ({config_path})", "Readable - check PasswordAuthentication settings")
                    break
            except:
                continue
    
    # Check for SSH keys
    ssh_dir = Path.home() / '.ssh'
    if ssh_dir.exists():
        key_files = ['id_rsa.pub', 'id_ed25519.pub', 'id_ecdsa.pub', 'authorized_keys']
        found_keys = []
        for key_file in key_files:
            if (ssh_dir / key_file).exists():
                found_keys.append(key_file)
        
        if found_keys:
            print_info("SSH Keys Found", ", ".join(found_keys), "success")
        else:
            print_info("SSH Keys", "No public keys or authorized_keys found", "warning")
    else:
        print_info("SSH Directory", "~/.ssh directory not found", "warning")
    
    # Show instructions or offer setup based on SSH availability
    if ssh_available and ssh_port_open:
        # Get network info for instructions
        try:
            # Get public IP
            public_ip = "Unable to determine"
            try:
                with urllib.request.urlopen('https://api.ipify.org?format=json', timeout=5) as response:
                    data = json.loads(response.read().decode())
                    public_ip = data.get('ip', 'Unable to determine')
            except:
                pass
            
            # Get local IP
            local_ip = "Unable to determine"
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.connect(("8.8.8.8", 80))
                local_ip = sock.getsockname()[0]
                sock.close()
            except:
                pass
            
            # Get current username
            username = os.getenv('USER') or os.getenv('USERNAME') or 'user'
            
            show_ssh_instructions(public_ip, local_ip, username)
            
        except Exception as e:
            print_info("Instructions", f"Unable to show connection details: {e}", "warning")
    
    elif not ssh_available:
        print(f"\n{Colors.WARNING}SSH is not currently available on this system.{Colors.ENDC}")
        try:
            response = input(f"{Colors.BOLD}Would you like to set up SSH access? (y/N): {Colors.ENDC}").strip().lower()
            if response in ['y', 'yes']:
                success = setup_ssh_access()
                if success:
                    print(f"\n{Colors.OKGREEN}SSH setup completed! You can now use SSH to connect to this system.{Colors.ENDC}")
                    # Re-run the check to show instructions
                    time.sleep(2)
                    check_ssh_access()
                else:
                    print(f"\n{Colors.FAIL}SSH setup failed. You may need to install SSH manually.{Colors.ENDC}")
            else:
                print(f"{Colors.OKCYAN}SSH setup skipped.{Colors.ENDC}")
        except (EOFError, KeyboardInterrupt):
            print(f"\n{Colors.OKCYAN}SSH setup skipped.{Colors.ENDC}")

def setup_vscode_web():
    """Setup VS Code web access (code-server)"""
    print(f"\n{Colors.HEADER}Setting up VS Code Web Access...{Colors.ENDC}")
    
    try:
        # Check if code-server is already installed
        code_server = run_command("which code-server")
        if code_server:
            print(f"{Colors.OKGREEN}code-server is already installed at {code_server}{Colors.ENDC}")
        else:
            print(f"{Colors.OKCYAN}Installing code-server...{Colors.ENDC}")
            
            # Install code-server using the official install script
            install_cmd = "curl -fsSL https://code-server.dev/install.sh | sh"
            print(f"Running: {install_cmd}")
            
            result = subprocess.run(install_cmd, shell=True, capture_output=True, text=True)
            
            if result.returncode == 0:
                print(f"{Colors.OKGREEN}code-server installed successfully!{Colors.ENDC}")
            else:
                print(f"{Colors.FAIL}Failed to install code-server: {result.stderr}{Colors.ENDC}")
                return False
        
        # Check if code-server is running
        code_server_proc = run_command("pgrep -f code-server")
        if not code_server_proc:
            print(f"{Colors.OKCYAN}Starting code-server...{Colors.ENDC}")
            
            # Create config directory if it doesn't exist
            config_dir = Path.home() / ".config" / "code-server"
            config_dir.mkdir(parents=True, exist_ok=True)
            
            # Start code-server in background
            start_cmd = "nohup code-server --bind-addr 0.0.0.0:8080 --auth password > ~/.config/code-server/code-server.log 2>&1 &"
            subprocess.run(start_cmd, shell=True)
            
            # Wait a moment for it to start
            time.sleep(3)
            
            # Check if it's running now
            code_server_proc = run_command("pgrep -f code-server")
            if code_server_proc:
                print(f"{Colors.OKGREEN}code-server started successfully!{Colors.ENDC}")
            else:
                print(f"{Colors.FAIL}Failed to start code-server{Colors.ENDC}")
                return False
        else:
            print(f"{Colors.OKGREEN}code-server is already running{Colors.ENDC}")
        
        # Show access instructions
        show_vscode_instructions()
        return True
        
    except Exception as e:
        print(f"{Colors.FAIL}Error setting up VS Code web access: {str(e)}{Colors.ENDC}")
        return False

def show_vscode_instructions():
    """Show VS Code web access instructions"""
    print(f"\n{Colors.HEADER}VS Code Web Access Instructions:{Colors.ENDC}")
    
    # Get network info for instructions
    network_info = get_network_info()
    public_ip = network_info.get('public_ip', 'YOUR_PUBLIC_IP')
    local_ip = network_info.get('local_ip', 'YOUR_LOCAL_IP')
    
    # Get password from config
    config_file = Path.home() / ".config" / "code-server" / "config.yaml"
    password = "check_config_file"
    
    if config_file.exists():
        try:
            with open(config_file, 'r') as f:
                content = f.read()
                for line in content.split('\n'):
                    if 'password:' in line:
                        password = line.split('password:')[1].strip()
                        break
        except:
            pass
    
    print(f"{Colors.OKGREEN}1. Local Access:{Colors.ENDC}")
    print(f"   URL: http://localhost:8080")
    print(f"   Password: {password}")
    
    if local_ip and local_ip != '127.0.0.1':
        print(f"\n{Colors.OKGREEN}2. Network Access:{Colors.ENDC}")
        print(f"   URL: http://{local_ip}:8080")
        print(f"   Password: {password}")
    
    if public_ip and public_ip != local_ip:
        print(f"\n{Colors.OKGREEN}3. Remote Access (if firewall allows):{Colors.ENDC}")
        print(f"   URL: http://{public_ip}:8080")
        print(f"   Password: {password}")
        print(f"   {Colors.WARNING}Note: Make sure port 8080 is open in your firewall/security groups{Colors.ENDC}")
    
    print(f"\n{Colors.OKCYAN}Configuration file: {config_file}{Colors.ENDC}")
    print(f"{Colors.OKCYAN}Log file: ~/.config/code-server/code-server.log{Colors.ENDC}")

def check_vscode_web():
    """Check VS Code web accessibility"""
    print_header("VS CODE WEB ACCESS CHECK")
    
    vscode_available = False
    
    # Check for code-server installation
    code_server = run_command("which code-server")
    if code_server:
        print_info("code-server", f"Installed at {code_server}", "success")
        vscode_available = True
        
        # Try to get version
        version = run_command("code-server --version")
        if version:
            print_info("code-server Version", version.split('\n')[0], "success")
    else:
        print_info("code-server", "Not installed", "warning")
    
    # Check for running code-server processes
    code_server_proc = run_command("pgrep -f code-server")
    if code_server_proc:
        print_info("code-server Process", "Running", "success")
        vscode_available = True
        
        # Try to find the port it's running on
        netstat_output = run_command("netstat -tlnp 2>/dev/null | grep code-server")
        if netstat_output:
            for line in netstat_output.split('\n'):
                if ':' in line:
                    port = line.split(':')[1].split()[0]
                    print_info("code-server Port", port, "success")
                    break
        
        # Show access instructions if running
        show_vscode_instructions()
    else:
        print_info("code-server Process", "Not running", "warning")
    
    # Check common VS Code web ports
    common_ports = [8080, 8443, 3000, 8000]
    for port in common_ports:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(1)
            result = sock.connect_ex(('localhost', port))
            sock.close()
            if result == 0:
                print_info(f"Port {port}", "Open (potential web service)", "success")
        except:
            continue
    
    # Check for other web-based IDEs
    jupyter_proc = run_command("pgrep -f jupyter")
    if jupyter_proc:
        print_info("Jupyter", "Process running", "success")
        vscode_available = True
    
    theia_proc = run_command("pgrep -f theia")
    if theia_proc:
        print_info("Theia IDE", "Process running", "success")
        vscode_available = True
    
    # Offer to set up VS Code web access if not available
    if not vscode_available or not code_server_proc:
        print(f"\n{Colors.WARNING}VS Code web access is not fully set up.{Colors.ENDC}")
        try:
            setup_choice = input(f"{Colors.OKCYAN}Would you like to set up VS Code web access now? (y/n): {Colors.ENDC}").strip().lower()
            if setup_choice in ['y', 'yes']:
                success = setup_vscode_web()
                if success:
                    print(f"\n{Colors.OKGREEN}VS Code web setup completed! You can now access VS Code through your browser.{Colors.ENDC}")
                    # Re-run the check to show updated status
                    time.sleep(2)
                    check_vscode_web()
                else:
                    print(f"\n{Colors.FAIL}VS Code web setup failed. You may need to install it manually.{Colors.ENDC}")
            else:
                print(f"{Colors.OKCYAN}VS Code web setup skipped.{Colors.ENDC}")
        except (EOFError, KeyboardInterrupt):
            print(f"\n{Colors.OKCYAN}VS Code web setup skipped.{Colors.ENDC}")

def check_python_environment():
    """Check Python environment and packages"""
    print_header("PYTHON ENVIRONMENT")
    
    # Python version
    python_version = f"{sys.version_info.major}.{sys.version_info.minor}.{sys.version_info.micro}"
    print_info("Python Version", python_version)
    print_info("Python Executable", sys.executable)
    
    # Virtual environment detection
    venv_info = []
    if hasattr(sys, 'real_prefix') or (hasattr(sys, 'base_prefix') and sys.base_prefix != sys.prefix):
        venv_info.append("Active virtual environment detected")
        if 'VIRTUAL_ENV' in os.environ:
            venv_info.append(f"VIRTUAL_ENV: {os.environ['VIRTUAL_ENV']}")
    
    # Conda environment detection
    if 'CONDA_DEFAULT_ENV' in os.environ:
        venv_info.append(f"Conda environment: {os.environ['CONDA_DEFAULT_ENV']}")
    
    if venv_info:
        for info in venv_info:
            print_info("Environment", info, "success")
    else:
        print_info("Environment", "System Python (no virtual environment)")
    
    # Check for conda installation
    conda_result = run_command("conda --version")
    if conda_result:
        print_info("Conda", conda_result.strip(), "success")
        
        # List conda environments
        conda_envs = run_command("conda env list")
        if conda_envs:
            env_lines = [line.strip() for line in conda_envs.split('\n') if line.strip() and not line.startswith('#')]
            if len(env_lines) > 1:  # More than just base
                print_info("Conda Environments", f"{len(env_lines)} environments found")
    else:
        print_info("Conda", "Not installed")
    
    # Check pip and installed packages
    try:
        import pkg_resources
        installed_packages = [d.project_name for d in pkg_resources.working_set]
        print_info("Installed Packages", f"{len(installed_packages)} packages")
        
        # Check for common data science/ML packages
        important_packages = ['numpy', 'pandas', 'matplotlib', 'scipy', 'scikit-learn', 
                            'tensorflow', 'torch', 'jupyter', 'notebook', 'fastapi', 
                            'flask', 'django', 'requests', 'opencv-python']
        found_packages = [pkg for pkg in important_packages if pkg in installed_packages]
        if found_packages:
            print_info("Key Packages", ", ".join(found_packages[:10]), "success")
    except ImportError:
        print_info("Package Info", "pkg_resources not available")
    
    # Check pip version
    pip_result = run_command(f"{sys.executable} -m pip --version")
    if pip_result:
        print_info("Pip Version", pip_result.strip())

def check_ros_installation():
    """Check ROS/ROS2 installation"""
    print_header("ROS/ROS2 INSTALLATION")
    
    # Check ROS 1 (Melodic, Noetic, etc.)
    ros1_found = False
    if os.path.exists('/opt/ros'):
        ros_dirs = [d for d in os.listdir('/opt/ros') if os.path.isdir(f'/opt/ros/{d}')]
        if ros_dirs:
            print_info("ROS 1 Distributions", ", ".join(ros_dirs), "success")
            ros1_found = True
    
    # Check ROS 1 environment
    if 'ROS_DISTRO' in os.environ:
        print_info("Active ROS 1", os.environ['ROS_DISTRO'], "success")
        ros1_found = True
        if 'ROS_MASTER_URI' in os.environ:
            print_info("ROS Master URI", os.environ['ROS_MASTER_URI'])
    
    if not ros1_found:
        print_info("ROS 1", "Not installed")
    
    # Check ROS 2
    ros2_found = False
    ros2_result = run_command("ros2 --version")
    if ros2_result:
        print_info("ROS 2 Version", ros2_result.strip(), "success")
        ros2_found = True
        
        # Check ROS 2 distro
        if 'ROS_DISTRO' in os.environ:
            print_info("ROS 2 Distro", os.environ['ROS_DISTRO'], "success")
        
        # List ROS 2 nodes if any are running
        nodes_result = run_command("ros2 node list")
        if nodes_result and nodes_result.strip():
            node_count = len([n for n in nodes_result.strip().split('\n') if n.strip()])
            print_info("Running ROS 2 Nodes", f"{node_count} nodes", "success")
    
    if not ros2_found:
        print_info("ROS 2", "Not installed")
    
    # Check for common ROS tools
    tools = ['catkin_make', 'colcon', 'rosdep', 'roscore', 'rostopic', 'rosnode']
    found_tools = []
    for tool in tools:
        if run_command(f"which {tool}"):
            found_tools.append(tool)
    
    if found_tools:
        print_info("ROS Tools", ", ".join(found_tools), "success")

def check_system_services():
    """Check system services and ports"""
    print_header("SYSTEM SERVICES & PORTS")
    
    # Check common development ports
    common_ports = {
        3000: "React/Node.js dev server",
        8000: "Django/Python dev server", 
        8080: "VS Code Server/Tomcat",
        8443: "HTTPS services",
        8888: "Jupyter Notebook",
        9000: "Various services",
        5000: "Flask dev server",
        4000: "Jekyll/Ruby dev server",
        3001: "Next.js dev server",
        11434: "Ollama API server",
        7860: "Gradio applications",
        6006: "TensorBoard",
        8501: "Streamlit applications"
    }
    
    open_ports = []
    for port, description in common_ports.items():
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(1)
        result = sock.connect_ex(('localhost', port))
        sock.close()
        if result == 0:
            open_ports.append(f"Port {port} ({description})")
    
    if open_ports:
        for port_info in open_ports:
            print_info("Open Port", port_info, "success")
    else:
        print_info("Development Ports", "No common dev ports open")
    
    # Check for specific services
    services_to_check = {
        'ollama': 'Ollama AI service',
        'docker': 'Docker daemon',
        'nginx': 'Nginx web server',
        'apache2': 'Apache web server',
        'mysql': 'MySQL database',
        'postgresql': 'PostgreSQL database',
        'redis-server': 'Redis cache',
        'mongodb': 'MongoDB database',
        'jupyter': 'Jupyter services'
    }
    
    running_services = []
    for service, description in services_to_check.items():
        # Check if process is running
        ps_result = run_command(f"pgrep -f {service}")
        if ps_result and ps_result.strip():
            running_services.append(f"{service} ({description})")
    
    if running_services:
        for service_info in running_services:
            print_info("Running Service", service_info, "success")
    else:
        print_info("Services", "No common services detected")
    
    # Check systemd services (Linux only)
    if platform.system() == "Linux":
        systemd_result = run_command("systemctl list-units --type=service --state=active --no-pager --no-legend")
        if systemd_result:
            active_services = len([line for line in systemd_result.split('\n') if line.strip()])
            print_info("Active Systemd Services", f"{active_services} services")
    
    # Check Docker containers if Docker is available
    docker_result = run_command("docker ps --format 'table {{.Names}}\t{{.Status}}'")
    if docker_result and 'NAMES' in docker_result:
        container_lines = [line for line in docker_result.split('\n')[1:] if line.strip()]
        if container_lines:
            print_info("Docker Containers", f"{len(container_lines)} running", "success")
        else:
            print_info("Docker Containers", "None running")
    else:
        print_info("Docker", "Not available or no containers")

def get_system_info():
    """Get general system information"""
    print_header("SYSTEM INFORMATION")
    
    print_info("Operating System", f"{platform.system()} {platform.release()}")
    print_info("Architecture", platform.machine())
    print_info("Python Version", platform.python_version())
    
    # Uptime
    uptime_output = run_command("uptime")
    if uptime_output:
        print_info("System Uptime", uptime_output)
    
    # Current user
    current_user = os.getenv('USER') or os.getenv('USERNAME') or 'unknown'
    print_info("Current User", current_user)
    
    # Check if running in container
    if os.path.exists('/.dockerenv'):
        print_info("Container", "Running in Docker container", "info")
    elif os.path.exists('/proc/1/cgroup'):
        try:
            with open('/proc/1/cgroup', 'r') as f:
                cgroup_content = f.read()
                if 'docker' in cgroup_content or 'containerd' in cgroup_content:
                    print_info("Container", "Likely running in container", "info")
        except:
            pass

def main():
    """Main function to run all checks"""
    print(f"{Colors.BOLD}{Colors.HEADER}")
    print("  ______ _                 _   ____   ____  ")
    print(" / _____|_)               | | |  _ \ |  _ \ ")
    print("| /     _  ___  _   _  ___| | | | | || | | |")
    print("| |    | |/ _ \| | | |/ _ \ | | | | || | | |")
    print("| \____| | (_) | |_| |  __/ | | |_| || |_| |")
    print(" \_____)_|\___/ \__,_|\___|_|  \___/ |____/ ")
    print("                                           ")
    print("        VM System Information Checker      ")
    print(f"{Colors.ENDC}")
    
    start_time = time.time()
    
    try:
        get_system_info()
        get_cpu_info()
        get_gpu_info()
        get_memory_info()
        get_disk_info()
        get_network_info()
        check_python_environment()
        check_ros_installation()
        check_system_services()
        check_ssh_access()
        check_vscode_web()
        
        end_time = time.time()
        
        print_header("SUMMARY")
        print_info("Check Duration", f"{end_time - start_time:.2f} seconds")
        print_info("Status", "System check completed successfully", "success")
        
        print(f"\n{Colors.BOLD}{Colors.OKCYAN}Tip: Save this output for reference when setting up remote access!{Colors.ENDC}")
        
    except KeyboardInterrupt:
        print(f"\n{Colors.WARNING}Check interrupted by user{Colors.ENDC}")
        sys.exit(1)
    except Exception as e:
        print(f"\n{Colors.FAIL}An error occurred: {e}{Colors.ENDC}")
        sys.exit(1)

if __name__ == "__main__":
    main()