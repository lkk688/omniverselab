#!/bin/bash

# Cloud VM System Information Checker (Shell Script Version)
# A lightweight alternative to the Python script
# Requirements: Basic Unix tools (no sudo required)
# Usage: bash vm_system_check.sh

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m' # No Color

# Function to print headers
print_header() {
    echo -e "\n${PURPLE}${BOLD}============================================================${NC}"
    echo -e "${PURPLE}${BOLD}$(printf '%*s' $(((60+${#1})/2)) "$1")${NC}"
    echo -e "${PURPLE}${BOLD}============================================================${NC}\n"
}

# Function to print info
print_info() {
    local label="$1"
    local value="$2"
    local status="${3:-info}"
    
    case $status in
        "success") color="$GREEN" ;;
        "warning") color="$YELLOW" ;;
        "error") color="$RED" ;;
        *) color="$CYAN" ;;
    esac
    
    echo -e "${BOLD}${label}:${NC} ${color}${value}${NC}"
}

# Function to check if command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# System Information
check_system_info() {
    print_header "SYSTEM INFORMATION"
    
    print_info "Operating System" "$(uname -s) $(uname -r)"
    print_info "Architecture" "$(uname -m)"
    print_info "Hostname" "$(hostname)"
    print_info "Current User" "$(whoami)"
    
    if [ -f /etc/os-release ]; then
        . /etc/os-release
        print_info "Distribution" "$PRETTY_NAME"
    fi
    
    if command_exists uptime; then
        print_info "System Uptime" "$(uptime | sed 's/.*up \([^,]*\).*/\1/')"
    fi
    
    # Check if running in container
    if [ -f /.dockerenv ]; then
        print_info "Container" "Running in Docker container" "info"
    elif [ -f /proc/1/cgroup ] && grep -q "docker\|containerd" /proc/1/cgroup 2>/dev/null; then
        print_info "Container" "Likely running in container" "info"
    fi
}

# CPU Information
check_cpu_info() {
    print_header "CPU INFORMATION"
    
    if [ -f /proc/cpuinfo ]; then
        cpu_model=$(grep "model name" /proc/cpuinfo | head -1 | cut -d: -f2 | sed 's/^ *//')
        cpu_cores=$(grep -c "^processor" /proc/cpuinfo)
        
        print_info "CPU Model" "${cpu_model:-Unknown}"
        print_info "CPU Cores" "$cpu_cores cores"
        
        if [ -f /proc/loadavg ]; then
            load_avg=$(cat /proc/loadavg | cut -d' ' -f1-3)
            print_info "Load Average" "$load_avg (1m, 5m, 15m)"
        fi
    else
        print_info "CPU Info" "Unable to read /proc/cpuinfo" "error"
    fi
}

# GPU Information
check_gpu_info() {
    print_header "GPU INFORMATION"
    
    # Check for NVIDIA GPU
    if command_exists nvidia-smi; then
        gpu_info=$(nvidia-smi --query-gpu=name,memory.total,driver_version --format=csv,noheader,nounits 2>/dev/null)
        if [ $? -eq 0 ] && [ -n "$gpu_info" ]; then
            print_info "GPU Detection" "NVIDIA GPU(s) found" "success"
            echo "$gpu_info" | while IFS=, read -r name memory driver; do
                print_info "GPU" "$(echo $name | sed 's/^ *//') ($(echo $memory | sed 's/^ *//') MB VRAM, Driver: $(echo $driver | sed 's/^ *//'))"
            done
        else
            print_info "NVIDIA GPU" "nvidia-smi failed or no GPU found" "warning"
        fi
    else
        print_info "nvidia-smi" "Not available" "warning"
    fi
    
    # Check for other GPUs using lspci
    if command_exists lspci; then
        gpu_lspci=$(lspci | grep -i "vga\|3d\|display")
        if [ -n "$gpu_lspci" ]; then
            print_info "GPU Detection (lspci)" "GPU(s) found" "success"
            echo "$gpu_lspci" | while read -r line; do
                gpu_name=$(echo "$line" | cut -d: -f3- | sed 's/^ *//')
                print_info "GPU" "$gpu_name"
            done
        fi
    else
        print_info "lspci" "Not available" "warning"
    fi
}

# Memory Information
check_memory_info() {
    print_header "MEMORY INFORMATION"
    
    if [ -f /proc/meminfo ]; then
        mem_total=$(grep "MemTotal:" /proc/meminfo | awk '{print $2}')
        mem_available=$(grep "MemAvailable:" /proc/meminfo | awk '{print $2}')
        mem_free=$(grep "MemFree:" /proc/meminfo | awk '{print $2}')
        
        if [ -n "$mem_total" ]; then
            mem_total_gb=$(echo "scale=2; $mem_total / 1024 / 1024" | bc 2>/dev/null || echo "$(($mem_total / 1024 / 1024))")
            print_info "Total RAM" "${mem_total_gb} GB"
        fi
        
        if [ -n "$mem_available" ]; then
            mem_available_gb=$(echo "scale=2; $mem_available / 1024 / 1024" | bc 2>/dev/null || echo "$(($mem_available / 1024 / 1024))")
            print_info "Available RAM" "${mem_available_gb} GB"
            
            if [ -n "$mem_total" ] && [ "$mem_total" -gt 0 ]; then
                usage_percent=$(echo "scale=1; ($mem_total - $mem_available) * 100 / $mem_total" | bc 2>/dev/null || echo "0")
                if [ "${usage_percent%.*}" -lt 80 ]; then
                    status="success"
                elif [ "${usage_percent%.*}" -lt 90 ]; then
                    status="warning"
                else
                    status="error"
                fi
                print_info "Memory Usage" "${usage_percent}%" "$status"
            fi
        elif [ -n "$mem_free" ]; then
            mem_free_gb=$(echo "scale=2; $mem_free / 1024 / 1024" | bc 2>/dev/null || echo "$(($mem_free / 1024 / 1024))")
            print_info "Free RAM" "${mem_free_gb} GB"
        fi
    else
        print_info "Memory Info" "Unable to read /proc/meminfo" "error"
    fi
}

# Disk Information
check_disk_info() {
    print_header "DISK INFORMATION"
    
    if command_exists df; then
        # Check root filesystem
        root_usage=$(df -h / 2>/dev/null | tail -1)
        if [ -n "$root_usage" ]; then
            filesystem=$(echo "$root_usage" | awk '{print $1}')
            size=$(echo "$root_usage" | awk '{print $2}')
            used=$(echo "$root_usage" | awk '{print $3}')
            available=$(echo "$root_usage" | awk '{print $4}')
            use_percent=$(echo "$root_usage" | awk '{print $5}' | tr -d '%')
            
            if [ "$use_percent" -lt 80 ]; then
                status="success"
            elif [ "$use_percent" -lt 90 ]; then
                status="warning"
            else
                status="error"
            fi
            
            print_info "Root Filesystem" "$used used / $size total (${use_percent}% used)" "$status"
        fi
        
        # Check current directory if different from root
        current_usage=$(df -h . 2>/dev/null | tail -1)
        current_mount=$(echo "$current_usage" | awk '{print $6}')
        if [ "$current_mount" != "/" ] && [ -n "$current_usage" ]; then
            used=$(echo "$current_usage" | awk '{print $3}')
            size=$(echo "$current_usage" | awk '{print $2}')
            use_percent=$(echo "$current_usage" | awk '{print $5}' | tr -d '%')
            print_info "Current Directory ($current_mount)" "$used used / $size total (${use_percent}% used)"
        fi
    else
        print_info "Disk Info" "df command not available" "error"
    fi
}

# Network Information
check_network_info() {
    print_header "NETWORK INFORMATION"
    
    # Local IP (try multiple methods)
    local_ip=""
    if command_exists ip; then
        local_ip=$(ip route get 8.8.8.8 2>/dev/null | grep -o 'src [0-9.]*' | cut -d' ' -f2 | head -1)
    fi
    
    if [ -z "$local_ip" ] && command_exists hostname; then
        local_ip=$(hostname -I 2>/dev/null | awk '{print $1}')
    fi
    
    if [ -z "$local_ip" ] && command_exists ifconfig; then
        local_ip=$(ifconfig 2>/dev/null | grep 'inet ' | grep -v '127.0.0.1' | head -1 | awk '{print $2}' | sed 's/addr://')
    fi
    
    print_info "Local IP" "${local_ip:-Unable to determine}"
    
    # Public IP
    public_ip=""
    for service in "https://api.ipify.org" "https://ipinfo.io/ip" "https://icanhazip.com"; do
        if command_exists curl; then
            public_ip=$(curl -s --connect-timeout 5 "$service" 2>/dev/null | tr -d '\n\r')
        elif command_exists wget; then
            public_ip=$(wget -qO- --timeout=5 "$service" 2>/dev/null | tr -d '\n\r')
        fi
        
        if [ -n "$public_ip" ] && [[ "$public_ip" =~ ^[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}$ ]]; then
            break
        fi
        public_ip=""
    done
    
    if [ -n "$public_ip" ]; then
        print_info "Public IP" "$public_ip" "success"
    else
        print_info "Public IP" "Unable to determine" "error"
    fi
    
    # Test connectivity
    connectivity="false"
    for target in "8.8.8.8" "1.1.1.1"; do
        if command_exists ping; then
            if ping -c 1 -W 3 "$target" >/dev/null 2>&1; then
                connectivity="true"
                break
            fi
        fi
    done
    
    if [ "$connectivity" = "true" ]; then
        print_info "Internet Connectivity" "Available" "success"
    else
        print_info "Internet Connectivity" "Limited or unavailable" "error"
    fi
}

# SSH Access Check
check_ssh_access() {
    print_header "SSH ACCESS CHECK"
    
    # Check if SSH daemon is running
    if command_exists pgrep; then
        if pgrep -f sshd >/dev/null 2>&1; then
            print_info "SSH Daemon" "Running" "success"
        else
            print_info "SSH Daemon" "Not running or not accessible" "error"
        fi
    else
        print_info "SSH Daemon" "Cannot check (pgrep not available)" "warning"
    fi
    
    # Check SSH port
    ssh_port=22
    if command_exists nc; then
        if nc -z localhost $ssh_port 2>/dev/null; then
            print_info "SSH Port $ssh_port" "Open" "success"
        else
            print_info "SSH Port $ssh_port" "Closed or filtered" "warning"
        fi
    elif command_exists telnet; then
        if timeout 3 telnet localhost $ssh_port </dev/null >/dev/null 2>&1; then
            print_info "SSH Port $ssh_port" "Open" "success"
        else
            print_info "SSH Port $ssh_port" "Closed or filtered" "warning"
        fi
    else
        print_info "SSH Port Check" "No suitable tool available (nc/telnet)" "warning"
    fi
    
    # Check SSH keys
    ssh_dir="$HOME/.ssh"
    if [ -d "$ssh_dir" ]; then
        key_files=""
        for key in "id_rsa.pub" "id_ed25519.pub" "id_ecdsa.pub" "authorized_keys"; do
            if [ -f "$ssh_dir/$key" ]; then
                key_files="$key_files $key"
            fi
        done
        
        if [ -n "$key_files" ]; then
            print_info "SSH Keys Found" "$(echo $key_files | sed 's/^ *//')" "success"
        else
            print_info "SSH Keys" "No public keys or authorized_keys found" "warning"
        fi
    else
        print_info "SSH Directory" "~/.ssh directory not found" "warning"
    fi
}

# VS Code Web Access Check
check_vscode_web() {
    print_header "VS CODE WEB ACCESS CHECK"
    
    # Check for code-server
    if command_exists code-server; then
        code_server_path=$(which code-server)
        print_info "code-server" "Installed at $code_server_path" "success"
        
        version=$(code-server --version 2>/dev/null | head -1)
        if [ -n "$version" ]; then
            print_info "code-server Version" "$version" "success"
        fi
    else
        print_info "code-server" "Not installed" "warning"
        print_info "Installation Tip" "Install with: curl -fsSL https://code-server.dev/install.sh | sh"
    fi
    
    # Check for running code-server
    if command_exists pgrep; then
        if pgrep -f code-server >/dev/null 2>&1; then
            print_info "code-server Process" "Running" "success"
        else
            print_info "code-server Process" "Not running" "warning"
        fi
    fi
    
    # Check common web ports
    for port in 8080 8443 3000 8000; do
        if command_exists nc; then
            if nc -z localhost $port 2>/dev/null; then
                print_info "Port $port" "Open (potential web service)" "success"
            fi
        fi
    done
    
    # Check for other web IDEs
    if command_exists pgrep; then
        if pgrep -f jupyter >/dev/null 2>&1; then
            print_info "Jupyter" "Process running" "success"
        fi
        
        if pgrep -f theia >/dev/null 2>&1; then
            print_info "Theia IDE" "Process running" "success"
        fi
    fi
}

# Main function
main() {
    echo -e "${BOLD}${PURPLE}"
    echo "  ______ _                 _   ____   ____  "
    echo " / _____|_)               | | |  _ \ |  _ \ "
    echo "| /     _  ___  _   _  ___| | | | | || | | |"
    echo "| |    | |/ _ \| | | |/ _ \ | | | | || | | |"
    echo "| \____| | (_) | |_| |  __/ | | |_| || |_| |"
    echo " \_____)_|\___/ \__,_|\___|_|  \___/ |____/ "
    echo "                                           "
    echo "        VM System Information Checker      "
    echo "              (Shell Version)             "
    echo -e "${NC}"
    
    start_time=$(date +%s)
    
    check_system_info
    check_cpu_info
    check_gpu_info
    check_memory_info
    check_disk_info
    check_network_info
    check_ssh_access
    check_vscode_web
    
    end_time=$(date +%s)
    duration=$((end_time - start_time))
    
    print_header "SUMMARY"
    print_info "Check Duration" "${duration} seconds"
    print_info "Status" "System check completed successfully" "success"
    
    echo -e "\n${BOLD}${CYAN}Tip: Save this output for reference when setting up remote access!${NC}"
}

# Run main function
main "$@"