#!/bin/bash

# Quick launcher script for VM System Check tools
# Usage: bash run_system_check.sh [python|shell|both]

set -e

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BOLD='\033[1m'
NC='\033[0m' # No Color

print_banner() {
    echo -e "${BLUE}${BOLD}"
    echo "╔══════════════════════════════════════════════════════════╗"
    echo "║                VM System Check Launcher                  ║"
    echo "║                                                          ║"
    echo "║  Choose your preferred system information tool:          ║"
    echo "║  • Python version (comprehensive, recommended)           ║"
    echo "║  • Shell version (lightweight, basic checks)            ║"
    echo "║  • Both versions (complete analysis)                     ║"
    echo "╚══════════════════════════════════════════════════════════╝"
    echo -e "${NC}"
}

check_file_exists() {
    local file="$1"
    local name="$2"
    
    if [ ! -f "$file" ]; then
        echo -e "${RED}Error: $name not found at $file${NC}"
        echo -e "${YELLOW}Please ensure you're running this script from the correct directory.${NC}"
        exit 1
    fi
}

run_python_version() {
    echo -e "${GREEN}${BOLD}Running Python Version (Comprehensive Analysis)...${NC}\n"
    
    check_file_exists "vm_system_check.py" "Python script"
    
    if command -v python3 >/dev/null 2>&1; then
        chmod +x vm_system_check.py 2>/dev/null || true
        python3 vm_system_check.py
    elif command -v python >/dev/null 2>&1; then
        chmod +x vm_system_check.py 2>/dev/null || true
        python vm_system_check.py
    else
        echo -e "${RED}Error: Python not found. Please install Python 3.6+ to use this version.${NC}"
        echo -e "${YELLOW}Falling back to shell version...${NC}\n"
        run_shell_version
    fi
}

run_shell_version() {
    echo -e "${GREEN}${BOLD}Running Shell Version (Lightweight Analysis)...${NC}\n"
    
    check_file_exists "vm_system_check.sh" "Shell script"
    
    chmod +x vm_system_check.sh 2>/dev/null || true
    bash vm_system_check.sh
}

run_both_versions() {
    echo -e "${BLUE}${BOLD}Running Both Versions for Complete Analysis...${NC}\n"
    
    echo -e "${YELLOW}═══════════════════════════════════════════════════════════${NC}"
    echo -e "${YELLOW}                    PYTHON VERSION RESULTS                 ${NC}"
    echo -e "${YELLOW}═══════════════════════════════════════════════════════════${NC}"
    run_python_version
    
    echo -e "\n${YELLOW}═══════════════════════════════════════════════════════════${NC}"
    echo -e "${YELLOW}                     SHELL VERSION RESULTS                 ${NC}"
    echo -e "${YELLOW}═══════════════════════════════════════════════════════════${NC}"
    run_shell_version
    
    echo -e "\n${GREEN}${BOLD}Both analyses completed successfully!${NC}"
}

show_help() {
    echo -e "${BOLD}VM System Check Launcher${NC}"
    echo ""
    echo "Usage: $0 [option]"
    echo ""
    echo "Options:"
    echo "  python    Run Python version (comprehensive analysis)"
    echo "  shell     Run shell version (lightweight analysis)"
    echo "  both      Run both versions (complete analysis)"
    echo "  help      Show this help message"
    echo ""
    echo "If no option is provided, you'll be prompted to choose."
    echo ""
    echo "Examples:"
    echo "  $0 python    # Run Python version only"
    echo "  $0 shell     # Run shell version only"
    echo "  $0 both      # Run both versions"
    echo "  $0           # Interactive mode"
}

interactive_mode() {
    print_banner
    
    echo -e "${BOLD}Please select an option:${NC}"
    echo "1) Python version (recommended)"
    echo "2) Shell version (lightweight)"
    echo "3) Both versions (complete)"
    echo "4) Help"
    echo "5) Exit"
    echo ""
    read -p "Enter your choice (1-5): " choice
    
    case $choice in
        1)
            echo ""
            run_python_version
            ;;
        2)
            echo ""
            run_shell_version
            ;;
        3)
            echo ""
            run_both_versions
            ;;
        4)
            echo ""
            show_help
            ;;
        5)
            echo -e "${GREEN}Goodbye!${NC}"
            exit 0
            ;;
        *)
            echo -e "${RED}Invalid choice. Please select 1-5.${NC}"
            exit 1
            ;;
    esac
}

# Main script logic
case "${1:-}" in
    "python")
        run_python_version
        ;;
    "shell")
        run_shell_version
        ;;
    "both")
        run_both_versions
        ;;
    "help" | "-h" | "--help")
        show_help
        ;;
    "")
        interactive_mode
        ;;
    *)
        echo -e "${RED}Unknown option: $1${NC}"
        echo -e "${YELLOW}Use '$0 help' for usage information.${NC}"
        exit 1
        ;;
esac

echo -e "\n${GREEN}${BOLD}System check completed! 🎉${NC}"
echo -e "${BLUE}Tip: Save this output for your records when setting up remote access.${NC}"