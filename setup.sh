#!/bin/bash

# Multi-Terminal Distrobox Script
# This script opens three separate terminals, each logging into a distrobox instance,
# changing to a specific directory, and running a specific command.

# Configuration - Fill in your details here
DISTRO_NAME_1="ubuntu-24.04"    # Replace with your distrobox name
DISTRO_NAME_2="ubuntu-24.04"    # Replace with your distrobox name  
DISTRO_NAME_3="ubuntu-24.04"    # Replace with your distrobox name

DIRECTORY_1="~/repos/mpac/mpac_mujoco/"         # Replace with your directory path
DIRECTORY_2="~/repos/mpac/mpac_go2/"         # Replace with your directory path
DIRECTORY_3="~/repos/CrossTele/"         # Replace with your directory path

COMMAND_1="build/simulate/unitree_mujoco"                # Replace with your command
COMMAND_2="build/ctrl --io_mode=mujoco"                # Replace with your command
COMMAND_3="echo Enter A Command..."                # Replace with your command

# Terminal emulator detection
detect_terminal() {
    if command -v gnome-terminal >/dev/null 2>&1; then
        echo "gnome-terminal"
    elif command -v konsole >/dev/null 2>&1; then
        echo "konsole"
    elif command -v xfce4-terminal >/dev/null 2>&1; then
        echo "xfce4-terminal"
    elif command -v alacritty >/dev/null 2>&1; then
        echo "alacritty"
    elif command -v kitty >/dev/null 2>&1; then
        echo "kitty"
    elif command -v terminator >/dev/null 2>&1; then
        echo "terminator"
    elif command -v xterm >/dev/null 2>&1; then
        echo "xterm"
    else
        echo "unknown"
    fi
}

# Function to open terminal with distrobox command
open_terminal_with_distrobox() {
    local distro_name="$1"
    local directory="$2"
    local command="$3"
    local terminal_title="$4"
    local terminal_emulator="$5"
    
    # Create the full command to run in distrobox
    local full_command="cd '$directory' && $command"
    
    case "$terminal_emulator" in
        "gnome-terminal")
            gnome-terminal --title="$terminal_title" -- bash -c "distrobox enter '$distro_name' -- bash -c '$full_command; exec bash'"
            ;;
        "konsole")
            konsole --title "$terminal_title" -e bash -c "distrobox enter '$distro_name' -- bash -c '$full_command; exec bash'"
            ;;
        "xfce4-terminal")
            xfce4-terminal --title="$terminal_title" --command="bash -c \"distrobox enter '$distro_name' -- bash -c '$full_command; exec bash'\""
            ;;
        "alacritty")
            alacritty --title "$terminal_title" -e bash -c "distrobox enter '$distro_name' -- bash -c '$full_command; exec bash'"
            ;;
        "kitty")
            kitty --title "$terminal_title" bash -c "distrobox enter '$distro_name' -- bash -c '$full_command; exec bash'"
            ;;
        "terminator")
            terminator --title="$terminal_title" --command="bash -c \"distrobox enter '$distro_name' -- bash -c '$full_command; exec bash'\""
            ;;
        "xterm")
            xterm -title "$terminal_title" -e bash -c "distrobox enter '$distro_name' -- bash -c '$full_command; exec bash'"
            ;;
        *)
            echo "Unsupported terminal emulator: $terminal_emulator"
            echo "Please install one of: gnome-terminal, konsole, xfce4-terminal, alacritty, kitty, terminator, xterm"
            return 1
            ;;
    esac
}

# Function to validate distrobox exists
check_distrobox() {
    local distro_name="$1"
    if ! distrobox list | grep -q "$distro_name"; then
        echo "Warning: Distrobox '$distro_name' not found!"
        echo "Available distroboxes:"
        distrobox list
        return 1
    fi
    return 0
}

# Main script
main() {
    echo "CrossTele Startup Utility"
    echo "=========================="
    
    # Check if distrobox is installed
    if ! command -v distrobox >/dev/null 2>&1; then
        echo "Error: distrobox is not installed!"
        echo "Please install distrobox first: https://github.com/89luca89/distrobox"
        exit 1
    fi
    
    # Detect terminal emulator
    TERMINAL=$(detect_terminal)
    echo "Detected terminal emulator: $TERMINAL"
    
    if [ "$TERMINAL" = "unknown" ]; then
        echo "Error: No supported terminal emulator found!"
        exit 1
    fi
    
    # Validate configuration
    echo "Checking configuration..."
    
    # Check if distroboxes exist
    check_distrobox "$DISTRO_NAME_1" || echo "Will attempt to use $DISTRO_NAME_1 anyway..."
    check_distrobox "$DISTRO_NAME_2" || echo "Will attempt to use $DISTRO_NAME_2 anyway..."
    check_distrobox "$DISTRO_NAME_3" || echo "Will attempt to use $DISTRO_NAME_3 anyway..."
    
    echo ""
    echo "Configuration:"
    echo "Terminal 1: $DISTRO_NAME_1 -> $DIRECTORY_1 -> $COMMAND_1"
    echo "Terminal 2: $DISTRO_NAME_2 -> $DIRECTORY_2 -> $COMMAND_2" 
    echo "Terminal 3: $DISTRO_NAME_3 -> $DIRECTORY_3 -> $COMMAND_3"
    echo ""
    
    # Ask for confirmation
    read -p "Proceed with launching terminals? (y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "Cancelled."
        exit 0
    fi
    
    echo "Launching terminals..."
    
    # Launch terminals with small delays to avoid race conditions
    open_terminal_with_distrobox "$DISTRO_NAME_1" "$DIRECTORY_1" "$COMMAND_1" "Distrobox 1: $DISTRO_NAME_1" "$TERMINAL" &
    sleep 0.5
    
    open_terminal_with_distrobox "$DISTRO_NAME_2" "$DIRECTORY_2" "$COMMAND_2" "Distrobox 2: $DISTRO_NAME_2" "$TERMINAL" &
    sleep 0.5
    
    open_terminal_with_distrobox "$DISTRO_NAME_3" "$DIRECTORY_3" "$COMMAND_3" "Distrobox 3: $DISTRO_NAME_3" "$TERMINAL" &
    
    echo "All terminals launched!"
    echo "Note: Each terminal will keep the shell open after the command completes."
}

# Alternative function for running commands that should exit after completion
open_terminal_run_and_exit() {
    local distro_name="$1"
    local directory="$2"
    local command="$3"
    local terminal_title="$4"
    local terminal_emulator="$5"
    
    # Create the full command to run in distrobox (exits after completion)
    local full_command="cd '$directory' && $command"
    
    case "$terminal_emulator" in
        "gnome-terminal")
            gnome-terminal --title="$terminal_title" -- bash -c "distrobox enter '$distro_name' -- bash -c '$full_command'"
            ;;
        "konsole")
            konsole --title "$terminal_title" -e bash -c "distrobox enter '$distro_name' -- bash -c '$full_command'"
            ;;
        *)
            # Add other terminal cases as needed
            bash -c "distrobox enter '$distro_name' -- bash -c '$full_command'"
            ;;
    esac
}

# Help function
show_help() {
    cat << EOF
Multi-Terminal Distrobox Script

Usage: $0 [OPTIONS]

This script opens three separate terminals, each logging into a distrobox instance,
changing to a specific directory, and running a specific command.

Before running, edit the script to configure:
- DISTRO_NAME_1, DISTRO_NAME_2, DISTRO_NAME_3: Your distrobox names
- DIRECTORY_1, DIRECTORY_2, DIRECTORY_3: Target directories
- COMMAND_1, COMMAND_2, COMMAND_3: Commands to run

Options:
  -h, --help    Show this help message
  -l, --list    List available distroboxes
  -t, --test    Test configuration without launching terminals

Examples:
  # Basic usage (after configuring the script)
  $0
  
  # List available distroboxes
  $0 --list

Supported terminal emulators:
- gnome-terminal, konsole, xfce4-terminal, alacritty, kitty, terminator, xterm

EOF
}

# Parse command line arguments
case "${1:-}" in
    -h|--help)
        show_help
        exit 0
        ;;
    -l|--list)
        echo "Available distroboxes:"
        distrobox list
        exit 0
        ;;
    -t|--test)
        echo "Testing configuration..."
        echo "Distrobox 1: $DISTRO_NAME_1"
        echo "Distrobox 2: $DISTRO_NAME_2" 
        echo "Distrobox 3: $DISTRO_NAME_3"
        echo "Directory 1: $DIRECTORY_1"
        echo "Directory 2: $DIRECTORY_2"
        echo "Directory 3: $DIRECTORY_3"
        echo "Command 1: $COMMAND_1"
        echo "Command 2: $COMMAND_2"
        echo "Command 3: $COMMAND_3"
        echo "Terminal: $(detect_terminal)"
        exit 0
        ;;
    "")
        main
        ;;
    *)
        echo "Unknown option: $1"
        show_help
        exit 1
        ;;
esac