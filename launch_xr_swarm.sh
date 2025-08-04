#!/bin/bash

# XR-Swarm-Bridge Launch Script
# This script launches all components of the XR-Swarm-Bridge system

set -e

echo "🚁 Starting XR-Swarm-Bridge System..."

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if running in Docker
if [ -f /.dockerenv ]; then
    print_status "Running in Docker container"
    DOCKER_MODE=true
else
    DOCKER_MODE=false
fi

# Check dependencies
check_dependencies() {
    print_status "Checking dependencies..."
    
    # Check ROS 2
    if ! command -v ros2 &> /dev/null; then
        print_error "ROS 2 not found. Please install ROS 2 Humble."
        exit 1
    fi
    
    # Check Node.js
    if ! command -v node &> /dev/null; then
        print_error "Node.js not found. Please install Node.js 18+."
        exit 1
    fi
    
    # Check Python packages
    python3 -c "import rclpy, websockets, cv2" 2>/dev/null || {
        print_warning "Some Python dependencies missing. Installing..."
        pip3 install rclpy websockets opencv-python numpy
    }
    
    print_status "Dependencies check completed ✓"
}

# Build ROS workspace
build_ros_workspace() {
    print_status "Building ROS 2 workspace..."
    
    cd ros2_ws
    
    # Source ROS 2
    source /opt/ros/humble/setup.bash
    
    # Install dependencies
    rosdep update
    rosdep install --from-paths src --ignore-src -r -y || print_warning "Some rosdep packages not found"
    
    # Build workspace
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
    
    if [ $? -eq 0 ]; then
        print_status "ROS workspace built successfully ✓"
    else
        print_error "ROS workspace build failed"
        exit 1
    fi
    
    cd ..
}

# Install webapp dependencies
setup_webapp() {
    print_status "Setting up webapp..."
    
    cd webapp
    
    # Install dependencies
    npm install
    
    if [ $? -eq 0 ]; then
        print_status "Webapp dependencies installed ✓"
    else
        print_error "Failed to install webapp dependencies"
        exit 1
    fi
    
    cd ..
}

# Start ROS components
start_ros_components() {
    print_status "Starting ROS 2 components..."
    
    # Source the workspace
    source ros2_ws/install/setup.bash
    
    # Start swarm coordinator
    print_status "Starting swarm coordinator..."
    ros2 run xr_swarm_core swarm_coordinator.py &
    COORDINATOR_PID=$!
    
    sleep 2
    
    # Start WebRTC bridge
    print_status "Starting WebRTC bridge..."
    ros2 run webrtc_bridge webrtc_server.py &
    WEBRTC_PID=$!
    
    sleep 2
    
    # Start example drone agents
    print_status "Starting example drone agents..."
    for i in {1..3}; do
        ros2 run robot_agents drone_agent.py drone_0$i &
        DRONE_PIDS[$i]=$!
        sleep 1
    done
    
    print_status "ROS components started ✓"
}

# Start webapp
start_webapp() {
    print_status "Starting webapp..."
    
    cd webapp
    
    # Development mode
    if [ "$1" = "dev" ]; then
        npm run dev &
        WEBAPP_PID=$!
    else
        # Production mode
        npm run build
        npm run preview &
        WEBAPP_PID=$!
    fi
    
    cd ..
    
    print_status "Webapp started ✓"
}

# Cleanup function
cleanup() {
    print_status "Shutting down XR-Swarm-Bridge..."
    
    # Kill all background processes
    if [ ! -z "$COORDINATOR_PID" ]; then
        kill $COORDINATOR_PID 2>/dev/null || true
    fi
    
    if [ ! -z "$WEBRTC_PID" ]; then
        kill $WEBRTC_PID 2>/dev/null || true
    fi
    
    if [ ! -z "$WEBAPP_PID" ]; then
        kill $WEBAPP_PID 2>/dev/null || true
    fi
    
    # Kill drone agents
    for pid in "${DRONE_PIDS[@]}"; do
        if [ ! -z "$pid" ]; then
            kill $pid 2>/dev/null || true
        fi
    done
    
    # Kill any remaining ROS processes
    pkill -f "ros2" 2>/dev/null || true
    pkill -f "python3.*swarm" 2>/dev/null || true
    
    print_status "Cleanup completed"
    exit 0
}

# Setup signal handlers
trap cleanup SIGINT SIGTERM

# Main execution
main() {
    print_status "XR-Swarm-Bridge Launch Script v1.0"
    print_status "======================================="
    
    # Parse arguments
    MODE="prod"
    if [ "$1" = "dev" ]; then
        MODE="dev"
        print_status "Running in development mode"
    fi
    
    # Run setup steps
    check_dependencies
    build_ros_workspace
    setup_webapp
    
    print_status "======================================="
    print_status "Starting system components..."
    
    # Start components
    start_ros_components
    start_webapp $MODE
    
    print_status "======================================="
    print_status "✅ XR-Swarm-Bridge is now running!"
    print_status ""
    print_status "🌐 Webapp: http://localhost:3000"
    print_status "🔗 WebRTC Bridge: ws://localhost:8443"
    print_status "🤖 ROS 2 nodes: Active"
    print_status ""
    print_status "Press Ctrl+C to stop all services"
    print_status "======================================="
    
    # Wait for user interrupt
    while true; do
        sleep 1
        
        # Check if processes are still running
        if ! kill -0 $COORDINATOR_PID 2>/dev/null; then
            print_error "Swarm coordinator stopped unexpectedly"
            cleanup
        fi
        
        if ! kill -0 $WEBAPP_PID 2>/dev/null; then
            print_error "Webapp stopped unexpectedly"
            cleanup
        fi
    done
}

# Create directories if they don't exist
mkdir -p logs
mkdir -p config

# Redirect output to log file while still showing on console
exec > >(tee -a logs/launch.log)
exec 2>&1

# Run main function
main "$@"