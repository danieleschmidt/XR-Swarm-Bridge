#!/bin/bash

# XR-Swarm-Bridge Simulation Launcher
# Starts multiple drone agents and the swarm coordinator for demonstration

echo "🚁 Starting XR-Swarm-Bridge Simulation..."

# Set environment variables
export PYTHONPATH="${PYTHONPATH}:$(pwd)/ros2_ws/src"
export ROS_DOMAIN_ID=42  # Isolated domain for simulation

# Function to cleanup on exit
cleanup() {
    echo "🛑 Shutting down simulation..."
    jobs -p | xargs -r kill
    wait
    echo "✅ Simulation stopped"
    exit 0
}

# Set up signal handlers
trap cleanup SIGINT SIGTERM

# Check if ROS2 is available
if ! command -v ros2 &> /dev/null; then
    echo "⚠️  ROS2 not found. Starting WebRTC bridge and webapp only..."
    
    # Start WebRTC bridge (Python fallback)
    cd ros2_ws/src/webrtc_bridge/scripts
    python3 webrtc_server.py &
    WEBRTC_PID=$!
    
    # Start webapp
    cd ../../../../webapp
    npm run dev &
    WEBAPP_PID=$!
    
    echo "🌐 WebRTC bridge started (PID: $WEBRTC_PID)"
    echo "🖥️  Webapp started (PID: $WEBAPP_PID)"
    echo "📱 Open http://localhost:5173 to access the interface"
    
    wait
    exit 0
fi

# Initialize ROS2 workspace
cd ros2_ws
source /opt/ros/humble/setup.bash 2>/dev/null || echo "⚠️  ROS2 Humble not sourced globally"
colcon build --symlink-install --continue-on-error
source install/setup.bash

echo "✅ ROS2 workspace built and sourced"

# Start ROS2 nodes
cd ..

# 1. Start WebRTC Bridge
echo "🌉 Starting WebRTC Bridge..."
ros2 run webrtc_bridge webrtc_server.py &
WEBRTC_PID=$!
sleep 2

# 2. Start Swarm Coordinator  
echo "🎯 Starting Swarm Coordinator..."
ros2 run xr_swarm_core swarm_coordinator.py &
COORDINATOR_PID=$!
sleep 2

# 3. Start multiple drone agents
echo "🚁 Starting drone swarm..."
DRONE_PIDS=()

for i in {01..05}; do
    DRONE_ID="drone_$i"
    echo "   Starting $DRONE_ID..."
    ros2 run robot_agents drone_agent.py $DRONE_ID &
    DRONE_PIDS+=($!)
    sleep 0.5
done

sleep 2

# 4. Start the webapp
echo "🖥️  Starting React webapp..."
cd webapp
npm install --silent &>/dev/null || echo "⚠️  npm install failed, continuing..."
npm run dev &
WEBAPP_PID=$!
cd ..

sleep 3

echo ""
echo "🎉 XR-Swarm-Bridge Simulation Started Successfully!"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "🌐 WebApp:           http://localhost:5173"
echo "🔗 WebSocket:        ws://localhost:8443"
echo "📊 Agents:           5 drones (drone_01 to drone_05)"
echo "🎮 Controls:         VR/AR supported, click VR/AR buttons"
echo "🤖 AI Commands:      GPT-4o integration available"
echo "📱 Mobile:           Touch controls supported"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "🧪 Try these commands in the webapp:"
echo "   • 'Take off all drones to 10 meters'"
echo "   • 'Form a circle formation'"  
echo "   • 'Search the area in a grid pattern'"
echo "   • 'Return all drones to base'"
echo ""
echo "🛑 Press Ctrl+C to stop the simulation"
echo ""

# Wait for user interrupt
wait

# Cleanup is handled by trap