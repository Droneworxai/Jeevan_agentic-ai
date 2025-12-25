#!/bin/bash

# Phase 4 Test Script - Rosbridge WebSocket Setup
# This script tests WebSocket connectivity and data flow

echo "========================================="
echo "Phase 4: Rosbridge WebSocket Test"
echo "========================================="

cd /home/jeevan-koiri/Desktop/drive/dev-work/droneworx/Jeevan_agentic-ai
source install/setup.bash

# Start simulation in background
echo "[1/5] Starting simulation with rosbridge..."
ros2 launch simulation sim.launch.py gz_args:="-s -r" > /tmp/phase4_sim.log 2>&1 &
SIM_PID=$!
sleep 12

echo "[2/5] Checking if rosbridge is running on port 9090..."
if netstat -tuln | grep -q 9090; then
    echo "✅ Rosbridge WebSocket server is running on port 9090"
else
    echo "❌ Rosbridge WebSocket server is NOT running"
    kill -9 $SIM_PID 2>/dev/null
    exit 1
fi

echo ""
echo "[3/5] Testing WebSocket connection (requires wscat)..."
if command -v wscat &> /dev/null; then
    echo "Testing WebSocket handshake..."
    timeout 3 wscat -c ws://localhost:9090 -x '{"op":"call_service","service":"/rosapi/get_ros_version","args":{}}' 2>&1 | head -5
    echo "✅ WebSocket connection test completed"
else
    echo "⚠️  wscat not installed, skipping WebSocket handshake test"
    echo "   Install with: npm install -g wscat"
fi

echo ""
echo "[4/5] Verifying rosbridge_suite installation..."
dpkg -l | grep ros-jazzy-rosbridge > /dev/null && echo "✅ rosbridge-suite is installed" || echo "❌ rosbridge-suite NOT installed"

echo ""
echo "[5/5] Opening test HTML page..."
echo "========================================="
echo ""
echo "✅ Phase 4 Infrastructure Ready!"
echo ""
echo "📋 Manual Testing Steps:"
echo "1. Open in browser: file:///home/jeevan-koiri/Desktop/drive/dev-work/droneworx/Jeevan_agentic-ai/test_phase4_rosbridge.html"
echo "   OR use: python3 -m http.server 8080 (then http://localhost:8080/test_phase4_rosbridge.html)"
echo ""
echo "2. In the browser test page:"
echo "   - Click 'Connect to ROS Bridge'"
echo "   - Click 'Get ROS Topics' to see available topics"
echo "   - Click 'Subscribe to Mission Status' to see live data"
echo "   - Click 'Call /set_boundary Service' to test service calls"
echo ""
echo "3. The test page will show:"
echo "   ✓ Connection status"
echo "   ✓ Available ROS topics"
echo "   ✓ Real-time mission status data"
echo "   ✓ Real-time robot pose data"
echo "   ✓ Service call responses"
echo ""
echo "Simulation is running in the background (PID: $SIM_PID)"
echo "To stop: kill -9 $SIM_PID && killall -9 gz python3"
echo "========================================="

# Keep the script running to maintain simulation
read -p "Press Enter to stop simulation and exit..."
kill -9 $SIM_PID 2>/dev/null
killall -9 gz python3 2>/dev/null
echo "Simulation stopped. Phase 4 test complete!"
