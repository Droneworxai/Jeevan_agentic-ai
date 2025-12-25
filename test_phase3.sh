#!/bin/bash

# Phase 3 Test Script - ROS 2 Node Communication
# This script tests topics, services, and node communication

echo "========================================="
echo "Phase 3: ROS 2 Node Communication Test"
echo "========================================="

cd /home/jeevan-koiri/Desktop/drive/dev-work/droneworx/Jeevan_agentic-ai
source install/setup.bash

# Start simulation in background
echo "[1/6] Starting simulation..."
ros2 launch simulation sim.launch.py gz_args:="-s -r" > /tmp/phase3_sim.log 2>&1 &
SIM_PID=$!
sleep 12

echo "[2/6] Testing topic availability..."
ros2 topic list | grep -E "(mission_status|robot_pose|farm_boundary|weed_status)" && echo "✅ Key topics found" || echo "❌ Missing topics"

echo ""
echo "[3/6] Testing services..."
echo "- Testing /set_boundary service..."
ros2 service call /set_boundary std_srvs/srv/Trigger "{}" && echo "✅ /set_boundary works" || echo "❌ /set_boundary failed"

echo "- Testing /load_mission service..."
ros2 service call /load_mission std_srvs/srv/Trigger "{}" && echo "✅ /load_mission works" || echo "❌ /load_mission failed"

echo "- Testing /reset_farm service..."
ros2 service call /reset_farm std_srvs/srv/Trigger "{}" && echo "✅ /reset_farm works" || echo "❌ /reset_farm failed"

echo ""
echo "[4/6] Testing mission_status topic..."
timeout 3 ros2 topic echo /mission_status --once && echo "✅ /mission_status publishing" || echo "❌ /mission_status not publishing"

echo ""
echo "[5/6] Testing robot_pose topic..."
timeout 3 ros2 topic echo /robot_pose --once && echo "✅ /robot_pose publishing" || echo "❌ /robot_pose not publishing"

echo ""
echo "[6/6] Checking node list..."
ros2 node list | grep -E "(eco_mission_planner|eco_boundary_manager|eco_weed_manager|eco_mission_monitor)" && echo "✅ All custom nodes running" || echo "❌ Missing nodes"

echo ""
echo "========================================="
echo "Cleaning up..."
kill -9 $SIM_PID 2>/dev/null
killall -9 gz 2>/dev/null
killall -9 python3 2>/dev/null
sleep 2

echo "Phase 3 test complete!"
echo "========================================="
