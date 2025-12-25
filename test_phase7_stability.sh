#!/bin/bash

# Source ROS 2 and workspace
source /opt/ros/jazzy/setup.bash
source /home/jeevan-koiri/Desktop/drive/dev-work/droneworx/Jeevan_agentic-ai/install/setup.bash

echo "--- Phase 7: System Stability & Performance Test ---"

# Function to cleanup on exit
cleanup() {
    echo "Cleaning up..."
    pkill -f "ros2 launch simulation sim.launch.py"
    pkill -f "gz sim"
    exit
}
trap cleanup SIGINT SIGTERM

# 1. Start Simulation
echo "Step 1: Starting Simulation (Headless)..."
ros2 launch simulation sim.launch.py gz_args:="-s -r" > /tmp/sim_output.log 2>&1 &
SIM_PID=$!

sleep 15

# Check if nodes are running
echo "Checking nodes..."
ros2 node list | grep eco_

# 2. Start Mission
echo "Step 2: Starting Mission Workflow..."
ros2 topic pub --once /mission/start std_msgs/msg/String "{data: '{\"farmName\":\"Stability Test\",\"boundary\":{\"type\":\"FeatureCollection\",\"features\":[{\"type\":\"Feature\",\"properties\":{},\"geometry\":{\"type\":\"Polygon\",\"coordinates\":[[[-1.755,52.175],[-1.754,52.175],[-1.754,52.176],[-1.755,52.176],[-1.755,52.175]]]}}]},\"planner\":\"boustrophedon\"}'}"
sleep 2
ros2 service call /set_boundary std_srvs/srv/Trigger {}
sleep 2
ros2 service call /load_mission std_srvs/srv/Trigger {}
sleep 2
ros2 topic pub --once /mission/execute std_msgs/msg/Empty {}

# 3. Monitor for 30 seconds
echo "Step 3: Monitoring system for 30 seconds..."
DURATION=30
START_TIME=$(date +%s)

while [ $(($(date +%s) - $START_TIME)) -lt $DURATION ]; do
    # Check if simulation is still running
    if ! kill -0 $SIM_PID 2>/dev/null; then
        echo "ERROR: Simulation process died!"
        break
    fi
    
    # Check node count
    NODE_COUNT=$(ros2 node list | grep eco_ | wc -l)
    if [ $NODE_COUNT -lt 5 ]; then
        echo "WARNING: Some eco_ nodes have died! Count: $NODE_COUNT"
        ros2 node list | grep eco_
    fi
    
    # Check CPU usage of ROS nodes
    CPU_USAGE=$(top -bn1 | grep -E "mission_|boundary_|weed_" | awk '{sum += $9} END {print sum}')
    echo "Time: $(($(date +%s) - $START_TIME))s | Nodes: $NODE_COUNT | CPU: $CPU_USAGE%"
    
    sleep 10
done

echo "Step 4: Aborting Mission..."
ros2 topic pub --once /mission/stop std_msgs/msg/Empty {}
sleep 2

echo "--- Stability Test Complete ---"
cleanup
