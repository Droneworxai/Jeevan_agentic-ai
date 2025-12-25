#!/bin/bash

# Quick start script for the entire EcoWeeder system
echo "Starting EcoWeeder Simulation System..."
echo "========================================"

# Check if ROS 2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "Sourcing ROS 2 Jazzy..."
    source /opt/ros/jazzy/setup.bash
fi

# Start the simulation in the background
echo "Starting Gazebo simulation and ROS nodes..."
cd "$(dirname "$0")"
./start_sim.sh &
SIM_PID=$!

# Wait a bit for the simulation to start
sleep 5

echo ""
echo "========================================"
echo "Simulation started successfully!"
echo "========================================"
echo ""
echo "To use the dashboard:"
echo "1. Navigate to the project root"
echo "2. Run: npm run dev"
echo "3. Open: http://localhost:3000/simulation"
echo ""
echo "Press Ctrl+C to stop the simulation"
echo ""

# Keep the script running
wait $SIM_PID
