#!/bin/bash

# Absolute path to the simulation directory
SIM_DIR="/home/jeevan-koiri/Desktop/drive/dev-work/droneworx/Jeevan_agentic-ai/gz_simulation"

# Fix for VS Code Snap environment libpthread issue
export LD_PRELOAD=/lib/x86_64-linux-gnu/libpthread.so.0

# Set Gazebo Resource Path to find local textures
export GZ_SIM_RESOURCE_PATH="$SIM_DIR/worlds:$GZ_SIM_RESOURCE_PATH"

# Source ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

# Source the workspace (CRITICAL: This makes the 'simulation' package available)
WORKSPACE_DIR="/home/jeevan-koiri/Desktop/drive/dev-work/droneworx/Jeevan_agentic-ai"
if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
    source "$WORKSPACE_DIR/install/setup.bash"
    echo "Workspace sourced from $WORKSPACE_DIR/install/setup.bash"
else
    echo "ERROR: Workspace not built! Run 'colcon build' first."
    exit 1
fi

echo "Starting Gazebo with Ecoweeder Bot (Fuel Models)..."
echo "-------------------------------------------------------"
echo "GUI NOTICE: If the Gazebo window does not appear or crashes,"
echo "it is likely due to the VS Code Snap environment sandbox."
echo ""
echo "FIX: Open a standard Ubuntu terminal (Ctrl+Alt+T) and run:"
echo "cd $SIM_DIR && ./start_sim.sh"
echo "-------------------------------------------------------"

# Launch the simulation (Gazebo + Bridge)
# We run this in the background so we can also start the mission planner
ros2 launch "$SIM_DIR/launch/sim.launch.py" &
LAUNCH_PID=$!

# Wait for Gazebo to start
sleep 10

echo "Starting ROS 2 Nodes (Architecture Components)..."
python3 "$SIM_DIR/simulation/boundary_manager.py" &
BOUNDARY_PID=$!

python3 "$SIM_DIR/simulation/weed_manager.py" &
WEED_PID=$!

python3 "$SIM_DIR/simulation/mission_monitor.py" &
MONITOR_PID=$!

echo "Starting Mission Planner..."
python3 "$SIM_DIR/simulation/mission_planner.py" &
PLANNER_PID=$!

# Wait for user interrupt
trap "kill $LAUNCH_PID $PLANNER_PID $BOUNDARY_PID $WEED_PID $MONITOR_PID; exit" SIGINT SIGTERM
wait
