#!/bin/bash

echo "--- EcoWeeder Shutdown ---"

# Function to kill processes by name pattern
kill_processes() {
    local pattern=$1
    local description=$2
    
    if pgrep -f "$pattern" > /dev/null; then
        echo "Stopping $description..."
        pkill -TERM -f "$pattern" 2>/dev/null
        sleep 2
        
        # Force kill if still running
        if pgrep -f "$pattern" > /dev/null; then
            echo "Force stopping $description..."
            pkill -KILL -f "$pattern" 2>/dev/null
        fi
        echo "$description stopped."
    else
        echo "$description not running."
    fi
}

# Stop Next.js development server
kill_processes "next dev" "Next.js Server"

# Stop ROS 2 nodes
kill_processes "mission_planner" "Mission Planner"
kill_processes "mission_monitor" "Mission Monitor"
kill_processes "boundary_manager" "Boundary Manager"
kill_processes "weed_manager" "Weed Manager"

# Stop rosbridge
kill_processes "rosbridge" "ROS Bridge WebSocket"

# Stop ROS-Gazebo bridge
kill_processes "ros_gz_bridge" "ROS-Gazebo Bridge"

# Stop Gazebo
kill_processes "gz sim" "Gazebo Simulation"
kill_processes "ruby.*gz.*sim" "Gazebo Launcher"

# Clean up log files (optional)
if [ -f "simulation.log" ]; then
    echo "Archiving simulation log..."
    mv simulation.log "simulation_$(date +%Y%m%d_%H%M%S).log"
fi

# Kill any remaining zombie processes
if pgrep -f "Jeevan_agentic-ai" > /dev/null; then
    echo "Cleaning up remaining processes..."
    pkill -KILL -f "Jeevan_agentic-ai.*node" 2>/dev/null
fi

echo ""
echo "✓ All EcoWeeder processes stopped."
echo "  You can restart with: ./start.sh"
