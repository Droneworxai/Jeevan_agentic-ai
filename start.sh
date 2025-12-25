#!/bin/bash

# EcoWeeder Full-Stack Startup Script
# IMPORTANT: Run chmod +x start.sh then ./start.sh

echo "--- EcoWeeder Startup ---"

# Stop any existing processes first
if [ -f "./stop.sh" ]; then
    echo "Cleaning up existing processes..."
    ./stop.sh
    sleep 2
fi

# 1. Environment Checks
echo "Checking for Node.js..."
node -v > /dev/null 2>&1 || { echo "Node.js is not installed. Please install it first."; exit 1; }

# Check for .env file
if [ ! -f .env ]; then
    echo ".env file not found. Creating basic .env..."
    echo "DATABASE_URL=\"mongodb://localhost:27017/ecoweeder\"" > .env
    echo "NEXT_PUBLIC_GEMINI_API_KEY=\"\"" >> .env
fi

# 2. Dependency Setup
echo "Installing dependencies..."
if [ -f package-lock.json ]; then
    npm ci
else
    npm install
fi

echo "Generating Prisma client..."
npx prisma generate

# 3. ROS 2 Simulation Stack
if command -v ros2 &> /dev/null; then
    echo "ROS 2 detected. Starting Simulation Stack..."
    
    # Kill any existing simulation processes to prevent duplicates
    echo "Cleaning up old simulation processes..."
    pkill -9 -f 'mission_planner|boundary_manager|weed_manager|mission_monitor|rosbridge_websocket|parameter_bridge|gz.*sim' > /dev/null 2>&1
    sleep 2
    
    # Source ROS 2 and Workspace
    [ -f /opt/ros/jazzy/setup.bash ] && source /opt/ros/jazzy/setup.bash
    [ -f /home/jeevan-koiri/Desktop/drive/dev-work/droneworx/Jeevan_agentic-ai/install/setup.bash ] && source /home/jeevan-koiri/Desktop/drive/dev-work/droneworx/Jeevan_agentic-ai/install/setup.bash
    
    # Launch simulation in background (Headless mode)
    echo "Launching Gazebo and ROS nodes (Headless)..."
    ros2 launch simulation sim.launch.py gz_args:="-s -r" > simulation.log 2>&1 &
    SIM_PID=$!
    echo "Simulation started with PID $SIM_PID (Logs: simulation.log)"
    
    # Kill background processes on exit
    trap "echo 'Shutting down simulation...'; pkill -9 -f 'mission_planner|boundary_manager|weed_manager|mission_monitor|rosbridge_websocket|parameter_bridge|gz.*sim'; exit" INT TERM EXIT
else
    echo "ROS 2 not detected. Skipping simulation stack."
fi

# 4. Next.js App
echo "Starting Next.js development server..."
npm run dev
