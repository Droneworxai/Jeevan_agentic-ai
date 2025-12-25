#!/bin/bash

# IMPORTANT in Linux: Run chmod +x start.sh then ./start.sh

echo "Checking for Node.js..."
node -v > /dev/null 2>&1 || { echo "Node.js is not installed. Please install it first."; exit 1; }

# Check for .env file
if [ ! -f .env ]; then
    echo ".env file not found. Creating from .env.example..."
    if [ -f .env.example ]; then
        cp .env.example .env
        echo "Please edit .env with your DATABASE_URL and other secrets."
    else
        echo "DATABASE_URL=\"mongodb://localhost:27017/ecoweeder\"" > .env
        echo "NEXT_PUBLIC_GEMINI_API_KEY=\"\"" >> .env
        echo "Created a basic .env file. Please update it with your actual credentials."
    fi
fi

echo "Installing dependencies..."
if [ -f package-lock.json ]; then
    npm ci
else
    npm install
fi

echo "Generating Prisma client..."
npx prisma generate

# Check for ROS Bridge
if command -v ros2 &> /dev/null; then
    echo "ROS 2 detected. Would you like to start the ROS Bridge? (y/n)"
    read -r start_ros
    if [ "$start_ros" = "y" ]; then
        echo "Starting ROS Bridge in background..."
        # Source ROS if not already sourced (common paths)
        [ -f /opt/ros/jazzy/setup.bash ] && source /opt/ros/jazzy/setup.bash
        [ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash
        [ -f /opt/ros/foxy/setup.bash ] && source /opt/ros/foxy/setup.bash
        
        ros2 launch rosbridge_server rosbridge_websocket_launch.xml > /dev/null 2>&1 &
        ROS_PID=$!
        echo "ROS Bridge started with PID $ROS_PID"
        
        echo "Starting Mission Planner node..."
        python3 simulation/scripts/mission_planner.py > /dev/null 2>&1 &
        PLANNER_PID=$!
        
        # Kill background processes on exit
        trap "kill $ROS_PID $PLANNER_PID; exit" INT TERM EXIT
    fi
fi

echo "Starting development server..."
npm run dev