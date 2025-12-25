#!/bin/bash

# Source ROS 2 and workspace
source /opt/ros/jazzy/setup.bash
source /home/jeevan-koiri/Desktop/drive/dev-work/droneworx/Jeevan_agentic-ai/install/setup.bash

echo "--- Phase 6: Mission Execution Integration Test ---"

# 1. Set Boundary
echo "Step 1: Setting Boundary..."
ros2 topic pub --once /mission/start std_msgs/msg/String "{data: '{\"farmName\":\"Sandfields Farm Ltd\",\"boundary\":{\"type\":\"FeatureCollection\",\"features\":[{\"type\":\"Feature\",\"properties\":{},\"geometry\":{\"type\":\"Polygon\",\"coordinates\":[[[-1.755,52.175],[-1.754,52.175],[-1.754,52.176],[-1.755,52.176],[-1.755,52.175]]]}}]},\"planner\":\"boustrophedon\"}'}"
ros2 service call /set_boundary std_srvs/srv/Trigger {}

# 2. Load Mission
echo "Step 2: Loading Mission..."
ros2 service call /load_mission std_srvs/srv/Trigger {}

# 3. Execute Mission
echo "Step 3: Executing Mission..."
ros2 topic pub --once /mission/execute std_msgs/msg/Empty {}

# 4. Monitor Progress
echo "Step 4: Monitoring Mission Status and Odometry (15 seconds)..."
# Check if robot is moving by looking at odom
timeout 15s ros2 topic echo --once /odom | grep -A 5 "position"
timeout 10s ros2 topic echo --once /mission_status

# 5. Abort Mission
echo "Step 5: Aborting Mission..."
ros2 topic pub --once /mission/stop std_msgs/msg/Empty {}

echo "--- Test Complete ---"
