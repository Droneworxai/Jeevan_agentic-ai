# EcoWeeder Agentic AI System - Complete Guide

## System Overview
This is a full-stack autonomous weeding robot control system with:
- **Frontend**: Next.js dashboard with real-time telemetry
- **Backend**: ROS 2 Jazzy nodes for mission control
- **Simulation**: Gazebo Ignition with EcoWeeder robot model
- **Location**: Sandfields Farm Ltd, Luddington, UK

## Quick Start

### 1. Start the Simulation
```bash
cd simulation
./quick_start.sh
```
This will launch:
- Gazebo simulation with the farm world
- ROS 2 bridge for communication
- Rosbridge WebSocket server
- All mission control nodes (boundary manager, mission planner, weed manager, mission monitor)

### 2. Start the Dashboard
```bash
npm run dev
```
Access the dashboard at: `http://localhost:3000`

### 3. Navigate to Simulation
Click "Launch Simulation" or go to: `http://localhost:3000/simulation`

## Architecture

### ROS 2 Nodes
- **boundary_manager_node**: Manages farm boundaries, publishes `/farm_boundary`
- **mission_planner_node**: Generates waypoints, publishes `/mission_waypoints`
- **weed_manager_node**: Detects and removes weeds, publishes `/weed_status`
- **mission_monitor_node**: Aggregates telemetry, publishes `/mission_status` and `/robot_pose`

### Topics
- `/selected_farm`: Current farm selection (String)
- `/farm_boundary`: Boundary polygon (Polygon)
- `/mission_waypoints`: Path to follow (Path)
- `/robot_pose`: Robot position (PoseStamped)
- `/mission_status`: Mission state JSON (String)
- `/weed_status`: Weeding events JSON (String)
- `/cmd_vel`: Velocity commands (Twist)
- `/odom`: Odometry data (Odometry)

### Services
- `/set_boundary`: Set farm boundaries
- `/load_mission`: Load mission parameters
- `/reset_farm`: Reset weed status

## Usage

### Web Dashboard Features
1. **Farm Selection**: Choose from predefined farms
2. **Boundary Definition**: Draw operational zones on the map
3. **Mission Planning**: Select algorithms (Boustrophedon, Spiral, Waypoints)
4. **Real-time Monitoring**: Live GPS position and mission status
5. **Manual Control**: Direct robot commands for testing

### Mission Workflow
1. Select "Sandfields Farm Ltd" from dropdown
2. Click "Set Boundary" to sync the operational area
3. Click "Load Mission" to prepare the path planner
4. Click "EXECUTE MISSION" to start autonomous operation
5. Monitor live telemetry in the sidebar
6. Click "ABORT MISSION" to stop

## Troubleshooting

### ROS Bridge Not Connected
- Ensure `rosbridge_server` is running: `ros2 launch rosbridge_server rosbridge_websocket_launch.xml`
- Check WebSocket URL: Default is `ws://localhost:9090`

### Gazebo Not Starting
- Run from a native terminal (not VS Code integrated terminal if using Snap)
- Source ROS 2: `source /opt/ros/jazzy/setup.bash`

### Nodes Not Publishing
- Check if nodes are running: `ros2 node list`
- Check topics: `ros2 topic list`
- View topic data: `ros2 topic echo /mission_status`

## File Structure
```
simulation/
├── scripts/
│   ├── mission_planner.py
│   ├── boundary_manager.py
│   ├── weed_manager.py
│   └── mission_monitor.py
├── launch/
│   └── sim.launch.py
├── robot_description/
│   └── urdf/
│       └── explorer_r2.urdf
├── worlds/
│   └── farm.sdf
├── start_sim.sh
└── quick_start.sh

app/
├── simulation/
│   └── page.tsx
└── dashboard/
    └── mission-control/
        └── page.tsx
```

## Development

### Adding New Missions
Edit `mission_planner.py` to add new path planning algorithms.

### Custom Boundaries
Use the dashboard drawing tools to define custom operational zones.

### Extending the System
- Add new ROS 2 nodes in `simulation/scripts/`
- Register them in `setup.py` entry points
- Update `start_sim.sh` to launch them

## Technical Stack
- **Simulation**: Gazebo Harmonic + ROS 2 Jazzy
- **Frontend**: Next.js 14 + TypeScript + Tailwind CSS
- **Maps**: Leaflet + OpenStreetMap
- **ROS Bridge**: rosbridge_suite
- **Communication**: WebSocket (rosbridge)
