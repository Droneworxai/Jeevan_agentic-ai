# Mission Planning and Execution Plan (Plan 2) - Refined

This plan outlines the implementation of a high-fidelity mission control system for the EcoWeeder autonomous robot, integrating a Next.js full-stack dashboard with a ROS 2 / Gazebo Ignition simulation environment.

## 1. GIS-Integrated Farm Selection
- **Target Location**: Sandfields Farm Ltd, Manor Farm, Luddington, United Kingdom (Approx. `52.175, -1.755`).
- **UI Implementation**: Enhance `app/dashboard/page.tsx` with a GIS-centric view.
- **Data Layer**: Use Prisma to manage `Farm` entities in MongoDB, storing GeoJSON polygons for field boundaries.
- **Map Engine**: Initialize `mission-control-map.tsx` using Leaflet/Mapbox centered on the Luddington site.

## 2. ROS 2 Architecture & Node Implementation
We will implement a modular ROS 2 architecture as follows:

### Nodes & Responsibilities
- **`boundary_manager_node`**: Manages farm boundaries; provides `/set_boundary` service and publishes `/farm_boundary` (Polygon).
- **`mission_planner_node`**: Generates coverage paths (Boustrophedon) and publishes `/mission_waypoints` (Path).
- **`robot_controller_node`**: Pure pursuit or PID controller to follow waypoints, publishing `/cmd_vel`.
- **`weed_manager_node`**: Monitors robot position relative to weed entities and triggers removal via Gazebo services.
- **`mission_monitor_node`**: Aggregates telemetry and publishes `/mission_status` (JSON string) for the dashboard.
- **`gazebo_bridge_node`**: Standard bridge for `/odom`, `/cmd_vel`, and `/tf`.

### Communication Interface (Topics & Services)
- **Topics**:
    - `/farm_boundary` (`geometry_msgs/Polygon`)
    - `/mission_waypoints` (`nav_msgs/Path`)
    - `/robot_pose` (`geometry_msgs/PoseStamped`)
    - `/mission_status` (`std_msgs/String`)
    - `/weed_status` (`std_msgs/String` - JSON)
- **Services**:
    - `/set_boundary` (Custom/Standard)
    - `/load_mission` (Custom/Standard)
- **Actions**:
    - `/execute_mission`: For long-running path following with feedback.

## 3. Precision Geo-Fencing & Path Planning
- **Interactive Boundary Definition**: Implement a drawing layer using `L.Draw` or `Mapbox Draw` to define operational zones (Geo-fencing).
- **Mission Planner Algorithms**:
    - **Boustrophedon Coverage**: Optimized lawnmower patterns for row crops.
    - **Waypoints**: Manual GPS coordinate injection for targeted weeding.
- **Coordinate Transformation**: Convert WGS84 (GPS) coordinates to local ENU (East-North-Up) coordinates for the Gazebo simulation.

## 4. Robotic Middleware Bridge (ROS 2 / Web)
- **Communication Layer**: Use `rosbridge_suite` to expose ROS 2 topics/services to the Next.js frontend via WebSockets.
- **Dynamic Parameter Reconfiguration**: Use ROS 2 parameters to dynamically update nodes with new boundaries and waypoints.
- **Lifecycle Management**: Trigger `ros2 launch simulation sim.launch.py` with environment variables for specific farm configurations.

## 5. Real-Time Telemetry & Digital Twin
- **Telemetry Stream**: Subscribe to `/odom` (Odometry) and `/gps/fix` topics.
- **State Estimation**: Visualize the robot's pose (Position + Orientation) on the dashboard map in real-time.
- **Digital Twin Sync**: Ensure the Gazebo robot's movement is mirrored exactly on the web-based GIS interface.

## 6. Autonomous Mission Execution & Analytics
- **Mission Logic**: Implement a state machine (Idle -> Navigating -> Weeding -> Returning).
- **Safety Interlocks**: Automatic "Stop" command if the robot breaches the defined Geo-fence.
- **Post-Mission Analytics**: Store mission paths and weeding events in MongoDB; generate reports on "Weeds Neutralized" vs "Area Covered".
