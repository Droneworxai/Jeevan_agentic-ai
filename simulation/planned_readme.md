# Comprehensive Simulation Plan: Sandfields Farm Ecoweeder

**Engineer:** Senior Robotics Engineer (10+ YOE)
**Stack:** ROS 2 Jazzy, Gazebo Sim (Jetty), `ros_gz` Bridge

## 1. Executive Summary
This plan outlines the development of a high-fidelity simulation for the Ecoweeder bot, replicating the operational environment of Sandfields Farm Ltd. The simulation focuses on autonomous Boustrophedon navigation and dynamic interaction with the environment (weeding).

## 2. Environment Architecture (`worlds/farm.sdf`)
The world is designed to mimic the large-scale agricultural rows seen at Manor Farm Luddington.
- **Ground Plane**: Integrated `RoboCup 3D Simulation Field` from Fuel to provide a professional, bounded simulation area.
- **Crop Topology**: 
    - Parallel rows of crops (Onion/Leek proxies) spaced at 1.0m intervals.
    - Each row extends 10m to simulate long-field operations.
- **Weed Distribution**: `grasspatch` models from Fuel randomly distributed within the crop rows.
- **Atmospherics**: High-intensity directional lighting with shadow mapping to simulate midday farm conditions.

## 3. Robot Integration (`explorer_r2`)
- **Model**: `EXPLORER_R2_VISUALS_ONLY` SDF model.
- **Control**: Differential drive system configured via `gz-sim-diff-drive-system`.
- **Feedback**: Odometry and Joint State publishing for closed-loop control in ROS 2.

## 4. Autonomous Mission Logic (`mission_planner.py`)
The mission planner implements a state-machine based **Boustrophedon Path**:
1. **State: ROW_FOLLOW**: Move straight for a defined distance (e.g., 8m).
2. **State: TURN_TRANSITION**: Execute a 90-degree turn.
3. **State: ROW_SHIFT**: Move laterally to the next row.
4. **State: TURN_ALIGN**: Execute a second 90-degree turn to align with the new row.
5. **Dynamic Weeding**: 
    - Real-time proximity monitoring between the robot's base and `grasspatch` entities.
    - Triggering the `/world/farm_world/remove` service to delete weeds upon "contact".

## 5. Implementation Roadmap
1. **Phase 1: World Construction**: Update `farm.sdf` with Fuel URIs and row logic.
2. **Phase 2: Launch Orchestration**: Configure `sim.launch.py` for SDF spawning and ROS bridging.
3. **Phase 3: Navigation Development**: Implement the Boustrophedon state machine.
4. **Phase 4: Validation**: End-to-end test of the weeding cycle.

## 6. Execution Commands
```bash
# Start Simulation
./simulation/start_sim.sh

# Start Mission
python3 ./simulation/scripts/mission_planner.py
```
