# Ecoweeder Simulation

This folder contains the ROS 2 Jazzy and Gazebo Sim (Jetty) simulation for the Ecoweeder bot.

## Structure

- `robot_description/urdf/bot.urdf`: The robot model.
- `worlds/farm.sdf`: The farm environment.
- `launch/sim.launch.py`: The ROS 2 launch file.
- `scripts/mission_planner.py`: (Placeholder) Script for autonomous weeding.
- `start_sim.sh`: Script to start the simulation.

## How to Run

To start the simulation in headless mode (recommended due to library conflicts in this environment):

```bash
./start_sim.sh
```

The simulation will start Gazebo Sim in the background and bridge the following topics:
- `/cmd_vel`: Control the robot's movement.
- `/odom`: Get the robot's odometry.
- `/tf`: Transform information.

## Verifying the Simulation

In a new terminal, source ROS 2 and check the topics:

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list
ros2 topic echo /odom
```

To move the robot:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

## Note on GUI

Due to a conflict between ROS 2 and Snap-installed Gazebo libraries (`libpthread.so.0`), the simulation defaults to headless mode. If you wish to try running with the GUI, you can modify `launch/sim.launch.py` and set `headless` to `false`, but it may crash with a symbol lookup error.
