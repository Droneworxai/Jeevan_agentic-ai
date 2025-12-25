# EcoWeeder Troubleshooting Guide

This guide helps resolve common issues with the ROS 2 simulation and Next.js dashboard integration.

## 1. ROS Bridge Connection Issues
**Symptoms:** Dashboard shows `ROS: disconnected` or `ROS: error`.

- **Check if simulation is running:**
  ```bash
  ros2 node list
  ```
  You should see `eco_` nodes and `rosbridge_websocket`.
- **Check Port 9090:**
  ```bash
  sudo lsof -i :9090
  ```
  If nothing is listening, the rosbridge server failed to start.
- **Check Logs:**
  Look at the terminal where you ran the launch file for `[rosbridge_websocket]: Rosbridge WebSocket server started on port 9090`.

## 2. Gazebo Simulation Issues
**Symptoms:** Robot doesn't move, or Gazebo crashes on startup.

- **GUI Symbol Lookup Error:**
  If you see `undefined symbol: __libc_pthread_init`, run Gazebo in headless mode:
  ```bash
  ros2 launch simulation sim.launch.py gz_args:="-s -r"
  ```
- **Robot Not Spawning:**
  Ensure you have an active internet connection for the first run, as Gazebo Fuel needs to download the `explorer_r2_visuals_only` model.
- **Slow Performance:**
  Gazebo is resource-intensive. Close other heavy applications.

## 3. Mission Execution Issues
**Symptoms:** "Execute Mission" button clicked but robot stays idle.

- **Check Mission State:**
  Look at the "Live Telemetry" card. If it says `IDLE`, ensure you have:
  1. Drawn a boundary.
  2. Clicked **Set Boundary**.
  3. Clicked **Load Mission**.
- **Check Topic Echo:**
  Verify the execute command is being sent:
  ```bash
  ros2 topic echo /mission/execute
  ```

## 4. Build Errors
**Symptoms:** `colcon build` fails.

- **Missing Dependencies:**
  Ensure all ROS 2 Jazzy dependencies are installed:
  ```bash
  rosdep install -i --from-path src --rosdistro jazzy -y
  ```
- **Python Errors:**
  Ensure you are using Python 3.12+ and have `setuptools` installed.

## 5. Map Issues
**Symptoms:** Robot marker not appearing or map is blank.

- **Internet Connection:** Leaflet requires an internet connection to load OpenStreetMap tiles.
- **Coordinate Offset:** The simulation uses a local coordinate system centered at `52.175, -1.755`. Ensure your drawn boundary is near this location.
