# GZ Simulation Fix Plan
**Problem**: Robot position stuck at [52.175000, -1.755000], Mission Planner shows "WAITING"

## Root Cause Analysis

### Issue 1: Static Robot Position
- **Problem**: Robot GPS coordinates are hardcoded in `useRosConnection.ts`
- **Location**: `hooks/useRosConnection.ts` line 10 sets `robotPosition` to `[52.175, -1.755]`
- **Impact**: Even when robot moves in Gazebo, web UI shows static position

### Issue 2: Mission Monitor Not Updating State
- **Problem**: Mission Monitor always publishes `state: "IDLE"` 
- **Location**: `gz_simulation/simulation/mission_monitor.py` line 20 sets `self.current_status = "IDLE"`
- **Impact**: Telemetry card always shows "Mission State: IDLE" and "Mission Planner: WAITING"

### Issue 3: Mission Monitor Doesn't Subscribe to Mission Planner State
- **Problem**: Mission Monitor has no connection to Mission Planner's actual state
- **Current**: Mission Monitor tracks weeds but not mission execution state
- **Impact**: Web UI can't show actual mission progress (MOVE_STRAIGHT, TURN_1, etc.)

## Implementation Plan

### Phase 1: Fix Robot Position Updates ✅ CRITICAL

#### File: `gz_simulation/simulation/mission_monitor.py`

**Changes Needed:**
1. **Add GPS conversion** in `odom_callback`:
   ```python
   def odom_callback(self, msg):
       self.last_odom = msg
       
       # Convert Gazebo XY to GPS lat/lon (Sandfields Farm: 52.175, -1.755)
       # Approximate: 1 meter ≈ 0.000009 degrees at this latitude
       base_lat = 52.175
       base_lon = -1.755
       meters_per_degree_lat = 111111.0
       meters_per_degree_lon = 111111.0 * math.cos(math.radians(base_lat))
       
       lat = base_lat + (msg.pose.pose.position.y / meters_per_degree_lat)
       lon = base_lon + (msg.pose.pose.position.x / meters_per_degree_lon)
       
       # Publish GPS pose
       pose_msg = PoseStamped()
       pose_msg.header = msg.header
       pose_msg.pose.position.x = lat
       pose_msg.pose.position.y = lon
       pose_msg.pose.position.z = 0.0
       self.pose_pub.publish(pose_msg)
   ```

2. **Update subscribe callback** to expect lat/lon format in web UI:
   ```typescript
   // In useRosConnection.ts - remove hardcoded default position
   const [robotPosition, setRobotPosition] = useState<[number, number] | null>(null);
   
   poseTopic.subscribe((message: any) => {
     // Message now contains GPS coordinates directly from mission_monitor
     setRobotPosition([message.pose.position.x, message.pose.position.y]);
   });
   ```

**Expected Result**: Robot icon moves on map as it drives in Gazebo

---

### Phase 2: Implement Mission State Publishing ✅ CRITICAL

#### File: `gz_simulation/simulation/mission_planner.py`

**Changes Needed:**
1. **Add state publisher**:
   ```python
   def __init__(self):
       # ... existing code ...
       self.state_pub = self.create_publisher(String, '/mission_state', 10)
   ```

2. **Publish state in control_loop**:
   ```python
   def control_loop(self):
       if self.current_pose is None:
           return
       
       # Publish current state
       state_msg = String()
       state_msg.data = json.dumps({
           'state': self.state,
           'row': self.row_count,
           'position': {
               'x': self.current_pose.position.x,
               'y': self.current_pose.position.y
           }
       })
       self.state_pub.publish(state_msg)
       
       # ... rest of control logic ...
   ```

#### File: `gz_simulation/simulation/mission_monitor.py`

**Changes Needed:**
1. **Subscribe to mission state**:
   ```python
   def __init__(self):
       # ... existing subscriptions ...
       self.mission_state_sub = self.create_subscription(
           String, 
           '/mission_state', 
           self.mission_state_callback, 
           10
       )
   
   def mission_state_callback(self, msg):
       try:
           data = json.loads(msg.data)
           self.current_status = data.get('state', 'IDLE')
           self.current_row = data.get('row', 0)
       except:
           pass
   ```

2. **Update status message**:
   ```python
   def publish_status(self):
       status = {
           "state": self.current_status,  # Now reflects actual mission state
           "row": self.current_row,
           "weeds_removed": self.weeds_removed,
           "position": { ... }
       }
       self.status_pub.publish(String(data=json.dumps(status)))
   ```

**Expected Result**: Telemetry shows "MOVE_STRAIGHT", "TURN_1", etc. instead of "IDLE"

---

### Phase 3: Add Progress Tracking 🔸 IMPORTANT

#### File: `gz_simulation/simulation/mission_planner.py`

**Changes Needed:**
1. **Track mission progress**:
   ```python
   def __init__(self):
       # ... existing code ...
       self.total_rows = 5  # Calculate from boundary
       self.mission_start_time = None
   
   def execute_callback(self, msg):
       self.state = 'MOVE_STRAIGHT'
       self.mission_start_time = self.get_clock().now()
       self.get_logger().info("Mission STARTED")
   ```

2. **Calculate area covered**:
   ```python
   def get_area_covered(self):
       if self.row_count <= 1:
           return 0.0
       return (self.row_count - 1) * self.row_spacing * self.row_length
   ```

3. **Add to state broadcast**:
   ```python
   state_msg.data = json.dumps({
       'state': self.state,
       'row': self.row_count,
       'total_rows': self.total_rows,
       'progress_pct': (self.row_count / self.total_rows) * 100,
       'area_covered': self.get_area_covered(),
       # ... position ...
   })
   ```

**Expected Result**: Web UI shows actual area covered and progress percentage

---

### Phase 4: Fix Mission Completion Detection 🔸 IMPORTANT

#### File: `gz_simulation/simulation/mission_planner.py`

**Changes Needed:**
1. **Detect mission end**:
   ```python
   def control_loop(self):
       # ... existing state machine ...
       
       # Check if mission complete
       if self.row_count > self.total_rows:
           if self.state != 'MISSION_COMPLETE':
               self.get_logger().info("🎉 MISSION COMPLETE!")
               self.state = 'MISSION_COMPLETE'
               twist = Twist()  # Stop the robot
               self.publisher_.publish(twist)
               return
       
       # Continue with normal state machine
       if self.state == 'MOVE_STRAIGHT':
           # ... existing logic ...
   ```

**Expected Result**: Robot stops automatically after completing all rows

---

### Phase 5: Improve Odometry Reliability 🔹 ENHANCEMENT

#### Problem: 
`/odom` topic might not be publishing from Gazebo bridge

#### File: `gz_simulation/launch/sim.launch.py`

**Verify bridge arguments include:**
```python
arguments=[
    '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
    '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',  # ✓ Already present
    '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',   # Add this for time sync
]
```

#### Test Command:
```bash
# Terminal 1: Launch sim
cd gz_simulation
./start_sim.sh

# Terminal 2: Check topics
ros2 topic list | grep odom
ros2 topic hz /odom  # Should show ~100 Hz
ros2 topic echo /odom --once
```

---

## Implementation Priority

### 🔴 CRITICAL (Do First):
1. **Fix GPS conversion** in mission_monitor.py → Enables robot movement visualization
2. **Add state publishing** in mission_planner.py → Shows actual mission status
3. **Remove hardcoded position** in useRosConnection.ts → Allows dynamic updates

### 🟡 IMPORTANT (Do Second):
4. **Add progress tracking** → Better user feedback
5. **Implement completion detection** → Auto-stop when done

### 🟢 ENHANCEMENT (Optional):
6. **Add time synchronization** via `/clock` topic
7. **Implement boundary-aware path planning** from web UI boundary data
8. **Add emergency stop** if robot goes out of bounds

---

## Testing Checklist

### Step 1: Verify Gazebo is Running
```bash
gz topic -l  # Should show /world/farm_world/...
gz topic -e -t /odom  # Should show pose updates
```

### Step 2: Verify ROS Bridge
```bash
ros2 topic list  # Should show /odom, /cmd_vel, /robot_pose, etc.
ros2 topic hz /robot_pose  # Should show ~10 Hz after fix
```

### Step 3: Test Web UI
1. Open http://localhost:3000/dashboard/simulation
2. Connect to ROS (green status indicator)
3. Draw boundary → Click "Set Boundary" (should succeed)
4. Click "Load Mission" (should succeed)
5. Click "EXECUTE MISSION"
6. **Expected**: 
   - ✅ Robot icon moves on map
   - ✅ Telemetry shows "MOVE_STRAIGHT", "TURN_1", etc.
   - ✅ ROS Terminal logs show position updates
   - ✅ Area covered increases

---

## Quick Fix Summary

### Minimal changes to get it working NOW:

**File 1: `gz_simulation/simulation/mission_monitor.py`**
- Add GPS conversion math to `odom_callback`
- Change lat/lon publishing format

**File 2: `hooks/useRosConnection.ts`**
- Remove `[52.175, -1.755]` default
- Set to `null` initially
- Update immediately when message received

**File 3: `gz_simulation/simulation/mission_planner.py`** 
- Add `/mission_state` publisher
- Broadcast state in every control loop iteration

**These 3 changes will fix both major issues!**

---

## Expected Timeline

- **Quick Fix (Files 1-3)**: 30 minutes
- **Testing & Debugging**: 1 hour  
- **Progress Features**: 1 hour
- **Polish & Documentation**: 30 minutes

**Total: ~3 hours to fully working simulation**

---

## Notes

- Current robot starts at Gazebo position `[9.6532, 4.6353, 0.0094]`
- This maps to GPS: `[52.175 + 4.6353/111111, -1.755 + 9.6532/111111]`
- Approximate: `[52.175042, -1.754913]` → Should see robot north-east of center
- Robot model from Gazebo Fuel: `EXPLORER_R2_VISUALS_ONLY`
- Differential drive with 1.2m wheel separation, 0.3m wheel radius
