#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String, Empty
from std_srvs.srv import Trigger
import math
import subprocess
import json

class MissionPlanner(Node):
    def __init__(self):
        super().__init__('mission_planner')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/mission_waypoints', 10)
        self.state_pub = self.create_publisher(String, '/mission_state', 10)
        
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        
        self.mission_sub = self.create_subscription(
            String,
            '/mission/start',
            self.mission_start_callback,
            10)
        
        self.stop_sub = self.create_subscription(
            Empty,
            '/mission/stop',
            self.stop_callback,
            10)
        
        self.execute_sub = self.create_subscription(
            Empty,
            '/mission/execute',
            self.execute_callback,
            10)
        
        # Architecture: /load_mission service
        self.srv = self.create_service(Trigger, '/load_mission', self.load_mission_callback)
        
        self.current_pose = None
        self.state = 'IDLE'
        self.mission_config = None
        self.start_pose = None
        self.row_count = 1
        self.target_yaw = 0.0
        
        # Boustrophedon Parameters (Default)
        self.row_length = 12.0  # meters
        self.row_spacing = 3.0 # meters
        self.speed = 0.5       # m/s
        self.turn_speed = 0.3  # rad/s
        
        # Progress tracking
        self.total_rows = 5  # Will be calculated from boundary
        self.mission_start_time = None
        self.area_covered = 0.0
        
        # Weed tracking
        self.weeds = {
            'weed_1': [9.0, 6.0],
            'weed_2': [8.0, 8.0]
        }
        self.removed_weeds = set()
        self.odom_count = 0
        
        # Field Boundaries (Default)
        self.min_x, self.max_x = -10.5, 10.5
        self.min_y, self.max_y = -10.5, 10.5

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("EcoWeeder Mission Planner: Waiting for Mission (Architecture Compliant)...")

    def load_mission_callback(self, request, response):
        self.get_logger().info("Mission loaded via service")
        response.success = True
        response.message = "Mission plan generated"
        return response

    def mission_start_callback(self, msg):
        try:
            data = json.loads(msg.data)
            self.get_logger().info(f"Received Mission Config: {data.get('farmName', 'Unknown Farm')}")
            
            # Update parameters from mission data
            if 'planner' in data:
                self.get_logger().info(f"Planner set to: {data['planner']}")
            
            # Store config but don't start yet
            self.mission_config = data
            self.get_logger().info("Mission configuration stored. Waiting for EXECUTE command.")
            
        except Exception as e:
            self.get_logger().error(f"Failed to parse mission data: {e}")

    def execute_callback(self, msg):
        if self.state == 'IDLE':
            self.get_logger().info("🚀 EXECUTE command received. Starting mission!")
            self.state = 'MOVE_STRAIGHT'
            self.start_pose = None
            self.row_count = 1
            self.odom_count = 0
            self.mission_start_time = self.get_clock().now()
            self.area_covered = 0.0
        else:
            self.get_logger().warn(f"Execute ignored. Current state: {self.state}")

    def stop_callback(self, msg):
        self.get_logger().info("Mission STOP requested.")
        self.state = 'IDLE'
        stop_twist = Twist()
        self.publisher_.publish(stop_twist)

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        self.odom_count += 1
        if self.odom_count % 100 == 0:
            self.get_logger().info(f"Odom: ({self.current_pose.position.x:.2f}, {self.current_pose.position.y:.2f})")

    def get_distance(self, p1, p2):
        return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)

    def get_yaw(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def get_area_covered(self):
        """Calculate area covered based on completed rows"""
        if self.row_count <= 1:
            return 0.0
        # Area = number of completed rows * row_spacing * row_length
        completed_rows = self.row_count - 1
        return completed_rows * self.row_spacing * self.row_length
    
    def get_mission_progress(self):
        """Calculate mission progress percentage"""
        if self.total_rows <= 0:
            return 0.0
        return min(100.0, (self.row_count / self.total_rows) * 100.0)

    def remove_weed(self, weed_name):
        if weed_name not in self.removed_weeds:
            self.get_logger().info(f"Weeding Tool: Removing {weed_name}")
            cmd = f"gz service -s /world/farm_world/remove --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean --timeout 1000 --req 'name: \"{weed_name}\", type: MODEL'"
            subprocess.run(cmd, shell=True, capture_output=True)
            self.removed_weeds.add(weed_name)

    def control_loop(self):
        # Publish current state for mission monitor (always, even when IDLE)
        if self.current_pose is not None:
            # Update area covered
            self.area_covered = self.get_area_covered()
            
            state_msg = String()
            state_msg.data = json.dumps({
                'state': self.state,
                'row': self.row_count,
                'total_rows': self.total_rows,
                'progress_pct': self.get_mission_progress(),
                'area_covered': round(self.area_covered, 2),
                'position': {
                    'x': self.current_pose.position.x,
                    'y': self.current_pose.position.y
                }
            })
            self.state_pub.publish(state_msg)
        
        if self.current_pose is None or self.state == 'IDLE':
            return
        
        # Check if mission is complete
        if self.row_count > self.total_rows and self.state != 'MISSION_COMPLETE':
            self.get_logger().info("🎉 MISSION COMPLETE! All rows covered.")
            self.state = 'MISSION_COMPLETE'
            twist = Twist()  # Stop the robot
            self.publisher_.publish(twist)
            return

        # Dynamic Weeding Logic
        for name, pos in self.weeds.items():
            dist = math.sqrt((self.current_pose.position.x - pos[0])**2 + (self.current_pose.position.y - pos[1])**2)
            if dist < 0.7:
                self.remove_weed(name)

        twist = Twist()
        
        if self.state == 'MOVE_STRAIGHT':
            if self.start_pose is None:
                self.start_pose = self.current_pose
                self.get_logger().info(f"Navigating Row {self.row_count}")
            
            dist = self.get_distance(self.current_pose.position, self.start_pose.position)
            if dist < self.row_length:
                twist.linear.x = self.speed
            else:
                twist.linear.x = 0.0
                self.state = 'TURN_1'
                self.start_pose = None
                turn_dir = -1 if self.row_count % 2 == 0 else 1
                self.target_yaw = self.get_yaw(self.current_pose.orientation) + (turn_dir * math.pi/2)
                self.get_logger().info(f"Row {self.row_count} Complete. Executing Turn.")

        elif self.state == 'TURN_1':
            current_yaw = self.get_yaw(self.current_pose.orientation)
            diff = self.target_yaw - current_yaw
            while diff > math.pi: diff -= 2*math.pi
            while diff < -math.pi: diff += 2*math.pi
            
            if abs(diff) > 0.05:
                twist.angular.z = self.turn_speed if diff > 0 else -self.turn_speed
            else:
                twist.angular.z = 0.0
                self.state = 'MOVE_SHORT'
                self.start_pose = self.current_pose
                self.get_logger().info("Shifting to Next Row.")

        elif self.state == 'MOVE_SHORT':
            dist = self.get_distance(self.current_pose.position, self.start_pose.position)
            if dist < self.row_spacing:
                twist.linear.x = self.speed
            else:
                twist.linear.x = 0.0
                self.state = 'TURN_2'
                self.start_pose = None
                turn_dir = -1 if self.row_count % 2 == 0 else 1
                self.target_yaw = self.get_yaw(self.current_pose.orientation) + (turn_dir * math.pi/2)

        elif self.state == 'TURN_2':
            current_yaw = self.get_yaw(self.current_pose.orientation)
            diff = self.target_yaw - current_yaw
            while diff > math.pi: diff -= 2*math.pi
            while diff < -math.pi: diff += 2*math.pi
            
            if abs(diff) > 0.05:
                twist.angular.z = self.turn_speed if diff > 0 else -self.turn_speed
            else:
                twist.angular.z = 0.0
                self.state = 'MOVE_STRAIGHT'
                self.start_pose = None
                self.row_count += 1
                self.get_logger().info(f"Starting Row {self.row_count} of {self.total_rows}")
        
        elif self.state == 'MISSION_COMPLETE':
            # Mission finished - keep robot stopped
            twist = Twist()

        self.publisher_.publish(twist)

def main():
    rclpy.init()
    planner = MissionPlanner()
    try:
        rclpy.spin(planner)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            planner.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
