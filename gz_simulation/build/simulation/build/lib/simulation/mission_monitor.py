#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import json
import math

class MissionMonitor(Node):
    def __init__(self):
        super().__init__('mission_monitor')
        self.status_pub = self.create_publisher(String, '/mission_status', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/robot_pose', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.weed_sub = self.create_subscription(String, '/weed_status', self.weed_callback, 10)
        self.mission_state_sub = self.create_subscription(
            String, 
            '/mission_state', 
            self.mission_state_callback, 
            10
        )
        
        self.current_status = "IDLE"
        self.current_row = 0
        self.total_rows = 0
        self.progress_pct = 0.0
        self.area_covered = 0.0
        self.weeds_removed = 0
        self.last_odom = None
        
        # GPS conversion constants (Sandfields Farm Ltd: 52.175°N, 1.755°W)
        self.base_lat = 52.175
        self.base_lon = -1.755
        self.meters_per_degree_lat = 111111.0
        self.meters_per_degree_lon = 111111.0 * math.cos(math.radians(self.base_lat))
        
        # Throttled publishing timer (5Hz)
        self.timer = self.create_timer(0.2, self.publish_status)
        
        self.get_logger().info("Mission Monitor Node Started (Architecture Compliant)")

    def odom_callback(self, msg):
        self.last_odom = msg
        
        # Convert Gazebo XY meters to GPS lat/lon coordinates
        # Gazebo origin (0, 0) maps to base GPS coordinates
        gazebo_x = msg.pose.pose.position.x
        gazebo_y = msg.pose.pose.position.y
        
        # Calculate GPS coordinates
        lat = self.base_lat + (gazebo_y / self.meters_per_degree_lat)
        lon = self.base_lon + (gazebo_x / self.meters_per_degree_lon)
        
        # Publish GPS pose to web UI
        pose_msg = PoseStamped()
        pose_msg.header = msg.header
        pose_msg.header.frame_id = "gps"
        pose_msg.pose.position.x = lat
        pose_msg.pose.position.y = lon
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation = msg.pose.pose.orientation
        self.pose_pub.publish(pose_msg)
        
        # Log position periodically for debugging
        if not hasattr(self, 'pose_log_count'):
            self.pose_log_count = 0
        self.pose_log_count += 1
        if self.pose_log_count % 50 == 0:  # Every ~0.5 seconds at 100Hz
            self.get_logger().info(
                f"Robot GPS: [{lat:.6f}, {lon:.6f}] | "
                f"Gazebo: [{gazebo_x:.2f}, {gazebo_y:.2f}]"
            )

    def publish_status(self):
        if self.last_odom is None:
            return

        # Periodically publish mission status with actual state from mission planner
        status = {
            "state": self.current_status,
            "row": self.current_row,
            "total_rows": self.total_rows,
            "progress_pct": self.progress_pct,
            "area_covered": self.area_covered,
            "weeds_removed": self.weeds_removed,
            "position": {
                "x": self.last_odom.pose.pose.position.x,
                "y": self.last_odom.pose.pose.position.y
            }
        }
        self.status_pub.publish(String(data=json.dumps(status)))

    def mission_state_callback(self, msg):
        """Subscribe to mission planner's actual state"""
        try:
            data = json.loads(msg.data)
            self.current_status = data.get('state', 'IDLE')
            self.current_row = data.get('row', 0)
            self.total_rows = data.get('total_rows', 0)
            self.progress_pct = data.get('progress_pct', 0.0)
            self.area_covered = data.get('area_covered', 0.0)
        except Exception as e:
            self.get_logger().error(f"Failed to parse mission state: {e}")

    def weed_callback(self, msg):
        try:
            data = json.loads(msg.data)
            if data.get("event") == "weed_removed":
                self.weeds_removed = data.get("total_removed", self.weeds_removed)
                self.current_status = "WEEDING"
        except:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = MissionMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
