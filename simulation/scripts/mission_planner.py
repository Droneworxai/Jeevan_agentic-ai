import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import subprocess

class MissionPlanner(Node):
    def __init__(self):
        super().__init__('mission_planner')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        
        self.current_pose = None
        self.state = 'MOVE_STRAIGHT'
        self.start_pose = None
        self.row_count = 1
        self.state = 'MOVE_STRAIGHT'
        self.target_yaw = 0.0
        
        # Boustrophedon Parameters
        self.row_length = 16.0  # meters
        self.row_spacing = 2.0 # meters
        self.speed = 0.5       # m/s
        self.turn_speed = 0.3  # rad/s
        
        # Weed tracking (Coordinates from farm.sdf)
        self.weeds = {
            'weed_1': [-6.0, -6.5],
            'weed_2': [-4.0, -6.5]
        }
        self.removed_weeds = set()
        self.odom_count = 0
        
        # Field Boundaries (20x20 centered at 0,0)
        self.min_x, self.max_x = -9.5, 9.5
        self.min_y, self.max_y = -9.5, 9.5

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Senior Mission Planner: Boustrophedon Path Active")

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        self.odom_count += 1
        if self.odom_count % 50 == 0:
            self.get_logger().info(f"Odom: ({self.current_pose.position.x:.2f}, {self.current_pose.position.y:.2f})")

    def get_distance(self, p1, p2):
        return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)

    def get_yaw(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def remove_weed(self, weed_name):
        if weed_name not in self.removed_weeds:
            self.get_logger().info(f"Weeding Tool: Removing {weed_name}")
            # Service call to Gazebo to remove the entity
            cmd = f"gz service -s /world/farm_world/remove --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean --timeout 1000 --req 'name: \"{weed_name}\", type: MODEL'"
            subprocess.run(cmd, shell=True, capture_output=True)
            self.removed_weeds.add(weed_name)

    def control_loop(self):
        if self.current_pose is None or self.odom_count < 10:
            return

        # Boundary Safety Check
        curr_x = self.current_pose.position.x
        curr_y = self.current_pose.position.y
        if curr_x < self.min_x or curr_x > self.max_x or curr_y < self.min_y or curr_y > self.max_y:
            self.get_logger().error(f"BOUNDARY BREACH! Stopping at ({curr_x:.2f}, {curr_y:.2f})")
            stop_twist = Twist()
            self.publisher_.publish(stop_twist)
            return

        # Dynamic Weeding Logic
        for name, pos in self.weeds.items():
            dist = math.sqrt((self.current_pose.position.x - pos[0])**2 + (self.current_pose.position.y - pos[1])**2)
            if dist < 0.7: # Effective weeding radius
                self.remove_weed(name)

        twist = Twist()
        
        if self.state == 'MOVE_STRAIGHT':
            if self.start_pose is None:
                self.start_pose = self.current_pose
                self.get_logger().info(f"Navigating Row {self.row_count}")
            
            dist = self.get_distance(self.current_pose.position, self.start_pose.position)
            if dist < self.row_length:
                twist.linear.x = self.speed
                if self.odom_count % 50 == 0: # Log every 5 seconds approx
                    self.get_logger().info(f"Row {self.row_count}: Dist {dist:.2f}m, Pose ({self.current_pose.position.x:.2f}, {self.current_pose.position.y:.2f})")
            else:
                twist.linear.x = 0.0
                self.state = 'TURN_1'
                self.start_pose = None
                # Alternate turn direction for Boustrophedon
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
