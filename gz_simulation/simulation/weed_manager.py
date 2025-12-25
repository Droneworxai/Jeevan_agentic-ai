#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Empty
import math
import subprocess
import json

class WeedManager(Node):
    def __init__(self):
        super().__init__('weed_manager')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        self.status_pub = self.create_publisher(String, '/weed_status', 10)
        
        # Architecture: /reset_farm service
        self.srv = self.create_service(Empty, '/reset_farm', self.reset_farm_callback)
        
        # Weed coordinates from farm.sdf
        self.weeds = {
            'weed_1': [9.0, 6.0],
            'weed_2': [8.0, 8.0]
        }
        self.removed_weeds = set()
        self.get_logger().info("Weed Manager Node Started (Architecture Compliant)")

    def reset_farm_callback(self, request, response):
        self.get_logger().info("Resetting farm weeds...")
        self.removed_weeds.clear()
        return response

    def odom_callback(self, msg):
        curr_x = msg.pose.pose.position.x
        curr_y = msg.pose.pose.position.y
        
        for name, pos in self.weeds.items():
            if name in self.removed_weeds:
                continue
                
            dist = math.sqrt((curr_x - pos[0])**2 + (curr_y - pos[1])**2)
            if dist < 0.7: # Weeding radius
                self.remove_weed(name)

    def remove_weed(self, weed_name):
        self.get_logger().info(f"Removing weed: {weed_name}")
        cmd = f"gz service -s /world/farm_world/remove --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean --timeout 1000 --req 'name: \"{weed_name}\", type: MODEL'"
        subprocess.run(cmd, shell=True, capture_output=True)
        self.removed_weeds.add(weed_name)
        
        # Publish status update
        status = {
            "event": "weed_removed",
            "name": weed_name,
            "total_removed": len(self.removed_weeds)
        }
        self.status_pub.publish(String(data=json.dumps(status)))

def main(args=None):
    rclpy.init(args=args)
    node = WeedManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
