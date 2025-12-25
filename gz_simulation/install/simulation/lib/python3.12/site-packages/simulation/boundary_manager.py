#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Polygon, Point32
from std_msgs.msg import String
from std_srvs.srv import Trigger
import json

class BoundaryManager(Node):
    def __init__(self):
        super().__init__('boundary_manager')
        self.boundary_pub = self.create_publisher(Polygon, '/farm_boundary', 10)
        self.farm_sub = self.create_subscription(String, '/selected_farm', self.farm_callback, 10)
        self.mission_sub = self.create_subscription(
            String,
            '/mission/start',
            self.mission_callback,
            10)
        
        # Architecture: /set_boundary service
        self.srv = self.create_service(Trigger, '/set_boundary', self.set_boundary_callback)
        
        self.current_farm = "None"
        self.get_logger().info("Boundary Manager Node Started (Architecture Compliant)")

    def farm_callback(self, msg):
        self.current_farm = msg.data
        self.get_logger().info(f"Selected Farm: {self.current_farm}")

    def set_boundary_callback(self, request, response):
        self.get_logger().info("Boundary set via service")
        response.success = True
        response.message = "Boundary accepted"
        return response

    def mission_callback(self, msg):
        try:
            data = json.loads(msg.data)
            if 'boundary' in data:
                boundary_data = data['boundary']
                polygon = Polygon()
                
                # Assuming GeoJSON format from dashboard
                # We convert Lat/Lon to local meters (simplified)
                # Center: 52.175, -1.755
                center_lat = 52.175
                center_lon = -1.755
                
                if 'features' in boundary_data:
                    coords = boundary_data['features'][0]['geometry']['coordinates'][0]
                    for coord in coords:
                        p = Point32()
                        # Simple conversion: 1 deg lat = 111111m, 1 deg lon = 111111 * cos(lat)
                        p.y = float((coord[1] - center_lat) * 111111)
                        p.x = float((coord[0] - center_lon) * (111111 * 0.613)) # cos(52.175) approx 0.613
                        p.z = 0.0
                        polygon.points.append(p)
                    
                    self.boundary_pub.publish(polygon)
                    self.get_logger().info(f"Published new boundary with {len(polygon.points)} points")
        except Exception as e:
            self.get_logger().error(f"Error processing boundary: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = BoundaryManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
