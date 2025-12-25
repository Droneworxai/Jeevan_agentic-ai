import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time

class TestEcoweeder(Node):
    def __init__(self):
        super().__init__('test_ecoweeder')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        self.last_odom = None

    def odom_callback(self, msg):
        self.last_odom = msg

    def run_test(self):
        print("Sending move command...")
        twist = Twist()
        twist.linear.x = 0.5
        self.publisher_.publish(twist)
        
        start_time = time.time()
        while time.time() - start_time < 5:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.last_odom:
                pos = self.last_odom.pose.pose.position
                print(f"Current Position: x={pos.x:.2f}, y={pos.y:.2f}")
        
        print("Stopping...")
        self.publisher_.publish(Twist())
        rclpy.spin_once(self, timeout_sec=1.0)

def main():
    rclpy.init()
    test = TestEcoweeder()
    try:
        test.run_test()
    except KeyboardInterrupt:
        pass
    finally:
        test.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
