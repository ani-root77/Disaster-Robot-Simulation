import rclpy
from sensor_msgs.msg import LaserScan

def callback(msg):
    print(f"Laser array length: {len(msg.ranges)}")
    rclpy.shutdown()

rclpy.init()
node = rclpy.create_node('scan_listener')
node.create_subscription(LaserScan, '/scan', callback, 10)
rclpy.spin(node)
