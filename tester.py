import time
import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.duration import Duration

rclpy.init()
node = Node("tf_test_node")
tf_buf = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buf, node)
time.sleep(0.5)  # wait for TFs to arrive

can = tf_buf.can_transform("odom", "base_link", rclpy.time.Time(), timeout=Duration(seconds=0.5))
print("can_transform odom->base_link:", int(can))
if can:
    trans = tf_buf.lookup_transform("odom", "base_link", rclpy.time.Time())
    print("translation:", trans.transform.translation.x, trans.transform.translation.y)
rclpy.shutdown()