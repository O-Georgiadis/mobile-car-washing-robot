import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LidarTestNode(Node):
    def __init__(self):
        Node.__init__(self, "lidar_test_node")
        self.subscriber_ = self.create_subscription(
            LaserScan, '/scan', self.callback_log_sensor, 10
        )
        self.get_logger().info("LiDARTestNode started")

    def callback_log_sensor(self, msg):
        self.get_logger().info(f"Ranges: {msg.ranges}, min: {msg.range_min}, max: {msg.range_max}")


def main(args=None):
    rclpy.init(args=args)
    node = LidarTestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

