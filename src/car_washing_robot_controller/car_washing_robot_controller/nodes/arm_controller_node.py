#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class ArmControllerNode(Node):
    def __init__(self):
        Node.__init__(self, 'arm_controller_node')

        self.joint1_pub = self.create_publisher(Float64, '/joint1/cmd_pos', 10)
        self.joint2_pub = self.create_publisher(Float64, '/joint2/cmd_pos', 10)
        self.joint3_pub = self.create_publisher(Float64, '/joint3/cmd_pos', 10)
        self.joint4_pub = self.create_publisher(Float64, '/joint4/cmd_pos', 10)
        self.joint5_pub = self.create_publisher(Float64, '/joint5/cmd_pos', 10)
        self.joint6_pub = self.create_publisher(Float64, '/joint6/cmd_pos', 10)

        self.timer = self.create_timer(1.0, self.move_to_home)
        self.home_executed = False

        self.get_logger().info('Arm controller node started')

    def move_to_home(self):
        if not self.home_executed:
            self.get_logger().info('Moving to HOME position')
            msg = Float64()
            msg.data = 0.0

            self.joint1_pub.publish(msg)
            self.joint2_pub.publish(msg)
            self.joint3_pub.publish(msg)
            self.joint4_pub.publish(msg)
            self.joint5_pub.publish(msg)
            self.joint6_pub.publish(msg)

            self.home_executed = True
            self.get_logger().info('HOME position sent')



def main(args=None):
    rclpy.init(args=args)
    node = ArmControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


