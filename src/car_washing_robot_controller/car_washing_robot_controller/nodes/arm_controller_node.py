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

        self.is_scrubbing = False
        self.scrub_position_up = True
        self.initializing_done = False

        self.timer = self.create_timer(2.0, self.callback_timer)
        self.get_logger().info('Arm controller node started')

    def callback_timer(self) -> None:
        if not self.initializing_done:
            self.get_logger().info("Initializing scrubbing position...")
            self.position_for_scrubbing()
            self.initializing_done = True
            self.is_scrubbing = True
        elif self.is_scrubbing:
            self.scrub_up_down()

    def position_for_scrubbing(self) -> None:
        self.get_logger().info('Getting into scrubbing position...')
        self.publish_joint_positions(0.0, 0.5, -0.3, 0.0, 0.0, 0.0)

    def scrub_up_down(self) -> None:
        msg = Float64()

        if self.scrub_position_up:
            msg.data = 0.3
            self.joint2_pub.publish(msg)
            self.get_logger().info("Scrubbing UP (joint2 = 0.3)")
            self.scrub_position_up = False
        else:
            msg.data = 0.5
            self.joint2_pub.publish(msg)
            self.get_logger().info("Scrubbing DOWN (joint2 = 0.5)")
            self.scrub_position_up = True

        msg.data = -0.3
        self.joint3_pub.publish(msg)

    def publish_joint_positions(
            self,
            joint1: float,
            joint2: float,
            joint3: float,
            joint4: float,
            joint5: float,
            joint6: float
    ) -> None:
        msg = Float64()

        msg.data = joint1
        self.joint1_pub.publish(msg)

        msg.data = joint2
        self.joint2_pub.publish(msg)

        msg.data = joint3
        self.joint3_pub.publish(msg)

        msg.data = joint4
        self.joint4_pub.publish(msg)

        msg.data = joint5
        self.joint5_pub.publish(msg)

        msg.data = joint6
        self.joint6_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArmControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


