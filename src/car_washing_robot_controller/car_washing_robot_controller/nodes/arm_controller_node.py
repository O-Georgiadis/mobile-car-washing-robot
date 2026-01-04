#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from ..utils.arm_pose import ArmPose



class ArmControllerNode(Node):
    def __init__(self):
        Node.__init__(self, 'arm_controller_node')

        self.joint1_pub = self.create_publisher(Float64, '/joint1/cmd_pos', 10)
        self.joint2_pub = self.create_publisher(Float64, '/joint2/cmd_pos', 10)
        self.joint3_pub = self.create_publisher(Float64, '/joint3/cmd_pos', 10)
        self.joint4_pub = self.create_publisher(Float64, '/joint4/cmd_pos', 10)
        self.joint5_pub = self.create_publisher(Float64, '/joint5/cmd_pos', 10)
        self.joint6_pub = self.create_publisher(Float64, '/joint6/cmd_pos', 10)

        self.joint2_position = 0.0
        self.joint2_up = True

        self.timer = self.create_timer(2.0, self.move_to_home)
        self.home_executed = False

        self.current_pose = ArmPose.HOME

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

    def callback_timer(self):
        if not self.home_executed:
            self.move_to_home()
        else:
            self.move_joint2()

        if self.current_pose == ArmPose.HOME:
            self.move_to_reach_forward()
            self.current_pose = ArmPose.REACH
        else:
            self.move_to_home_pose()
            self.current_pose = ArmPose.HOME

    def move_to_home_pose(self):
        self.get_logger().info('Moving to HOME position')
        self.publish_joint_positions(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    def move_to_reach_forward(self):
        self.get_logger().info('Moving to REACH FORWARD pose...')
        self.publish_joint_positions(0.0, 0.5, -0.3, 0.0, 0.0, 0.0)

    def publish_joint_positions(self, j1, j2, j3, j4, j5, j6):
        msg = Float64()

        msg.data = j1
        self.joint1_pub.publish(msg)

        msg.data = j2
        self.joint2_pub.publish(msg)

        msg.data = j3
        self.joint3_pub.publish(msg)

        msg.data = j4
        self.joint4_pub.publish(msg)

        msg.data = j5
        self.joint5_pub.publish(msg)

        msg.data = j6
        self.joint6_pub.publish(msg)

    def move_joint2(self):
        msg = Float64()

        if self.joint2_up:
            msg.data = 0.5
            self.get_logger().info("Joint 2: Moving UP to 0.5")
            self.joint2_up = False
        else:
            msg.data = 0.0
            self.get_logger().info("Joint 2: Moving DOWN to 0.0")
            self.joint2_up = True

        self.joint2_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArmControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


