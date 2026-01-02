#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# from car_washing_robot_controller.car_washing_robot_controller.utils.constants import APPROACH_TARGET_DISTANCE, \
#     APPROACH_SPEED


class ApproachNode(Node):
    def __init__(self):
        Node.__init__(self, 'approach_node')
        self.target_distance = 0.7
        self.approach_speed = 0.2
        # self.target_distance = APPROACH_TARGET_DISTANCE
        # self.approach_speed = APPROACH_SPEED
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.callback_scan,
            10
        )
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('ApproachNode Node created')

    def callback_scan(self, msg: LaserScan):
        # extract front distance
        num_readings = len(msg.ranges)
        front_idx = num_readings // 2

        front_distance = msg.ranges[front_idx]

        # validate
        if front_distance == float('inf'):
            self.get_logger().warn('No obstacle in front (infinite distance)')
            self.stop_robot()
            return

        if front_distance != front_distance:
            self.get_logger().warn('Invalid sensor reading')
            self.stop_robot()
            return

        self.get_logger().info(f"Front obstacle at {front_distance:.2f} m")

        cmd = Twist()
        if front_distance > self.target_distance:
            cmd.linear.x = self.approach_speed
            cmd.angular.z = 0.0

            distance_to_target = front_distance - self.target_distance
            self.get_logger().info(f"Moving forward, distance to goal: {distance_to_target:.2f} m")
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.get_logger().info(f"Target reached")

        self.cmd_vel_publisher.publish(cmd)

    def stop_robot(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_publisher.publish(cmd)
        self.get_logger().info(f"Emergency stop")


def main(args=None):
    rclpy.init(args=args)
    node = ApproachNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

