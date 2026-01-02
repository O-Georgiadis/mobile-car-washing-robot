#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist



class ApproachNode(Node):
    def __init__(self):
        Node.__init__(self, 'approach_node')
        self.target_distance = 0.7
        self.approach_speed = 0.2
        self.wall_target_distance = 0.5
        self.wall_follow_speed = 0.15
        self.wall_distance_tolerance = 0.1
        self.steering_gain = 0.3

        self.state = 'APPROACH'

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

        side_idx = num_readings // 4
        side_distance = msg.ranges[side_idx]

        # validate
        if front_distance == float('inf') or front_distance != front_distance:
            front_distance = 999.0

        if side_distance == float('inf') or side_distance != side_distance:
            side_distance = 999.0

        if self.state == 'APPROACH':
            self.handle_approach(front_distance, side_distance)
        elif self.state == 'FOLLOW':
            self.handle_wall(front_distance, side_distance)

    def handle_approach(self, front_distance, side_distance):
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
            self.state = 'FOLLOW'

        self.cmd_vel_publisher.publish(cmd)

    def handle_wall(self, front_distance, side_distance):
        self.get_logger().info(f"FOLLOW - FRONT: {front_distance:.2f} m, SIDE: {side_distance:.2f} m")
        cmd = Twist()

        error = side_distance - self.wall_target_distance

        if abs(error) < self.wall_distance_tolerance:
            cmd.linear.x = self.wall_follow_speed
            cmd.angular.z = 0.0
            self.get_logger().info(f"Side distance {side_distance:.2f} okay, moving straight")
        elif error < 0:
            cmd.linear.x = self.wall_follow_speed
            cmd.angular.z = self.steering_gain * abs(error)
            self.get_logger().info(f"Error {error:.2f}, steering away")
        else:
            cmd.linear.x = self.wall_follow_speed
            cmd.angular.z = -self.steering_gain * error
            self.get_logger().info(f"Error {error:.2f}, steering towards goal")

        if front_distance < 0.3:
            cmd.linear.x = 0.0
            self.get_logger().warn("Obstacle ahead, stopping")

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

