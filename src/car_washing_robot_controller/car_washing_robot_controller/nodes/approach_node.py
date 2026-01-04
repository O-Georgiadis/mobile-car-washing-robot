#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import time
import math
from ..utils.constants import (
    APPROACH_SPEED, TARGET_DISTANCE,
    WALL_TARGET_DISTANCE, WALL_FOLLOW_SPEED, WALL_DISTANCE_TOLERANCE, STEERING_GAIN,
    CORNER_DETECTION_THRESHOLD, TURNING_SPEED, TURN_DURATION,
    ZERO, MAX_DISTANCE, OBSTACLE_STOP_DISTANCE
)
from ..utils.robot_state import RobotState


class ApproachNode(Node):
    def __init__(self):
        Node.__init__(self, 'approach_node')
        self.state = RobotState.APPROACH

        self.turn_count = 0
        self.total_turns = 4
        self.turn_start_time = None

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
        if not math.isfinite(front_distance):
            front_distance = MAX_DISTANCE

        if not math.isfinite(side_distance):
            side_distance = MAX_DISTANCE

        match self.state:
            case RobotState.APPROACH:
                self.handle_approach(front_distance, side_distance)
            case RobotState.FOLLOW:
                self.handle_wall(front_distance, side_distance)
            case RobotState.TURNING:
                self.handle_turning(front_distance, side_distance)
            case RobotState.COMPLETE:
                self.handle_complete()

    def handle_approach(self, front_distance, side_distance):
        self.get_logger().info(f"Front obstacle at {front_distance:.2f} m")

        if front_distance > TARGET_DISTANCE:
            self.publish_velocity(APPROACH_SPEED, ZERO)
            distance_to_target = front_distance - TARGET_DISTANCE
            self.get_logger().info(f"Moving forward, distance to goal: {distance_to_target:.2f} m")
        else:
            self.publish_velocity(ZERO, ZERO)
            self.get_logger().info(f"Target reached")
            self.state = RobotState.FOLLOW

    def handle_wall(self, front_distance, side_distance):
        self.get_logger().info(f"FOLLOW - FRONT: {front_distance:.2f} m, SIDE: {side_distance:.2f} m")

        # detect corners
        if side_distance > CORNER_DETECTION_THRESHOLD:
            self.state = RobotState.TURNING
            self.turn_start_time = time.time()

            self.get_logger().info(f'CORNER DETECTED! (side={side_distance:.2f}m) → TURNING')
            self.publish_velocity(ZERO, TURNING_SPEED) # start rotation
            return  # Exit early

        error = side_distance - WALL_TARGET_DISTANCE

        if abs(error) < WALL_DISTANCE_TOLERANCE:
            self.publish_velocity(WALL_FOLLOW_SPEED, ZERO)
            self.get_logger().info(f"Side distance {side_distance:.2f} okay, moving straight")
        elif error < 0:
            self.publish_velocity(WALL_FOLLOW_SPEED, STEERING_GAIN * abs(error))
            self.get_logger().info(f"Error {error:.2f}, steering away")
        else:
            self.publish_velocity(WALL_FOLLOW_SPEED, -STEERING_GAIN * error)
            self.get_logger().info(f"Error {error:.2f}, steering towards goal")

        if front_distance < OBSTACLE_STOP_DISTANCE:
            self.publish_velocity(ZERO, ZERO)
            self.get_logger().warn("Obstacle ahead, stopping")


    def handle_turning(self, front_distance, side_distance):
        elapsed_time = time.time() - self.turn_start_time

        self.get_logger().info(f'[TURNING] Elapsed: {elapsed_time:.1f}s / {TURN_DURATION:.1f}s')

        if elapsed_time >= TURN_DURATION:
            self.turn_count += 1

            self.get_logger().info(f'Turn complete! ({self.turn_count}/{self.total_turns} turns done)')

            # Check if we've completed all turns
            if self.turn_count >= self.total_turns:
                # ====== ALL TURNS DONE - MISSION COMPLETE ======
                self.state = RobotState.COMPLETE
                self.get_logger().info('MISSION COMPLETE!')
            else:
                # ====== MORE TURNS TO GO - RESUME FOLLOWING ======
                self.state = RobotState.FOLLOW
                self.get_logger().info('TRANSITION: TURNING -> FOLLOW')

            # Stop rotating
            self.publish_velocity(ZERO, ZERO)

        else:
            # ====== STILL TURNING ======
            # Continue rotating
            self.publish_velocity(ZERO, TURNING_SPEED) # don't move forward, keep rotating
            self.get_logger().info(f'Rotating... ({elapsed_time:.1f}s / {TURN_DURATION:.1f}s)')


    def handle_complete(self):
        self.publish_velocity(ZERO, ZERO)

    def publish_velocity(self, linear, angular):
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.cmd_vel_publisher.publish(cmd)


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

