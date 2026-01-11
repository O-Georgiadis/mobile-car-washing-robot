import unittest
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from car_washing_robot_controller.nodes.approach_node import ApproachNode
from car_washing_robot_controller.utils.robot_state import RobotState
from car_washing_robot_controller.utils.constants import (
    TARGET_DISTANCE, CORNER_DETECTION_THRESHOLD, TURN_DURATION,
    WALL_TARGET_DISTANCE, OBSTACLE_STOP_DISTANCE
)


class VelocityCapture(Node):
    """Helper node to capture velocity commands published by ApproachNode."""

    def __init__(self):
        Node.__init__(self, 'velocity_capture')
        self.commands = []
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.velocity_callback,
            10
        )

    def velocity_callback(self, msg):
        """Store received velocity commands."""
        self.commands.append({
            'linear': msg.linear.x,
            'angular': msg.angular.z,
            'timestamp': time.time()
        })

    def get_latest_command(self):
        """Get the most recent velocity command."""
        return self.commands[-1] if self.commands else None

    def clear_commands(self):
        """Clear the command history."""
        self.commands.clear()


class TestApproachIntegration(unittest.TestCase):
    """Integration tests for ApproachNode with actual ROS communication."""

    @classmethod
    def setUpClass(cls):
        """Initialize ROS once for all tests."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS after all tests."""
        rclpy.shutdown()

    def setUp(self):
        """Set up test fixtures before each test."""
        # Create the approach node
        self.approach_node = ApproachNode()

        # Create velocity capture node
        self.velocity_capture = VelocityCapture()

        # Create a publisher for LaserScan messages
        self.scan_publisher = self.velocity_capture.create_publisher(
            LaserScan,
            '/scan',
            10
        )

        # Give nodes time to initialize
        time.sleep(0.1)

    def tearDown(self):
        """Clean up after each test."""
        self.approach_node.destroy_node()
        self.velocity_capture.destroy_node()

    def spin_nodes(self, duration=0.2):
        """Spin both nodes for a short duration to process messages."""
        start_time = time.time()
        while time.time() - start_time < duration:
            rclpy.spin_once(self.approach_node, timeout_sec=0.01)
            rclpy.spin_once(self.velocity_capture, timeout_sec=0.01)

    def create_laser_scan(self, front_distance, side_distance, num_readings=720):
        """Create a LaserScan message with specified front and side distances."""
        msg = LaserScan()
        msg.header.stamp = self.velocity_capture.get_clock().now().to_msg()
        msg.header.frame_id = 'lidar_link'

        msg.angle_min = -3.14159
        msg.angle_max = 3.14159
        msg.angle_increment = (msg.angle_max - msg.angle_min) / num_readings
        msg.time_increment = 0.0
        msg.scan_time = 0.1
        msg.range_min = 0.05
        msg.range_max = 10.0

        # Create ranges array with default distance
        ranges = [5.0] * num_readings

        # Set specific distances
        front_idx = num_readings // 2
        side_idx = num_readings // 4

        ranges[front_idx] = front_distance
        ranges[side_idx] = side_distance

        msg.ranges = ranges
        msg.intensities = [0.0] * num_readings

        return msg

    def publish_scan_and_wait(self, front_distance, side_distance):
        """Publish a LaserScan message and wait for processing."""
        scan_msg = self.create_laser_scan(front_distance, side_distance)
        self.scan_publisher.publish(scan_msg)
        self.spin_nodes(duration=0.2)

    # ========== APPROACH STATE INTEGRATION TESTS ==========

    def test_approach_state_far_from_target(self):
        """Test robot moves forward when far from target."""
        self.velocity_capture.clear_commands()
        self.approach_node.state = RobotState.APPROACH

        self.publish_scan_and_wait(front_distance=2.0, side_distance=5.0)

        cmd = self.velocity_capture.get_latest_command()
        self.assertIsNotNone(cmd, "No velocity command received")
        self.assertGreater(cmd['linear'], 0, "Robot should move forward")
        self.assertEqual(cmd['angular'], 0.0)
        self.assertEqual(self.approach_node.state, RobotState.APPROACH)

    def test_approach_state_reaches_target(self):
        """Test state transition when target distance reached."""
        self.velocity_capture.clear_commands()
        self.approach_node.state = RobotState.APPROACH

        self.publish_scan_and_wait(front_distance=TARGET_DISTANCE - 0.1, side_distance=5.0)

        cmd = self.velocity_capture.get_latest_command()
        self.assertIsNotNone(cmd)
        self.assertEqual(cmd['linear'], 0.0, "Robot should stop")
        self.assertEqual(cmd['angular'], 0.0)
        self.assertEqual(self.approach_node.state, RobotState.FOLLOW,
                        "Should transition to FOLLOW state")

    # ========== FOLLOW STATE INTEGRATION TESTS ==========

    def test_follow_state_maintains_wall_distance(self):
        """Test robot follows wall at target distance."""
        self.velocity_capture.clear_commands()
        self.approach_node.state = RobotState.FOLLOW

        self.publish_scan_and_wait(
            front_distance=5.0,
            side_distance=WALL_TARGET_DISTANCE
        )

        cmd = self.velocity_capture.get_latest_command()
        self.assertIsNotNone(cmd)
        self.assertGreater(cmd['linear'], 0, "Robot should move forward")
        self.assertEqual(cmd['angular'], 0.0, "Robot should move straight")
        self.assertEqual(self.approach_node.state, RobotState.FOLLOW)

    def test_follow_state_steers_when_too_close(self):
        """Test robot steers away when too close to wall."""
        self.velocity_capture.clear_commands()
        self.approach_node.state = RobotState.FOLLOW

        self.publish_scan_and_wait(
            front_distance=5.0,
            side_distance=WALL_TARGET_DISTANCE - 0.2
        )

        cmd = self.velocity_capture.get_latest_command()
        self.assertIsNotNone(cmd)
        self.assertGreater(cmd['linear'], 0)
        self.assertGreater(cmd['angular'], 0, "Should steer away from wall")

    def test_follow_state_steers_when_too_far(self):
        """Test robot steers towards wall when too far."""
        self.velocity_capture.clear_commands()
        self.approach_node.state = RobotState.FOLLOW

        self.publish_scan_and_wait(
            front_distance=5.0,
            side_distance=WALL_TARGET_DISTANCE + 0.3
        )

        cmd = self.velocity_capture.get_latest_command()
        self.assertIsNotNone(cmd)
        self.assertGreater(cmd['linear'], 0)
        self.assertLess(cmd['angular'], 0, "Should steer towards wall")

    def test_follow_state_detects_corner(self):
        """Test corner detection and transition to TURNING."""
        self.velocity_capture.clear_commands()
        self.approach_node.state = RobotState.FOLLOW

        self.publish_scan_and_wait(
            front_distance=5.0,
            side_distance=CORNER_DETECTION_THRESHOLD + 0.5
        )

        cmd = self.velocity_capture.get_latest_command()
        self.assertIsNotNone(cmd)
        self.assertEqual(cmd['linear'], 0.0, "Should stop forward motion")
        self.assertGreater(cmd['angular'], 0, "Should start rotating")
        self.assertEqual(self.approach_node.state, RobotState.TURNING,
                        "Should transition to TURNING")
        self.assertIsNotNone(self.approach_node.turn_start_time)

    def test_follow_state_stops_for_obstacle(self):
        """Test robot stops when obstacle detected ahead."""
        self.velocity_capture.clear_commands()
        self.approach_node.state = RobotState.FOLLOW

        self.publish_scan_and_wait(
            front_distance=OBSTACLE_STOP_DISTANCE - 0.1,
            side_distance=WALL_TARGET_DISTANCE
        )

        cmd = self.velocity_capture.get_latest_command()
        self.assertIsNotNone(cmd)
        self.assertEqual(cmd['linear'], 0.0, "Should stop for obstacle")
        self.assertEqual(cmd['angular'], 0.0)

    # ========== TURNING STATE INTEGRATION TESTS ==========

    def test_turning_state_rotates(self):
        """Test robot rotates during TURNING state."""
        self.velocity_capture.clear_commands()
        self.approach_node.state = RobotState.TURNING
        self.approach_node.turn_start_time = time.time()

        # Publish scan to trigger turning handler
        self.publish_scan_and_wait(front_distance=5.0, side_distance=5.0)

        cmd = self.velocity_capture.get_latest_command()
        self.assertIsNotNone(cmd)
        self.assertEqual(cmd['linear'], 0.0, "Should not move forward")
        self.assertGreater(cmd['angular'], 0, "Should rotate")

    def test_turning_state_completes_turn(self):
        """Test turn completion after duration elapsed."""
        self.velocity_capture.clear_commands()
        self.approach_node.state = RobotState.TURNING
        self.approach_node.turn_count = 0
        # Simulate turn started in the past
        self.approach_node.turn_start_time = time.time() - TURN_DURATION - 0.1

        self.publish_scan_and_wait(front_distance=5.0, side_distance=5.0)

        cmd = self.velocity_capture.get_latest_command()
        self.assertIsNotNone(cmd)
        self.assertEqual(cmd['linear'], 0.0)
        self.assertEqual(cmd['angular'], 0.0, "Should stop rotating")
        self.assertEqual(self.approach_node.turn_count, 1)
        self.assertEqual(self.approach_node.state, RobotState.FOLLOW,
                        "Should transition back to FOLLOW")

    # ========== COMPLETE STATE INTEGRATION TESTS ==========

    def test_complete_state_stops_robot(self):
        """Test robot remains stopped in COMPLETE state."""
        self.velocity_capture.clear_commands()
        self.approach_node.state = RobotState.COMPLETE

        self.publish_scan_and_wait(front_distance=5.0, side_distance=5.0)

        cmd = self.velocity_capture.get_latest_command()
        self.assertIsNotNone(cmd)
        self.assertEqual(cmd['linear'], 0.0)
        self.assertEqual(cmd['angular'], 0.0)

    # ========== END-TO-END SCENARIO TESTS ==========

    def test_full_approach_to_follow_transition(self):
        """Test complete transition from APPROACH to FOLLOW."""
        self.velocity_capture.clear_commands()
        self.approach_node.state = RobotState.APPROACH

        # Robot approaches target
        self.publish_scan_and_wait(front_distance=2.0, side_distance=5.0)
        self.assertEqual(self.approach_node.state, RobotState.APPROACH)

        # Robot reaches target
        self.publish_scan_and_wait(front_distance=0.6, side_distance=0.5)
        self.assertEqual(self.approach_node.state, RobotState.FOLLOW)

        # Robot starts following wall
        self.publish_scan_and_wait(front_distance=5.0, side_distance=0.5)
        cmd = self.velocity_capture.get_latest_command()
        self.assertGreater(cmd['linear'], 0)

    def test_full_follow_to_turning_transition(self):
        """Test complete transition from FOLLOW to TURNING."""
        self.velocity_capture.clear_commands()
        self.approach_node.state = RobotState.FOLLOW

        # Robot follows wall normally
        self.publish_scan_and_wait(front_distance=5.0, side_distance=0.5)
        self.assertEqual(self.approach_node.state, RobotState.FOLLOW)

        # Corner detected
        self.publish_scan_and_wait(front_distance=5.0, side_distance=2.5)
        self.assertEqual(self.approach_node.state, RobotState.TURNING)

        # Continue turning
        self.publish_scan_and_wait(front_distance=5.0, side_distance=5.0)
        cmd = self.velocity_capture.get_latest_command()
        self.assertGreater(cmd['angular'], 0)


if __name__ == '__main__':
    unittest.main()
