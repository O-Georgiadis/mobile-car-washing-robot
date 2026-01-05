import unittest
from unittest.mock import Mock, MagicMock, patch
import time

# We'll import the actual ROS types since they're available
import rclpy
from geometry_msgs.msg import Twist

from car_washing_robot_controller.utils.robot_state import RobotState
from car_washing_robot_controller.utils.constants import (
    APPROACH_SPEED, TARGET_DISTANCE,
    WALL_TARGET_DISTANCE, WALL_FOLLOW_SPEED, WALL_DISTANCE_TOLERANCE, STEERING_GAIN,
    CORNER_DETECTION_THRESHOLD, TURNING_SPEED, TURN_DURATION,
    ZERO, OBSTACLE_STOP_DISTANCE
)


class TestApproachNode(unittest.TestCase):
    """Test cases for ApproachNode logic."""

    @classmethod
    def setUpClass(cls):
        """Initialize ROS once for all tests."""
        if not rclpy.ok():
            rclpy.init()

    def setUp(self):
        """Set up test fixtures before each test."""
        # Import and patch at the same time
        with patch('car_washing_robot_controller.nodes.approach_node.Node.__init__', return_value=None), \
             patch('car_washing_robot_controller.nodes.approach_node.Node.create_subscription'), \
             patch('car_washing_robot_controller.nodes.approach_node.Node.create_publisher'), \
             patch('car_washing_robot_controller.nodes.approach_node.Node.get_logger'):
            from car_washing_robot_controller.nodes.approach_node import ApproachNode

            self.node = ApproachNode()

            # Manually set attributes that __init__ would set
            self.node.state = RobotState.APPROACH
            self.node.turn_count = 0
            self.node.total_turns = 4
            self.node.turn_start_time = None

            # Mock logger
            mock_logger = Mock()
            mock_logger.info = Mock()
            mock_logger.warn = Mock()
            self.node.get_logger = Mock(return_value=mock_logger)

            # Create a real Twist publisher mock that captures messages
            self.published_messages = []

            def capture_publish(msg):
                self.published_messages.append(msg)

            self.node.cmd_vel_publisher = Mock()
            self.node.cmd_vel_publisher.publish = Mock(side_effect=capture_publish)

    def get_last_published_velocity(self):
        """Helper to get the last published velocity command."""
        if not self.published_messages:
            return None
        return self.published_messages[-1]

    def test_initialization(self):
        """Test that node initializes in APPROACH state."""
        self.assertEqual(self.node.state, RobotState.APPROACH)
        self.assertEqual(self.node.turn_count, 0)
        self.assertEqual(self.node.total_turns, 4)
        self.assertIsNone(self.node.turn_start_time)

    # ========== APPROACH STATE TESTS ========== 

    def test_handle_approach_moves_forward_when_far(self):
        """Test robot moves forward when distance > target."""
        self.node.state = RobotState.APPROACH
        self.node.handle_approach(2.0)

        # Verify velocity command
        cmd = self.get_last_published_velocity()
        self.assertIsNotNone(cmd)
        self.assertEqual(cmd.linear.x, APPROACH_SPEED)
        self.assertEqual(cmd.angular.z, ZERO)

        # State should remain APPROACH
        self.assertEqual(self.node.state, RobotState.APPROACH)

    def test_handle_approach_stops_at_target_distance(self):
        """Test robot stops and transitions when reaching target."""
        self.node.state = RobotState.APPROACH
        self.node.handle_approach(TARGET_DISTANCE)

        # Verify stop command
        cmd = self.get_last_published_velocity()
        self.assertIsNotNone(cmd)
        self.assertEqual(cmd.linear.x, ZERO)
        self.assertEqual(cmd.angular.z, ZERO)

        # State should transition to FOLLOW
        self.assertEqual(self.node.state, RobotState.FOLLOW)

    def test_handle_approach_stops_when_closer_than_target(self):
        """Test robot stops when distance < target."""
        self.node.state = RobotState.APPROACH
        self.node.handle_approach(0.5)

        # Verify stop command
        cmd = self.get_last_published_velocity()
        self.assertIsNotNone(cmd)
        self.assertEqual(cmd.linear.x, ZERO)
        self.assertEqual(cmd.angular.z, ZERO)

        # State should transition to FOLLOW
        self.assertEqual(self.node.state, RobotState.FOLLOW)

    # ========== FOLLOW STATE TESTS ========== 

    def test_handle_wall_moves_straight_when_at_target_distance(self):
        """Test robot moves straight when side distance is at target."""
        self.node.state = RobotState.FOLLOW
        front_distance = 5.0
        side_distance = WALL_TARGET_DISTANCE

        self.node.handle_wall(front_distance, side_distance)

        # Verify straight movement
        cmd = self.get_last_published_velocity()
        self.assertIsNotNone(cmd)
        self.assertEqual(cmd.linear.x, WALL_FOLLOW_SPEED)
        self.assertEqual(cmd.angular.z, ZERO)

        # State should remain FOLLOW
        self.assertEqual(self.node.state, RobotState.FOLLOW)

    def test_handle_wall_moves_straight_within_tolerance(self):
        """Test robot moves straight when within distance tolerance."""
        self.node.state = RobotState.FOLLOW
        front_distance = 5.0
        side_distance = WALL_TARGET_DISTANCE + WALL_DISTANCE_TOLERANCE * 0.5

        self.node.handle_wall(front_distance, side_distance)

        cmd = self.get_last_published_velocity()
        self.assertIsNotNone(cmd)
        self.assertEqual(cmd.linear.x, WALL_FOLLOW_SPEED)
        self.assertEqual(cmd.angular.z, ZERO)

    def test_handle_wall_steers_away_when_too_close(self):
        """Test robot steers away when too close to wall."""
        self.node.state = RobotState.FOLLOW
        front_distance = 5.0
        side_distance = WALL_TARGET_DISTANCE - 0.2  # Too close

        self.node.handle_wall(front_distance, side_distance)

        cmd = self.get_last_published_velocity()
        self.assertIsNotNone(cmd)
        self.assertEqual(cmd.linear.x, WALL_FOLLOW_SPEED)
        # Should steer away (positive angular)
        self.assertGreater(cmd.angular.z, 0)
        error = abs(side_distance - WALL_TARGET_DISTANCE)
        expected_angular = STEERING_GAIN * error
        self.assertAlmostEqual(cmd.angular.z, expected_angular, places=5)

    def test_handle_wall_steers_towards_when_too_far(self):
        """Test robot steers towards wall when too far."""
        self.node.state = RobotState.FOLLOW
        front_distance = 5.0
        side_distance = WALL_TARGET_DISTANCE + 0.3  # Too far

        self.node.handle_wall(front_distance, side_distance)

        cmd = self.get_last_published_velocity()
        self.assertIsNotNone(cmd)
        self.assertEqual(cmd.linear.x, WALL_FOLLOW_SPEED)
        # Should steer towards (negative angular)
        self.assertLess(cmd.angular.z, 0)
        error = side_distance - WALL_TARGET_DISTANCE
        expected_angular = -STEERING_GAIN * error
        self.assertAlmostEqual(cmd.angular.z, expected_angular, places=5)

    def test_handle_wall_detects_corner_and_starts_turning(self):
        """Test robot detects corner and transitions to TURNING."""
        self.node.state = RobotState.FOLLOW
        front_distance = 5.0
        side_distance = CORNER_DETECTION_THRESHOLD + 0.5  # Corner detected

        self.node.handle_wall(front_distance, side_distance)

        # Should transition to TURNING
        self.assertEqual(self.node.state, RobotState.TURNING)
        # Should start rotation
        cmd = self.get_last_published_velocity()
        self.assertIsNotNone(cmd)
        self.assertEqual(cmd.linear.x, ZERO)
        self.assertEqual(cmd.angular.z, TURNING_SPEED)
        # Turn start time should be set
        self.assertIsNotNone(self.node.turn_start_time)

    def test_handle_wall_corner_detection_threshold(self):
        """Test corner detection exactly at threshold."""
        self.node.state = RobotState.FOLLOW
        front_distance = 5.0
        side_distance = CORNER_DETECTION_THRESHOLD + 0.01  # Just above threshold

        self.node.handle_wall(front_distance, side_distance)

        self.assertEqual(self.node.state, RobotState.TURNING)

    def test_handle_wall_stops_for_obstacle_ahead(self):
        """Test robot stops when obstacle detected ahead."""
        self.node.state = RobotState.FOLLOW
        front_distance = OBSTACLE_STOP_DISTANCE - 0.1  # Obstacle ahead
        side_distance = WALL_TARGET_DISTANCE

        self.node.handle_wall(front_distance, side_distance)

        # Should stop
        cmd = self.get_last_published_velocity()
        self.assertIsNotNone(cmd)
        self.assertEqual(cmd.linear.x, ZERO)
        self.assertEqual(cmd.angular.z, ZERO)

    def test_handle_wall_obstacle_overrides_steering(self):
        """Test obstacle detection overrides steering commands."""
        self.node.state = RobotState.FOLLOW
        front_distance = OBSTACLE_STOP_DISTANCE - 0.05
        side_distance = WALL_TARGET_DISTANCE + 0.3  # Would normally steer

        self.node.handle_wall(front_distance, side_distance)

        # Should stop despite steering command
        cmd = self.get_last_published_velocity()
        self.assertIsNotNone(cmd)
        self.assertEqual(cmd.linear.x, ZERO)
        self.assertEqual(cmd.angular.z, ZERO)

    # ========== TURNING STATE TESTS ========== 

    def test_handle_turning_continues_rotation(self):
        """Test robot continues rotating during turn."""
        self.node.state = RobotState.TURNING
        self.node.turn_start_time = time.time()

        self.node.handle_turning()

        # Should continue rotating
        cmd = self.get_last_published_velocity()
        self.assertIsNotNone(cmd)
        self.assertEqual(cmd.linear.x, ZERO)
        self.assertEqual(cmd.angular.z, TURNING_SPEED)

        # State should remain TURNING
        self.assertEqual(self.node.state, RobotState.TURNING)
        # Turn count should not increment yet
        self.assertEqual(self.node.turn_count, 0)

    def test_handle_turning_completes_turn_after_duration(self):
        """Test turn completion after duration elapsed."""
        self.node.state = RobotState.TURNING
        self.node.turn_count = 0
        # Simulate turn started in the past
        self.node.turn_start_time = time.time() - TURN_DURATION - 0.1

        self.node.handle_turning()

        # Should stop rotation
        cmd = self.get_last_published_velocity()
        self.assertIsNotNone(cmd)
        self.assertEqual(cmd.linear.x, ZERO)
        self.assertEqual(cmd.angular.z, ZERO)

        # Turn count should increment
        self.assertEqual(self.node.turn_count, 1)
        # Should transition back to FOLLOW
        self.assertEqual(self.node.state, RobotState.FOLLOW)

    def test_handle_turning_completes_mission_after_four_turns(self):
        """Test mission completes after 4 turns."""
        self.node.state = RobotState.TURNING
        self.node.turn_count = 3  # This will be the 4th turn
        self.node.turn_start_time = time.time() - TURN_DURATION - 0.1

        self.node.handle_turning()

        # Turn count should be 4
        self.assertEqual(self.node.turn_count, 4)
        # Should transition to COMPLETE
        self.assertEqual(self.node.state, RobotState.COMPLETE)

    def test_handle_turning_exact_duration(self):
        """Test turn completion at exact duration."""
        self.node.state = RobotState.TURNING
        self.node.turn_count = 0
        self.node.turn_start_time = time.time() - TURN_DURATION

        self.node.handle_turning()

        # Should complete turn
        self.assertEqual(self.node.turn_count, 1)
        self.assertEqual(self.node.state, RobotState.FOLLOW)

    # ========== COMPLETE STATE TESTS ========== 

    def test_handle_complete_stops_robot(self):
        """Test robot stops in COMPLETE state."""
        self.node.state = RobotState.COMPLETE

        self.node.handle_complete()

        # Should publish stop command
        cmd = self.get_last_published_velocity()
        self.assertIsNotNone(cmd)
        self.assertEqual(cmd.linear.x, ZERO)
        self.assertEqual(cmd.angular.z, ZERO)

    # ========== HELPER METHOD TESTS ========== 

    def test_publish_velocity(self):
        """Test velocity publishing helper method."""
        linear = 0.5
        angular = 0.3

        self.node.publish_velocity(linear, angular)

        # Verify publish was called
        cmd = self.get_last_published_velocity()
        self.assertIsNotNone(cmd)
        self.assertEqual(cmd.linear.x, linear)
        self.assertEqual(cmd.angular.z, angular)

    def test_publish_velocity_zero(self):
        """Test publishing zero velocity."""
        self.node.publish_velocity(ZERO, ZERO)

        cmd = self.get_last_published_velocity()
        self.assertIsNotNone(cmd)
        self.assertEqual(cmd.linear.x, ZERO)
        self.assertEqual(cmd.angular.z, ZERO)

    # ========== EDGE CASE TESTS ========== 

    def test_multiple_turns_sequence(self):
        """Test completing multiple turns in sequence."""
        self.node.state = RobotState.TURNING
        self.node.turn_count = 0

        for turn_num in range(1, 5):
            self.node.turn_start_time = time.time() - TURN_DURATION - 0.1
            self.node.handle_turning()

            self.assertEqual(self.node.turn_count, turn_num)

            if turn_num < 4:
                self.assertEqual(self.node.state, RobotState.FOLLOW)
            else:
                self.assertEqual(self.node.state, RobotState.COMPLETE)

            # Transition back to TURNING for next iteration
            if turn_num < 4:
                self.node.state = RobotState.TURNING

    def test_wall_follow_with_negative_side_distance(self):
        """Test handling negative side distance (error case)."""
        self.node.state = RobotState.FOLLOW
        front_distance = 5.0
        side_distance = -1.0  # Invalid but possible sensor reading

        # Should not crash, will be treated as very close
        self.node.handle_wall(front_distance, side_distance)

        # Should steer away from wall
        cmd = self.get_last_published_velocity()
        self.assertIsNotNone(cmd)
        self.assertGreater(cmd.angular.z, 0)


if __name__ == '__main__':
    unittest.main()
