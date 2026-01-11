#!/bin/bash
# Manual test script for APPROACH state
# Tests robot approaching a car from a distance

echo "========================================="
echo "Testing APPROACH State"
echo "========================================="
echo ""
echo "This script will publish LiDAR scans simulating:"
echo "1. Robot far from car (2.0m)"
echo "2. Robot approaching (1.5m, 1.0m, 0.8m)"
echo "3. Robot reaches target (0.7m)"
echo ""
echo "Watch /cmd_vel topic for velocity commands"
echo "Expected behavior:"
echo "  - Robot moves forward (linear.x > 0) when distance > 0.7m"
echo "  - Robot stops (linear.x = 0) when distance <= 0.7m"
echo "  - State transitions from APPROACH to FOLLOW"
echo ""
echo "Press Enter to start..."
read

# Function to publish a LaserScan message
publish_scan() {
    local front_distance=$1
    local side_distance=$2

    echo "Publishing scan: front=${front_distance}m, side=${side_distance}m"

    # Create a simple LaserScan with 720 readings
    ros2 topic pub --once /scan sensor_msgs/msg/LaserScan "{
        header: {
            stamp: {sec: 0, nanosec: 0},
            frame_id: 'lidar_link'
        },
        angle_min: -3.14159,
        angle_max: 3.14159,
        angle_increment: 0.0087266,
        time_increment: 0.0,
        scan_time: 0.1,
        range_min: 0.05,
        range_max: 10.0,
        ranges: [$(seq -s, 1 180)$front_distance, $(seq -s, 1 180)$side_distance, $(seq -s, 1 360)5.0],
    }"

    sleep 1
}

echo ""
echo "Step 1: Robot far from car (2.0m)"
publish_scan 2.0 5.0

echo ""
echo "Step 2: Robot getting closer (1.5m)"
publish_scan 1.5 5.0

echo ""
echo "Step 3: Robot approaching (1.0m)"
publish_scan 1.0 5.0

echo ""
echo "Step 4: Robot near target (0.8m)"
publish_scan 0.8 5.0

echo ""
echo "Step 5: Robot reaches target (0.7m) - Should STOP and transition to FOLLOW"
publish_scan 0.7 0.5

echo ""
echo "========================================="
echo "Test complete!"
echo "========================================="
echo ""
echo "To verify results, check the node output or run:"
echo "  ros2 topic echo /cmd_vel"
