#!/bin/bash
# Manual test script for FOLLOW (wall-following) state
# Tests robot following a wall with proportional steering

echo "========================================="
echo "Testing WALL FOLLOW State"
echo "========================================="
echo ""
echo "This script simulates wall-following scenarios:"
echo "1. Robot at target distance (0.5m) - moves straight"
echo "2. Robot too close to wall (0.3m) - steers away"
echo "3. Robot too far from wall (0.8m) - steers towards"
echo "4. Obstacle ahead - stops"
echo ""
echo "Watch /cmd_vel topic for velocity commands"
echo "Expected behavior:"
echo "  - angular.z = 0 when at target distance"
echo "  - angular.z > 0 when too close (steer away)"
echo "  - angular.z < 0 when too far (steer towards)"
echo "  - linear.x = 0 when obstacle ahead"
echo ""
echo "IMPORTANT: Make sure the node is in FOLLOW state"
echo "Press Enter to start..."
read

# Function to publish a LaserScan message with proper structure
publish_scan() {
    local front_distance=$1
    local side_distance=$2
    local description=$3

    echo ""
    echo "=== $description ==="
    echo "Front: ${front_distance}m, Side: ${side_distance}m"

    # Generate 720 ranges with front (index 360) and side (index 180) set
    ranges="["
    for i in {1..720}; do
        if [ $i -eq 360 ]; then
            ranges="${ranges}${front_distance}"
        elif [ $i -eq 180 ]; then
            ranges="${ranges}${side_distance}"
        else
            ranges="${ranges}5.0"
        fi
        if [ $i -lt 720 ]; then
            ranges="${ranges},"
        fi
    done
    ranges="${ranges}]"

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
        ranges: ${ranges}
    }"

    sleep 2
}

echo ""
echo "Scenario 1: At target distance (0.5m) - Should move STRAIGHT"
publish_scan 5.0 0.5 "Perfect wall distance"

echo ""
echo "Scenario 2: Too close to wall (0.3m) - Should STEER AWAY"
publish_scan 5.0 0.3 "Too close to wall"

echo ""
echo "Scenario 3: Too far from wall (0.8m) - Should STEER TOWARDS"
publish_scan 5.0 0.8 "Too far from wall"

echo ""
echo "Scenario 4: Within tolerance (0.55m) - Should move STRAIGHT"
publish_scan 5.0 0.55 "Within tolerance range"

echo ""
echo "Scenario 5: Obstacle ahead (0.2m) - Should STOP"
publish_scan 0.2 0.5 "Obstacle detected ahead"

echo ""
echo "========================================="
echo "Test complete!"
echo "========================================="
echo ""
echo "Review the /cmd_vel output to verify:"
echo "  1. Straight movement when at target"
echo "  2. Positive angular.z when too close"
echo "  3. Negative angular.z when too far"
echo "  4. Zero velocity when obstacle ahead"
