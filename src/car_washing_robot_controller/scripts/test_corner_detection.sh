#!/bin/bash
# Manual test script for corner detection and TURNING state
# Tests transition from FOLLOW to TURNING and turn completion

echo "========================================="
echo "Testing CORNER DETECTION & TURNING"
echo "========================================="
echo ""
echo "This script simulates:"
echo "1. Normal wall following"
echo "2. Corner detection (side distance > 2.0m)"
echo "3. Transition to TURNING state"
echo "4. Turn completion after 3.2 seconds"
echo ""
echo "Watch /cmd_vel and node logs"
echo "Expected behavior:"
echo "  - State: FOLLOW -> TURNING -> FOLLOW"
echo "  - Rotation starts when corner detected"
echo "  - Rotation continues for 3.2 seconds"
echo "  - Turn counter increments"
echo ""
echo "IMPORTANT: Make sure the node is in FOLLOW state"
echo "Press Enter to start..."
read

# Function to publish a LaserScan message
publish_scan() {
    local front_distance=$1
    local side_distance=$2
    local description=$3

    echo ""
    echo "=== $description ==="
    echo "Front: ${front_distance}m, Side: ${side_distance}m"

    # Generate 720 ranges
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

    sleep 1
}

echo ""
echo "Step 1: Normal wall following (side = 0.5m)"
publish_scan 5.0 0.5 "Normal wall following"

echo ""
echo "Step 2: Approaching corner (side = 1.5m)"
publish_scan 5.0 1.5 "Approaching corner"

echo ""
echo "Step 3: CORNER DETECTED! (side = 2.5m)"
echo "Robot should START TURNING (angular.z = 0.5 rad/s)"
publish_scan 5.0 2.5 "Corner detected - Start turning"

echo ""
echo "Step 4: Continue turning (publish scans during turn)"
echo "The robot will turn for 3.2 seconds..."
for i in {1..4}; do
    echo "  Turn in progress... ${i}/4"
    publish_scan 5.0 5.0 "Turning"
    sleep 1
done

echo ""
echo "Step 5: Turn should complete now"
echo "Expected: Stop rotation, transition back to FOLLOW, turn_count += 1"
publish_scan 5.0 0.5 "After turn - back to following"

echo ""
echo "========================================="
echo "Test complete!"
echo "========================================="
echo ""
echo "Verify in node logs:"
echo "  1. 'CORNER DETECTED' message"
echo "  2. State transition: FOLLOW -> TURNING"
echo "  3. 'Turn complete' after ~3.2 seconds"
echo "  4. State transition: TURNING -> FOLLOW"
echo "  5. turn_count incremented (e.g., 1/4 turns)"
echo ""
echo "Verify in /cmd_vel:"
echo "  1. angular.z = 0.5 during turn"
echo "  2. linear.x = 0 during turn"
echo "  3. Velocities change after turn completes"
