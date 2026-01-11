#!/bin/bash
# Helper script to monitor robot behavior during testing
# Opens multiple terminal windows to watch different topics

echo "========================================="
echo "Robot Monitoring Dashboard"
echo "========================================="
echo ""
echo "This script helps you monitor the robot's behavior"
echo "by displaying relevant ROS topics and node output."
echo ""
echo "Available monitoring options:"
echo "  1. Velocity commands (/cmd_vel)"
echo "  2. LiDAR scans (/scan)"
echo "  3. Node logs"
echo "  4. All of the above (split terminal)"
echo ""
read -p "Select option (1-4): " option

case $option in
    1)
        echo ""
        echo "Monitoring /cmd_vel (Velocity Commands)"
        echo "Press Ctrl+C to stop"
        echo ""
        ros2 topic echo /cmd_vel
        ;;

    2)
        echo ""
        echo "Monitoring /scan (LiDAR data)"
        echo "Showing only front and side distances"
        echo "Press Ctrl+C to stop"
        echo ""
        ros2 topic echo /scan --field ranges[360],ranges[180]
        ;;

    3)
        echo ""
        echo "Monitoring ApproachNode logs"
        echo "Press Ctrl+C to stop"
        echo ""
        ros2 node info /approach_node
        ;;

    4)
        echo ""
        echo "Opening split monitoring (requires tmux)"
        echo ""

        # Check if tmux is installed
        if ! command -v tmux &> /dev/null; then
            echo "ERROR: tmux not installed"
            echo "Install with: sudo apt-get install tmux"
            exit 1
        fi

        # Create a new tmux session with split panes
        tmux new-session -d -s robot_monitor

        # Split horizontally
        tmux split-window -h
        # Split left pane vertically
        tmux select-pane -t 0
        tmux split-window -v

        # Pane 0 (top-left): Velocity commands
        tmux select-pane -t 0
        tmux send-keys "echo 'Velocity Commands (/cmd_vel)'; ros2 topic echo /cmd_vel" C-m

        # Pane 1 (bottom-left): LiDAR data
        tmux select-pane -t 1
        tmux send-keys "echo 'LiDAR Front & Side Distances'; ros2 topic echo /scan --field ranges[360],ranges[180]" C-m

        # Pane 2 (right): Node info
        tmux select-pane -t 2
        tmux send-keys "echo 'Node Information'; ros2 node info /approach_node; echo ''; echo 'Waiting for updates...'" C-m

        # Attach to the session
        echo "Starting monitoring dashboard..."
        echo "Press Ctrl+B then D to detach"
        echo "Run 'tmux attach -t robot_monitor' to reattach"
        echo ""
        sleep 2
        tmux attach -t robot_monitor
        ;;

    *)
        echo "Invalid option"
        exit 1
        ;;
esac
