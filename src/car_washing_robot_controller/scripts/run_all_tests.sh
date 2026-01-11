#!/bin/bash
# Master test script - runs all automated tests

set -e  # Exit on error

echo "========================================="
echo "Car Washing Robot - Test Suite"
echo "========================================="
echo ""

# Colors for output
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_ROOT="$(dirname "$(dirname "$SCRIPT_DIR")")"

cd "$PROJECT_ROOT"

echo "Project root: $PROJECT_ROOT"
echo ""

# Check if ROS workspace is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}ERROR: ROS environment not sourced${NC}"
    echo "Please run: source /opt/ros/<distro>/setup.bash"
    echo "And: source install/setup.bash"
    exit 1
fi

echo -e "${YELLOW}ROS Distribution: $ROS_DISTRO${NC}"
echo ""

# ========== UNIT TESTS ==========
echo "========================================="
echo "Running Unit Tests (Mocked ROS)"
echo "========================================="
echo ""

if python3 -m pytest src/car_washing_robot_controller/tests/unit/ -v; then
    echo -e "${GREEN}✓ Unit tests PASSED${NC}"
else
    echo -e "${RED}✗ Unit tests FAILED${NC}"
    exit 1
fi

echo ""

# ========== INTEGRATION TESTS ==========
echo "========================================="
echo "Running Integration Tests (Real ROS)"
echo "========================================="
echo ""
echo -e "${YELLOW}Note: Integration tests require ROS 2 runtime${NC}"
echo ""

if python3 -m pytest src/car_washing_robot_controller/tests/integration/ -v -s; then
    echo -e "${GREEN}✓ Integration tests PASSED${NC}"
else
    echo -e "${RED}✗ Integration tests FAILED${NC}"
    exit 1
fi

echo ""

# ========== CODE QUALITY CHECKS ==========
echo "========================================="
echo "Running Code Quality Checks"
echo "========================================="
echo ""

# Flake8 (Python linting)
echo "Running flake8..."
if flake8 src/car_washing_robot_controller/car_washing_robot_controller/ --count --select=E9,F63,F7,F82 --show-source --statistics; then
    echo -e "${GREEN}✓ Flake8 PASSED${NC}"
else
    echo -e "${RED}✗ Flake8 FAILED${NC}"
    exit 1
fi

echo ""

# ========== SUMMARY ==========
echo "========================================="
echo -e "${GREEN}ALL TESTS PASSED!${NC}"
echo "========================================="
echo ""
echo "Test Summary:"
echo "  ✓ Unit tests"
echo "  ✓ Integration tests"
echo "  ✓ Code quality checks"
echo ""
echo "Manual testing scripts available in:"
echo "  $SCRIPT_DIR/"
echo ""
echo "Run manual tests with:"
echo "  ./scripts/test_approach_state.sh"
echo "  ./scripts/test_wall_follow.sh"
echo "  ./scripts/test_corner_detection.sh"
echo ""
