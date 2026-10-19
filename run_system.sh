#!/usr/bin/env bash
# Runner script with automatic console logging to ~/workspace/catkin_ws_ov/logs/

WORKSPACE_DIR="/home/bhadwabhindi/workspace/catkin_ws_ov"
LOGS_DIR="${WORKSPACE_DIR}/logs"
mkdir -p "${LOGS_DIR}"

TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
RUN_LOG="${LOGS_DIR}/vio_run_${TIMESTAMP}.log"
LATEST_LOG="${LOGS_DIR}/latest_run.log"

source /opt/ros/humble/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"

echo "=========================================================="
echo " Starting VIO System with Automatic Logging"
echo " Trajectory File:  ${LOGS_DIR}/latest_trajectory.txt"
echo " Full Session Log: ${RUN_LOG}"
echo " Latest Run Link:  ${LATEST_LOG}"
echo "=========================================================="
echo ""

# Run and pipe all terminal output (stdout + stderr) to console AND log files
ros2 launch esp32_imu_driver full_system_features.launch.py 2>&1 | tee "${RUN_LOG}" | tee "${LATEST_LOG}"
