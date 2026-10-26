#!/usr/bin/env bash
# Source in a clean bash. Task outputs and ROS caches stay in this workspace.
export ROS_HOME=/home/mint/ws_calib/evidence/uwb_ros1/ros_home
export ROS_LOG_DIR=/home/mint/ws_calib/evidence/uwb_ros1/ros_logs
export PYTHONDONTWRITEBYTECODE=1
export TMPDIR=/home/mint/ws_calib/evidence/uwb_ros1/tmp
mkdir -p "$ROS_HOME" "$ROS_LOG_DIR" "$TMPDIR"
source /opt/ros/noetic/setup.bash
source /home/mint/ws_calib/devel_uwb_ros1/setup.bash
