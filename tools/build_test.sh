#!/usr/bin/env bash
set -eo pipefail
ws=/home/mint/ws_calib
export TMPDIR="$ws/evidence/uwb_ros1/tmp"
export PYTHONDONTWRITEBYTECODE=1
mkdir -p "$TMPDIR"
source /opt/ros/noetic/setup.bash
catkin config --workspace "$ws" --profile uwb_ros1 --extend /opt/ros/noetic \
 --build-space build_uwb_ros1 --devel-space devel_uwb_ros1 \
 --install-space install_uwb_ros1 --log-space logs_uwb_ros1 \
 --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DPYTHON_EXECUTABLE=/usr/bin/python3
catkin build --workspace "$ws" --profile uwb_ros1 nlink_parser liuo_uwb_msgs liuo_uwb_protocol liuo_uwb_bridge -j4 -p2 --no-status
for package in liuo_uwb_protocol liuo_uwb_bridge; do
 (cd "$ws/build_uwb_ros1/$package" && ctest --output-on-failure)
done
