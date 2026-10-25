#!/usr/bin/env bash
# Invoke with data-run -- tools/build_test.sh. Output never enters the repository.
set -euo pipefail
root=$(cd "$(dirname "$0")/.." && pwd)
out=/home/dev/ws_iap/uwb_ros2
set +u
source /opt/ros/jazzy/setup.bash
set -u
mkdir -p "$out/evidence"
colcon --log-base "$out/log" build --base-paths "$root/packages" \
 --build-base "$out/build" --install-base "$out/install" --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
colcon --log-base "$out/log" test --base-paths "$root/packages" --build-base "$out/build" --install-base "$out/install" --test-result-base "$out/test-results" --return-code-on-test-failure
colcon --log-base "$out/log" test-result --test-result-base "$out/test-results" --verbose
