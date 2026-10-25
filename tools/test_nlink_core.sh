#!/usr/bin/env bash
set -euo pipefail
root=$(cd "$(dirname "$0")/.." && pwd)
out=/home/dev/ws_iap/uwb_ros2/nlink-sanitizer
mkdir -p "$out"
g++ -std=c++17 -g -fsanitize=address,undefined -fno-omit-frame-pointer \
  -I "$root/packages/nlink_parser/include" \
  "$root/packages/nlink_parser/src/ros2/framer.cpp" \
  "$root/packages/nlink_parser/test/framer_test.cpp" -o "$out/framer_test"
ASAN_OPTIONS=verify_asan_link_order=0 "$out/framer_test"
