#!/usr/bin/env bash
set -euo pipefail
root=$(cd "$(dirname "$0")/.." && pwd)
out=/home/dev/ws_iap/uwb_ros2/nlink-sanitizer
mkdir -p "$out"
vendor="$root/packages/nlink_parser/src/utils/nlink_unpack"
objects=()
for file in "$vendor"/nlink_utils.c "$vendor"/nlink_linktrack_{anchorframe0,tagframe0,nodeframe0,nodeframe1,nodeframe2,nodeframe3,nodeframe4,nodeframe5,nodeframe6}.c; do
  object="$out/$(basename "$file").o"
  gcc -std=c11 -g -fsanitize=address,undefined -fno-omit-frame-pointer -I "$vendor" -c "$file" -o "$object"
  objects+=("$object")
done
g++ -std=c++17 -g -fsanitize=address,undefined -fno-omit-frame-pointer \
  -I "$root/packages/nlink_parser/include" -I "$vendor" \
  "$root/packages/nlink_parser/src/ros2/framer.cpp" \
  "$root/packages/nlink_parser/test/framer_test.cpp" "${objects[@]}" -o "$out/framer_test"
ASAN_OPTIONS=verify_asan_link_order=0 UBSAN_OPTIONS=halt_on_error=1 "$out/framer_test"
gcc -std=c11 -g -fsanitize=address,undefined -fno-omit-frame-pointer -Dmalloc=uwb_test_malloc -I "$vendor" \
 -c "$vendor/nlink_linktrack_nodeframe3.c" -o "$out/frame3_fail.o"
g++ -std=c++17 -g -fsanitize=address,undefined -fno-omit-frame-pointer \
 -I "$root/packages/nlink_parser/include" -I "$vendor" \
 "$root/packages/nlink_parser/src/ros2/framer.cpp" "$root/packages/nlink_parser/test/allocation_failure_test.cpp" \
 "$out/frame3_fail.o" "$out/nlink_utils.c.o" -o "$out/allocation_failure_test"
ASAN_OPTIONS=verify_asan_link_order=0 UBSAN_OPTIONS=halt_on_error=1 "$out/allocation_failure_test"
