#!/usr/bin/env bash
set -euo pipefail
if [[ $# != 1 || $1 != /home/dev/ws_iap/* ]]; then echo 'usage: data-run -- tools/record.sh /home/dev/ws_iap/path/new_bag' >&2; exit 2; fi
set +u
source /opt/ros/jazzy/setup.bash
source /home/dev/ws_iap/uwb_ros2/install/setup.bash
set -u
root=$(cd "$(dirname "$0")/.." && pwd)
exec ros2 bag record --output "$1" --qos-profile-overrides-path "$root/packages/liuo_uwb_bridge/config/bag_qos.yaml" \
 /uwb/n0/nlink_linktrack_nodeframe0 /uwb/n0/nlink_linktrack_nodeframe2 /uwb/n0/nlink_linktrack_nodeframe3 /uwb/n0/nlink_linktrack_nodeframe5 /uwb/n0/nlink_linktrack_nodeframe6 \
 /uwb/n0/nlink_linktrack_nodeframe0_stamped /uwb/n0/nlink_linktrack_nodeframe2_stamped /uwb/n0/nlink_linktrack_nodeframe3_stamped /uwb/n0/nlink_linktrack_nodeframe5_stamped /uwb/n0/nlink_linktrack_nodeframe6_stamped \
 /liuo/uwb/observations /liuo/uwb/diagnostics /uwb/n0/nlink_diagnostics
