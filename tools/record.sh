#!/usr/bin/env bash
set -euo pipefail
bag=${1:?Usage: record.sh OUTPUT.bag [namespace [range_frame [data_frame]]]}
ns=${2:-/uwb/n0}
range=${3:-3}
data=${4:-0}
exec rosbag record -O "$bag" \
 "$ns/nlink_linktrack_nodeframe${range}_stamped" \
 "$ns/nlink_linktrack_nodeframe${data}_stamped" \
 /liuo/uwb/observations /liuo/uwb/diagnostics "$ns/nlink_diagnostics"
