#!/usr/bin/env bash
# Run after build_test.sh, with a fresh evidence directory. Owns its master only.
set -eo pipefail
root=$(cd "$(dirname "$0")/.." && pwd)
source "$root/tools/ros1_env.sh"
out=${1:-/home/mint/ws_calib/evidence/uwb_ros1/run-$(date +%Y%m%d-%H%M%S)}
mkdir -p "$out"
port=$(python3 -c 'import socket; s=socket.socket(); s.bind(("127.0.0.1",0)); print(s.getsockname()[1]); s.close()')
export ROS_MASTER_URI="http://127.0.0.1:$port"
roscore -p "$port" > "$out/master.log" 2>&1 &
master_pid=$!
trap 'kill -INT "$master_pid" 2>/dev/null || true; wait "$master_pid" 2>/dev/null || true' EXIT
python3 - <<'PY'
import os,time,xmlrpc.client
master=xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'])
for _ in range(100):
    try:
        if master.getPid('/acceptance')[0]==1:break
    except OSError:pass
    time.sleep(.1)
else:raise RuntimeError('test master did not start')
PY
{
 git -C "$root" rev-parse HEAD
 git -C "$root" status --short
 env | sort | rg 'ROS|CMAKE_PREFIX|PYTHON|TMPDIR'
 python3 --version
 cmake --version
 g++ --version
} > "$out/environment.txt"
for test in pty_driver_test legacy_tx_test pty_bridge_test state_integration_test topic_visibility_test ros1_contract_test bag_smoke; do
 echo "Running $test"
 python3 "$root/tools/$test.py" --output "$out/$test" > "$out/$test.log" 2>&1
done
echo "PASS: ROS1 software integration suite; evidence: $out"
