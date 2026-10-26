#!/usr/bin/env python3
"""Installed launch: selected topics only, with both local and remote PTY input.

Run in an unused ROS_MASTER_URI; all serial inputs are synthetic.
"""
import argparse
import json
import os
from pathlib import Path
import pty
import signal
import subprocess
import time

import rospy
from ros1_test_support import TestNode, spin_wall
import yaml
import rospkg
def get_package_share_directory(name): return rospkg.RosPack().get_path(name)
from liuo_uwb_msgs.msg import UwbObservationBatch
from pty_bridge_test import data_frame, encode, packet, range_frame, root


def check(node, out, frame, data, namespace="/uwb/n0"):
    out.mkdir(parents=True, exist_ok=True)
    share = Path(get_package_share_directory('liuo_uwb_bridge'))
    config = yaml.safe_load((share / 'config/uwb.yaml').read_text())
    params = config
    assert params['publish_debug_topics'] is False
    params.update(range_frame=frame, data_frame=data, simulator=True)
    config_path = out / 'uwb.yaml'
    config_path.write_text(yaml.safe_dump(config))
    # Match the deployed robot mapping; don't depend on example fixed-node entries.
    registry = out / 'registry.yaml'
    registry.write_text(yaml.safe_dump({'nodes': [
        dict(system_id=0, operating_mode='unconfirmed', raw_role=0,
             raw_id=raw, logical_id=logical, is_fixed=logical != 0)
        for raw, logical in [(19, 0), (12, 12), (1, 1)]
    ]}))
    received = []
    sub = node.subscribe(UwbObservationBatch, '/liuo/uwb/observations',
                                   received.append, 100)
    master, slave = pty.openpty()
    log = (out / 'launch.log').open('w')
    proc = subprocess.Popen([
        'roslaunch', 'liuo_uwb_bridge', 'robot.launch',
        f'port:={os.ttyname(slave)}', f'config:={config_path}',
        f'registry:={registry}', f'namespace:={namespace}', 'baud:=460800',
    ], stdout=log, stderr=log, start_new_session=True)

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            spin_wall(node, timeout_sec=.02)

    def topics():
        return {topic for topic, owners in node.publishers().items()
                if namespace+'/linktrack' in owners and topic.startswith(namespace+'/')}

    expected = {namespace+'/nlink_diagnostics'} | {
        f'{namespace}/nlink_linktrack_nodeframe{i}_stamped' for i in [frame, data]
    }
    try:
        end = time.monotonic() + 8
        while time.monotonic() < end:
            spin(.1)
            if topics() == expected and node.count_publishers('/liuo/uwb/observations') == 1:
                break
        assert topics() == expected, (topics(), expected)
        spin(.5)
        # Even receiving every supported frame must not create disabled publishers.
        fixtures = json.loads((root / 'packages/nlink_parser/test/linktrack_fixtures.json').read_text())
        for fixture in fixtures:
            os.write(master, bytes.fromhex(fixture))
            spin(.03)
        spin(.1)
        received.clear()
        os.write(master, range_frame(100, frame, receiver=19, peers=(12,)))
        os.write(master, data_frame([(12, encode(packet(12, 100, 100, peers=(1,))), 0)],
                                    data, receiver=19))
        spin(.6)
        edges = {(o.kind, o.node_a, o.node_b) for m in received for o in m.observations}
        assert edges == {(0, 0, 12), (1, 12, 1)}, edges
        assert all(m.receiver_id == 0 for m in received)
        assert topics() == expected, topics()
        assert proc.poll() is None
        assert rospy.get_param(namespace+'/linktrack/baud_rate')==460800
        assert rospy.get_param('/liuo_uwb_bridge/input_namespace')==namespace
        business=set(node.publishers())-{'/rosout','/rosout_agg','/clock'}
        assert business==expected|{'/liuo/uwb/observations','/liuo/uwb/diagnostics'},business
        return dict(range_frame=frame, data_frame=data, status='PASS',
                    driver_topics=sorted(topics()), edges=sorted(edges))
    finally:
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGINT)
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait()
        log.close()
        os.close(master)
        os.close(slave)
        sub.unregister()
        spin(.3)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    node = TestNode('topic_visibility_acceptance')
    try:
        results = [check(node, Path(args.output) / f'{frame}-{data}', frame, data)
                   for frame, data in [(3, 0), (2, 0), (5, 6)]]
        results.append(check(node,Path(args.output)/'namespace',3,0,'/test/override'))
        print(json.dumps(results, indent=2))
    finally:
        node.close()
        pass


if __name__ == '__main__':
    main()
