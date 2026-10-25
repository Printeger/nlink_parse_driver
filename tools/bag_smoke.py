#!/usr/bin/env python3
"""Installed launch/parameters/QoS + actual rosbag2 record and replay smoke."""
import argparse,json,os,signal,subprocess,time
from pathlib import Path
import rclpy
from rclpy.serialization import deserialize_message
import rosbag2_py
from nlink_parser.msg import LinktrackNodeframe3Stamped
from pty_bridge_test import Harness,range_frame,root

def stop(p):
    if p.poll() is None:
        os.killpg(p.pid,signal.SIGINT)
        try:p.wait(timeout=5)
        except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()

p=argparse.ArgumentParser();p.add_argument('--output',required=True);a=p.parse_args();out=Path(a.output);out.mkdir(parents=True,exist_ok=True)
rclpy.init();h=Harness(out/'harness')
try:
    # Replace initial direct nodes with the installed launch, preserving the PTY and observer.
    for proc in h.procs:stop(proc)
    h.start(['ros2','launch','liuo_uwb_bridge','robot.launch.py',f'port:={h.port}',f'registry:={h.registry}','baud:=460800','namespace:=/uwb/n0'],'launch')
    h.spin(2)
    value=subprocess.check_output(['ros2','param','get','/uwb/n0/linktrack','baud_rate'],text=True)
    assert '460800' in value,value
    bag=out/'recorded'
    h.start([str(root/'tools/record.sh'),str(bag)],'record');recorder=h.procs[-1];h.spin(2)
    for i in range(20):h.send(range_frame(i+1));h.spin(.02)
    h.spin(.5);stop(recorder)
    reader=rosbag2_py.SequentialReader();reader.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id=''),rosbag2_py.ConverterOptions('',''))
    frames=[]
    while reader.has_next():
        topic,raw,t=reader.read_next()
        if topic.endswith('nodeframe3_stamped'):frames.append(deserialize_message(raw,LinktrackNodeframe3Stamped))
    assert len(frames)==20,len(frames)
    for proc in h.procs:stop(proc)
    h.messages.clear()
    h.start(['ros2','run','liuo_uwb_bridge','range_bridge','--ros-args','-p',f'registry_path:={h.registry}','-p','replay_mode:=true'],'replay_bridge');h.spin(1)
    epochs=[]
    for run in range(2):
        if run:
            result=subprocess.check_output(['ros2','service','call','/liuo_uwb_bridge/reset_replay','std_srvs/srv/Trigger','{}'],text=True);assert 'success=True' in result,result
        h.messages.clear()
        h.start(['ros2','bag','play',str(bag),'--topics','/uwb/n0/nlink_linktrack_nodeframe3_stamped','--qos-profile-overrides-path',str(root/'packages/liuo_uwb_bridge/config/bag_qos.yaml')],f'play-{run}')
        h.spin(3)
        assert len(h.messages)==20,len(h.messages)
        recorded=[(x.context.header.stamp.sec,x.context.header.stamp.nanosec) for x in frames]
        actual=[(x.observations[0].received_stamp.sec,x.observations[0].received_stamp.nanosec) for _,x in h.messages]
        assert recorded==actual
        epochs.append(h.messages[0][1].driver_session_id)
    assert epochs[0]!=epochs[1]
    report={'status':'PASS','launch_baud':460800,'recorded_stamped':len(frames),'replay_passes':2,'received_times_preserved':True,'dataset_sessions':epochs,'bag':str(bag)}
    (out/'report.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
finally:h.close();rclpy.shutdown()
