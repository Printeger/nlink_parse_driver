#!/usr/bin/env python3
"""Software PTY -> public ROS2 raw/stamped regression. No radio hardware."""
import argparse
import json
import os
from pathlib import Path
import pty
import signal
import subprocess
import time
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
import nlink_parser.msg as msgs

parser=argparse.ArgumentParser();parser.add_argument('--output',required=True);args=parser.parse_args()
out=Path(args.output);out.mkdir(parents=True,exist_ok=True)
root=Path(__file__).resolve().parents[1]
fixtures=[bytes.fromhex(x) for x in json.loads((root/'packages/nlink_parser/test/linktrack_fixtures.json').read_text())]
rclpy.init();node=rclpy.create_node('pty_driver_acceptance')
received={};subs=[]
qos=QoSProfile(depth=100,reliability=ReliabilityPolicy.BEST_EFFORT)
for name in ['Anchorframe0','Tagframe0']+[f'Nodeframe{i}' for i in range(7)]:
    received[name]=[]
    subs.append(node.create_subscription(getattr(msgs,'Linktrack'+name),'/test/nlink_linktrack_'+name.lower(),lambda m,n=name:received[n].append(m),qos))
    if name in ['Nodeframe0','Nodeframe2','Nodeframe3','Nodeframe5','Nodeframe6']:
        received[name+'Stamped']=[]
        subs.append(node.create_subscription(getattr(msgs,'Linktrack'+name+'Stamped'),'/test/nlink_linktrack_'+name.lower()+'_stamped',lambda m,n=name:received[n+'Stamped'].append(m),qos))
def spin(seconds):
    end=time.monotonic()+seconds
    while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=0.01)
def wait_count(name,n):
    end=time.monotonic()+3
    while len(received[name])<n and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=0.01)
    assert len(received[name])==n,(name,n,len(received[name]))
master,slave=pty.openpty();port=os.ttyname(slave)
log=(out/'driver.log').open('w')
proc=subprocess.Popen(['ros2','run','nlink_parser','linktrack','--ros-args','-r','__ns:=/test','-p',f'port_name:={port}'],stdout=log,stderr=log,start_new_session=True)
try:
    spin(2)
    for wire in fixtures:os.write(master,wire);spin(0.1)
    spin(0.3)
    for name,local,system,voltage in [('Anchorframe0',33313,32000,4.995),('Tagframe0',44556,71627,4.948),('Nodeframe1',34304,33000,4.936),('Nodeframe2',9157,1926842,4.973),('Nodeframe3',463352,7262319,4.954),('Nodeframe4',106020,106020,4.44),('Nodeframe5',463352,7262319,4.954)]:
        m=received[name][-1];assert m.local_time==local and m.system_time==system,(name,m)
        assert abs(m.voltage-voltage)<0.001
    for name in ['Nodeframe0','Nodeframe6']:
        m=received[name][-1];assert [len(x.data) for x in m.nodes]==[9,37]
    for name,dist in [('Nodeframe2',[3.179,5.548,6.728,4.651]),('Nodeframe3',[2.85,6.051,7.304,5.35]),('Nodeframe5',[2.85,6.051,7.304,5.35])]:
        for actual,expected in zip(received[name][-1].nodes,dist):assert abs(actual.dis-expected)<0.001
        assert received[name][-1]==received[name+'Stamped'][-1].frame
    f3=fixtures[5];initial=len(received['Nodeframe3'])
    for cut in range(1,len(f3)):
        os.write(master,f3[:cut]);spin(0.015)
        assert len(received['Nodeframe3'])==initial+cut-1
        os.write(master,f3[cut:]);wait_count('Nodeframe3',initial+cut)
    before=len(received['Nodeframe3']);os.write(master,f3*3);wait_count('Nodeframe3',before+3)
    wrapped=received['Nodeframe3Stamped'][-3:]
    assert len({x.context.rx_frame_seq for x in wrapped})==3
    assert all(x.frame.header==x.context.header for x in wrapped)
    before=len(received['Nodeframe3'])
    bad=bytearray(f3);bad[2:4]=b'\xff\xff'
    os.write(master,b'garbage'+bad+f3);wait_count('Nodeframe3',before+1)
    before=len(received['Nodeframe3']);bad=bytearray(f3);bad[-1]^=1
    os.write(master,bad+f3);wait_count('Nodeframe3',before+1)
    print(json.dumps({'status':'PASS','source':'software PTY','raw_frames':{k:len(v) for k,v in received.items()},'checks':['all imported LinkTrack types','device milliseconds','raw/stamped equal','Frame3 every split','concatenation sequences','bad length/checksum recovery']},indent=2))
finally:
    os.killpg(proc.pid,signal.SIGINT)
    try:proc.wait(timeout=5)
    except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
    log.close();os.close(master);os.close(slave);node.destroy_node();rclpy.shutdown()
