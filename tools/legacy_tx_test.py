#!/usr/bin/env python3
import argparse,json,os,pty,signal,subprocess,time
from pathlib import Path
import rclpy
from std_msgs.msg import String
from rclpy.qos import QoSProfile,ReliabilityPolicy
p=argparse.ArgumentParser();p.add_argument('--output',required=True);a=p.parse_args();out=Path(a.output);out.mkdir(parents=True,exist_ok=True)
rclpy.init();node=rclpy.create_node('tx_acceptance');pub=node.create_publisher(String,'/tx/nlink_linktrack_data_transmission',QoSProfile(depth=8,reliability=ReliabilityPolicy.BEST_EFFORT))
def spin(seconds):
 end=time.monotonic()+seconds
 while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
def read(fd):
 try:return os.read(fd,4096)
 except BlockingIOError:return b''
for enabled in [False,True]:
 master,slave=pty.openpty();os.set_blocking(master,False);log=(out/f'{enabled}.log').open('w')
 proc=subprocess.Popen(['ros2','run','nlink_parser','linktrack','--ros-args','-r','__ns:=/tx','-p',f'port_name:={os.ttyname(slave)}','-p',f'enable_legacy_tx:={str(enabled).lower()}'],stdout=log,stderr=log,start_new_session=True)
 try:
  spin(1.5);pub.publish(String(data='~U:test\n'));spin(.2)
  assert read(master)==(b'~U:test\n' if enabled else b'')
  pub.publish(String(data='x'*101));spin(.2);assert read(master)==b''
 finally:
  os.killpg(proc.pid,signal.SIGINT);proc.wait(timeout=5);log.close();os.close(master);os.close(slave)
node.destroy_node();rclpy.shutdown();print(json.dumps({'status':'PASS','default_no_write':True,'enabled_write':True,'oversize_rejected':True}))
