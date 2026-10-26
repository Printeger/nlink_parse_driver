#!/usr/bin/env python3
import argparse,json,os,pty,signal,subprocess,time
from pathlib import Path
import rospy
from ros1_test_support import TestNode, spin_wall
from std_msgs.msg import String
p=argparse.ArgumentParser();p.add_argument('--output',required=True);a=p.parse_args();out=Path(a.output);out.mkdir(parents=True,exist_ok=True)
node=TestNode('tx_acceptance');pub=node.advertise(String,'/tx/nlink_linktrack_data_transmission',8)
def spin(seconds):
 end=time.monotonic()+seconds
 while time.monotonic()<end:spin_wall(node,timeout_sec=.01)
def read(fd):
 try:return os.read(fd,4096)
 except BlockingIOError:return b''
for enabled in [False,True]:
 master,slave=pty.openpty();os.set_blocking(master,False);log=(out/f'{enabled}.log').open('w')
 proc=subprocess.Popen(['rosrun','nlink_parser','linktrack','__ns:=/tx',f'_port_name:={os.ttyname(slave)}',f'_enable_legacy_tx:={str(enabled).lower()}'],stdout=log,stderr=log,start_new_session=True)
 try:
  spin(1.5);pub.publish(String(data='~U:test\n'));spin(.2)
  assert read(master)==(b'~U:test\n' if enabled else b'')
  pub.publish(String(data='x'*101));spin(.2);assert read(master)==b''
 finally:
  os.killpg(proc.pid,signal.SIGINT);proc.wait(timeout=5);log.close();os.close(master);os.close(slave)
node.close();pass;print(json.dumps({'status':'PASS','default_no_write':True,'enabled_write':True,'oversize_rejected':True}))
