#!/usr/bin/env python3
"""Bounded resource faults on PTY and explicit stamped scheduling/clock faults."""
import argparse,json,time,signal,os,subprocess
from pathlib import Path
import rospy
from ros1_test_support import TestNode, spin_wall
from nlink_parser.msg import LinktrackNodeframe3Stamped,LinktrackNodeframe0Stamped,LinktrackNode2,LinktrackNode0
from pty_bridge_test import Harness,range_frame,data_frame,packet,encode
p=argparse.ArgumentParser();p.add_argument('--output',required=True);a=p.parse_args();out=Path(a.output);out.mkdir(parents=True,exist_ok=True)
report={}
h=Harness(out/'resources',keys=16)
try:
    for i in range(1,101):h.expect(data_frame([(1,encode(packet(1,i,i)),0)]),1,.015)
    h.expect(data_frame([(1,encode(packet(1,1,1)),0)]),0)
    # Exhaust the bounded retired boot list; subsequent new boots fail closed.
    for boot in range(43,107):h.expect(data_frame([(1,encode(packet(1,1,1,boot)),0)]),1,.015)
    h.expect(data_frame([(1,encode(packet(1,1,1,107)),0)]),0)
    h.expect(data_frame([(1,encode(packet(1,100,100,42)),0)]),0)
    h.spin(1)
    diag={v.key:v.value for v in h.diags[-1].status[0].values}
    assert int(diag['packet_cache_keys'])<=16 and int(diag['observation_cache_keys'])<=16
    assert int(diag['tombstones'])==64
    report['resources']={'status':'PASS','diagnostics':diag}
finally:h.close()
h=Harness(out/'ordering')
try:
    driver=h.procs[0];os.killpg(driver.pid,signal.SIGINT);driver.wait(timeout=5)
    qos=100
    local=h.node.advertise(LinktrackNodeframe3Stamped,'/uwb/n0/nlink_linktrack_nodeframe3_stamped',qos)
    remote=h.node.advertise(LinktrackNodeframe0Stamped,'/uwb/n0/nlink_linktrack_nodeframe0_stamped',qos);h.spin(1)
    t=time.monotonic_ns();ros=time.time_ns()
    def context(m,mono,wall,seq,mark):
        m.context.header.seq=7  # Deliberately constant ROS1 transport field, not protocol identity.
        m.context.host_monotonic_ns=mono;m.context.port_session_id=777;m.context.rx_frame_seq=seq;m.context.function_mark=mark
        m.context.header.stamp.secs=wall//1000000000;m.context.header.stamp.nsecs=wall%1000000000
    d=LinktrackNodeframe0Stamped();context(d,t,ros,2,2);n=LinktrackNode0();n.id=1;n.data=list(encode(packet()));d.frame.nodes=[n]
    remote.publish(d);h.spin(.03)
    m=LinktrackNodeframe3Stamped();context(m,t-10000000,ros-10000000,1,5);n=LinktrackNode2();n.id=1;n.dis=10.;n.fp_rssi=-80.;n.rx_rssi=-75.;m.frame.nodes=[n];m.frame.local_time=100
    local.publish(m);h.spin(.1);assert h.count()==2
    epochs=[b.driver_session_id for _,b in h.messages];assert len(set(epochs))==1
    base_mono=t-10000000
    for i in range(16):
        t=time.monotonic_ns();wall=time.time_ns();context(m,t,wall,i+3,5);m.frame.local_time=100+(t-base_mono)//1000000;local.publish(m);h.spin(.02)
    assert h.messages[-1][1].observations[0].time_basis==1
    prev=h.messages[-1][1].observations[0].source_session_id
    context(m,time.monotonic_ns(),time.time_ns()+30000000000,20,5);m.frame.local_time+=20;local.publish(m);h.spin(.1)
    o=h.messages[-1][1].observations[0];assert o.time_basis==0 and o.source_frame_stamp.secs==0 and o.source_session_id!=prev
    # Rates use current steady time after all streams stop, not last receive time.
    h.spin(11)
    diag={v.key:v.value for v in h.diags[-1].status[0].values}
    assert float(diag['source_frame_rate_10s'])==0 and float(diag['unique_packet_rate_10s'])==0,diag
    report['ordering_clock_stale']={'status':'PASS','cross_topic_kept':2,'clock_jump_invalidated':True,'stopped_10s_rates_zero':True}
finally:h.close()
h=Harness(out/'remote-clock',mode='DR_MODE0',shared=True)
try:
    driver=h.procs[0];os.killpg(driver.pid,signal.SIGINT);driver.wait(timeout=5)
    qos=100
    local=h.node.advertise(LinktrackNodeframe3Stamped,'/uwb/n0/nlink_linktrack_nodeframe3_stamped',qos)
    remote=h.node.advertise(LinktrackNodeframe0Stamped,'/uwb/n0/nlink_linktrack_nodeframe0_stamped',qos);h.spin(1)
    base=time.monotonic_ns();m=LinktrackNodeframe3Stamped();n=LinktrackNode2();n.id=1;n.dis=10.;m.frame.nodes=[n]
    for i in range(24):
        t=time.monotonic_ns();context(m,t,time.time_ns(),i+1,5);m.frame.local_time=1000+(t-base)//1000000;m.frame.system_time=m.frame.local_time;local.publish(m);h.spin(.02)
    d=LinktrackNodeframe0Stamped();n=LinktrackNode0();n.id=1
    from dataclasses import replace
    n.data=list(encode(replace(packet(),source_system_ms=m.frame.system_time)));d.frame.nodes=[n]
    context(d,time.monotonic_ns(),time.time_ns(),25,2);remote.publish(d);h.spin(.05)
    assert h.messages[-1][1].observations[0].time_basis==2
    n.data=list(encode(replace(packet(seq=2,cycle=2),source_system_ms=m.frame.system_time)));d.frame.nodes=[n]
    context(d,time.monotonic_ns(),time.time_ns()+30000000000,26,2);remote.publish(d);h.spin(.05)
    obs=h.messages[-1][1].observations[0];assert obs.time_basis==0 and obs.source_frame_stamp.secs==0
    report['remote_only_ros_jump']={'status':'PASS','previous_time_basis':2,'after_jump':0}
finally:h.close();pass
(out/'report.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
