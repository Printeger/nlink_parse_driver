#!/usr/bin/env python3
"""Synthetic PTY -> real ROS1 bag; direct and reset/seek bridge replay."""
import argparse,json,os,signal,subprocess
from pathlib import Path
import rospy,rosbag
from std_srvs.srv import Trigger
from pty_bridge_test import Harness,range_frame,root

def stop(p):
    if p.poll() is None:
        os.killpg(p.pid,signal.SIGINT)
        try:p.wait(timeout=8)
        except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()

p=argparse.ArgumentParser();p.add_argument('--output',required=True);a=p.parse_args()
out=Path(a.output);out.mkdir(parents=True,exist_ok=True)
h=Harness(out/'harness')
try:
    for proc in h.procs:stop(proc)
    h.start(['roslaunch','liuo_uwb_bridge','robot.launch',f'port:={h.port}',f'registry:={h.registry}','baud:=460800','namespace:=/uwb/n0'],'launch')
    h.spin(2)
    assert rospy.get_param('/uwb/n0/linktrack/baud_rate')==460800
    assert rospy.get_param('/uwb/n0/linktrack/port_name')==str(h.port)
    assert rospy.get_param('/liuo_uwb_bridge/registry_path')==str(h.registry)
    bag=out/'recorded.bag'
    h.start([str(root/'tools/record.sh'),str(bag)],'record');recorder=h.procs[-1];h.spin(2)
    for i in range(20):h.send(range_frame(i+1));h.spin(.02)
    h.spin(.5);stop(recorder)
    topic='/uwb/n0/nlink_linktrack_nodeframe3_stamped'
    with rosbag.Bag(str(bag)) as reader:
        frames=list(reader.read_messages(topics=[topic]))
        original=list(reader.read_messages(topics=['/liuo/uwb/observations']))
    assert len(frames)==20 and len(original)==20,(len(frames),len(original))
    for proc in h.procs:stop(proc)
    h.messages.clear();h.spin(.3)
    h.start(['rosbag','play',str(bag),'--wait-for-subscribers','--delay','1','--topics','/liuo/uwb/observations'],'direct')
    h.procs[-1].wait(timeout=15);h.spin(.5)
    assert len(h.messages)==20,len(h.messages)
    assert [m.batch_seq for _,m in h.messages]==[m.batch_seq for _,m,_ in original]
    h.start(['rosrun','liuo_uwb_bridge','range_bridge',f'_registry_path:={h.registry}','_replay_mode:=true'],'replay_bridge');h.spin(1)
    reset=rospy.ServiceProxy('/liuo_uwb_bridge/reset_replay',Trigger)
    epochs=[]
    for run in range(3):
        if run:assert reset().success
        h.messages.clear()
        # Third pass seeks into the bag, with an explicit fresh dataset reset.
        cmd=['rosbag','play',str(bag),'--wait-for-subscribers','--delay','1','--topics',topic]
        if run==2:cmd+=['--start','0.1']
        h.start(cmd,f'play-{run}');h.procs[-1].wait(timeout=15);h.spin(.5)
        actual=[m.observations[0].received_stamp.to_nsec() for _,m in h.messages]
        recorded=[m.context.header.stamp.to_nsec() for _,m,_ in frames]
        if run<2:assert actual==recorded,(len(actual),len(recorded))
        else:assert actual and actual==recorded[-len(actual):]
        epochs.append(h.messages[0][1].driver_session_id)
        assert h.node.count_publishers('/liuo/uwb/observations')==1
    assert len(set(epochs))==3
    # Actual rosbag /clock pause/resume, bridge-only replay launch.
    for proc in h.procs:stop(proc)
    h.start(['roslaunch','liuo_uwb_bridge','replay.launch',f'registry:={h.registry}'],'sim_replay')
    h.spin(1);h.messages.clear()
    pause_log=(out/'paused-play.log').open('w');h.logs.append(pause_log)
    paused=subprocess.Popen(['rosbag','play',str(bag),'--clock','--pause','--rate','0.2',
                             '--wait-for-subscribers','--delay','1','--topics',topic],
                            stdin=subprocess.PIPE,stdout=pause_log,stderr=pause_log,start_new_session=True)
    h.procs.append(paused);h.spin(1.5)
    assert not h.messages
    paused.stdin.write(b' ');paused.stdin.flush();h.spin(.35)
    paused.stdin.write(b' ');paused.stdin.flush();h.spin(.15)
    before=len(h.messages);diagnostics_before=len(h.diags);h.spin(1.2)
    assert 0<before<20 and len(h.messages)==before,(before,len(h.messages))
    assert len(h.diags)>diagnostics_before,'steady diagnostics stalled with /clock'
    paused.stdin.write(b' ');paused.stdin.flush();paused.wait(timeout=15);h.spin(.5)
    assert [m.observations[0].received_stamp.to_nsec() for _,m in h.messages]==recorded
    assert h.node.count_publishers('/liuo/uwb/observations')==1
    report=dict(clock_pause_resume=True,paused_diagnostics=True,status='PASS',launch_baud=460800,recorded_stamped=20,direct_observations=20,
                replay_passes=2,seek_reset=True,received_times_preserved=True,dataset_sessions=epochs,bag=str(bag))
    (out/'report.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
finally:
    h.close();rospy.set_param('/use_sim_time',False)
