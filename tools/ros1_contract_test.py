#!/usr/bin/env python3
"""ROS1 private parameters, registry failures, Header.seq and paused clock."""
import argparse,json,subprocess,time
from pathlib import Path
import rospy,yaml
from rosgraph_msgs.msg import Clock
from pty_bridge_test import Harness,range_frame,root
p=argparse.ArgumentParser();p.add_argument('--output',required=True);a=p.parse_args()
out=Path(a.output);out.mkdir(parents=True,exist_ok=True)
h=Harness(out/'clock')
report={}
try:
    # Invalid mapping is observable through diagnostics, not accepted as N0.
    h.expect(range_frame(1,receiver=19,peers=(12,)),0)
    h.spin(1.1)
    diag={v.key:v.value for v in h.diags[-1].status[0].values}
    assert int(diag['unknown_receiver'])==1
    report['wrong_mapping_diagnostic']=True
    # Restart these processes after setting the global ROS1 parameter. No /clock
    # publisher exists yet: both input and diagnostic paths must keep running.
    from bag_test_utils import stop
    for proc in h.procs:stop(proc)
    h.procs.clear()
    rospy.set_param('/use_sim_time',True)
    h.start(['rosrun','nlink_parser','linktrack','__ns:=/uwb/n0',f'_port_name:={h.port}'],'sim_driver')
    h.start(['rosrun','liuo_uwb_bridge','range_bridge',f'_registry_path:={h.registry}'],'sim_bridge')
    h.spin(1.5);h.messages.clear();h.diags.clear();h.driver_diags.clear()
    for i in range(12):h.expect(range_frame(i+1),2,.02)
    assert all(o.received_stamp.to_nsec()==0 and o.time_basis==0 and o.source_frame_stamp.to_nsec()==0 for _,b in h.messages for o in b.observations)
    h.spin(1.1);assert h.diags and h.driver_diags
    report['no_clock_serial_and_diagnostics']=True
    clock=rospy.Publisher('/clock',Clock,queue_size=1);time.sleep(.3)
    clock.publish(Clock(clock=rospy.Time(100)));h.spin(.1)
    before=h.count()
    for i in range(12,24):h.expect(range_frame(i+1),2,.02)
    assert h.count()==before+24
    assert all(o.time_basis==0 for _,b in h.messages[-12:] for o in b.observations)
    clock.publish(Clock(clock=rospy.Time(10)));h.spin(.1)
    h.expect(range_frame(25),2)
    assert h.messages[-1][1].observations[0].received_stamp.secs==10
    clock.publish(Clock(clock=rospy.Time(200)));h.spin(.1);h.expect(range_frame(26),2)
    h.spin(1.1)
    report['paused_and_backward_forward_clock']=True
    clock.unregister()
finally:
    h.close();rospy.set_param('/use_sim_time',False)

base=yaml.safe_load((root/'packages/liuo_uwb_bridge/config/registry.yaml').read_text())
assert any(n['raw_id']==19 and n['logical_id']==0 and not n['is_fixed'] for n in base['nodes'])
for label,change in [
    ('duplicate_raw',lambda x:x['nodes'].append(dict(x['nodes'][0],logical_id=200,is_fixed=True))),
    ('duplicate_logical',lambda x:x['nodes'].append(dict(x['nodes'][1],raw_id=222))),
    ('missing_n0',lambda x:x['nodes'].pop(0)),
    ('fixed_conflict',lambda x:x['nodes'][0].update(is_fixed=True))]:
    cfg=yaml.safe_load(yaml.safe_dump(base));change(cfg);path=out/(label+'.yaml');path.write_text(yaml.safe_dump(cfg))
    result=subprocess.run(['rosrun','liuo_uwb_bridge','range_bridge',f'_registry_path:={path}'],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=8)
    (out/(label+'.log')).write_text(result.stdout)
    assert result.returncode!=0,(label,result.stdout)
    report[label]='rejected'
for key,value in [('range_frame',4),('data_frame',3)]:
    result=subprocess.run(['rosrun','nlink_parser','linktrack',f'_{key}:={value}'],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=8)
    assert result.returncode!=0
    report[key]='invalid rejected'
report['status']='PASS'
(out/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
