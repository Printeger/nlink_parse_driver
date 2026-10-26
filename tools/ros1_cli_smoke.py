#!/usr/bin/env python3
"""Exercise installed consumer, rosmsg, rostopic echo/hz with synthetic PTY."""
import argparse,json,subprocess
from pathlib import Path
from pty_bridge_test import Harness,range_frame
from bag_test_utils import stop
p=argparse.ArgumentParser();p.add_argument('--output',required=True);a=p.parse_args()
out=Path(a.output);out.mkdir(parents=True,exist_ok=True);h=Harness(out/'harness')
try:
    h.start(['rosrun','liuo_uwb_bridge','observation_reader.py'],'reader')
    h.start(['rostopic','echo','-n','1','/liuo/uwb/observations'],'echo');echo=h.procs[-1]
    h.start(['rostopic','hz','-w','10','/uwb/n0/nlink_linktrack_nodeframe3_stamped'],'hz')
    h.spin(1)
    for i in range(150):h.send(range_frame(i+1));h.spin(.02)
    echo.wait(timeout=5)
    for proc in h.procs[2:]:stop(proc)
    assert 'distance_m' in (out/'harness/echo.log').read_text()
    assert 'average rate:' in (out/'harness/hz.log').read_text()
    assert 'edge=N0->N1' in (out/'harness/reader.log').read_text()
    definitions={}
    for package in ['nlink_parser','liuo_uwb_msgs']:
        names=subprocess.check_output(['rosmsg','package',package],text=True).split()
        for name in names:
            definitions[name]={'md5':subprocess.check_output(['rosmsg','md5',name],text=True).strip(),
                               'definition':subprocess.check_output(['rosmsg','show',name],text=True)}
    (out/'message-definitions.json').write_text(json.dumps(definitions,indent=2))
    report=dict(status='PASS',source='synthetic PTY',installed_reader=True,echo=True,hz=True,message_types=len(definitions))
    (out/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
finally:h.close()
