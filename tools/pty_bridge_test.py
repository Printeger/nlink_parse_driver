#!/usr/bin/env python3
"""Raw serial PTY -> public ROS1 observations; explicitly synthetic inputs."""
import argparse
from collections import Counter
from dataclasses import replace
import json
import math
import os
from pathlib import Path
import pty
import signal
import struct
import subprocess
import sys
import time
import rospy
from ros1_test_support import TestNode, spin_wall
from liuo_uwb_msgs.msg import UwbObservationBatch
from diagnostic_msgs.msg import DiagnosticArray
root=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(root/'packages/liuo_uwb_protocol/tests'))
from protocol_reference import Packet,Edge,encode

def checksum(raw):return raw+bytes([sum(raw)&255])
def range_frame(seq,frame=3,receiver=0,peers=(1,2),role=0,peer_role=0,local=None):
    local=(seq*20)&0xffffffff if local is None else local
    wide=frame==5;function={2:4,3:5,5:8}[frame];fixed={2:119,3:21,5:24}[frame]
    raw=bytearray(fixed);raw[0:2]=bytes([0x55,function]);raw[4]=role
    if wide:struct.pack_into('<I',raw,5,receiver)
    else:raw[5]=receiver
    if frame==2:struct.pack_into('<I',raw,102,local);struct.pack_into('<I',raw,6,local)
    else:struct.pack_into('<II',raw,9 if wide else 6,local,local)
    raw[-1]=len(peers)
    for peer in peers:
        raw+=bytes([peer_role])+(struct.pack('<I',peer) if wide else bytes([peer]))
        raw+=(10000).to_bytes(3,'little')+bytes([160,150])
        if frame==2:raw+=struct.pack('<I',local)+bytes(2)
    struct.pack_into('<H',raw,2,len(raw)+1)
    return checksum(raw)
def data_frame(blocks,frame=0,receiver=0,role=0):
    wide=frame==6;raw=bytearray(14 if wide else 11);raw[0:2]=bytes([0x55,9 if wide else 2]);raw[4]=role
    if wide:struct.pack_into('<I',raw,5,receiver)
    else:raw[5]=receiver
    raw[-1]=len(blocks)
    for sender,payload,sender_role in blocks:
        raw+=bytes([sender_role])+(struct.pack('<I',sender) if wide else bytes([sender]))+struct.pack('<H',len(payload))+payload
    struct.pack_into('<H',raw,2,len(raw)+1)
    return checksum(raw)
def packet(reporter=1,seq=1,cycle=1,boot=42,peers=(2,),shard=0,shards=1):
    return Packet(7,reporter,boot,cycle,seq,seq*20&0xffffffff,seq*20&0xffffffff,12,shard,shards,tuple(Edge(p,3,10000,160,150) for p in peers))

class Harness:
    def __init__(self,out,frame=3,data=0,mode='unconfirmed',wide=False,lp=False,keys=10000,shared=False):
        self.out=out;out.mkdir(parents=True,exist_ok=True);self.messages=[];self.diags=[];self.driver_diags=[];self.latencies=[]
        self.node=TestNode('bridge_acceptance')
        qos=100
        self.sub=self.node.subscribe(UwbObservationBatch,'/liuo/uwb/observations',self.receive,qos)
        self.diag=self.node.subscribe(DiagnosticArray,'/liuo/uwb/diagnostics',lambda m:self.diags.append(m),10)
        self.driver_diag=self.node.subscribe(DiagnosticArray,'/uwb/n0/nlink_diagnostics',lambda m:self.driver_diags.append(m),10)
        self.master,self.slave=pty.openpty();self.port=out/'serial';self.port.symlink_to(os.ttyname(self.slave))
        self.registry=out/'registry.yaml'
        rows=[]
        for i in range(20):
            raw=0x10203000+i if wide else (0 if lp else i)
            role=(2 if i==0 else i) if lp else 0
            rows.append(f'  - {{system_id: 0, operating_mode: {mode}, raw_role: {role}, raw_id: {raw}, logical_id: {i}, is_fixed: {str(i!=0).lower()}}}')
        # LP N0 TAG0 and fixed ANCHOR0; avoid other role collisions.
        if lp:rows=rows[:2]+[f'  - {{system_id: 0, operating_mode: LP, raw_role: 1, raw_id: {i}, logical_id: {i+1}, is_fixed: true}}' for i in range(1,19)]
        self.registry.write_text('nodes:\n'+'\n'.join(rows)+'\n')
        self.procs=[];self.logs=[]
        self.start(['rosrun','nlink_parser','linktrack','__ns:=/uwb/n0',f'_port_name:={self.port}',f'_range_frame:={frame}',f'_data_frame:={data}'],'driver')
        self.start(['rosrun','liuo_uwb_bridge','range_bridge',f'_registry_path:={self.registry}',f'_operating_mode:={mode}',f'_range_frame:={frame}',f'_data_frame:={data}','_simulator:=true',f'_dedup_max_entries:={keys}',f'_shared_system_clock:={str(shared).lower()}','_clock_domain:=test_clock'],'bridge')
        self.spin(2)
    def receive(self,m):self.messages.append((time.monotonic(),m))
    def start(self,cmd,name):
        log=(self.out/(name+'.log')).open('w');self.logs.append(log)
        self.procs.append(subprocess.Popen(cmd,stdout=log,stderr=log,start_new_session=True))
    def spin(self,seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:spin_wall(self.node,timeout_sec=min(.01,max(0,end-time.monotonic())))
    def send(self,b):os.write(self.master,b)
    def count(self):return sum(len(m.observations) for _,m in self.messages)
    def expect(self,b,additional,timeout=.3):
        before=self.count();self.send(b);self.spin(timeout)
        deadline=time.monotonic()+1
        while additional and self.count()<before+additional and time.monotonic()<deadline:self.spin(.005)
        assert self.count()==before+additional,(before,additional,self.count())
        assert all(p.poll() is None for p in self.procs),'process died'
    def close(self):
        for p in reversed(self.procs):
            if p.poll() is None:
                os.killpg(p.pid,signal.SIGINT)
                try:p.wait(timeout=5)
                except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
        for log in self.logs:log.close()
        os.close(self.master);os.close(self.slave);self.node.close()
    def reconnect(self):
        os.close(self.master);os.close(self.slave);self.spin(.7)
        self.master,self.slave=pty.openpty();self.port.unlink();self.port.symlink_to(os.ttyname(self.slave));self.spin(1)

def faults(out):
    h=Harness(out/'faults')
    try:
        h.expect(range_frame(1),2)
        p=packet();wire=encode(p)
        h.expect(data_frame([(1,wire,0)]),1)
        h.expect(data_frame([(1,wire,0)]),0)
        h.expect(data_frame([(1,encode(replace(p,tx_cycle_seq=2)),0)]),0)
        h.expect(data_frame([(1,encode(replace(p,tx_cycle_seq=3,source_frame_seq=2)),0)]),1)
        h.expect(data_frame([(2,encode(packet(2,1,1,43,(1,))),0)]),1)
        h.expect(data_frame([(2,encode(packet(1,10,10)),0)]),0)
        # Independent sender streams, including interleaved fragments.
        a=encode(packet(1,3,4));b=encode(packet(2,2,2,43,(1,)))
        h.expect(data_frame([(1,a[:31],0),(2,b[:31],0)]),0,timeout=.05)
        h.expect(data_frame([(2,b[31:],0),(1,a[31:],0)]),2)
        # Missing second shard cannot delay the first five edges.
        h.expect(data_frame([(1,encode(packet(1,4,5,42,(2,3,4,5,6),0,2)),0)]),5)
        # Fresh boot and old boot delay.
        h.expect(data_frame([(1,encode(packet(1,1,1,44)),0)]),1)
        h.expect(data_frame([(1,wire,0)]),0)
        # uint32 sequence rollover.
        h.expect(data_frame([(3,encode(packet(3,0xfffffffe,0xfffffffe,45,(1,))),0)]),1)
        h.expect(data_frame([(3,encode(packet(3,0xffffffff,0xffffffff,45,(1,))),0)]),1)
        h.expect(data_frame([(3,encode(packet(3,0,0,45,(1,))),0)]),1)
        # CRC, oversize line, wrong sender, bad outer block/checksum.
        corrupt=bytearray(encode(packet(1,2,2,44)));corrupt[20]=ord('A') if corrupt[20]!=ord('A') else ord('B')
        h.expect(data_frame([(1,bytes(corrupt),0)]),0)
        h.expect(data_frame([(1,b'~U:'+b'x'*150,0)]),0)
        h.expect(data_frame([(1,encode(packet(1,2,2,44)),0)]),1)
        bad=bytearray(data_frame([(1,wire,0)]));bad[13:15]=b'\xff\xff'
        h.expect(bytes(bad)+range_frame(2),2)
        # Device reset rotates local session; no variance/freshness fabrication.
        previous=h.messages[-1][1].observations[0].source_session_id
        h.expect(range_frame(3,local=0),2)
        assert h.messages[-1][1].observations[0].source_session_id!=previous
        for _,batch in h.messages:
            for o in batch.observations:
                assert math.isnan(o.time_uncertainty_s) and not o.pair_measurement_freshness_verified
                if o.kind==1:assert o.time_basis==0 and o.source_frame_stamp.secs==0
        # Reconnect clears partial serial and sender envelopes.
        h.expect(data_frame([(1,encode(packet(1,3,3,44))[:30],0)]),0)
        h.send(range_frame(4)[:12]);h.spin(.02);h.reconnect()
        h.expect(data_frame([(1,encode(packet(1,3,3,44))[30:],0)]),0)
        h.expect(range_frame(5),2)
        h.expect(data_frame([(1,encode(packet(1,3,3,44)),0)]),1)
        return {'status':'PASS','observations':h.count(),'checks':['packet/observation dedup','unchanged new range','bidirectional','sender fragmentation isolation','partial shard immediate','boot tombstone','uint32 wrap','CRC/length/overflow recovery','device reset','PTY reconnect clears halves','unknown timestamps']}
    finally:h.close()

def adapters(out):
    results=[]
    for f,d,mode,wide,lp in [(2,0,'unconfirmed',False,False),(5,6,'DR_MODE1',True,False),(3,0,'LP',False,True)]:
        h=Harness(out/f'adapter-{f}-{mode}',f,d,mode,wide,lp)
        try:
            receiver=0x10203000 if wide else 0;peer=0x10203001 if wide else (0 if lp else 1)
            h.expect(range_frame(1,f,receiver,(peer,),2 if lp else 0,1 if lp else 0),1)
            h.expect(data_frame([(peer,encode(packet(1,1,1,42,(2,))),1 if lp else 0)],d,receiver,2 if lp else 0),1)
            o=h.messages[-1][1].observations[0];assert o.reporter_id==1 and o.node_b==2
            if wide:assert not o.source_system_time_valid and o.time_basis==0
            results.append({'frame':f,'data':d,'mode':mode,'status':'PASS'})
        finally:h.close()
    return results

def load(out,seconds):
    h=Harness(out/'load');memory=[];next_memory=0;start=time.monotonic();next_local=start;next_remote=start;seq=0;cycle=0;peers=tuple(range(1,10));sent=Counter();arrivals=[]
    # This subscriber deliberately stalls on a separate executor/process, never the driver.
    slow=out/'slow.py';slow.write_text('import rospy,time\nfrom liuo_uwb_msgs.msg import UwbObservationBatch\nrospy.init_node("slow_subscriber");s=rospy.Subscriber("/liuo/uwb/observations",UwbObservationBatch,lambda m:time.sleep(.2),queue_size=2,tcp_nodelay=True);rospy.spin()\n')
    h.start([sys.executable,str(slow)],'slow')
    h.spin(1); h.messages.clear()
    start=time.monotonic(); next_local=start; next_remote=start
    expected={}; sent_windows=[Counter() for _ in range(int(seconds)//10)]
    try:
        while time.monotonic()<start+seconds:
            now=time.monotonic()
            if now>=next_memory:
                def rss_tree(pid):
                    total=0
                    try:
                        status=Path(f'/proc/{pid}/status').read_text()
                        total=int(next(x.split()[1] for x in status.splitlines() if x.startswith('VmRSS:')))
                        children=Path(f'/proc/{pid}/task/{pid}/children').read_text().split()
                        total+=sum(rss_tree(int(c)) for c in children)
                    except (FileNotFoundError,StopIteration,ProcessLookupError):pass
                    return total
                memory.append({'elapsed_s':now-start,'driver_kib':rss_tree(h.procs[0].pid),'bridge_kib':rss_tree(h.procs[1].pid)})
                next_memory=now+1
            if now>=next_local:
                seq+=1;h.send(range_frame(seq,peers=peers));sent['local_frames']+=1;next_local+=.02
                for peer in peers:expected[(0,peer,seq)]=int((now-start)//10)
            if now>=next_remote:
                cycle+=1
                for sender in range(1,4):
                    edges=tuple(i for i in range(1,10) if i!=sender)
                    for shard in range(2):
                        h.send(data_frame([(sender,encode(packet(sender,seq,cycle,100+sender,edges[5*shard:5*shard+5],shard,2)),0)]));sent['remote_packets']+=1
                    for peer in edges:expected[(sender,peer,seq)]=int((now-start)//10)
                next_remote+=.2
            spin_wall(h.node,timeout_sec=.001)
        h.spin(.3)
        counts=Counter();windows=[Counter() for _ in range(int(seconds)//10)]
        actual=Counter();latencies=[]
        for when,b in h.messages:
            for o in b.observations:
                identity=(o.reporter_id,o.node_b,o.source_frame_seq)
                # Local source seq counts all serial frames, so correlate each local
                # frame through its device millisecond counter instead.
                if o.kind==0:identity=(0,o.node_b,o.source_local_ms//20)
                actual[identity]+=1
                assert identity in expected,identity
                k=identity[:2];counts[k]+=1
                win=expected[identity]
                if win<len(windows):windows[win][k]+=1
        assert set(actual)==set(expected),(len(actual),len(expected))
        assert all(n==1 for n in actual.values()),'duplicate observations'
        for (reporter,peer,seq),win in expected.items():
            if win<len(sent_windows):sent_windows[win][(reporter,peer)]+=1
        assert windows==sent_windows,'per-edge 10s send/receive mismatch'
        local={f'{a}->{b}':counts[(a,b)]/seconds for a,b in [(0,i) for i in peers]}
        remote={f'{a}->{b}':counts[(a,b)]/seconds for a in range(1,4) for b in range(1,10) if a!=b}
        assert min(local.values())>=20,local
        for w in windows:assert all(w[(0,i)]/10>=20 for i in peers),w
        # Every simulated sender is logged; RX is reported independently.
        diag={v.key:v.value for v in h.diags[-1].status[0].values} if h.diags else {}
        assert int(diag.get('packet_cache_keys','10001'))<=10000
        assert int(diag.get('observation_cache_keys','10001'))<=10000
        return {'status':'PASS','source':'software PTY, no RF','seconds':seconds,'memory_samples':memory,'sent':dict(sent),'expected_observations':len(expected),'received_observations':sum(actual.values()),'per_edge_exact_match':True,'window_basis':'send time; tail drained, no extra input','local_edge_hz':local,'remote_edge_hz':remote,'10s_windows':[{f'{a}->{b}':n/10 for (a,b),n in w.items()} for w in windows],'diagnostics':diag,'driver_diagnostics':{v.key:v.value for v in h.driver_diags[-1].status[0].values} if h.driver_diags else {}}
    finally:h.close()

def main():
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--load-seconds',type=int,default=60);p.add_argument('--only',choices=['all','faults','load','adapters'],default='all');a=p.parse_args()
    out=Path(a.output);out.mkdir(parents=True,exist_ok=True);report={}
    try:
        if a.only in ['all','faults']:report['faults']=faults(out)
        if a.only in ['all','adapters']:report['adapters']=adapters(out)
        if a.only in ['all','load']:report['load']=load(out,a.load_seconds)
        (out/'report.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
    finally:pass
if __name__=='__main__':main()
