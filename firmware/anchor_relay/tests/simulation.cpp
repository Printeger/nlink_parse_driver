#include "anchor_relay/relay.hpp"
#include <cassert>
#include <iostream>
#include <map>
#include <vector>
using namespace liuo::uwb;using namespace liuo::uwb::mcu;
struct Fake:Hal {uint64_t now{};bool partial{};std::vector<Packet> packets;uint64_t monotonic_ms()override{return now;}size_t write(const char* p,size_t n)override{Packet q;assert(decode({p,n},q)==Error::ok);packets.push_back(q);return partial?n-1:n;}};
Snapshot snapshot(size_t n,uint64_t now,uint32_t seq){Snapshot s;s.reporter=1;s.received_ms=now;s.seq=seq;s.local_ms=uint32_t(now);s.count=n;for(size_t i=0;i<n;++i)s.edges[i]={uint8_t(i+2),3,10000,160,150};return s;}
int main(){
 for(size_t neighbors:{8u,18u}){
  Fake hal;Config config;config.simulator=true;config.phase_ms=0;config.jitter_ms=0;Relay relay(1,42,hal,config);
  for(hal.now=0;hal.now<1000;++hal.now){if(hal.now%20==0)relay.local_frame(snapshot(neighbors,hal.now,uint32_t(hal.now/20+1)));relay.tick();}
  assert(relay.sent==(neighbors==8?10u:20u));std::map<uint32_t,std::vector<Packet>> cycles;
  for(auto p:hal.packets)cycles[p.tx_cycle_seq].push_back(p);assert(cycles.size()==5);
  for(auto c:cycles){size_t edges=0;uint32_t seq=c.second[0].source_frame_seq;for(auto p:c.second){assert(p.source_frame_seq==seq);edges+=p.edge_count;}assert(edges==neighbors);}
  size_t before=hal.packets.size();hal.now=2000;relay.tick();assert(hal.packets.size()==before);
  std::cout<<"neighbors="<<neighbors<<" complete_cycles=5 packets="<<relay.sent<<" each_edge_hz=5 (simulation)\n";
 }
 Fake hal;Config config;config.simulator=true;config.phase_ms=0;config.jitter_ms=0;Relay relay(1,42,hal,config);
 auto s=snapshot(8,0,1);relay.local_frame(s);relay.tick();hal.now=100;hal.partial=true;relay.tick();assert(relay.short_writes==1 && relay.sent==1);
 hal.now=200;relay.local_frame(snapshot(50,200,2));relay.tick();assert(relay.overload==1 && relay.pending()==0);
 hal.now=400;relay.local_frame(snapshot(8,400,3));relay.tick();hal.now=551;relay.tick();assert(relay.late>0);
 auto lookup=[](uint8_t role,uint32_t id)->std::optional<Node>{if(role || id>20)return {};return Node{uint8_t(id),id!=0};};
 Receiver rx(1,relay,lookup);
 const uint8_t data[]={0x55,2,12,0,0,1,0,0,0,0,0,0x64}; // no blocks, additive checksum
 rx.feed(data,sizeof(data),600);assert(rx.ignored_data==1);
 std::cout<<"PASS: frozen source, full rounds, stale/late/overload/partial TX and no re-relay\n";
}
