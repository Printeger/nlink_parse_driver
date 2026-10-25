#include "anchor_relay/relay.hpp"
#include <algorithm>
#include <bitset>
#include <stdexcept>
namespace liuo::uwb::mcu {
Relay::Relay(uint8_t reporter,uint32_t boot,Hal& hal,Config config):reporter_(reporter),boot_(boot),hal_(hal),config_(config) {
 if(!reporter || reporter==255 || !boot || config.period_ms!=200 || config.hold_ms>250 || !config.hold_ms || config.queue_ms>50 || config.gap_ms<25 || config.jitter_ms>2 || (!config.simulator && !config.budget_bytes_per_cycle))throw std::invalid_argument("invalid/unverified relay capacity");
 uint32_t phase=config.phase_ms==UINT32_MAX?((uint32_t(reporter)*2654435761u)^boot)%200:config.phase_ms;
 if(phase>=200)throw std::invalid_argument("phase outside cycle");
 next_cycle_=hal_.monotonic_ms()+phase;
}
void Relay::reset(uint32_t boot){if(!boot || boot==boot_)throw std::invalid_argument("boot must change");boot_=boot;have_=false;shards_=next_shard_=0;cycle_=0;}
bool Relay::local_frame(const Snapshot& frame){
 if(frame.reporter!=reporter_ || frame.count>254){++invalid;return false;}
 Snapshot clean=frame;clean.count=0;std::bitset<256> ids;
 for(size_t i=0;i<frame.count;++i){auto e=frame.edges[i];if(!e.peer_id || e.peer_id==255 || e.peer_id==reporter_ || ids[e.peer_id]){++invalid;continue;}
  Packet check;check.reporter_id=reporter_;check.boot_id=boot_;check.edge_count=1;check.edges[0]=e;if(validate(check)!=Error::ok){++invalid;continue;}
  ids.set(e.peer_id);clean.edges[clean.count++]=e;
 }
 std::sort(clean.edges.begin(),clean.edges.begin()+clean.count,[](auto a,auto b){return a.peer_id<b.peer_id;});latest_=clean;have_=true;return true;
}
void Relay::tick(){
 now_=hal_.monotonic_ms();
 if(now_>=next_cycle_){
  discarded+=pending();shards_=next_shard_=0;++cycle_;++cycles;
  cycle_start_=next_cycle_;next_cycle_+=((now_-next_cycle_)/config_.period_ms+1)*config_.period_ms;
  if(now_-cycle_start_>config_.queue_ms){++late;return;}
  if(!have_ || now_<latest_.received_ms || now_-latest_.received_ms>config_.hold_ms || !latest_.count){++stale;return;}
  frozen_=latest_;size_t needed=(frozen_.count+4)/5;
  size_t wire_bytes=0;for(size_t i=0;i<needed;++i){size_t count=std::min<size_t>(5,frozen_.count-i*5);wire_bytes+=4+4*((32+8*count+2)/3);}
  if(needed*config_.gap_ms>config_.period_ms || (config_.budget_bytes_per_cycle && wire_bytes>config_.budget_bytes_per_cycle)){++overload;return;}
  shards_=needed;
 }
 if(next_shard_>=shards_)return;
 int jitter=config_.jitter_ms?int((boot_^cycle_^uint32_t(next_shard_)*17)%(2*config_.jitter_ms+1))-int(config_.jitter_ms):0;
 uint64_t due=cycle_start_+next_shard_*config_.period_ms/shards_;if(next_shard_ && jitter<0)due-=uint64_t(-jitter);else if(jitter>0)due+=uint64_t(jitter);
 if(now_<due)return;
 if(now_-due>config_.queue_ms){++late;++next_shard_;return;}
 if(now_<frozen_.received_ms || now_-frozen_.received_ms>config_.hold_ms){++stale;discarded+=pending();next_shard_=shards_;return;}
 if(transmitted_ && now_-last_tx_<config_.gap_ms)return;
 Packet p;p.flags=uint8_t(6|(frozen_.system_valid?1:0));p.reporter_id=reporter_;p.boot_id=boot_;p.tx_cycle_seq=cycle_;p.source_frame_seq=frozen_.seq;p.source_local_ms=frozen_.local_ms;p.source_system_ms=frozen_.system_valid?frozen_.system_ms:0;p.hold_age_ms=uint16_t(now_-frozen_.received_ms);p.shard_index=uint8_t(next_shard_);p.shard_count=uint8_t(shards_);
 size_t start=next_shard_*5;p.edge_count=uint8_t(std::min<size_t>(5,frozen_.count-start));for(size_t i=0;i<p.edge_count;++i)p.edges[i]=frozen_.edges[start+i];
 Wire wire;if(encode(p,wire)!=Error::ok){++invalid;++next_shard_;return;}
 last_tx_=now_;transmitted_=true;if(hal_.write(wire.bytes.data(),wire.size)==wire.size)++sent;else ++short_writes;++next_shard_;
}
namespace {
uint32_t u32(const uint8_t* p){return uint32_t(p[0])|uint32_t(p[1])<<8|uint32_t(p[2])<<16|uint32_t(p[3])<<24;}
int32_t i24(const uint8_t* p){uint32_t v=uint32_t(p[0])|uint32_t(p[1])<<8|uint32_t(p[2])<<16;return v&0x800000?int32_t(v)-0x1000000:int32_t(v);}
}
void Receiver::feed(const uint8_t* bytes,size_t length,uint64_t mono_ms){auto cb=[&](const uint8_t* p,size_t n){frame(p,n,mono_ms);};framer_.expire(mono_ms*1000000,cb);framer_.feed(bytes,length,mono_ms*1000000,cb);}
void Receiver::frame(const uint8_t* p,size_t,uint64_t now){
 if(p[1]!=4 && p[1]!=5 && p[1]!=8){++ignored_data;return;}
 bool wide=p[1]==8;uint32_t raw=wide?u32(p+5):p[5];auto self=registry_(p[4],raw);if(!self || !self->fixed || self->logical!=reporter_){++bad_identity;return;}
 Snapshot s;s.reporter=reporter_;s.received_ms=now;s.system_valid=system_valid_&&!wide;
 size_t offset,count,stride;
 if(p[1]==4){s.local_ms=u32(p+102);s.system_ms=u32(p+6);offset=119;count=p[118];stride=13;}
 else{size_t at=wide?9:6;s.local_ms=u32(p+at);s.system_ms=u32(p+at+4);offset=wide?24:21;count=p[offset-1];stride=wide?10:7;}
 if(have_time_ && s.local_ms-previous_time_>10000){++resets;relay_.device_reset();sequence_=0;}
 have_time_=true;previous_time_=s.local_ms;s.seq=++sequence_;++parsed;
 for(size_t i=0;i<count;++i){const auto* n=p+offset+i*stride;auto peer=registry_(n[0],wide?u32(n+1):n[1]);if(!peer || !peer->fixed || peer->logical==reporter_)continue;
  size_t at=wide?5:2;int32_t distance=i24(n+at);if(distance<=0)continue;
  s.edges[s.count++]={peer->logical,3,distance,n[at+3],n[at+4]};
 }
 relay_.local_frame(s);
}
}
