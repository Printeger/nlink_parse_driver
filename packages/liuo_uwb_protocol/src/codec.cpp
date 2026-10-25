#include "liuo_uwb_protocol/codec.hpp"
#include <algorithm>
namespace liuo::uwb {
namespace {
constexpr char alphabet[]="ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
void put(uint8_t* p,uint32_t v,size_t n) { for(size_t i=0;i<n;++i) p[i]=uint8_t(v>>(8*i)); }
uint32_t get(const uint8_t* p,size_t n) { uint32_t v=0; for(size_t i=0;i<n;++i) v|=uint32_t(p[i])<<(8*i); return v; }
void b64(const uint8_t* raw,size_t n,Wire& w) {
  w.size=3; w.bytes[0]='~'; w.bytes[1]='U'; w.bytes[2]=':';
  for(size_t i=0;i<n;i+=3) {
    uint32_t v=uint32_t(raw[i])<<16;
    if(i+1<n) v|=uint32_t(raw[i+1])<<8;
    if(i+2<n) v|=raw[i+2];
    w.bytes[w.size++]=alphabet[(v>>18)&63]; w.bytes[w.size++]=alphabet[(v>>12)&63];
    w.bytes[w.size++]=i+1<n?alphabet[(v>>6)&63]:'=';
    w.bytes[w.size++]=i+2<n?alphabet[v&63]:'=';
  }
  w.bytes[w.size++]='\n';
}
}
uint16_t crc16(const uint8_t* data,size_t n) {
  uint16_t c=0xffff;
  for(size_t i=0;i<n;++i) { c^=uint16_t(data[i])<<8; for(int j=0;j<8;++j) c=uint16_t((c<<1)^((c&0x8000)?0x1021:0)); }
  return c;
}
Error validate(const Packet& p,int32_t limit) {
  if(p.flags&~7) return Error::flags;
  if(!p.reporter_id || p.reporter_id==255 || !p.boot_id) return Error::identity;
  if(!p.edge_count || p.edge_count>5) return Error::length;
  if(!p.shard_count || p.shard_count>51 || p.shard_index>=p.shard_count) return Error::shard;
  if(((p.flags&4)!=0)==(p.hold_age_ms==65535) || (!(p.flags&1)&&p.source_system_ms) || (!(p.flags&2)&&p.source_local_ms)) return Error::time;
  uint8_t prev=0;
  for(size_t i=0;i<p.edge_count;++i) {
    const auto& e=p.edges[i];
    if(!e.peer_id || e.peer_id==255 || e.peer_id==p.reporter_id) return Error::identity;
    if(e.peer_id<=prev) return Error::order;
    prev=e.peer_id;
    if((e.flags&~3) || !(e.flags&1) || (!(e.flags&2)&&(e.fp_code||e.rx_code))) return Error::flags;
    if(e.distance_mm<=0 || e.distance_mm>0x7fffff || e.distance_mm>limit) return Error::range;
  }
  return Error::ok;
}
Error encode(const Packet& p,Wire& w,int32_t limit) {
  w.size=0; auto err=validate(p,limit); if(err!=Error::ok) return err;
  std::array<uint8_t,72> raw{};
  raw[0]='L'; raw[1]='R'; raw[2]=1; raw[3]=p.flags; raw[4]=p.reporter_id; raw[5]=p.edge_count;
  put(raw.data()+6,p.boot_id,4); put(raw.data()+10,p.tx_cycle_seq,4); put(raw.data()+14,p.source_frame_seq,4);
  put(raw.data()+18,p.source_local_ms,4); put(raw.data()+22,p.source_system_ms,4); put(raw.data()+26,p.hold_age_ms,2);
  raw[28]=p.shard_index; raw[29]=p.shard_count;
  for(size_t i=0;i<p.edge_count;++i) { auto* r=raw.data()+30+i*8; const auto& e=p.edges[i]; r[0]=e.peer_id; r[1]=e.flags; put(r+2,uint32_t(e.distance_mm),4); r[6]=e.fp_code; r[7]=e.rx_code; }
  size_t n=30+8*p.edge_count; put(raw.data()+n,crc16(raw.data(),n),2); b64(raw.data(),n+2,w); return Error::ok;
}
Error decode(std::string_view wire,Packet& result,int32_t limit) {
  if(wire.size()<4 || wire.size()>100 || wire.substr(0,3)!="~U:" || wire.back()!='\n') return Error::envelope;
  auto s=wire.substr(3,wire.size()-4); if(s.empty() || s.size()%4) return Error::base64;
  std::array<uint8_t,72> raw{}; size_t n=0;
  for(size_t i=0;i<s.size();i+=4) {
    uint32_t v=0; size_t pad=0;
    for(size_t j=0;j<4;++j) {
      char c=s[i+j]; v<<=6;
      if(c=='=') { if(i+4!=s.size() || j<2) return Error::base64; ++pad; }
      else { if(pad) return Error::base64; const char* a=std::find(alphabet,alphabet+64,c); if(a==alphabet+64) return Error::base64; v|=uint32_t(a-alphabet); }
    }
    if(n+3-pad>raw.size()) return Error::length;
    raw[n++]=uint8_t(v>>16); if(pad<2) raw[n++]=uint8_t(v>>8); if(!pad) raw[n++]=uint8_t(v);
  }
  Wire canonical; b64(raw.data(),n,canonical); if(canonical.view()!=wire) return Error::base64;
  if(n<40 || raw[5]<1 || raw[5]>5 || n!=32+8*size_t(raw[5])) return Error::length;
  if(raw[0]!='L' || raw[1]!='R' || raw[2]!=1) return Error::version;
  if(get(raw.data()+n-2,2)!=crc16(raw.data(),n-2)) return Error::crc;
  Packet p; p.flags=raw[3]; p.reporter_id=raw[4]; p.edge_count=raw[5]; p.boot_id=get(raw.data()+6,4);
  p.tx_cycle_seq=get(raw.data()+10,4); p.source_frame_seq=get(raw.data()+14,4);
  p.source_local_ms=get(raw.data()+18,4); p.source_system_ms=get(raw.data()+22,4); p.hold_age_ms=get(raw.data()+26,2);
  p.shard_index=raw[28]; p.shard_count=raw[29];
  for(size_t i=0;i<p.edge_count;++i) { const auto* r=raw.data()+30+8*i; p.edges[i]={r[0],r[1],static_cast<int32_t>(get(r+2,4)),r[6],r[7]}; }
  auto err=validate(p,limit); if(err==Error::ok) result=p; return err;
}
}
