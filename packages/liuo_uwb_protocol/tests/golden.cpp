#include "liuo_uwb_protocol/codec.hpp"
#include <cassert>
using namespace liuo::uwb;
int main() {
  constexpr auto golden="~U:TFIBBwEFQDAgEAcAAADSBAAAQOIBAAjiAQAMAAABAgMGKAAAopgDA4EoAACjmQQD/CgAAKSaBQN3KQAApZsGA/IpAACmnEZn\n";
  Packet p; assert(decode(golden,p)==Error::ok);
  assert(p.boot_id==0x10203040 && p.source_frame_seq==1234 && p.edge_count==5);
  assert(p.edges[0].distance_mm==10246 && p.edges[4].distance_mm==10738);
  Wire w; assert(encode(p,w)==Error::ok && w.view()==golden);
  Packet untouched=p;assert(decode("~U:bad\n",untouched)!=Error::ok);assert(untouched.boot_id==p.boot_id);
  assert(decode(golden,untouched,1000)==Error::range);
  p.flags=0;p.source_local_ms=0;p.source_system_ms=0;p.hold_age_ms=65535;
  for(auto& edge:p.edges){edge.flags=1;edge.fp_code=0;edge.rx_code=0;}
  assert(encode(p,w)==Error::ok);Packet unknown;assert(decode(w.view(),unknown)==Error::ok && unknown.flags==0 && unknown.edges[0].flags==1);
  assert(crc16(reinterpret_cast<const uint8_t*>("123456789"),9)==0x29b1);
}
