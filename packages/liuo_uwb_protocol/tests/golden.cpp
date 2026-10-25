#include "liuo_uwb_protocol/codec.hpp"
#include <cassert>
using namespace liuo::uwb;
int main() {
  constexpr auto golden="~U:TFIBBwEFQDAgEAcAAADSBAAAQOIBAAjiAQAMAAABAgMGKAAAopgDA4EoAACjmQQD/CgAAKSaBQN3KQAApZsGA/IpAACmnEZn\n";
  Packet p; assert(decode(golden,p)==Error::ok);
  assert(p.boot_id==0x10203040 && p.source_frame_seq==1234 && p.edge_count==5);
  assert(p.edges[0].distance_mm==10246 && p.edges[4].distance_mm==10738);
  Wire w; assert(encode(p,w)==Error::ok && w.view()==golden);
  assert(crc16(reinterpret_cast<const uint8_t*>("123456789"),9)==0x29b1);
}
