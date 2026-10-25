#include "nlink_parser/framer.hpp"
#include "fixtures.hpp"
#include <cassert>
#include <iostream>
#include "nlink_linktrack_anchorframe0.h"
#include "nlink_linktrack_tagframe0.h"
#include "nlink_linktrack_nodeframe0.h"
#include "nlink_linktrack_nodeframe1.h"
#include "nlink_linktrack_nodeframe2.h"
#include "nlink_linktrack_nodeframe3.h"
#include "nlink_linktrack_nodeframe4.h"
#include "nlink_linktrack_nodeframe5.h"
#include "nlink_linktrack_nodeframe6.h"

bool unpack(const uint8_t* p,size_t n) {
 switch(p[1]) {
 case 0:return nlt_anchorframe0_.UnpackData(p,n);
 case 1:return g_nlt_tagframe0.UnpackData(p,n);
 case 2:return g_nlt_nodeframe0.UnpackData(p,n);
 case 3:return g_nlt_nodeframe1.UnpackData(p,n);
 case 4:return g_nlt_nodeframe2.UnpackData(p,n);
 case 5:return g_nlt_nodeframe3.UnpackData(p,n);
 case 6:return g_nlt_nodeframe4.UnpackData(p,n);
 case 8:return g_nlt_nodeframe5.UnpackData(p,n);
 case 9:return g_nlt_nodeframe6.UnpackData(p,n);
 default:return false;
 }
}
int main() {
  size_t splits=0;
  for(const auto& bytes:fixtures) {
    size_t expected=bytes[1]==6?4:1;
    for(size_t split=0;split<=bytes.size();++split) {
      nlink::Framer f; size_t count=0;
      auto cb=[&](const uint8_t* p,size_t n,uint64_t){assert(nlink::validate(p,n));assert(unpack(p,n));++count;};
      f.feed(bytes.data(),split,1,cb);f.feed(bytes.data()+split,bytes.size()-split,2,cb);
      assert(count==expected && f.pending()==0);++splits;
    }
  }
  const auto good=fixtures[5];
  std::vector<std::vector<uint8_t>> bad;
  auto x=good;x[2]=255;x[3]=255;bad.push_back(x);
  x=good;x[20]=255;bad.push_back(x);
  x=good;x.back()^=1;bad.push_back(x);
  x=fixtures[2];x[13]=255;x[14]=255;bad.push_back(x);
  x=fixtures[8];x[19]=255;x[20]=255;bad.push_back(x);
  for(auto bytes:bad) {
    nlink::Framer f;size_t count=0;auto cb=[&](const uint8_t* p,size_t n,uint64_t){assert(n==good.size());assert(std::equal(p,p+n,good.begin()));++count;};
    bytes.insert(bytes.end(),good.begin(),good.end());f.feed(bytes.data(),bytes.size(),1,cb);f.expire(200000000,cb);assert(count==1);
  }
  nlink::Framer f;size_t count=0;auto cb=[&](const uint8_t*,size_t,uint64_t){++count;};
  std::vector<uint8_t> garbage(1000000,0xff);f.feed(garbage.data(),garbage.size(),1,cb);assert(f.pending()<=1 && f.peak<=16384);
  f.feed(good.data(),10,2,cb);f.clear();f.feed(good.data(),good.size(),3,cb);assert(count==1);
  // Delayed recovery must retain the read which completed the embedded good frame.
  nlink::Framer delayed;uint64_t completed=0;
  std::vector<uint8_t> prefixed={0x55,2,0,32,0,0,0,0,0,0,1};prefixed.insert(prefixed.end(),good.begin(),good.end());
  auto got=[&](const uint8_t*,size_t,uint64_t stamp){completed=stamp;};
  delayed.feed(prefixed.data(),prefixed.size(),100,got);uint8_t noise=0xff;delayed.feed(&noise,1,200,got);
  delayed.expire(200000000,got);assert(completed==100);
  nlink::Framer mixed;completed=0;const uint8_t prefix[]={0x55,2,0,32,0,0,0,0,0,0,1};
  mixed.feed(prefix,sizeof(prefix),1,got);mixed.feed(good.data(),10,90000000,got);mixed.expire(101000000,got);
  mixed.feed(good.data()+10,good.size()-10,110000000,got);assert(completed==110000000);
  nlink::Framer short_header;short_header.feed(prefix,2,1,got);short_header.expire(101000000,got);assert(short_header.pending()==0);
  std::cout<<"PASS: "<<splits<<" fixture splits, bad length/block/checksum recovery, 1MB garbage bound, reconnect clear\n";
}
