#include "nlink_parser/framer.hpp"
#include "fixtures.hpp"
#include <cassert>
#include <iostream>
int main() {
  size_t splits=0;
  for(const auto& bytes:fixtures) {
    size_t expected=bytes[1]==6?4:1;
    for(size_t split=0;split<=bytes.size();++split) {
      nlink::Framer f; size_t count=0;
      auto cb=[&](const uint8_t* p,size_t n){assert(nlink::validate(p,n));++count;};
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
    nlink::Framer f;size_t count=0;auto cb=[&](const uint8_t* p,size_t n){assert(n==good.size());assert(std::equal(p,p+n,good.begin()));++count;};
    bytes.insert(bytes.end(),good.begin(),good.end());f.feed(bytes.data(),bytes.size(),1,cb);f.expire(200000000,cb);assert(count==1);
  }
  nlink::Framer f;size_t count=0;auto cb=[&](const uint8_t*,size_t){++count;};
  std::vector<uint8_t> garbage(1000000,0xff);f.feed(garbage.data(),garbage.size(),1,cb);assert(f.pending()<=1 && f.peak<=16384);
  f.feed(good.data(),10,2,cb);f.clear();f.feed(good.data(),good.size(),3,cb);assert(count==1);
  std::cout<<"PASS: "<<splits<<" fixture splits, bad length/block/checksum recovery, 1MB garbage bound, reconnect clear\n";
}
