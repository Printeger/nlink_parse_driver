#include "nlink_parser/framer.hpp"
namespace nlink {
namespace {
size_t u16(const uint8_t* p) { return p[0]+256u*p[1]; }
size_t minimum(uint8_t f) {
  switch(f) { case 0:return 896; case 1:return 128; case 2:return 12; case 3:return 28; case 4:return 120; case 5:case 6:return 22; case 8:return 25; case 9:return 15; default:return 0; }
}
bool shape(const uint8_t* p,size_t n) {
  switch(p[1]) {
    case 0: return n==896 && p[n-1]==0xee;
    case 1: return n==128;
    case 3: return n==28+20u*p[26];
    case 4: return n==120+13u*p[118];
    case 5: return n==22+7u*p[20];
    case 8: return n==25+10u*p[23];
    case 2:case 9: {
      size_t pos=p[1]==2?11:14, header=p[1]==2?4:7;
      size_t count=p[pos-1];
      for(size_t i=0;i<count;++i) {
        if(pos+header>n-1) return false;
        size_t len=u16(p+pos+header-2);
        // Vendored result arrays are 224 bytes per sender block.
        if(len>224 || len>n-1-pos-header) return false;
        pos+=header+len;
      }
      return pos==n-1;
    }
    case 6: {
      size_t pos=21; if(p[20]>16) return false;
      for(size_t i=0;i<p[20];++i) {
        if(pos+5>n-1 || p[pos+4]>16) return false;
        size_t len=5+4u*p[pos+4]; if(pos+len>n-1) return false; pos+=len;
      }
      return pos==n-1;
    }
  }
  return false;
}
}
bool validate(const uint8_t* p,size_t n) {
  if(n<2 || p[0]!=0x55 || !minimum(p[1]) || n<minimum(p[1])) return false;
  if(p[1]>1 && u16(p+2)!=n) return false;
  if(!shape(p,n)) return false;
  if(p[1]==0) return true; // Anchor0 has a fixed tail, not an additive checksum.
  uint8_t sum=0; for(size_t i=0;i<n-1;++i) sum+=p[i]; return sum==p[n-1];
}
void Framer::feed(const uint8_t* p,size_t n,uint64_t mono,const Handler& handler) {
  last_byte_=mono;
  for(size_t i=0;i<n;++i) {
    if(buffer_.size()==capacity_) { buffer_.erase(buffer_.begin()); ++overflow; }
    buffer_.push_back(p[i]); peak=std::max<uint64_t>(peak,buffer_.size());
    drain(handler,false);
  }
}
void Framer::expire(uint64_t mono,const Handler& handler) {
  if(!buffer_.empty() && mono>=last_byte_ && mono-last_byte_>100000000) drain(handler,true);
}
void Framer::drain(const Handler& handler,bool expired) {
  while(buffer_.size()>=2) {
    auto* p=buffer_.data(); size_t min=minimum(p[1]);
    if(p[0]!=0x55 || !min) { buffer_.erase(buffer_.begin()); ++rejected; continue; }
    if(p[1]>1 && buffer_.size()<4) break;
    size_t len=p[1]<2?min:u16(p+2);
    bool bad=len<min || len>maximum_;
    // Fixed-record counts let us reject inconsistent lengths before waiting.
    if(!bad && buffer_.size()>=min-1 && (p[1]==3 || p[1]==4 || p[1]==5 || p[1]==8)) bad=!shape(p,len);
    if(bad || (expired && buffer_.size()<len)) { buffer_.erase(buffer_.begin()); ++rejected; continue; }
    if(buffer_.size()<len) break;
    if(!validate(p,len)) { buffer_.erase(buffer_.begin()); ++rejected; continue; }
    handler(p,len); buffer_.erase(buffer_.begin(),buffer_.begin()+len);
  }
  if(expired && buffer_.size()==1) {buffer_.clear(); ++rejected;}
}
}
