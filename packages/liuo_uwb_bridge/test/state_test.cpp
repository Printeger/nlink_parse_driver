#include "liuo_uwb_bridge/state.hpp"
#include <cassert>
#include <iostream>
using namespace liuo::uwb;
int main(){
 SequenceWindow w;
 assert(w.accept(0xfffffffe,1,1));assert(w.accept(0xffffffff,1,2));assert(w.accept(0,1,3));assert(w.accept(1,1,4));
 assert(!w.accept(0,1,5));assert(w.accept(0,2,6));assert(!w.accept(0xffff0000,2,7));
 w.expire(11000000000);assert(!w.accept(1,1,11000000001));assert(w.accept(2,1,11000000002));
 SequenceWindow bound(2);for(uint32_t i=0;i<100000;++i)assert(bound.accept(i,1,i));assert(bound.size()==2);assert(!bound.accept(0,1,100001));
 EnvelopeStream a,b;size_t lines=0;
 auto cb=[&](const std::string& s){assert(s=="~U:abc\n");++lines;};
 a.feed({'~','U',':','a'},1,cb);b.feed({'b','c','\n'},2,cb);assert(lines==0);
 a.feed({'b','c','\n'},3,cb);assert(lines==1);
 a.feed(std::vector<uint8_t>(1000,'x'),4,cb);assert(a.pending()<=100);
 a.feed({'~','U',':','a'},5,cb);a.feed({'b','c','\n'},300000000,cb);assert(lines==1);
 a.feed({'~','U',':','a','b','c','\n','~','U',':','a','b','c','\n'},300000001,cb);assert(lines==3);
 ClockMapper clock;ClockMapper::Result r;
 for(uint32_t i=0;i<100;++i)r=clock.observe(0xffffff00u+20*i,1000000000ull+20002000ull*i,1700000000000000000ll+20002000ll*i);
 assert(r.mapped && std::abs(clock.alpha-1.0001)<1e-6);
 r=clock.observe(1,4000000000ull,1700000003000000000ll);assert(r.reset && !r.mapped);
 for(uint32_t i=1;i<20;++i)r=clock.observe(1+20*i,4000000000ull+20000000ull*i,1700000003000000000ll+20000000ll*i);
 assert(r.mapped);r=clock.observe(401,4400000000ull,1700000033400000000ll);assert(r.reset&&!r.mapped);
 std::cout<<"PASS: sequence wrap/out-of-order/retirement/capacity, sender isolation/resync/expiry, clock drift/wrap/restart/ROS jump\n";
}
