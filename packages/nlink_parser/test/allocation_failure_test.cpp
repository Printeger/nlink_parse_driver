#include "nlink_parser/framer.hpp"
#include "nlink_linktrack_nodeframe3.h"
#include "fixtures.hpp"
#include <cstdlib>
#include <cassert>
#include <iostream>
static bool fail=true;
extern "C" void* uwb_test_malloc(size_t n){return fail?nullptr:std::malloc(n);}
int main(){
 nlink::Framer framer;size_t delivered=0;
 auto callback=[&](const uint8_t* p,size_t n,uint64_t){if(g_nlt_nodeframe3.UnpackData(p,n))++delivered;};
 const auto& bytes=fixtures[5];framer.feed(bytes.data(),bytes.size(),1,callback);assert(delivered==0);
 fail=false;framer.feed(bytes.data(),bytes.size(),2,callback);assert(delivered==1);
 assert(g_nlt_nodeframe3.result.local_time==463352);
 std::cout<<"PASS: failed vendor allocation returns failure, next valid frame recovers\n";
}
