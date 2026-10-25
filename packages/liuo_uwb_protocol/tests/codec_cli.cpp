#include "liuo_uwb_protocol/codec.hpp"
#include <iostream>
#include <string>
int main() {
  std::string line;
  while(std::getline(std::cin,line)) {
    liuo::uwb::Packet p; liuo::uwb::Wire w; line+='\n';
    auto e=liuo::uwb::decode(line,p);
    if(e==liuo::uwb::Error::ok) e=liuo::uwb::encode(p,w);
    if(e==liuo::uwb::Error::ok) std::cout<<w.view(); else std::cout<<"ERROR "<<int(e)<<'\n';
  }
}
