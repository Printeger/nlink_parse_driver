#pragma once
#include <yaml-cpp/yaml.h>
#include <map>
#include <set>
#include <stdexcept>
#include <tuple>
namespace liuo::uwb {
struct Identity {uint8_t logical;bool fixed;};
class Registry {
 public:
  using Key=std::tuple<uint32_t,std::string,uint8_t,uint32_t>;
  explicit Registry(const std::string& path) {
    auto root=YAML::LoadFile(path);auto nodes=root["nodes"];
    if(!nodes.IsSequence() || nodes.size()>255 || nodes.size()<1)throw std::invalid_argument("registry requires 1..255 identities");
    for(auto n:nodes) {
      auto role=n["raw_role"].as<unsigned>();auto id=n["logical_id"].as<unsigned>();bool fixed=n["is_fixed"].as<bool>();
      if(role>255 || id>254 || fixed!=(id!=0))throw std::invalid_argument("registry role/logical/fixed conflict");
      Key key{n["system_id"].as<uint32_t>(),n["operating_mode"].as<std::string>(),uint8_t(role),n["raw_id"].as<uint32_t>()};
      if(entries_.count(key) || logical_.count(id))throw std::invalid_argument("duplicate raw or logical identity");
      entries_[key]={uint8_t(id),fixed};logical_.insert(id);
    }
    if(!logical_.count(0))throw std::invalid_argument("registry has no N0");
  }
  const Identity* lookup(uint32_t system,const std::string& mode,uint8_t role,uint32_t raw) const {
    auto it=entries_.find({system,mode,role,raw});return it==entries_.end()?nullptr:&it->second;
  }
  bool fixed(uint8_t logical) const{return logical!=0 && logical_.count(logical);}
  const std::set<unsigned>& ids()const{return logical_;}
 private:std::map<Key,Identity> entries_;std::set<unsigned> logical_;
};
}
