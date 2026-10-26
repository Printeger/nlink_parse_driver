#pragma once
#include <algorithm>
#include <array>
#include <bitset>
#include <cmath>
#include <cstdint>
#include <deque>
#include <map>
#include <optional>
#include <string>
#include <stdexcept>
#include <vector>
namespace liuo::uwb {
// Each window has a permanent retirement floor. TTL/capacity eviction never
// makes an old identifier new again. Modulo comparisons require <2^31 advance.
class SequenceWindow {
  struct Record {uint64_t time;std::bitset<256> keys;};
 public:
  SequenceWindow(size_t capacity=32):capacity_(capacity) {if(!capacity)throw std::invalid_argument("zero sequence capacity");}
  bool accept(uint32_t seq,uint8_t key,uint64_t now) {
    expire(now);
    int64_t index=seq;
    if(initialized_) {
      uint32_t delta=seq-uint32_t(high_);
      int64_t signed_delta=delta<0x80000000u?delta:int64_t(delta)-0x100000000ll;
      index=high_+signed_delta;
      if(signed_delta>1000000 || index<=floor_ || index<high_-127) return false;
    } else initialized_=true;
    high_=std::max(high_,index);
    auto it=records_.find(index);
    if(it!=records_.end() && it->second.keys[key]) return false;
    if(it==records_.end()) {
      while(records_.size()>=capacity_) retire();
      if(index<=floor_) return false;
      it=records_.emplace(index,Record{now,{}}).first;
    }
    it->second.keys.set(key);return true;
  }
  void expire(uint64_t now) {
    while(!records_.empty() && now>=records_.begin()->second.time && now-records_.begin()->second.time>10000000000ull) retire();
  }
  size_t size() const {return records_.size();}
  size_t keys() const {size_t n=0;for(const auto& r:records_)n+=r.second.keys.count();return n;}
  uint64_t oldest() const {return records_.empty()?UINT64_MAX:records_.begin()->second.time;}
  void evict_one(){if(!records_.empty())retire();}
  uint64_t evictions{};
 private:
  void retire() {floor_=std::max(floor_,records_.begin()->first);records_.erase(records_.begin());++evictions;}
  size_t capacity_;bool initialized_{};int64_t high_{-1},floor_{-1};std::map<int64_t,Record> records_;
};
class EnvelopeStream {
 public:
  template<class F> void feed(const std::vector<uint8_t>& bytes,uint64_t now,F on_line) {
    if(last_ && (now<last_ || now-last_>250000000)) {line_.clear();++timeouts;}
    last_=now;
    for(auto byte:bytes) {
      char c=char(byte);
      if(c=='~') {if(!line_.empty())++resyncs;line_="~";continue;}
      if(line_.empty()) continue;
      if((line_.size()==1 && c!='U') || (line_.size()==2 && c!=':')) {line_.clear();++resyncs;continue;}
      if(line_.size()>=100) {line_.clear();++overflows;continue;}
      line_+=c;
      if(c=='\n') {on_line(line_);line_.clear();}
    }
  }
  void clear(){line_.clear();last_=0;}
  size_t pending()const{return line_.size();}
  uint64_t timeouts{},resyncs{},overflows{};
 private:std::string line_;uint64_t last_{};
};
class ClockMapper {
 public:
  enum class State {initializing,valid,invalid};
  struct Result {bool reset{};bool mapped{};int64_t stamp_ns{};};
  Result observe(uint32_t device,uint64_t mono,int64_t ros) {
    bool reset=false;
    if(have_) {
      uint32_t delta=device-previous_;
      auto dm=mono>=last_mono_?mono-last_mono_:UINT64_MAX;
      if(delta>10000 || dm>10000000000ull || ros<=0 || (mono>last_mono_ && ros<=last_ros_) || std::abs(double(ros-last_ros_)-double(dm))>100000000.0) {clear();reset=true;}
      else unwrapped_+=delta;
    }
    if(!have_){unwrapped_=device;base_device_=unwrapped_;base_mono_=mono;}
    have_=true;previous_=device;last_mono_=mono;last_ros_=ros;
    samples_.push_back({double(unwrapped_-base_device_)*1e6,double(mono-base_mono_)});
    if(samples_.size()>128)samples_.pop_front();
    state=State::initializing;
    if(samples_.size()>=8 && samples_.back().first-samples_.front().first>=100000000) {
      double mx=0,my=0;for(auto s:samples_){mx+=s.first;my+=s.second;}mx/=samples_.size();my/=samples_.size();
      double xx=0,xy=0;for(auto s:samples_){xx+=(s.first-mx)*(s.first-mx);xy+=(s.first-mx)*(s.second-my);}
      if(xx>0){alpha=xy/xx;beta=my-alpha*mx; residual_ns=0;for(auto s:samples_)residual_ns=std::max(residual_ns,std::abs(s.second-alpha*s.first-beta));
        state=(alpha>=0.995 && alpha<=1.005 && residual_ns<50000000)?State::valid:State::invalid;}
    }
    if(state!=State::valid || ros<=0)return {reset,false,0};
    int64_t mapped=int64_t(base_mono_+alpha*double(unwrapped_-base_device_)*1e6+beta)+(ros-int64_t(mono));
    return {reset,true,mapped};
  }
  std::optional<int64_t> map(uint32_t device) const {
    if(state!=State::valid)return {};
    uint32_t d=device-previous_;int64_t delta=d<0x80000000u?d:int64_t(d)-0x100000000ll;
    if(std::abs(delta)>10000)return {};
    return int64_t(base_mono_+alpha*(double(unwrapped_-base_device_)+delta)*1e6+beta)+(last_ros_-int64_t(last_mono_));
  }
  void clear(){samples_.clear();have_=false;state=State::invalid;}
  State state{State::initializing};double alpha{1},beta{},residual_ns{};
 private:
  bool have_{};uint32_t previous_{};uint64_t unwrapped_{},base_device_{},base_mono_{},last_mono_{};int64_t last_ros_{};
  std::deque<std::pair<double,double>> samples_;
};
}
