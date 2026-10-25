#include "liuo_uwb_bridge/registry.hpp"
#include "liuo_uwb_bridge/state.hpp"
#include <liuo_uwb_protocol/codec.hpp>
#include <liuo_uwb_msgs/msg/uwb_observation_batch.hpp>
#include <nlink_parser/msg/linktrack_nodeframe0_stamped.hpp>
#include <nlink_parser/msg/linktrack_nodeframe2_stamped.hpp>
#include <nlink_parser/msg/linktrack_nodeframe3_stamped.hpp>
#include <nlink_parser/msg/linktrack_nodeframe5_stamped.hpp>
#include <nlink_parser/msg/linktrack_nodeframe6_stamped.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <limits>
#include <random>
using namespace liuo::uwb;
using Observation=liuo_uwb_msgs::msg::UwbObservation;
using Batch=liuo_uwb_msgs::msg::UwbObservationBatch;
using Context=nlink_parser::msg::NLinkRxContext;
namespace {
uint64_t new_session(){std::random_device r;return ((uint64_t(r())<<32)|r())|1;}
uint64_t steady(){return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();}
int64_t ns(const builtin_interfaces::msg::Time& t){return int64_t(t.sec)*1000000000+t.nanosec;}
builtin_interfaces::msg::Time stamp(int64_t n){return rclcpp::Time(std::max<int64_t>(0,n));}
struct Source {
 uint32_t boot{};uint64_t session{};std::vector<uint32_t> dead;
 SequenceWindow packets,observations;EnvelopeStream stream;
 struct Cycle {uint8_t expected;std::bitset<51> shards;uint64_t time;};
 std::map<uint32_t,Cycle> cycles;
 bool activate(uint32_t b,uint64_t s){
   if(b==boot)return true;
   if(std::find(dead.begin(),dead.end(),b)!=dead.end() || dead.size()>=64)return false;
   if(boot)dead.push_back(boot);
   boot=b;session=s;packets=SequenceWindow{};observations=SequenceWindow{};cycles.clear();return true;
 }
};
// Bounded per-edge second buckets; unique keys are limited by the registry.
struct Rate {
 std::array<uint32_t,60> counts{};std::array<uint64_t,60> seconds{};uint64_t last{};
 void add(uint64_t t){uint64_t sec=t/1000000000;size_t k=sec%60;if(seconds[k]!=sec){seconds[k]=sec;counts[k]=0;}if(counts[k]<1000000)++counts[k];last=t;}
 double rate(uint64_t t,size_t window)const{uint64_t sec=t/1000000000;uint64_t count=0;for(size_t k=0;k<60;++k)if(seconds[k]<=sec && sec-seconds[k]<window)count+=counts[k];return double(count)/window;}
};
}
class Bridge:public rclcpp::Node {
 public:
  Bridge():Node("liuo_uwb_bridge"),registry_(declare_parameter<std::string>("registry_path","")) {
    network_=declare_parameter<std::string>("network_instance","unconfigured");
    mode_=declare_parameter<std::string>("operating_mode","unconfirmed");system_=declare_parameter<int>("system_id",0);
    if(network_.empty() || system_<0)throw std::invalid_argument("network/system required");
    replay_=declare_parameter<bool>("replay_mode",false);
    shared_=declare_parameter<bool>("shared_system_clock",false);clock_domain_=declare_parameter<std::string>("clock_domain","");
    for(const auto& name:{"fixed_to_fixed_local_range","simultaneous_ranging_and_data","hundred_byte_payload","pair_freshness_guaranteed"}) capabilities_[name]=declare_parameter<bool>(name,false);
    capacity_=declare_parameter<int>("capacity",0);simulator_=declare_parameter<bool>("simulator",false);
    if(declare_parameter<bool>("allow_rate_degrade",false) || declare_parameter<bool>("forward_received_reports",false))throw std::invalid_argument("rate degradation/forwarding unsupported in v1");
    limit_=declare_parameter<int>("max_distance_mm",0x7fffff);max_keys_=declare_parameter<int>("dedup_max_entries",10000);
    if(limit_<1 || limit_>0x7fffff || max_keys_<1 || max_keys_>10000)throw std::invalid_argument("invalid bounds");
    int range=declare_parameter<int>("range_frame",3),data=declare_parameter<int>("data_frame",0);
    auto prefix=declare_parameter<std::string>("input_namespace","/uwb/n0");
    auto qos=rclcpp::QoS(100).durability_volatile();if(declare_parameter<bool>("reliable_qos",false))qos.reliable();else qos.best_effort();
    output_=create_publisher<Batch>("/liuo/uwb/observations",qos);
    diag_=create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/liuo/uwb/diagnostics",rclcpp::QoS(10).reliable());
    auto topic=[&](int n){return prefix+"/nlink_linktrack_nodeframe"+std::to_string(n)+"_stamped";};
    if(range==2)subs_.push_back(create_subscription<nlink_parser::msg::LinktrackNodeframe2Stamped>(topic(2),qos,[this](nlink_parser::msg::LinktrackNodeframe2Stamped::ConstSharedPtr m){local(*m);}));
    else if(range==3)subs_.push_back(create_subscription<nlink_parser::msg::LinktrackNodeframe3Stamped>(topic(3),qos,[this](nlink_parser::msg::LinktrackNodeframe3Stamped::ConstSharedPtr m){local(*m);}));
    else if(range==5)subs_.push_back(create_subscription<nlink_parser::msg::LinktrackNodeframe5Stamped>(topic(5),qos,[this](nlink_parser::msg::LinktrackNodeframe5Stamped::ConstSharedPtr m){local(*m);}));
    else throw std::invalid_argument("range_frame must be 2/3/5");
    if(data==0)subs_.push_back(create_subscription<nlink_parser::msg::LinktrackNodeframe0Stamped>(topic(0),qos,[this](nlink_parser::msg::LinktrackNodeframe0Stamped::ConstSharedPtr m){remote(*m);}));
    else if(data==6)subs_.push_back(create_subscription<nlink_parser::msg::LinktrackNodeframe6Stamped>(topic(6),qos,[this](nlink_parser::msg::LinktrackNodeframe6Stamped::ConstSharedPtr m){remote(*m);}));
    else throw std::invalid_argument("data_frame must be 0/6");
    for(auto id:registry_.ids())if(id)sources_.try_emplace(uint8_t(id));
    reset_dataset();
    reset_=create_service<std_srvs::srv::Trigger>("~/reset_replay",[this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> res){res->success=replay_;if(replay_){reset_dataset();res->message="new replay dataset session";}else res->message="live reset denied";});
    timer_=create_wall_timer(std::chrono::seconds(1),[this]{diagnostics();});
  }
 private:
  void reset_dataset(){driver_session_=new_session();session_counter_=0;port_=0;last_rx_seq_=0;last_mono_=0;dead_ports_.clear();local_session_=++session_counter_;clock_.clear();system_clock_.clear();for(auto& s:sources_)s.second=Source{};rates_.clear();source_rate_=Rate{};packet_rate_=Rate{};}
  bool context(const Context& c){
    if(!c.port_session_id || !c.rx_frame_seq){++errors_["invalid_context"];return false;}
    if(replay_ && port_==c.port_session_id && c.host_monotonic_ns<last_mono_){reset_dataset();++errors_["replay_dataset_reset"];}
    if(c.port_session_id!=port_){
      if(std::find(dead_ports_.begin(),dead_ports_.end(),c.port_session_id)!=dead_ports_.end() || dead_ports_.size()>=64){++errors_["old_port_session"];return false;}
      if(port_)dead_ports_.push_back(port_);port_=c.port_session_id;last_rx_seq_=0;last_mono_=0;local_session_=++session_counter_;clock_.clear();system_clock_.clear();
      for(auto& s:sources_)s.second.stream.clear();
    }
    if(c.host_monotonic_ns<last_mono_){++errors_["out_of_order_context"];return false;}
    last_mono_=c.host_monotonic_ns;
    if(!replay_){auto now_mono=steady();if(now_mono<c.host_monotonic_ns || now_mono-c.host_monotonic_ns>1000000000){++errors_["stale_context"];return false;}latency_peak_ns_=std::max(latency_peak_ns_,now_mono-c.host_monotonic_ns);}
    return true;
  }
  bool receiver(uint8_t role,uint32_t raw){auto id=registry_.lookup(system_,mode_,role,raw);if(!id || id->logical!=0){++errors_["unknown_receiver"];return false;}return true;}
  Observation base(const Context& c){Observation o;o.received_stamp=c.header.stamp;o.time_uncertainty_s=std::numeric_limits<float>::quiet_NaN();o.source_hold_age_ms=65535;o.fp_rssi=o.rx_rssi=std::numeric_limits<float>::quiet_NaN();o.source_frame_seq_valid=true;return o;}
  void publish(std::vector<Observation> observations,uint8_t trigger){
    if(observations.empty())return;
    Batch b;b.header.stamp=now();b.driver_session_id=driver_session_;b.batch_seq=++batch_seq_;b.trigger_type=trigger;b.receiver_id=0;b.observations=std::move(observations);
    for(const auto& o:b.observations){
      auto key=std::make_pair(o.reporter_id,o.node_b);
      if(!rates_.count(key) && rates_.size()>=1024){auto oldest=std::min_element(rates_.begin(),rates_.end(),[](const auto& a,const auto& b){return a.second.last<b.second.last;});rates_.erase(oldest);++errors_["rate_state_eviction"];}
      rates_[key].add(last_mono_);
    }
    output_->publish(b);
  }
  template<class M>void local(const M& m){
    if(!receiver(m.frame.role,m.frame.id) || !context(m.context))return;
    if(m.context.rx_frame_seq<=last_rx_seq_){++errors_["local_duplicate"];return;}last_rx_seq_=m.context.rx_frame_seq;
    auto result=clock_.observe(uint32_t(m.frame.local_time),m.context.host_monotonic_ns,ns(m.context.header.stamp));
    if(result.reset){local_session_=++session_counter_;system_clock_.clear();++errors_["clock_reset"];}
    // High 32 bits form a new local sequence domain on rx_frame_seq wrap.
    uint64_t high=m.context.rx_frame_seq>>32;if(high!=rx_high_){rx_high_=high;local_session_=++session_counter_;}
    bool sys_valid=shared_ && !clock_domain_.empty() && mode_!="DR_MODE1" && mode_!="unconfirmed";
    if(sys_valid)system_clock_.observe(m.frame.system_time,m.context.host_monotonic_ns,ns(m.context.header.stamp));
    source_rate_.add(last_mono_);std::vector<Observation> obs;std::bitset<256> peers;
    for(const auto& n:m.frame.nodes){
      auto id=registry_.lookup(system_,mode_,n.role,n.id);
      if(!id || !id->fixed){++errors_["unknown_peer"];continue;}
      if(peers[id->logical]){++errors_["duplicate_peer"];continue;}peers.set(id->logical);
      if(!std::isfinite(n.dis) || n.dis<=0 || n.dis*1000.0>limit_+0.5){++errors_["invalid_distance"];continue;}
      auto o=base(m.context);o.kind=Observation::ROBOT_ANCHOR;o.node_a=0;o.node_b=id->logical;o.reporter_id=0;o.source_session_id=local_session_;o.source_frame_seq=uint32_t(m.context.rx_frame_seq);
      o.shard_count=1;o.source_local_ms=uint32_t(m.frame.local_time);o.source_system_ms=m.frame.system_time;o.source_local_time_valid=true;o.source_system_time_valid=sys_valid;
      if(result.mapped){o.source_frame_stamp=stamp(result.stamp_ns);o.time_basis=Observation::TIME_FRAME_LOCAL_MAPPED;}
      o.distance_m=n.dis;o.rssi_valid=std::isfinite(n.fp_rssi)&&std::isfinite(n.rx_rssi)&&n.fp_rssi<=0&&n.rx_rssi<=0&&n.fp_rssi>=-127.5&&n.rx_rssi>=-127.5;
      if(o.rssi_valid){o.fp_rssi=n.fp_rssi;o.rx_rssi=n.rx_rssi;}
      obs.push_back(o);
    }
    publish(std::move(obs),Batch::LOCAL_RANGE);
  }
  template<class M>void remote(const M& m){
    if(!receiver(m.frame.role,m.frame.id) || !context(m.context))return;
    for(const auto& n:m.frame.nodes){
      auto id=registry_.lookup(system_,mode_,n.role,n.id);if(!id || !id->fixed){++errors_["unknown_sender"];continue;}
      auto& source=sources_.at(id->logical);
      source.stream.feed(n.data,m.context.host_monotonic_ns,[&](const std::string& wire){packet(id->logical,wire,m.context);});
    }
  }
  void packet(uint8_t sender,const std::string& wire,const Context& c){
    Packet p;auto e=decode(wire,p,limit_);if(e!=Error::ok){++errors_["codec_"+std::to_string(int(e))];return;}
    if(sender!=p.reporter_id){++errors_["reporter_mismatch"];return;}
    for(size_t i=0;i<p.edge_count;++i)if(!registry_.fixed(p.edges[i].peer_id)){++errors_["unregistered_peer"];return;}
    auto& s=sources_.at(sender);
    if(!s.activate(p.boot_id,session_counter_+1)){++errors_["old_or_exhausted_boot"];return;}
    session_counter_=std::max(session_counter_,s.session);
    if(!s.packets.accept(p.tx_cycle_seq,p.shard_index,c.host_monotonic_ns)){++errors_["packet_duplicate_or_old"];return;}
    packet_rate_.add(last_mono_);
    auto it=s.cycles.find(p.tx_cycle_seq);
    if(it==s.cycles.end()){
      if(s.cycles.size()>=32){count_missing(s.cycles.begin()->second);s.cycles.erase(s.cycles.begin());}
      it=s.cycles.emplace(p.tx_cycle_seq,Source::Cycle{p.shard_count,{},last_mono_}).first;
    }
    if(it->second.expected!=p.shard_count){++errors_["inconsistent_shards"];return;}it->second.shards.set(p.shard_index);
    std::vector<Observation> obs;
    for(size_t i=0;i<p.edge_count;++i){const auto& edge=p.edges[i];
      if(!s.observations.accept(p.source_frame_seq,edge.peer_id,c.host_monotonic_ns)){++errors_["observation_duplicate_or_old"];continue;}
      auto o=base(c);o.kind=Observation::ANCHOR_ANCHOR;o.reporter_id=o.node_a=sender;o.node_b=edge.peer_id;o.source_session_id=s.session;o.source_frame_seq=p.source_frame_seq;o.tx_cycle_seq=p.tx_cycle_seq;o.shard_index=p.shard_index;o.shard_count=p.shard_count;
      o.source_local_ms=p.source_local_ms;o.source_system_ms=p.source_system_ms;o.source_local_time_valid=p.flags&2;
      o.source_system_time_valid=(p.flags&1) && mode_!="DR_MODE1";
      o.source_hold_age_ms=p.hold_age_ms;o.source_hold_age_valid=p.flags&4;
      if(o.source_system_time_valid && shared_ && !clock_domain_.empty() && mode_!="unconfirmed")if(auto mapped=system_clock_.map(p.source_system_ms)){o.source_frame_stamp=stamp(*mapped);o.time_basis=Observation::TIME_FRAME_SYSTEM_MAPPED;}
      o.distance_m=edge.distance_mm/1000.0f;o.rssi_valid=edge.flags&2;if(o.rssi_valid){o.fp_rssi=-edge.fp_code/2.0f;o.rx_rssi=-edge.rx_code/2.0f;}obs.push_back(o);
    }
    bound_caches();publish(std::move(obs),Batch::REMOTE_EDGE_REPORT);
  }
  void count_missing(const Source::Cycle& c){errors_["missing_shards_estimate"]+=c.expected-c.shards.count();}
  void bound_caches(){
    uint64_t expiry=replay_?last_mono_:steady();
    for(int which=0;which<2;++which)for(;;){size_t count=0;Source* oldest=nullptr;uint64_t time=UINT64_MAX;
      for(auto& pair:sources_){auto& w=which?pair.second.observations:pair.second.packets;w.expire(expiry);count+=w.keys();if(w.oldest()<time){time=w.oldest();oldest=&pair.second;}}
      if(count<=size_t(max_keys_) || !oldest)break;
      (which?oldest->observations:oldest->packets).evict_one();++errors_["cache_capacity_eviction"];
    }
  }
  void diagnostics(){
    bound_caches();diagnostic_msgs::msg::DiagnosticArray a;a.header.stamp=now();diagnostic_msgs::msg::DiagnosticStatus s;s.name="liuo_uwb_bridge";s.hardware_id=network_;s.level=1;s.message="hardware UNKNOWN; frame reports are not verified TWR freshness";
    auto add=[&](std::string k,std::string v){diagnostic_msgs::msg::KeyValue kv;kv.key=std::move(k);kv.value=std::move(v);s.values.push_back(kv);};
    add("mode",mode_);add("capacity",std::to_string(capacity_));add("simulator",simulator_?"true":"false");add("network_instance",network_);add("dataset_session",std::to_string(driver_session_));
    for(auto cap:capabilities_)add(cap.first,cap.second?"configured_verified":"false");
    add("shared_system_clock",shared_?"configured_verified":"false");add("clock_domain",clock_domain_);add("pair_measurement_rate","unknown");add("wire_tx_rate","unknown (MCU log required)");add("radio_loss","unknown (TX log required)");add("degraded","false");
    add("source_frame_rate_10s",std::to_string(source_rate_.rate(last_mono_,10)));add("source_frame_rate_60s",std::to_string(source_rate_.rate(last_mono_,60)));add("unique_packet_rate_10s",std::to_string(packet_rate_.rate(last_mono_,10)));
    uint64_t t=last_mono_;if(!replay_)t=steady();
    for(const auto& r:rates_){auto prefix="edge_"+std::to_string(r.first.first)+"_"+std::to_string(r.first.second);add(prefix+"_10s_hz",std::to_string(r.second.rate(t,10)));add(prefix+"_60s_hz",std::to_string(r.second.rate(t,60)));add(prefix+"_stale",t>r.second.last+250000000?"true":"false");}
    size_t pk=0,ob=0,fragments=0,tombs=dead_ports_.size();
    for(auto& pair:sources_){auto& src=pair.second;pk+=src.packets.keys();ob+=src.observations.keys();fragments+=src.stream.pending();tombs+=src.dead.size();
      for(auto it=src.cycles.begin();it!=src.cycles.end();)if(t>=it->second.time && t-it->second.time>1000000000){count_missing(it->second);it=src.cycles.erase(it);}else ++it;
    }
    add("packet_cache_keys",std::to_string(pk));add("observation_cache_keys",std::to_string(ob));add("fragment_bytes",std::to_string(fragments));add("tombstones",std::to_string(tombs));add("processing_latency_peak_ms",std::to_string(latency_peak_ns_/1e6));
    add("clock_state",std::to_string(int(clock_.state)));add("clock_alpha",std::to_string(clock_.alpha));add("clock_residual_ms",std::to_string(clock_.residual_ns/1e6));add("time_uncertainty","unknown; one-way offset unidentifiable");
    for(auto err:errors_)add(err.first,std::to_string(err.second));a.status.push_back(s);diag_->publish(a);
  }
  Registry registry_;std::string network_,mode_,clock_domain_;int system_{},capacity_{},limit_{},max_keys_{};bool replay_{},shared_{},simulator_{};
  std::map<std::string,bool> capabilities_;std::map<std::string,uint64_t> errors_;
  std::map<uint8_t,Source> sources_;std::map<std::pair<uint8_t,uint8_t>,Rate> rates_;Rate source_rate_,packet_rate_;
  uint64_t driver_session_{},session_counter_{},local_session_{},batch_seq_{},port_{},last_rx_seq_{},last_mono_{},rx_high_{},latency_peak_ns_{};std::vector<uint64_t> dead_ports_;
  ClockMapper clock_,system_clock_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;rclcpp::Publisher<Batch>::SharedPtr output_;rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_;rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char** argv){rclcpp::init(argc,argv);try{rclcpp::spin(std::make_shared<Bridge>());}catch(const std::exception& e){fprintf(stderr,"bridge: %s\n",e.what());rclcpp::shutdown();return 1;}rclcpp::shutdown();}
