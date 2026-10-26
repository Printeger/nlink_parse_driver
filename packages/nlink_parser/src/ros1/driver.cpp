#include "publisher.hpp"
#include "nlink_parser/framer.hpp"
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <chrono>
#include <deque>
#include <random>
using namespace std::chrono_literals;
namespace {
uint64_t mono() { return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count(); }
uint64_t session() { std::random_device rd; return (uint64_t(rd())<<32 | rd()) | 1; }
}
class Driver {
  ros::NodeHandle nh_, private_{"~"};
  template<class T> T parameter(const std::string& key, const T& value) { if(!private_.hasParam(key)) return value; T result; if(!private_.getParam(key,result)) throw std::invalid_argument("invalid parameter type: ~"+key); return result; }

 public:
  Driver():framer_(parameter<int>("max_nlink_rx_frame_bytes",8192),parameter<int>("rx_ring_bytes",16384)),
      publisher_(nh_,private_) {
    port_=parameter<std::string>("port_name","/dev/ttyACM0");
    int baud=parameter<int>("baud_rate",921600); baud_=baud;
    int timeout=parameter<int>("serial_timeout_ms",10);
    int reconnect=parameter<int>("reconnect_interval_ms",500);
    chunk_=parameter<int>("max_read_chunk",4096);
    legacy_=parameter<bool>("enable_legacy_tx",false);
    if(baud<=0 || timeout<1 || timeout>100 || reconnect<50 || chunk_<1 || chunk_>8192) throw std::invalid_argument("invalid serial parameters");
    reconnect_ns_=uint64_t(reconnect)*1000000;
    serial_.setPort(port_); serial_.setBaudrate(baud);
    auto serial_timeout=serial::Timeout::simpleTimeout(timeout);
    serial_.setTimeout(serial_timeout);
    if(legacy_) tx_sub_=nh_.subscribe<std_msgs::String>("nlink_linktrack_data_transmission",8,[this](std_msgs::String::ConstPtr m){
      if(m->data.empty() || m->data.size()>100 || tx_.size()>=8 || !serial_.isOpen()) {++tx_dropped_; return;}
      tx_.push_back({mono(),m->data}); tx_peak_=std::max(tx_peak_,tx_.size());
    });
    diagnostics_=nh_.advertise<diagnostic_msgs::DiagnosticArray>("nlink_diagnostics",10,false);
    timer_=nh_.createSteadyTimer(ros::WallDuration(0.001),[this](const ros::SteadyTimerEvent&){ tick(); });
    diag_timer_=nh_.createSteadyTimer(ros::WallDuration(1),[this](const ros::SteadyTimerEvent&){ diagnostics(); });
  }
 private:
  void frame(const uint8_t* p,size_t n,uint64_t completed) {
    auto read=std::find_if(read_contexts_.begin(),read_contexts_.end(),[&](const auto& c){return c.host_monotonic_ns==completed;});
    if(read==read_contexts_.end()){++unpack_errors_;return;}
    auto frame_context=*read;frame_context.function_mark=p[1];frame_context.rx_frame_seq=++seq_;
    try { if(publisher_.publish(p,n,frame_context)) ++frames_; else ++unpack_errors_; }
    catch(const std::bad_alloc&) {++unpack_errors_;}
  }
  void tick() {
    auto t=mono();
    if(!serial_.isOpen()) {
      if(t<next_open_) return;
      next_open_=t+reconnect_ns_;
      try {serial_.open(); context_.port_session_id=session(); framer_.clear(); read_contexts_.clear();tx_.clear(); ++opens_;}
      catch(const std::exception& e) {++open_failures_; ROS_WARN_THROTTLE(5,"serial open %s baud=%d: %s",port_.c_str(),baud_,e.what());return;}
    }
    try {
      auto cb=[this](const uint8_t* p,size_t n,uint64_t completed){frame(p,n,completed);};
      framer_.expire(t,cb);
      // waitReadable is bounded by configured serial timeout and detects PTY EOF.
      if(serial_.available() || serial_.waitReadable()) {
        std::array<uint8_t,8192> buffer{};
        size_t available=serial_.available();
        size_t n=serial_.read(buffer.data(),std::min<size_t>(available?available:1,chunk_));
        auto received_ros=ros::Time::now(); auto received_mono=mono();
        if(!n) throw std::runtime_error("serial EOF");
        context_.header.stamp=received_ros; context_.host_monotonic_ns=received_mono;read_contexts_.push_back(context_);
        bytes_+=n; auto second=received_mono/1000000000;auto bucket=second%60;if(byte_seconds_[bucket]!=second){byte_seconds_[bucket]=second;byte_counts_[bucket]=0;}byte_counts_[bucket]+=n; framer_.feed(buffer.data(),n,received_mono,cb);
      }
      while(!read_contexts_.empty() && read_contexts_.front().host_monotonic_ns<framer_.oldest_stamp())read_contexts_.pop_front();
      t=mono();
      while(!tx_.empty() && t-tx_.front().first>50000000) {tx_.pop_front();++tx_dropped_;}
      if(!tx_.empty() && t-last_tx_>=25000000) {
        auto payload=std::move(tx_.front().second); tx_.pop_front(); last_tx_=t;
        if(serial_.write(payload)!=payload.size()) ++tx_dropped_; else ++tx_sent_;
      }
    } catch(const std::exception& e) {
      ROS_WARN("serial %s: %s",port_.c_str(),e.what());
      try {serial_.close();} catch(...) {}
      framer_.clear();read_contexts_.clear();tx_.clear();next_open_=mono()+reconnect_ns_;++disconnects_;
    }
  }
  void diagnostics() {
    diagnostic_msgs::DiagnosticArray a; a.header.stamp=ros::Time::now();
    diagnostic_msgs::DiagnosticStatus s; s.name="nlink_parser";s.hardware_id=port_;s.level=serial_.isOpen()?0:1;s.message=serial_.isOpen()?"receiving; hardware capabilities unconfirmed":"disconnected";
    auto add=[&](std::string key,auto value){ diagnostic_msgs::KeyValue v;v.key=key;v.value=std::to_string(value);s.values.push_back(v);};
    auto second=mono()/1000000000;
    uint64_t short_bytes=0,long_bytes=0;for(size_t i=0;i<60;++i)if(second>=byte_seconds_[i]){auto age=second-byte_seconds_[i];if(age<60)long_bytes+=byte_counts_[i];if(age<10)short_bytes+=byte_counts_[i];}
    add("rx_bytes_per_s_10s",short_bytes/10.0);add("rx_bytes_per_s_60s",long_bytes/60.0);add("rx_utilization_10s",short_bytes/10.0*10/baud_);
    if(short_bytes/10.0*10/baud_>0.7){s.level=1;s.message="UART RX utilization above 70 percent";}
    add("rx_bytes",bytes_);add("frames",frames_);add("rx_rejected_bytes",framer_.rejected);add("rx_overflow",framer_.overflow);add("rx_peak_bytes",framer_.peak);add("rx_pending_bytes",framer_.pending());
    add("unpack_errors",unpack_errors_);add("opens",opens_);add("open_failures",open_failures_);add("disconnects",disconnects_);add("tx_dropped",tx_dropped_);add("tx_sent",tx_sent_);add("tx_peak",tx_peak_);
    add("port_session_id",context_.port_session_id);a.status.push_back(s);diagnostics_.publish(a);
  }
  nlink::Framer framer_; nlink::Publisher publisher_; serial::Serial serial_;
  std::array<uint64_t,60> byte_counts_{},byte_seconds_{};int baud_{};
  std::string port_;size_t chunk_{};bool legacy_{};uint64_t reconnect_ns_{},next_open_{},seq_{},last_tx_{};
  uint64_t bytes_{},frames_{},unpack_errors_{},opens_{},open_failures_{},disconnects_{},tx_dropped_{},tx_sent_{};size_t tx_peak_{};
  nlink_parser::NLinkRxContext context_;
  std::deque<nlink_parser::NLinkRxContext> read_contexts_;
  std::deque<std::pair<uint64_t,std::string>> tx_;
  ros::Subscriber tx_sub_;
  ros::Publisher diagnostics_;
  ros::SteadyTimer timer_,diag_timer_;
};
int main(int argc,char** argv) {
  ros::init(argc,argv,"linktrack");
  try {Driver driver; ros::spin();} catch(const std::exception& e) {fprintf(stderr,"linktrack: %s\n",e.what());ros::shutdown();return 1;}
  ros::shutdown();return 0;
}
