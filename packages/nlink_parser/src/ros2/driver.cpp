#include "publisher.hpp"
#include "nlink_parser/framer.hpp"
#include <serial/serial.h>
#include <std_msgs/msg/string.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <chrono>
#include <deque>
#include <random>
using namespace std::chrono_literals;
namespace {
uint64_t mono() { return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count(); }
uint64_t session() { std::random_device rd; return (uint64_t(rd())<<32 | rd()) | 1; }
}
class Driver: public rclcpp::Node {
 public:
  Driver():Node("linktrack"),framer_(declare_parameter<int>("max_nlink_rx_frame_bytes",8192),declare_parameter<int>("rx_ring_bytes",16384)),
      publisher_(*this,make_qos()) {
    port_=declare_parameter<std::string>("port_name","/dev/ttyUSB0");
    int baud=declare_parameter<int>("baud_rate",921600);
    int timeout=declare_parameter<int>("serial_timeout_ms",10);
    int reconnect=declare_parameter<int>("reconnect_interval_ms",500);
    chunk_=declare_parameter<int>("max_read_chunk",4096);
    legacy_=declare_parameter<bool>("enable_legacy_tx",false);
    if(baud<=0 || timeout<1 || timeout>100 || reconnect<50 || chunk_<1 || chunk_>8192) throw std::invalid_argument("invalid serial parameters");
    reconnect_ns_=uint64_t(reconnect)*1000000;
    serial_.setPort(port_); serial_.setBaudrate(baud);
    auto serial_timeout=serial::Timeout::simpleTimeout(timeout);
    serial_.setTimeout(serial_timeout);
    if(legacy_) tx_sub_=create_subscription<std_msgs::msg::String>("nlink_linktrack_data_transmission",rclcpp::QoS(8).best_effort(),[this](std_msgs::msg::String::ConstSharedPtr m){
      if(m->data.empty() || m->data.size()>100 || tx_.size()>=8 || !serial_.isOpen()) {++tx_dropped_; return;}
      tx_.push_back({mono(),m->data}); tx_peak_=std::max(tx_peak_,tx_.size());
    });
    diagnostics_=create_publisher<diagnostic_msgs::msg::DiagnosticArray>("nlink_diagnostics",rclcpp::QoS(10).reliable());
    timer_=create_wall_timer(1ms,[this]{ tick(); });
    diag_timer_=create_wall_timer(1s,[this]{ diagnostics(); });
  }
 private:
  rclcpp::QoS make_qos() { auto q=rclcpp::QoS(100).durability_volatile(); if(declare_parameter<bool>("reliable_qos",false)) q.reliable(); else q.best_effort(); return q; }
  void frame(const uint8_t* p,size_t n) {
    context_.function_mark=p[1]; context_.rx_frame_seq=++seq_;
    try { if(publisher_.publish(p,n,context_)) ++frames_; else ++unpack_errors_; }
    catch(const std::bad_alloc&) {++unpack_errors_;}
  }
  void tick() {
    auto t=mono();
    if(!serial_.isOpen()) {
      if(t<next_open_) return;
      next_open_=t+reconnect_ns_;
      try {serial_.open(); context_.port_session_id=session(); framer_.clear(); tx_.clear(); ++opens_;}
      catch(const std::exception&) {++open_failures_;return;}
    }
    try {
      auto cb=[this](const uint8_t* p,size_t n){frame(p,n);};
      framer_.expire(t,cb);
      // waitReadable is bounded by configured serial timeout and detects PTY EOF.
      if(serial_.available() || serial_.waitReadable()) {
        std::array<uint8_t,8192> buffer{};
        size_t available=serial_.available();
        size_t n=serial_.read(buffer.data(),std::min<size_t>(available?available:1,chunk_));
        auto received_ros=now(); auto received_mono=mono();
        if(!n) throw std::runtime_error("serial EOF");
        context_.header.stamp=received_ros; context_.host_monotonic_ns=received_mono;
        bytes_+=n; framer_.feed(buffer.data(),n,received_mono,cb);
      }
      t=mono();
      while(!tx_.empty() && t-tx_.front().first>50000000) {tx_.pop_front();++tx_dropped_;}
      if(!tx_.empty() && t-last_tx_>=25000000) {
        auto payload=std::move(tx_.front().second); tx_.pop_front(); last_tx_=t;
        if(serial_.write(payload)!=payload.size()) ++tx_dropped_; else ++tx_sent_;
      }
    } catch(const std::exception&) {
      try {serial_.close();} catch(...) {}
      framer_.clear();tx_.clear();next_open_=mono()+reconnect_ns_;++disconnects_;
    }
  }
  void diagnostics() {
    diagnostic_msgs::msg::DiagnosticArray a; a.header.stamp=now();
    diagnostic_msgs::msg::DiagnosticStatus s; s.name="nlink_parser";s.hardware_id=port_;s.level=serial_.isOpen()?0:1;s.message=serial_.isOpen()?"receiving; hardware capabilities unconfirmed":"disconnected";
    auto add=[&](std::string key,auto value){ diagnostic_msgs::msg::KeyValue v;v.key=key;v.value=std::to_string(value);s.values.push_back(v);};
    add("rx_bytes",bytes_);add("frames",frames_);add("rx_rejected_bytes",framer_.rejected);add("rx_overflow",framer_.overflow);add("rx_peak_bytes",framer_.peak);add("rx_pending_bytes",framer_.pending());
    add("unpack_errors",unpack_errors_);add("opens",opens_);add("open_failures",open_failures_);add("disconnects",disconnects_);add("tx_dropped",tx_dropped_);add("tx_sent",tx_sent_);add("tx_peak",tx_peak_);
    add("port_session_id",context_.port_session_id);a.status.push_back(s);diagnostics_->publish(a);
  }
  nlink::Framer framer_; nlink::Publisher publisher_; serial::Serial serial_;
  std::string port_;size_t chunk_{};bool legacy_{};uint64_t reconnect_ns_{},next_open_{},seq_{},last_tx_{};
  uint64_t bytes_{},frames_{},unpack_errors_{},opens_{},open_failures_{},disconnects_{},tx_dropped_{},tx_sent_{};size_t tx_peak_{};
  nlink_parser::msg::NLinkRxContext context_;
  std::deque<std::pair<uint64_t,std::string>> tx_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr tx_sub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_;
  rclcpp::TimerBase::SharedPtr timer_,diag_timer_;
};
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  try {rclcpp::spin(std::make_shared<Driver>());} catch(const std::exception& e) {fprintf(stderr,"linktrack: %s\n",e.what());rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
