#pragma once
#include <liuo_uwb_protocol/codec.hpp>
#include <nlink_parser/framer.hpp>
#include <array>
#include <functional>
#include <optional>
namespace liuo::uwb::mcu {
// Board adapters supply a monotonic clock and nonblocking, bounded UART write.
// Return actual bytes written. A short write is a failed envelope, never retried.
struct Hal {virtual ~Hal()=default;virtual uint64_t monotonic_ms()=0;virtual size_t write(const char*,size_t)=0;};
struct Snapshot {
 uint8_t reporter{};uint32_t seq{},local_ms{},system_ms{};bool system_valid{};uint64_t received_ms{};
 std::array<Edge,254> edges{};size_t count{};
};
struct Config {uint32_t period_ms{200},hold_ms{250},queue_ms{50},gap_ms{25},jitter_ms{2};uint32_t phase_ms{UINT32_MAX};size_t budget_bytes_per_cycle{0};bool simulator{false};};
class Relay {
 public:
  Relay(uint8_t reporter,uint32_t boot,Hal& hal,Config config={});
  bool local_frame(const Snapshot&); // Only this reporter's freshly decoded local frame.
  void reset(uint32_t boot);
  void device_reset(){reset(boot_==UINT32_MAX?1:boot_+1);}
  void tick();
  uint64_t sent{},short_writes{},stale{},late{},overload{},invalid{},cycles{},discarded{};
  size_t pending()const{return shards_>next_shard_?shards_-next_shard_:0;}
 private:
  uint8_t reporter_;uint32_t boot_;Hal& hal_;Config config_;Snapshot latest_{},frozen_{};bool have_{};
  uint32_t cycle_{};uint64_t next_cycle_{},cycle_start_{},last_tx_{},now_{};bool transmitted_{};size_t shards_{},next_shard_{};
};
// Lookup preserves raw role and 32-bit ID; nullopt rejects unknown identities.
struct Node {uint8_t logical;bool fixed;};
using Lookup=std::function<std::optional<Node>(uint8_t,uint32_t)>;
class Receiver {
 public:
  Receiver(uint8_t reporter,Relay& relay,Lookup registry,bool system_valid=false):reporter_(reporter),relay_(relay),registry_(std::move(registry)),system_valid_(system_valid){}
  void feed(const uint8_t*,size_t,uint64_t mono_ms);
  uint64_t ignored_data{},bad_identity{},parsed{},resets{};
 private:
  void frame(const uint8_t*,size_t,uint64_t);
  uint8_t reporter_;Relay& relay_;Lookup registry_;bool system_valid_,have_time_{};uint32_t previous_time_{},sequence_{};
  nlink::Framer framer_;
};
}
