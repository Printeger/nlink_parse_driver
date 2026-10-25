#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>
namespace liuo::uwb {
struct Edge { uint8_t peer_id{}, flags{}; int32_t distance_mm{}; uint8_t fp_code{}, rx_code{}; };
struct Packet {
  uint8_t flags{}, reporter_id{}, edge_count{};
  uint32_t boot_id{}, tx_cycle_seq{}, source_frame_seq{}, source_local_ms{}, source_system_ms{};
  uint16_t hold_age_ms{65535}; uint8_t shard_index{}, shard_count{1};
  std::array<Edge,5> edges{};
};
struct Wire { std::array<char,100> bytes{}; size_t size{}; std::string_view view() const { return {bytes.data(),size}; } };
enum class Error { ok, envelope, base64, length, version, crc, flags, identity, shard, time, range, order };
uint16_t crc16(const uint8_t*,size_t);
Error validate(const Packet&, int32_t max_distance_mm=0x7fffff);
Error encode(const Packet&,Wire&,int32_t max_distance_mm=0x7fffff);
Error decode(std::string_view,Packet&,int32_t max_distance_mm=0x7fffff);
}
