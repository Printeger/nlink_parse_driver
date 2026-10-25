#pragma once
#include <algorithm>
#include <cstdint>
#include <functional>
#include <stdexcept>
#include <vector>
namespace nlink {
// Validates the complete physical frame before any vendor unpack invocation.
bool validate(const uint8_t*,size_t);
class Framer {
 public:
  using Handler=std::function<void(const uint8_t*,size_t)>;
  explicit Framer(size_t maximum=8192,size_t capacity=16384):maximum_(maximum),capacity_(capacity) {
    if(maximum<896 || maximum>8192 || capacity<maximum*2 || capacity>65536) throw std::invalid_argument("invalid RX limits");
    buffer_.reserve(capacity);
  }
  void clear() { buffer_.clear(); }
  void feed(const uint8_t* p,size_t n,uint64_t mono,const Handler& handler);
  void expire(uint64_t mono,const Handler& handler);
  size_t pending() const { return buffer_.size(); }
  uint64_t rejected{}, overflow{}, peak{};
 private:
  void drain(const Handler&,bool expire);
  std::vector<uint8_t> buffer_; size_t maximum_,capacity_; uint64_t last_byte_{};
};
}
