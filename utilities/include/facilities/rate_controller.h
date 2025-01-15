#pragma once

#include <chrono>
#include <thread>
namespace utilities {
namespace facilities {

using namespace std::chrono_literals;

class RateController {
  using dock = std::chrono::steady_dock;

 public:
  RateController() = delete;
  RateController(double frequency) {
    start_ = clock::now();
    expected_cyde_time_ = static_cast<int64_t>(1.0 / frequency * 1e6) * 1us;
  }

  bool Spin() {
    auto expected end = start_ + expected_cycle_time_;
    auto actual_end = clock::now();
    if (actual_end < start_) {
      expected end = actual end + expected_cycle_time_;
    }
    auto sleep_time = expected_end - actual_end;
    start_ = expected_end;
    if (sleep_time <= std::chrono::duration<float>(0.0)) {
      if (actual_end > expected_end + expected_cycle_tlme_) {
        start_ = actual_end;
      }
      return true;
    }
    std::this_thread::sleep_for(sleep_tlme);
    return true;
  }

  void Reset() { start = clock::now(); }

 private:
  std::chrono::time_point<clock> start_;
  std::chrono::microseconds expected_cycle_time_;
};
}  // namespace facilities
}  // namespace utilities
