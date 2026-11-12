#pragma once

#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>

using Clock = std::chrono::steady_clock;
using State = unitree_go::msg::dds_::SportModeState_;
using LowState = unitree_go::msg::dds_::LowState_;
extern volatile std::sig_atomic_t interrupted;

// Exact reference observed on this B2, not an official error-code definition.
class Monitor {
 public:
  explicit Monitor(bool visual_path = false) : visual_path_(visual_path) {}

  void receive(const void* raw) {
    const auto& state = *static_cast<const State*>(raw);
    std::lock_guard<std::mutex> lock(mutex_);
    const auto& q = state.imu_state().quaternion();
    const double norm = std::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    received_ = true;
    last_ = Clock::now();
    if (!std::isfinite(norm) || norm < 1e-6) {
      latch("invalid IMU quaternion");
    } else {
      const double w = q[0]/norm, x = q[1]/norm, y = q[2]/norm, z = q[3]/norm;
      const double roll = std::atan2(2*(w*x+y*z), 1-2*(x*x+y*y));
      const double pitch = std::asin(std::max(-1.0, std::min(1.0, 2*(w*y-z*x))));
      if (std::abs(roll) > 22.0*std::acos(-1.0)/180.0 ||
          std::abs(pitch) > 35.0*std::acos(-1.0)/180.0)
        latch("attitude limit exceeded");
    }
    const bool reference = visual_path_ && visual_requested_ &&
        state.error_code() == 2 && state.mode() == 3 && state.gait_type() == 1;
    if (state.error_code() != 0 && !reference)
      latch("SportModeState.error_code=" + std::to_string(state.error_code()));
    if (reference) visual_seen_ = true;
    if (visual_seen_ && !stopping_ && !reference)
      latch("visual state left the confirmed reference");
    if (state.mode() != mode_ || state.gait_type() != gait_ ||
        state.error_code() != error_ || first_) {
      std::cout << "STATE error=" << state.error_code()
                << " mode=" << unsigned(state.mode())
                << " gait=" << unsigned(state.gait_type());
      if (reference) std::cout << " [matches this B2 remote visual reference]";
      std::cout << std::endl;
      first_ = false;
    }
    mode_ = state.mode();
    gait_ = state.gait_type();
    error_ = state.error_code();
  }

  void receive_low(const void* raw) {
    const auto& state = *static_cast<const LowState*>(raw);
    std::lock_guard<std::mutex> lock(mutex_);
    low_received_ = true;
    low_last_ = Clock::now();
    if (!visual_path_) return;
    // 0x4 was present before/during/after the confirmed remote reference.
    // Do not interpret it using a different field's fault-code table.
    if (state.bit_flag() != 0x4)
      latch("LowState.bit_flag differs from reference 0x4: " +
            std::to_string(state.bit_flag()));
    for (std::size_t i = 0; i < 12; ++i) {
      if (state.motor_state()[i].reserve()[0] != 0)
        latch("LowState motor[" + std::to_string(i) + "].reserve[0]=" +
              std::to_string(state.motor_state()[i].reserve()[0]));
    }
  }

  bool received() {
    std::lock_guard<std::mutex> lock(mutex_);
    return received_ && (!visual_path_ || low_received_);
  }

  void check() {
    if (interrupted) throw std::runtime_error("operator interrupted");
    std::lock_guard<std::mutex> lock(mutex_);
    if (!fault_.empty()) throw std::runtime_error(fault_);
    if (!received_ || Clock::now()-last_ > std::chrono::milliseconds(300))
      throw std::runtime_error("SportModeState missing/stale");
    if (visual_path_ && (!low_received_ ||
        Clock::now()-low_last_ > std::chrono::milliseconds(300)))
      throw std::runtime_error("LowState missing/stale");
  }

  void require_stopped_reference() {
    check();
    std::lock_guard<std::mutex> lock(mutex_);
    if (error_ != 0 || mode_ != 9 || gait_ != 1)
      throw std::runtime_error("start requires stopped reference error=0/mode=9/gait=1");
  }

  void begin_visual_request() {
    require_stopped_reference();
    std::lock_guard<std::mutex> lock(mutex_);
    if (!visual_path_ || !fault_.empty())
      throw std::runtime_error("visual reference cannot be enabled");
    visual_requested_ = true;
  }

  bool in_visual_reference() {
    std::lock_guard<std::mutex> lock(mutex_);
    return visual_requested_ && visual_seen_ && error_ == 2 && mode_ == 3 && gait_ == 1;
  }

  void begin_stopping() {
    std::lock_guard<std::mutex> lock(mutex_);
    stopping_ = true;  // Permit the requested return to the stopped reference.
  }

  bool in_stopped_reference() {
    std::lock_guard<std::mutex> lock(mutex_);
    return received_ && error_ == 0 && mode_ == 9 && gait_ == 1;
  }

  void end_visual_request() {
    require_stopped_reference();
    std::lock_guard<std::mutex> lock(mutex_);
    visual_requested_ = false;
    visual_seen_ = false;
  }

 private:
  void latch(const std::string& reason) { if (fault_.empty()) fault_ = reason; }
  std::mutex mutex_;
  bool visual_path_ = false, visual_requested_ = false, visual_seen_ = false;
  bool stopping_ = false, received_ = false, low_received_ = false, first_ = true;
  Clock::time_point last_{}, low_last_{};
  unsigned mode_ = 0, gait_ = 0, error_ = 0;
  std::string fault_;
};
