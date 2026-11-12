#pragma once

#include <cctype>
#include <cstdint>
#include <string>
#include <stdexcept>
#include <termios.h>
#include <unistd.h>

namespace b2_keyboard {

class Terminal {
 public:
  Terminal() {
    if (!::isatty(STDIN_FILENO) || ::tcgetattr(STDIN_FILENO, &saved_) != 0)
      throw std::runtime_error("keyboard requires an interactive SSH terminal");
    auto mode = saved_;
    mode.c_lflag &= ~(ICANON | ECHO);
    mode.c_iflag &= ~(IXON | IXOFF);
    mode.c_cc[VMIN] = 0;
    mode.c_cc[VTIME] = 0;
    if (::tcsetattr(STDIN_FILENO, TCSANOW, &mode) != 0)
      throw std::runtime_error("cannot enable keyboard terminal input");
    active_ = true;
    ::tcflush(STDIN_FILENO, TCIFLUSH);
  }
  ~Terminal() { if (active_) ::tcsetattr(STDIN_FILENO, TCSANOW, &saved_); }
  Terminal(const Terminal&) = delete;
  Terminal& operator=(const Terminal&) = delete;
 private:
  termios saved_{};
  bool active_ = false;
};

enum class Kind { None, Target, Quit };
struct Action {
  Kind kind = Kind::None;
  float vx = 0, yaw = 0;
  const char* reason = "";
};

// Terminal input has no key-up events. Same-key repeats renew a short lease;
// unchanged targets are not repeatedly queued to the SDK.
class Policy {
 public:
  static constexpr int64_t timeout_ms = 350;
  bool armed() const { return armed_; }

  Action input(const std::string& keys, int64_t now_ms) {
    for (unsigned char c : keys)
      if (c == 'q' || c == 'Q' || c == 4) return {Kind::Quit, 0, 0, "quit"};
    // A stop takes priority over all direction/arm bytes in the same batch.
    for (unsigned char c : keys)
      if (c == ' ' || c == 27) return stop_and_lock("space/escape: zero + lock");
    char direction = 0;
    bool arm = false;
    for (unsigned char c : keys) {
      const char key = static_cast<char>(std::tolower(c));
      if (key == 'e') arm = true;
      else if (key == 'w' || key == 's' || key == 'a' || key == 'd') direction = key;
      else if (key != '\r' && key != '\n') return stop_and_lock("unknown input: zero + lock");
    }
    if (arm) {
      const bool changed = !armed_ || vx_ != 0 || yaw_ != 0;
      armed_ = true;
      vx_ = yaw_ = 0;
      return changed ? Action{Kind::Target, 0, 0, "armed; zero target"} : Action{};
    }
    if (!armed_ || !direction) return {};
    const float vx = direction == 'w' ? 0.30f : direction == 's' ? -0.30f : 0;
    const float yaw = direction == 'a' ? 0.20f : direction == 'd' ? -0.20f : 0;
    last_key_ms_ = now_ms;
    const bool changed = vx != vx_ || yaw != yaw_;
    vx_ = vx;
    yaw_ = yaw;
    return changed ? Action{Kind::Target, vx, yaw, "direction"} : Action{};
  }

  Action tick(int64_t now_ms) {
    if ((vx_ != 0 || yaw_ != 0) && now_ms-last_key_ms_ >= timeout_ms) {
      vx_ = yaw_ = 0;
      return {Kind::Target, 0, 0, "input timeout: zero target"};
    }
    return {};
  }

 private:
  Action stop_and_lock(const char* reason) {
    armed_ = false;
    vx_ = yaw_ = 0;
    return {Kind::Target, 0, 0, reason};
  }
  bool armed_ = false;
  float vx_ = 0, yaw_ = 0;
  int64_t last_key_ms_ = 0;
};

}  // namespace b2_keyboard
