// Exercises the real callback/guard code; never initializes DDS or SportClient.
#include "state_monitor.hpp"
#include <functional>
#include <thread>

volatile std::sig_atomic_t interrupted = 0;

void require(bool value, const char* label) {
  if (!value) throw std::runtime_error(label);
}

void rejects(const std::function<void()>& action, const std::string& expected) {
  try { action(); }
  catch (const std::runtime_error& e) {
    if (std::string(e.what()).find(expected) != std::string::npos) return;
    throw;
  }
  throw std::runtime_error("guard failed to reject: " + expected);
}

struct Fixture {
  Monitor monitor;
  State sport;
  LowState low;
  explicit Fixture(bool visual = true) : monitor(visual) {
    sport.imu_state().quaternion()[0] = 1.0f;
    sport.mode(9);
    sport.gait_type(1);
    low.bit_flag(0x4);
    monitor.receive(&sport);
    if (visual) monitor.receive_low(&low);
    monitor.check();
  }
  void enter() {
    monitor.begin_visual_request();
    sport.error_code(2);
    sport.mode(3);
    monitor.receive(&sport);
    monitor.check();
    require(monitor.in_visual_reference(), "visual reference missing");
  }
};

int main() {
  try {
    { Fixture f; f.enter(); }
    { Fixture f; f.sport.mode(18); f.monitor.receive(&f.sport);
      rejects([&] { f.monitor.begin_visual_request(); }, "start requires stopped reference"); }
    // Exactly the same numeric value is blocked outside the visual context.
    for (bool visual : {false, true}) {
      Fixture f(visual);
      f.sport.error_code(2);
      f.sport.mode(3);
      f.monitor.receive(&f.sport);
      rejects([&] { f.monitor.check(); }, "error_code=2");
    }
    for (unsigned error : {1u, 3u, 4u, 6u}) {
      Fixture f; f.enter();
      f.sport.error_code(error);
      f.monitor.receive(&f.sport);
      rejects([&] { f.monitor.check(); }, "error_code=");
    }
    for (bool wrong_mode : {false, true}) {
      Fixture f; f.monitor.begin_visual_request();
      f.sport.error_code(2);
      f.sport.mode(wrong_mode ? 9 : 3);
      f.sport.gait_type(wrong_mode ? 1 : 2);
      f.monitor.receive(&f.sport);
      rejects([&] { f.monitor.check(); }, "error_code=2");
    }
    { Fixture f; f.enter(); f.low.bit_flag(0x0); f.monitor.receive_low(&f.low);
      rejects([&] { f.monitor.check(); }, "bit_flag"); }
    { Fixture f; f.enter(); f.low.motor_state()[3].reserve()[0] = 0x4;
      f.monitor.receive_low(&f.low);
      rejects([&] { f.monitor.check(); }, "motor[3]"); }
    { Fixture f; f.enter(); f.sport.error_code(0); f.sport.mode(9);
      f.monitor.receive(&f.sport);
      rejects([&] { f.monitor.check(); }, "left the confirmed reference"); }
    { Fixture f; f.enter(); f.monitor.begin_stopping();
      f.sport.error_code(0); f.sport.mode(9); f.monitor.receive(&f.sport);
      f.monitor.check(); require(f.monitor.in_stopped_reference(), "stop reference missing");
      f.monitor.end_visual_request(); f.sport.error_code(2); f.sport.mode(3);
      f.monitor.receive(&f.sport);
      rejects([&] { f.monitor.check(); }, "error_code=2"); }
    { Fixture f; f.enter(); f.sport.imu_state().quaternion().fill(0);
      f.monitor.receive(&f.sport);
      rejects([&] { f.monitor.check(); }, "quaternion"); }
    { Fixture f; f.enter();
      f.sport.imu_state().quaternion() =
          {float(std::cos(0.25)), float(std::sin(0.25)), 0.0f, 0.0f};
      f.monitor.receive(&f.sport);
      rejects([&] { f.monitor.check(); }, "attitude limit"); }
    { Fixture f; f.enter();
      std::this_thread::sleep_for(std::chrono::milliseconds(320));
      f.monitor.receive(&f.sport);  // Sport fresh, LowState expired.
      rejects([&] { f.monitor.check(); }, "LowState missing/stale"); }
    { Fixture f; f.enter();
      std::this_thread::sleep_for(std::chrono::milliseconds(320));
      f.monitor.receive_low(&f.low);  // LowState fresh, Sport expired.
      rejects([&] { f.monitor.check(); }, "SportModeState missing/stale"); }
    { Fixture f; f.enter(); interrupted = 1;
      rejects([&] { f.monitor.check(); }, "operator interrupted"); interrupted = 0; }
    std::cout << "Visual reference and rejection checks passed (no DDS).\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "Guard regression failed: " << e.what() << std::endl;
    return 1;
  }
}
