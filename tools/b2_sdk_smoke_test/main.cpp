// Direct calls to Unitree's B2 SportClient. No ROS services or hand-built RPC.
#include "state_monitor.hpp"
#include "keyboard_control.hpp"
#include <unitree/robot/b2/sport/sport_client.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <poll.h>
#include <unistd.h>
#include <termios.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

using Client = unitree::robot::b2::SportClient;
volatile std::sig_atomic_t interrupted = 0;

void on_signal(int) { interrupted = 1; }


void wait_checked(Monitor& monitor, double seconds) {
  const auto end = Clock::now() + std::chrono::duration<double>(seconds);
  while (Clock::now() < end) {
    monitor.check();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  monitor.check();
}

void require_ok(const char* function, int32_t code) {
  std::cout << function << " returned " << code << std::endl;
  if (code != 0) throw std::runtime_error(std::string(function) + " failed");
}

void run_keyboard(Client& client, Monitor& monitor) {
  b2_keyboard::Terminal terminal;  // Discard queued keys, restore TTY on exit.
  client.SetTimeout(0.2f);  // Bound blocking in the interactive motion loop.
  b2_keyboard::Policy policy;
  const auto keyboard_started = Clock::now();
  std::cout << "READY：视觉模式持续保持，不会在10秒后退出。\n"
               "E 解锁方向输入；W/S 前后；A/D 转向；空格归零并锁定方向输入；Q 退出。\n"
               "方向输入超时0.35秒归零，视觉模式继续保持；不要使用方向箭头。\n"
               "先在平地逐项核对方向及归零停车，再测试楼梯；Ctrl+C退出。"
            << std::endl;
  while (true) {
    monitor.check();
    pollfd input{STDIN_FILENO, POLLIN, 0};
    int ready = ::poll(&input, 1, 10);
    if (ready < 0) {
      monitor.check();
      throw std::runtime_error("keyboard poll failed");
    }
    if (input.revents & (POLLHUP | POLLERR | POLLNVAL))
      throw std::runtime_error("keyboard terminal disconnected");
    std::string keys;
    // Consume a batch before dispatch, so queued stop/quit wins over directions.
    while (ready > 0 && (input.revents & POLLIN)) {
      char bytes[256];
      const auto count = ::read(STDIN_FILENO, bytes, sizeof(bytes));
      if (count <= 0) throw std::runtime_error("keyboard input closed");
      keys.append(bytes, static_cast<std::size_t>(count));
      if (keys.size() > 4096) throw std::runtime_error("keyboard input backlog");
      input.revents = 0;
      ready = ::poll(&input, 1, 0);
      if (ready < 0 || (input.revents & (POLLHUP | POLLERR | POLLNVAL)))
        throw std::runtime_error("keyboard terminal disconnected");
    }
    const auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        Clock::now().time_since_epoch()).count();
    auto action = keys.empty() ? b2_keyboard::Action{} : policy.input(keys, now_ms);
    // Ignored characters must not suppress the movement-input watchdog.
    if (action.kind == b2_keyboard::Kind::None) action = policy.tick(now_ms);
    if (action.kind == b2_keyboard::Kind::Quit) return;
    if (action.kind == b2_keyboard::Kind::Target) {
      monitor.check();
      if (!monitor.in_visual_reference())
        throw std::runtime_error("keyboard motion requires confirmed visual reference");
      std::cout << "KEY t=" << std::chrono::duration<double>(
          Clock::now()-keyboard_started).count() << " " << action.reason
                << " armed=" << policy.armed()
                << " vx=" << action.vx << " yaw=" << action.yaw << std::endl;
      // Zero velocity stops translation/turning without requesting mode exit.
      require_ok("Move(keyboard)", client.Move(action.vx, 0.0f, action.yaw));
    }
  }
}

// The SDK reply cannot prove that the robot reached the remote's visual mode.
// Require an on-site observation in the same activation before sending Move.
void confirm_visual_motion(Monitor& monitor) {
  std::cout << "Confirm the expected visual mode ON SITE. Type MOVE then Enter"
               " within 30s to authorize one forward pulse; Ctrl+C cancels."
            << std::endl;
  const auto deadline = Clock::now() + std::chrono::seconds(30);
  std::string line;
  while (Clock::now() < deadline) {
    monitor.check();
    pollfd input{STDIN_FILENO, POLLIN, 0};
    const int ready = ::poll(&input, 1, 20);
    if (ready < 0) {
      monitor.check();
      throw std::runtime_error("operator input unavailable");
    }
    if (ready == 0) continue;
    if (input.revents & (POLLIN | POLLHUP)) {
      char ch;
      if (::read(STDIN_FILENO, &ch, 1) != 1)
        throw std::runtime_error("operator input closed; no Move sent");
      if (ch == '\n') {
        if (line != "MOVE")
          throw std::runtime_error("visual mode not confirmed; no Move sent");
        monitor.check();
        return;
      }
      if (ch != '\r') line += ch;
      if (line.size() > 32)
        throw std::runtime_error("invalid operator confirmation");
    } else {
      throw std::runtime_error("operator input unavailable");
    }
  }
  throw std::runtime_error("visual confirmation timed out; no Move sent");
}

// Signal handlers only set a flag. SDK calls run in normal control flow.
// Main exits, errors and Ctrl+C all pass through this cleanup.
class StopOnExit {
 public:
  explicit StopOnExit(Client& client, bool visual_path = false)
      : client_(client), visual_path_(visual_path) {}
  ~StopOnExit() { if (active_) stop(); }
  void arm() { active_ = true; }
  void mark_vision_requested() { vision_requested_ = true; }
  bool stop() noexcept {
    bool ok = true;
    try {
      const auto code = client_.StopMove();
      std::cout << "StopMove returned " << code << std::endl;
      ok = code == 0;
    } catch (const std::exception& e) {
      std::cerr << "StopMove exception: " << e.what() << std::endl;
      ok = false;
    }
    if (visual_path_) {
      // A failed/timed-out enable call may still have taken effect.
      if (vision_requested_) {
        try {
          const auto code = client_.VisionWalk(false);
          std::cout << "VisionWalk(false) returned " << code << std::endl;
          ok = ok && code == 0;
        } catch (const std::exception& e) {
          std::cerr << "VisionWalk(false) exception: " << e.what() << std::endl;
          ok = false;
        }
        // Request zero motion again after the mode-change request.
        try {
          const auto code = client_.StopMove();
          std::cout << "Final StopMove returned " << code << std::endl;
          ok = ok && code == 0;
        } catch (const std::exception& e) {
          std::cerr << "Final StopMove exception: " << e.what() << std::endl;
          ok = false;
        }
      }
    } else {
      try {
        const auto code = client_.SwitchGait(0);
        std::cout << "SwitchGait(0) returned " << code << std::endl;
        ok = ok && code == 0;
      } catch (const std::exception& e) {
        std::cerr << "SwitchGait(0) exception: " << e.what() << std::endl;
        ok = false;
      }
    }
    active_ = false;
    if (!ok) std::cerr << "STOP UNCONFIRMED: use operator stop" << std::endl;
    return ok;
  }
 private:
  Client& client_;
  bool active_ = false;
  bool visual_path_ = false, vision_requested_ = false;
};

int main(int argc, char** argv) {
  std::string stage = "arguments";
  if (argc != 5) {
    std::cerr << "Usage: " << argv[0]
              << " INTERFACE keyboard|vision|vision-forward|step|forward|backward SECONDS"
              << " --dry-run|--execute\n"
              << "vision/step: 0.2..10 s; translation: 0.2..1 s at +/-0.30 m/s\n"
              << "keyboard: SECONDS must be 0 (until Q/interrupt); interactive TTY required\n"
              << "vision-forward requires on-site MOVE confirmation before Move\n";
    return 2;
  }
  try {
    const std::string mode = argv[2], action = argv[4];
    std::size_t parsed = 0;
    const double seconds = std::stod(argv[3], &parsed);
    const bool step = mode == "step";
    const bool keyboard = mode == "keyboard";
    const bool visual = keyboard || mode == "vision" || mode == "vision-forward";
    const bool observation = step || mode == "vision";
    if (!step && !visual && mode != "forward" && mode != "backward")
      throw std::runtime_error("invalid mode");
    if (parsed != std::string(argv[3]).size() ||
        !std::isfinite(seconds) || (keyboard ? seconds != 0.0 :
        seconds < 0.2 || seconds > (observation ? 10.0 : 1.0)))
      throw std::runtime_error("duration outside allowed limits");
    if (action != "--dry-run" && action != "--execute")
      throw std::runtime_error("explicit --execute is required");
    std::cout << "B2 native SDK test [vision_keyboard_v3]: " << mode;
    if (keyboard) std::cout << " until Q/interrupt";
    else std::cout << " for " << seconds << " s";
    std::cout << "; " << (visual
                  ? "stopped reference + LowState checks -> BalanceStand -> wait 0.5s -> "
                    "VisionWalk(true) -> wait reference 2/3/1 (max 2s) -> check 0.3s"
                  : step
                  ? "ClassicWalk(true) -> wait 3s -> BalanceStand -> "
                    "wait 0.5s -> SwitchGait(2)"
                  : "BalanceStand -> ClassicWalk(true) -> wait 3s -> Move")
              << (keyboard ? " -> keyboard until Q (E to arm; 0.35s input lease)" :
                  mode == "vision-forward"
                  ? " -> on-site MOVE confirmation (max 30s) -> Move(+0.30,0,0)"
                  : mode == "vision" ? " -> observe (no Move)" : "")
              << (visual ? " -> StopMove -> VisionWalk(false) -> StopMove"
                         : " -> StopMove -> SwitchGait(0)") << std::endl;
    if (action == "--dry-run") return 0;  // No DDS initialization.
    if (keyboard && !::isatty(STDIN_FILENO))
      throw std::runtime_error("keyboard requires an interactive SSH terminal; no DDS started");

    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);
    std::signal(SIGHUP, on_signal);
    std::signal(SIGPIPE, on_signal);
    std::signal(SIGTSTP, on_signal);  // Ctrl+Z must not suspend a moving controller.
    stage = "DDS initialization";
    unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
    Monitor monitor(visual);
    unitree::robot::ChannelSubscriber<State> subscriber(
        "rt/lf/sportmodestate");
    subscriber.InitChannel(
        std::bind(&Monitor::receive, &monitor, std::placeholders::_1), 1);
    std::unique_ptr<unitree::robot::ChannelSubscriber<LowState>> low_subscriber;
    if (visual) {
      low_subscriber = std::make_unique<unitree::robot::ChannelSubscriber<LowState>>(
          "rt/lf/lowstate");
      low_subscriber->InitChannel(
          std::bind(&Monitor::receive_low, &monitor, std::placeholders::_1), 1);
    }
    Client client;
    client.SetTimeout(1.0f);
    client.Init();
    StopOnExit cleanup(client, visual);  // Destroyed before client/monitor.
    const auto discovery_end = Clock::now() + std::chrono::seconds(3);
    while (!monitor.received() && Clock::now() < discovery_end) {
      if (interrupted) return 1;
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    monitor.check();
    if (visual) monitor.require_stopped_reference();
    cleanup.arm();  // An unsuccessful/timeout call may still have executed.
    if (visual) {
      stage = "BalanceStand before visual mode";
      require_ok("BalanceStand", client.BalanceStand());
      wait_checked(monitor, 0.5);
      stage = "VisionWalk(true) / visual mode request";
      monitor.begin_visual_request();
      cleanup.mark_vision_requested();
      require_ok("VisionWalk(true)", client.VisionWalk(true));
      stage = "waiting for confirmed remote visual reference";
      const auto ready_deadline = Clock::now() + std::chrono::seconds(2);
      while (!monitor.in_visual_reference()) {
        monitor.check();
        if (Clock::now() >= ready_deadline)
          throw std::runtime_error("visual reference error=2/mode=3/gait=1 not observed");
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
      wait_checked(monitor, 0.3);
      if (keyboard) {
        stage = "continuous visual keyboard control";
        run_keyboard(client, monitor);
      } else if (mode == "vision") {
        stage = "visual mode observation (no Move)";
        std::cout << "Observing for " << seconds
                  << " s after visual reference confirmation; no Move will be sent."
                  << std::endl;
        wait_checked(monitor, seconds);
      } else {
        stage = "on-site visual mode confirmation";
        confirm_visual_motion(monitor);
        stage = "Move / visual forward pulse";
        const auto started = Clock::now();
        require_ok("Move", client.Move(0.30f, 0.0f, 0.0f));
        const double elapsed = std::chrono::duration<double>(
            Clock::now() - started).count();
        wait_checked(monitor, std::max(0.0, seconds - elapsed));
      }
      stage = "StopMove / VisionWalk(false) / StopMove";
      monitor.begin_stopping();
      if (!cleanup.stop()) return 1;
      stage = "checking return to stopped reference";
      const auto stopped_deadline = Clock::now() + std::chrono::seconds(1);
      while (!monitor.in_stopped_reference()) {
        monitor.check();
        if (Clock::now() >= stopped_deadline)
          throw std::runtime_error("STOP UNCONFIRMED: stopped reference absent; use operator stop");
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
      monitor.end_visual_request();
      wait_checked(monitor, 0.3);
      std::cout << "Visual test calls completed; mode, motion and stop require"
                   " on-site verification. No stair success is inferred."
                << std::endl;
      return 0;
    }
    if (step) {
      // Select the controller first, then unlock that selected controller.
      // The AI documentation requires SwitchGait in a movable state;
      // ClassicWalk success alone does not prove that condition.
      stage = "ClassicWalk(true), then 3s settling";
      require_ok("ClassicWalk(true)", client.ClassicWalk(true));
      wait_checked(monitor, 3.0);
      stage = "BalanceStand after ClassicWalk, then 0.5s monitoring";
      require_ok("BalanceStand", client.BalanceStand());
      wait_checked(monitor, 0.5);
    } else {
      // Preserve the forward sequence already verified on this B2.
      stage = "BalanceStand";
      require_ok("BalanceStand", client.BalanceStand());
      monitor.check();
      stage = "ClassicWalk(true), then 3s settling";
      require_ok("ClassicWalk(true)", client.ClassicWalk(true));
      wait_checked(monitor, 3.0);
    }
    monitor.check();
    const auto action_started = Clock::now();
    if (step) {
      stage = "SwitchGait(2) / sustained stepping";
      require_ok("SwitchGait(2)", client.SwitchGait(2));
    } else {
      stage = "Move / translation";
      const float vx = mode == "forward" ? 0.30f : -0.30f;
      // Official Move is called once, not repeatedly queued.
      require_ok("Move", client.Move(vx, 0.0f, 0.0f));
    }
    const double call_elapsed = std::chrono::duration<double>(
        Clock::now() - action_started).count();
    wait_checked(monitor, std::max(0.0, seconds - call_elapsed));
    stage = "StopMove / SwitchGait(0)";
    if (!cleanup.stop()) return 1;
    std::cout << "Calls completed. Verify physical motion and stop on site."
              << std::endl;
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "ABORT at [" << stage << "]: " << e.what() << std::endl;
    return 1;
  }
}
