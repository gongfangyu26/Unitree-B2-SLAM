#include "keyboard_control.hpp"
#include <pty.h>
#include <iostream>

using namespace b2_keyboard;
void require(bool ok, const char* message) {
  if (!ok) throw std::runtime_error(message);
}

void policy_tests() {
  Policy p;
  require(p.input("w", 0).kind == Kind::None, "motion allowed before E");
  require(p.input("e", 1).kind == Kind::Target && p.armed(), "E did not arm at zero");
  require(p.input("w", 10).vx == 0.30f, "forward target wrong");
  require(p.input("w", 300).kind == Kind::None, "same target redundantly queued");
  require(p.tick(649).kind == Kind::None, "repeated key did not renew lease");
  auto action = p.tick(650);
  require(action.kind == Kind::Target && action.vx == 0 && action.yaw == 0,
          "input watchdog did not zero at lease expiry");
  require(p.tick(700).kind == Kind::None, "timeout repeatedly queues zero");
  require(p.input("s", 710).vx == -0.30f, "backward target wrong");
  require(p.input("a", 720).yaw == 0.20f, "positive yaw target wrong");
  require(p.input("d", 730).yaw == -0.20f, "negative yaw target wrong");
  action = p.input("wa", 740);
  require(action.vx == 0 && action.yaw == 0.20f, "combined axis motion allowed");
  action = p.input(" ew", 750);
  require(action.kind == Kind::Target && action.vx == 0 && !p.armed(),
          "space lost priority over queued arm/direction");
  require(p.input("wwww", 760).kind == Kind::None, "space failed to lock repeat input");
  p.input("e", 800); p.input("w", 810);
  require(p.input("\n", 1150).kind == Kind::None, "newline should be ignored");
  require(p.tick(1160).kind == Kind::Target, "ignored input prevented watchdog");
  p.input("e", 1200); p.input("w", 1210);
  action = p.input("\x1b[A", 1220);
  require(action.kind == Kind::Target && !p.armed(), "arrow escape not locked");
  p.input("e", 1300); p.input("w", 1310);
  require(p.input("p", 1320).kind == Kind::Target && !p.armed(), "unknown key not locked");
  require(p.input("wqe", 1400).kind == Kind::Quit, "quit did not dominate batch");
  require(p.input(std::string(1, 4), 1401).kind == Kind::Quit, "Ctrl+D did not quit");
}

void terminal_tests() {
  int master = -1, slave = -1;
  if (::openpty(&master, &slave, nullptr, nullptr, nullptr) != 0)
    throw std::runtime_error("cannot create test PTY");
  const int original = ::dup(STDIN_FILENO);
  require(original >= 0, "cannot save test stdin");
  require(::dup2(slave, STDIN_FILENO) >= 0, "cannot attach test PTY");
  termios before{}, during{}, after{};
  try {
    require(::tcgetattr(STDIN_FILENO, &before) == 0, "cannot read test TTY");
    {
      Terminal terminal;
      require(::tcgetattr(STDIN_FILENO, &during) == 0, "cannot inspect raw TTY");
      require(!(during.c_lflag & (ICANON | ECHO)), "TTY remains canonical/echoing");
      require((during.c_lflag & ISIG) == (before.c_lflag & ISIG), "signal keys disabled");
    }
    require(::tcgetattr(STDIN_FILENO, &after) == 0, "cannot inspect restored TTY");
    require(before.c_lflag == after.c_lflag && before.c_iflag == after.c_iflag &&
            before.c_cc[VMIN] == after.c_cc[VMIN] && before.c_cc[VTIME] == after.c_cc[VTIME],
            "TTY not restored");
    try { Terminal terminal; throw 1; } catch (int) {}
    ::tcgetattr(STDIN_FILENO, &after);
    require(before.c_lflag == after.c_lflag, "TTY not restored after exception");
  } catch (...) {
    ::dup2(original, STDIN_FILENO);
    ::close(original); ::close(slave); ::close(master);
    throw;
  }
  ::dup2(original, STDIN_FILENO);
  ::close(original); ::close(slave); ::close(master);
}

int main() {
  try {
    policy_tests();
    terminal_tests();
    std::cout << "Keyboard policy and PTY restoration checks passed (no DDS).\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return 1;
  }
}
