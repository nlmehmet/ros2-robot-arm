#include "robot_arm_control/stm32_mock.hpp"

#include <iostream>
#include <stdexcept>
#include <utility>

namespace mock = robot_arm_control::stm32_mock;
using namespace std::chrono_literals;

namespace
{
constexpr mock::Frame request_one{0x53, 0x4d, 1, 1, 1, 0, 0, 0, 0x8c, 0x9c};
constexpr mock::Frame reply_one{0x53, 0x4d, 1, 2, 1, 0, 0, 0, 0x5e, 0x72};
constexpr mock::Frame reply_two{0x53, 0x4d, 1, 2, 2, 0, 0, 0, 0x82, 0xe9};
constexpr mock::Frame reply_three{0x53, 0x4d, 1, 2, 3, 0, 0, 0, 0x36, 0x9f};
constexpr mock::Probe::Clock::time_point start{};

void check(bool condition, const char * message)
{
  if (!condition) {
    throw std::runtime_error(message);
  }
}

struct ScriptedTransport final : mock::Transport
{
  bool online = true;
  bool writable = true;
  unsigned sends = 0;
  mock::Frame sent{};
  std::optional<mock::Frame> reply;

  bool connected() const override {return online;}
  bool send(const mock::Frame & frame) override
  {
    ++sends;
    sent = frame;
    return writable;
  }
  std::optional<mock::Frame> receive() override {return std::exchange(reply, std::nullopt);}
};

void check_framing()
{
  mock::Loopback loopback;
  check(!loopback.send(request_one), "Offline link accepted a request");
  loopback.set_connected(true);
  for (std::size_t index = 0; index < request_one.size(); ++index) {
    auto corrupt = request_one;
    corrupt[index] ^= 1;
    check(!loopback.send(corrupt), "Corrupt request was accepted");
    check(!loopback.receive(), "Corrupt request produced a response");
  }
  check(!loopback.send(reply_one), "Reply was accepted as a request");
  check(loopback.send(request_one), "Known request frame was rejected");
  check(!loopback.send(request_one), "Unread reply was overwritten");
  check(loopback.receive() == reply_one, "Reply did not match independent CRC vector");
  check(!loopback.receive(), "Reply was delivered twice");
  check(loopback.send(request_one), "Loopback did not accept another request");
  loopback.set_connected(false);
  loopback.set_connected(true);
  check(!loopback.receive(), "Reconnect retained a queued reply");
  mock::Probe probe(loopback, 10ms);
  check(probe.start(start) == mock::Status::pending, "Loopback probe did not start");
  check(probe.poll(start + 1ms) == mock::Status::acknowledged, "Loopback probe was not acknowledged");
  std::cout << "PASS framing, checksum, and in-memory round trip\n";
}

void check_deadline()
{
  ScriptedTransport transport;
  mock::Probe probe(transport, 10ms);
  check(probe.poll(start) == mock::Status::idle, "Probe did not start idle");
  check(probe.start(start) == mock::Status::pending, "Probe did not start");
  check(transport.sent == request_one, "Request did not match independent CRC vector");
  check(probe.start(start + 6ms) == mock::Status::pending && transport.sends == 1,
    "A second start resent the frame");
  check(probe.poll(start + 9ms) == mock::Status::pending, "Probe timed out early");
  transport.reply = reply_one;
  check(probe.poll(start + 10ms) == mock::Status::timed_out, "Deadline reply was accepted");
  check(probe.poll(start + 11ms) == mock::Status::timed_out, "Timeout status was lost");

  ScriptedTransport silent;
  mock::Probe missing(silent, 10ms);
  missing.start(start);
  check(missing.poll(start + 10ms) == mock::Status::timed_out, "Missing reply did not time out");
  std::cout << "PASS missing and late replies and single in-flight probe\n";
}

void check_faults()
{
  ScriptedTransport transport;
  mock::Probe probe(transport, 10ms);
  transport.online = false;
  check(probe.start(start) == mock::Status::disconnected && transport.sends == 0,
    "Offline probe attempted a send");
  transport.online = true;
  probe.start(start);
  transport.online = false;
  check(probe.poll(start + 1ms) == mock::Status::disconnected, "Link loss was missed");
  transport.online = true;
  check(probe.poll(start + 2ms) == mock::Status::disconnected && transport.sends == 1,
    "Reconnect automatically replayed the probe");
  probe.start(start + 3ms);
  transport.reply = reply_one;
  check(probe.poll(start + 4ms) == mock::Status::invalid_reply, "Stale reply was accepted");
  probe.start(start + 5ms);
  transport.reply = reply_three;
  check(probe.poll(start + 6ms) == mock::Status::acknowledged, "Reconnect probe failed");

  ScriptedTransport reconnected;
  mock::Probe recovery(reconnected, 10ms);
  reconnected.writable = false;
  check(recovery.start(start) == mock::Status::send_error, "Send failure was hidden");
  reconnected.writable = true;
  recovery.start(start + 1ms);
  reconnected.reply = reply_two;
  check(recovery.poll(start + 2ms) == mock::Status::acknowledged, "Fresh retry failed");

  ScriptedTransport corrupt;
  mock::Probe checksum_probe(corrupt, 10ms);
  checksum_probe.start(start);
  corrupt.reply = reply_one;
  (*corrupt.reply)[9] ^= 1;
  check(checksum_probe.poll(start + 1ms) == mock::Status::invalid_reply, "Bad CRC was accepted");
  std::cout << "PASS disconnect, explicit retry, stale reply, send failure, and corrupt reply\n";
}

void check_heartbeat()
{
  using State = mock::LinkState;
  ScriptedTransport transport;
  mock::Heartbeat heartbeat(transport, 20ms, 10ms);
  check(!heartbeat.reset_fault(), "Reset succeeded before any fault");
  check(heartbeat.update(start) == State::awaiting_heartbeat && transport.sent == request_one,
    "First heartbeat did not start in awaiting state");
  transport.reply = reply_one;
  check(heartbeat.update(start + 1ms) == State::healthy && !heartbeat.fault(),
    "First acknowledgement did not establish health");
  check(heartbeat.update(start + 19ms) == State::healthy && transport.sends == 1,
    "Heartbeat sent before its period");
  check(heartbeat.update(start + 20ms) == State::healthy && transport.sends == 2,
    "Periodic heartbeat did not send precisely when due");
  check(heartbeat.update(start + 29ms) == State::healthy, "Pending heartbeat faulted early");
  transport.reply = reply_two;
  check(heartbeat.update(start + 30ms) == State::faulted &&
    heartbeat.fault() == mock::Status::timed_out, "Late heartbeat did not latch timeout");
  check(heartbeat.update(start + 60ms) == State::faulted && transport.sends == 2,
    "Faulted heartbeat sent another request");
  check(heartbeat.reset_fault() && !heartbeat.fault(), "Connected reset failed");
  check(heartbeat.update(start + 61ms) == State::awaiting_heartbeat && transport.sends == 3,
    "Reset did not schedule a fresh heartbeat");
  transport.reply = reply_three;
  check(heartbeat.update(start + 62ms) == State::healthy, "Fresh acknowledgement did not recover");
  check(heartbeat.update(start + 91ms) == State::faulted && transport.sends == 3 &&
    heartbeat.fault() == mock::Status::timed_out, "Missed update window did not fault");
  std::cout << "PASS heartbeat cadence, deadline, missed update window, and explicit recovery\n";
}

void check_heartbeat_fault_causes()
{
  using State = mock::LinkState;
  ScriptedTransport transport;
  mock::Heartbeat heartbeat(transport, 20ms, 10ms);
  heartbeat.update(start);
  transport.reply = reply_one;
  heartbeat.update(start + 1ms);
  transport.online = false;
  check(heartbeat.update(start + 2ms) == State::faulted &&
    heartbeat.fault() == mock::Status::disconnected, "Disconnect between probes was missed");
  check(!heartbeat.reset_fault(), "Offline fault reset succeeded");
  transport.online = true;
  check(heartbeat.update(start + 3ms) == State::faulted && transport.sends == 1,
    "Reconnect cleared the fault or replayed a request");
  check(heartbeat.reset_fault(), "Reconnect reset failed");
  heartbeat.update(start + 4ms);
  transport.reply = reply_one;
  check(heartbeat.update(start + 5ms) == State::faulted &&
    heartbeat.fault() == mock::Status::invalid_reply, "Stale heartbeat did not latch invalid reply");
  transport.online = false;
  heartbeat.update(start + 6ms);
  check(heartbeat.fault() == mock::Status::invalid_reply, "Latched cause was overwritten");
  transport.online = true;
  transport.writable = false;
  check(heartbeat.reset_fault(), "Reset before send failure failed");
  check(heartbeat.update(start + 7ms) == State::faulted &&
    heartbeat.fault() == mock::Status::send_error, "Failed send did not latch");
  std::cout << "PASS heartbeat disconnect, invalid reply, send fault, and cause retention\n";
}
}  // namespace

int main()
{
  try {
    check_framing();
    check_deadline();
    check_faults();
    check_heartbeat();
    check_heartbeat_fault_causes();
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
