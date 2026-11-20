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
}  // namespace

int main()
{
  try {
    check_framing();
    check_deadline();
    check_faults();
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
