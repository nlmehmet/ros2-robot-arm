#ifndef ROBOT_ARM_CONTROL__STM32_MOCK_HPP_
#define ROBOT_ARM_CONTROL__STM32_MOCK_HPP_

#include <array>
#include <chrono>
#include <cstdint>
#include <optional>

namespace robot_arm_control::stm32_mock
{

// Mock frames are complete packets. No device or wire transport is selected.
using Frame = std::array<std::uint8_t, 10>;

class Transport
{
public:
  virtual ~Transport() = default;
  virtual bool connected() const = 0;
  virtual bool send(const Frame & request) = 0;
  virtual std::optional<Frame> receive() = 0;
};

class Loopback final : public Transport
{
public:
  void set_connected(bool connected);
  bool connected() const override;
  bool send(const Frame & request) override;
  std::optional<Frame> receive() override;

private:
  bool connected_{false};
  std::optional<Frame> reply_;
};

enum class Status {idle, pending, acknowledged, disconnected, send_error, timed_out, invalid_reply};

class Probe
{
public:
  using Clock = std::chrono::steady_clock;

  Probe(Transport & transport, Clock::duration timeout);
  Status start(Clock::time_point now);
  Status poll(Clock::time_point now);

private:
  Transport & transport_;
  Clock::duration timeout_;
  Clock::time_point deadline_{};
  std::uint32_t sequence_{0};
  Status status_{Status::idle};
};

enum class LinkState {awaiting_heartbeat, healthy, faulted};

class Heartbeat
{
public:
  using Clock = Probe::Clock;

  Heartbeat(Transport & transport, Clock::duration period, Clock::duration reply_timeout);
  LinkState update(Clock::time_point now);
  std::optional<Status> fault() const;
  bool reset_fault();

private:
  Transport & transport_;
  Probe probe_;
  Clock::duration period_;
  Clock::duration reply_timeout_;
  std::optional<Clock::time_point> next_due_;
  std::optional<Status> fault_;
  LinkState state_{LinkState::awaiting_heartbeat};
  bool pending_{false};
};

}  // namespace robot_arm_control::stm32_mock

#endif
