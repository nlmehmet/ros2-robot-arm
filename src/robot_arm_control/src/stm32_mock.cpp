#include "robot_arm_control/stm32_mock.hpp"

#include <stdexcept>
#include <utility>

namespace robot_arm_control::stm32_mock
{
namespace
{
constexpr std::uint8_t request_kind = 1;
constexpr std::uint8_t reply_kind = 2;

// CRC-16/CCITT-FALSE covers the magic, version, kind and little-endian sequence.
std::uint16_t checksum(const Frame & frame)
{
  std::uint16_t crc = 0xffff;
  for (std::size_t index = 0; index < 8; ++index) {
    crc ^= static_cast<std::uint16_t>(frame[index]) << 8;
    for (int bit = 0; bit < 8; ++bit) {
      crc = static_cast<std::uint16_t>((crc << 1) ^ ((crc & 0x8000) ? 0x1021 : 0));
    }
  }
  return crc;
}

Frame encode(std::uint8_t kind, std::uint32_t sequence)
{
  Frame frame{0x53, 0x4d, 1, kind};
  for (std::size_t index = 0; index < 4; ++index) {
    frame[4 + index] = static_cast<std::uint8_t>(sequence >> (8 * index));
  }
  const auto crc = checksum(frame);
  frame[8] = static_cast<std::uint8_t>(crc);
  frame[9] = static_cast<std::uint8_t>(crc >> 8);
  return frame;
}

std::uint32_t read_sequence(const Frame & frame)
{
  std::uint32_t sequence = 0;
  for (std::size_t index = 0; index < 4; ++index) {
    sequence |= static_cast<std::uint32_t>(frame[4 + index]) << (8 * index);
  }
  return sequence;
}
}  // namespace

void Loopback::set_connected(bool connected)
{
  connected_ = connected;
  if (!connected_) {
    reply_.reset();
  }
}

bool Loopback::connected() const {return connected_;}

bool Loopback::send(const Frame & request)
{
  const auto sequence = read_sequence(request);
  if (!connected_ || reply_ || request != encode(request_kind, sequence)) {
    return false;
  }
  reply_ = encode(reply_kind, sequence);
  return true;
}

std::optional<Frame> Loopback::receive()
{
  return std::exchange(reply_, std::nullopt);
}

void BufferedTransport::set_connected(bool connected)
{
  connected_ = connected;
  if (!connected_) {
    rx_size_ = 0;
    tx_offset_ = tx_.size();
  }
}

bool BufferedTransport::connected() const {return connected_;}

bool BufferedTransport::send(const Frame & frame)
{
  if (!connected_ || tx_offset_ < tx_.size()) {
    return false;
  }
  tx_ = frame;
  tx_offset_ = 0;
  return true;
}

std::optional<Frame> BufferedTransport::receive()
{
  if (rx_size_ != rx_.size()) {
    return std::nullopt;
  }
  rx_size_ = 0;
  return rx_;
}

bool BufferedTransport::accept_byte(std::uint8_t byte)
{
  if (!connected_ || rx_size_ == rx_.size()) {
    return false;
  }
  rx_[rx_size_++] = byte;
  return true;
}

std::optional<std::uint8_t> BufferedTransport::take_byte()
{
  if (tx_offset_ == tx_.size()) {
    return std::nullopt;
  }
  return tx_[tx_offset_++];
}

Probe::Probe(Transport & transport, Clock::duration timeout)
: transport_(transport), timeout_(timeout)
{
  if (timeout_ <= Clock::duration::zero()) {
    throw std::invalid_argument("Probe timeout must be positive");
  }
}

Status Probe::start(Clock::time_point now)
{
  if (status_ == Status::pending) {
    return status_;
  }
  if (!transport_.connected()) {
    return status_ = Status::disconnected;
  }
  if (!transport_.send(encode(request_kind, ++sequence_))) {
    return status_ = Status::send_error;
  }
  deadline_ = now + timeout_;
  return status_ = Status::pending;
}

Status Probe::poll(Clock::time_point now)
{
  if (status_ != Status::pending) {
    return status_;
  }
  if (!transport_.connected()) {
    return status_ = Status::disconnected;
  }
  // Consume at most one complete frame and never acknowledge at or after the deadline.
  const auto reply = transport_.receive();
  if (now >= deadline_) {
    return status_ = Status::timed_out;
  }
  if (reply) {
    status_ = *reply == encode(reply_kind, sequence_) ? Status::acknowledged : Status::invalid_reply;
  }
  return status_;
}

Heartbeat::Heartbeat(Transport & transport, Clock::duration period, Clock::duration reply_timeout)
: transport_(transport), probe_(transport, reply_timeout), period_(period),
  reply_timeout_(reply_timeout)
{
  if (period_ < reply_timeout) {
    throw std::invalid_argument("Heartbeat period must be at least the reply timeout");
  }
}

LinkState Heartbeat::update(Clock::time_point now)
{
  if (fault_) {
    return state_;
  }

  Status status;
  if (pending_) {
    status = probe_.poll(now);
  } else if (!transport_.connected()) {
    status = Status::disconnected;
  } else if (next_due_ && now - *next_due_ >= reply_timeout_) {
    status = Status::timed_out;
  } else if (!next_due_ || now >= *next_due_) {
    status = probe_.start(now);
    next_due_ = now + period_;
  } else {
    return state_;
  }

  pending_ = status == Status::pending;
  if (status == Status::acknowledged) {
    state_ = LinkState::healthy;
  } else if (!pending_) {
    fault_ = status;
    state_ = LinkState::faulted;
  }
  return state_;
}

std::optional<Status> Heartbeat::fault() const {return fault_;}

bool Heartbeat::reset_fault()
{
  if (!fault_ || !transport_.connected()) {
    return false;
  }
  fault_.reset();
  next_due_.reset();
  state_ = LinkState::awaiting_heartbeat;
  return true;
}

}  // namespace robot_arm_control::stm32_mock
