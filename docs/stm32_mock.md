# STM32 communication mock

The Phase 5 host checkpoint exercises a request and acknowledgement over an
in-memory transport. `robot_arm_control` builds the mock as a separate static
library used by its tests. The Fortress controller and launch have no dependency
on it. The probe has no motor command or feedback fields.

The mock exchanges complete ten-byte packets. Bytes 0–1 contain `53 4d` in
hexadecimal, byte 2 is version `1`, and byte 3 is kind `1` for requests or `2`
for acknowledgements. Bytes 4–7 hold the little-endian sequence number and
bytes 8–9 hold the little-endian CRC-16/CCITT-FALSE of bytes 0–7. The packet
format is a test fixture. Physical framing and the STM32 wire protocol remain
design decisions for the hardware phase.

The transport interface is nonblocking and used from a single caller thread.
The probe allows one outstanding request. It accepts only a matching response
before the deadline, using caller-supplied steady-clock timestamps. Disconnect,
failed send, invalid reply, and timeout are explicit outcomes. Reconnection
requires a fresh probe, and the sequence persists across reconnects for the
life of the probe. The in-memory endpoint starts disconnected and clears its
queued acknowledgement on disconnect.

## Heartbeat supervision

Call `Heartbeat::update(now)` regularly with nondecreasing steady-clock times.
The first update sends a probe. The link is `awaiting_heartbeat` until a valid
acknowledgement arrives. It then stays `healthy` while the next probe is within
its reply window. The heartbeat period is measured from each send and must be
at least the positive reply timeout. A reply at its deadline, or an update
that misses a scheduled send by the full timeout, latches a timeout fault.

Disconnect, failed send, and invalid reply also latch `faulted` and retain the
first cause. A fault stops further probes. Once connected, an explicit
`reset_fault()` permits a new probe on the next update. A fresh matching
acknowledgement is required to restore health. The probe sequence persists
across resets so a stale reply cannot restore health. All supervision runs in
the caller's update loop.

## Buffered byte transport

`BufferedTransport` implements the same packet interface with one fixed-size
buffer per direction. `send()` queues a whole frame and `take_byte()` drains
it in order. `accept_byte()` accumulates incoming bytes until `receive()` can
return a complete frame. A full buffer refuses input without overwriting it.
Callers retain any byte refused by the receiver until space is available.
Disconnect flushes both directions, including partially transferred frames.

The host harness starts each connection at a frame boundary. CRC and sequence
validation remain in the probe. Byte insertion or loss can invalidate alignment,
so the harness reconnects both endpoints to clear buffers before recovery.
The adapter uses fixed memory and has no serial port, CAN, or device access.

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths src --symlink-install
source install/setup.bash
colcon test --base-paths src --event-handlers console_direct+
colcon test-result --verbose
```

The C++ test uses independent packet fixtures and a scripted transport to check
corruption, missing and late replies, send failure, disconnect, stale replies,
and recovery. Deterministic heartbeat cases cover cadence, watchdog expiry,
fault latching, reconnect, and explicit reset. The existing bringup tests
exercise Fortress separately.
