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

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths src --symlink-install
source install/setup.bash
colcon test --base-paths src --event-handlers console_direct+
colcon test-result --verbose
```

The C++ test uses independent packet fixtures and a scripted transport to check
corruption, missing and late replies, send failure, disconnect, stale replies,
and recovery. The existing bringup tests exercise Fortress separately.
