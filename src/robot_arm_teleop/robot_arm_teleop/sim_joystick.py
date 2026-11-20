"""Translate one joystick axis into bounded Fortress trajectory waypoints."""

import math
import signal
from time import monotonic

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rosgraph_msgs.msg import Clock as ClockMessage
from sensor_msgs.msg import JointState, Joy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


COMMAND_PERIOD = 0.05
COMMAND_HORIZON = 0.20
FEEDBACK_TIMEOUT = 0.25
DEADZONE = 0.10


class SimJoystick(Node):
    def __init__(self):
        super().__init__('sim_joystick')

        def startup_parameter(name, default):
            return self.declare_parameter(
                name, default, ParameterDescriptor(read_only=True),
            ).value

        self.joint = startup_parameter('joint', '')
        self.axis = startup_parameter('axis', 0)
        self.button = startup_parameter('deadman_button', 0)
        self.lower = startup_parameter('lower_rad', 0.0)
        self.upper = startup_parameter('upper_rad', 0.0)
        self.rate = startup_parameter('scale_rad_s', 0.4)
        max_rate = startup_parameter('max_rate_rad_s', 0.0)
        self.timeout = startup_parameter('input_timeout_s', 0.25)
        if (
            not self.joint or min(self.axis, self.button) < 0
            or not all(map(math.isfinite, (self.lower, self.upper, self.rate, max_rate, self.timeout)))
            or not self.lower < self.upper or not 0.0 < self.rate <= max_rate
            or not COMMAND_PERIOD < self.timeout <= 1.0
        ):
            raise ValueError('Invalid joystick mapping, simulation limits, rate or input timeout')

        self.joy_at = self.position_at = self.clock_at = -math.inf
        self.clock_stamp = None
        self.position = 0.0
        self.disarm()
        latest = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Joy, 'joy', self.on_joy, latest)
        self.create_subscription(JointState, 'joint_states', self.on_state, latest)
        self.create_subscription(ClockMessage, 'clock', self.on_clock, latest)
        self.publisher = self.create_publisher(JointTrajectory, '~/joint_trajectory', 1)
        self.create_timer(COMMAND_PERIOD, self.publish_command, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def disarm(self):
        self.released = False
        self.direction = 0.0

    def feedback_ready(self, now):
        return now - self.clock_at <= FEEDBACK_TIMEOUT and now - self.position_at <= FEEDBACK_TIMEOUT

    def on_clock(self, message):
        stamp = message.clock.sec * 10**9 + message.clock.nanosec
        if self.clock_stamp is not None:
            if stamp > self.clock_stamp:
                self.clock_at = monotonic()
            elif stamp < self.clock_stamp:
                self.clock_at = -math.inf
                self.position_at = -math.inf
                self.disarm()
        self.clock_stamp = stamp

    def on_state(self, message):
        if len(message.name) == len(message.position) and message.name.count(self.joint) == 1:
            position = message.position[message.name.index(self.joint)]
            if self.lower <= position <= self.upper:
                self.position = position
                self.position_at = monotonic()
                return
        self.position_at = -math.inf
        self.disarm()

    def on_joy(self, message):
        now = monotonic()
        if now - self.joy_at > self.timeout:
            self.disarm()
        self.joy_at = now
        if (
            not self.feedback_ready(now) or self.axis >= len(message.axes)
            or self.button >= len(message.buttons)
            or not -1.0 <= message.axes[self.axis] <= 1.0
            or message.buttons[self.button] not in (0, 1)
        ):
            self.disarm()
            return
        if message.buttons[self.button] == 0:
            self.released = True
            self.direction = 0.0
        elif self.released:
            axis = message.axes[self.axis]
            self.direction = axis if abs(axis) > DEADZONE else 0.0

    def publish_command(self):
        now = monotonic()
        if now - self.joy_at > self.timeout or not self.feedback_ready(now):
            self.disarm()
            return
        if self.direction == 0.0:
            return
        target = self.position + self.direction * self.rate * COMMAND_HORIZON
        target = min(self.upper, max(self.lower, target))
        if target == self.position:
            return
        point = JointTrajectoryPoint(positions=[target])
        point.time_from_start.nanosec = round(COMMAND_HORIZON * 10**9)
        self.publisher.publish(JointTrajectory(joint_names=[self.joint], points=[point]))


def main(args=None):
    stopping = False

    def stop(_signum, _frame):
        nonlocal stopping
        stopping = True

    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    previous_handlers = {sig: signal.signal(sig, stop) for sig in (signal.SIGINT, signal.SIGTERM)}
    node = None
    try:
        node = SimJoystick()
        while rclpy.ok() and not stopping:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
        for sig, handler in previous_handlers.items():
            signal.signal(sig, handler)
