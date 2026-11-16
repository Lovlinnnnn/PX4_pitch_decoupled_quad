#!/usr/bin/env python3
"""Bridge MAVROS TUNNEL to the pitch_decoupled_quad Gazebo Classic joints."""

import math
import struct
import threading
import time


COMMAND_TYPE = 32778
FEEDBACK_TYPE = 32780
TUNNEL_CAPACITY = 128
MAX_FEEDBACK_AGE_US = 1_000_000
NANOSECONDS = 1_000_000_000
COMMAND = struct.Struct("<f")
FEEDBACK = struct.Struct("<IIfI")
VALID = 1
FAULT = 2


def decode_command(message, system_id, component_id, limit_deg):
    """Return the legacy common angle in radians, or None for other payload types."""
    if message.payload_type != COMMAND_TYPE:
        return None
    if (message.target_system, message.target_component) != (system_id, component_id):
        raise ValueError("32778 target does not match this bridge")
    if message.payload_length != COMMAND.size or len(message.payload) != TUNNEL_CAPACITY:
        raise ValueError("32778 requires 4 payload bytes in a 128-byte Tunnel array")
    angle_deg = COMMAND.unpack(bytes(message.payload[:COMMAND.size]))[0]
    if not math.isfinite(angle_deg) or abs(angle_deg) > limit_deg:
        raise ValueError("32778 angle is outside the finite command range")
    return math.radians(angle_deg)


def feedback_payload(sequence, sample_age_us, angle_rad, flags):
    encoded = FEEDBACK.pack(sequence, sample_age_us, angle_rad, flags)
    return list(encoded.ljust(TUNNEL_CAPACITY, b"\0"))


def positive_float(value, name, maximum):
    value = float(value)
    if not math.isfinite(value) or not 0 < value <= maximum:
        raise ValueError("{} must be in (0, {}]".format(name, maximum))
    return value


class JointFeedback:
    """Join one physical JointState sample with ROS and wall-clock freshness."""

    def __init__(self, names, signs, zeros, angle_limit_rad, skew_limit_rad,
                 sim_timeout, wall_timeout, skew_fault_delay):
        self.names = names
        self.signs = signs
        self.zeros = zeros
        self.angle_limit = angle_limit_rad
        self.skew_limit = skew_limit_rad
        self.sim_timeout_ns = int(sim_timeout * NANOSECONDS)
        self.wall_timeout = wall_timeout
        self.skew_fault_delay_ns = int(skew_fault_delay * NANOSECONDS)
        self._last_sim_ns = 0
        self._latest_stamp_ns = 0
        self._last_sent_stamp_ns = 0
        self._last_angle = 0.0
        self._sample = None
        self._skew_since_ns = None

    def _sync_clock(self, sim_ns):
        if sim_ns < self._last_sim_ns:
            self._latest_stamp_ns = 0
            self._last_sent_stamp_ns = 0
            self._last_angle = 0.0
            self._sample = None
            self._skew_since_ns = None
        self._last_sim_ns = sim_ns

    def accept(self, message, sim_ns, wall_time):
        self._sync_clock(sim_ns)
        stamp = message.header.stamp.to_nsec()
        if sim_ns <= 0 or stamp <= 0:
            return
        if stamp <= self._latest_stamp_ns or abs(stamp - sim_ns) > self.sim_timeout_ns:
            return

        # A missing or stale sample breaks the evidence of continuous joint skew.
        if self._sample is not None:
            previous_stamp, previous_wall, _, _ = self._sample
            if stamp - previous_stamp > self.sim_timeout_ns or wall_time - previous_wall > self.wall_timeout:
                self._skew_since_ns = None
        self._latest_stamp_ns = stamp

        angle = self._joint_angles(message)
        if angle is None:
            self._skew_since_ns = None
            self._sample = (stamp, wall_time, self._last_angle, FAULT)
            return

        front, rear = angle
        if abs(front - rear) > self.skew_limit:
            if self._skew_since_ns is None:
                self._skew_since_ns = stamp
            flags = FAULT if stamp - self._skew_since_ns >= self.skew_fault_delay_ns else 0
            self._sample = (stamp, wall_time, self._last_angle, flags)
        else:
            self._skew_since_ns = None
            self._sample = (stamp, wall_time, (front + rear) * 0.5, VALID)

    def _joint_angles(self, message):
        if len(message.name) != len(message.position):
            return None
        positions = dict(zip(message.name, message.position))
        if len(positions) != len(message.name) or any(name not in positions for name in self.names):
            return None
        angles = tuple(sign * (positions[name] - zero)
                       for name, sign, zero in zip(self.names, self.signs, self.zeros))
        if any(not math.isfinite(angle) or abs(angle) > self.angle_limit for angle in angles):
            return None
        return angles

    def next_report(self, sim_ns, wall_time):
        """Return (angle, age_us, flags), or None while waiting for a new sample."""
        self._sync_clock(sim_ns)
        if self._sample is None:
            return self._last_angle, 0, FAULT

        stamp, received_wall, angle, flags = self._sample
        age_ns = sim_ns - stamp
        if age_ns > self.sim_timeout_ns or wall_time - received_wall > self.wall_timeout:
            self._skew_since_ns = None
            return self._last_angle, 0, FAULT
        # ROS /clock and JointState can arrive through different callbacks.
        if age_ns < 0 or stamp <= self._last_sent_stamp_ns:
            return None

        self._last_sent_stamp_ns = stamp
        if flags == VALID:
            self._last_angle = angle
        return angle, age_ns // 1000, flags


class TiltJointFromTunnel:
    def __init__(self):
        import rospy
        from mavros_msgs.msg import Tunnel
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Float64

        self.ros = rospy
        self.Tunnel = Tunnel
        self.Float64 = Float64
        self.lock = threading.Lock()
        rospy.init_node("tilt_joint_from_tunnel")
        if not rospy.get_param("/use_sim_time", False):
            raise ValueError("set /use_sim_time=true before starting this Gazebo bridge")

        self.system_id = int(rospy.get_param("~system_id", 1))
        self.component_id = int(rospy.get_param("~component_id", 191))
        self.autopilot_component = int(rospy.get_param("~autopilot_component", 1))
        if not 1 <= self.system_id <= 255 or self.component_id != 191 or self.autopilot_component != 1:
            raise ValueError("expected a valid system ID, MAVROS component 191, and PX4 component 1")
        if rospy.get_param("~payload_type", COMMAND_TYPE) != COMMAND_TYPE:
            raise ValueError("the command payload type is 32778")
        if rospy.get_param("~command_format", "single_float") != "single_float":
            raise ValueError("32778 uses a single float32 command in degrees")

        model = rospy.get_param("~model_name", "pitch_decoupled_quad")
        names = (rospy.get_param("~front_joint", "front_tilt_joint"),
                 rospy.get_param("~rear_joint", "behind_tilt_joint"))
        signs = tuple(float(rospy.get_param("~{}_sign".format(side), 1.0)) for side in ("front", "rear"))
        zeros = tuple(float(rospy.get_param("~{}_zero_rad".format(side), 0.0)) for side in ("front", "rear"))
        if not model or not all(names) or names[0] == names[1]:
            raise ValueError("provide a model name and two distinct JointState names")
        if any(sign not in (-1.0, 1.0) for sign in signs) or any(not math.isfinite(zero) for zero in zeros):
            raise ValueError("feedback calibration uses +/-1 signs and finite zero positions")
        self.limit_deg = positive_float(rospy.get_param("~angle_limit_deg", 35.0), "angle_limit_deg", 55.0)
        measured_deg = positive_float(rospy.get_param("~feedback_angle_limit_deg", 35.5),
                                      "feedback_angle_limit_deg", 89.0)
        skew_deg = positive_float(rospy.get_param("~skew_limit_deg", 2.0), "skew_limit_deg", 90.0)
        self.rate_hz = positive_float(rospy.get_param("~rate", 50.0), "rate", 1000.0)
        self.command_timeout = positive_float(rospy.get_param("~command_timeout", 0.5), "command_timeout", 60.0)
        feedback_timeout = positive_float(rospy.get_param("~feedback_timeout", 0.3),
                                          "feedback_timeout", MAX_FEEDBACK_AGE_US / 1e6)
        feedback_wall_timeout = positive_float(rospy.get_param("~feedback_wall_timeout", 1.5),
                                               "feedback_wall_timeout", 60.0)
        skew_delay = float(rospy.get_param("~skew_fault_delay", 0.06))
        if not math.isfinite(skew_delay) or skew_delay < 0:
            raise ValueError("skew_fault_delay must be finite and non-negative")
        self.feedback = JointFeedback(names, signs, zeros, math.radians(measured_deg),
                                      math.radians(skew_deg), feedback_timeout, feedback_wall_timeout,
                                      skew_delay)
        self.sequence = 0
        self.last_command_wall = None
        self.command_timed_out = False
        self.next_fault_wall = 0.0
        self.last_feedback_flags = None
        namespace = "/" + model.strip("/")
        self.target_pub = rospy.Publisher(rospy.get_param("~target_topic", namespace + "/tilt/command"),
                                          Float64, queue_size=1, latch=False)
        self.feedback_pub = rospy.Publisher(rospy.get_param("~feedback_topic", "/mavros/tunnel/in"),
                                            Tunnel, queue_size=1, latch=False)
        self.command_sub = rospy.Subscriber(rospy.get_param("~command_topic", "/mavros/tunnel/out"),
                                            Tunnel, self.tunnel_cb, queue_size=1, tcp_nodelay=True)
        self.joint_sub = rospy.Subscriber(rospy.get_param("~joint_state_topic", namespace + "/joint_states"),
                                          JointState, self.joints_cb, queue_size=1, tcp_nodelay=True)
        rospy.loginfo("Tilt bridge: 32778 deg to %s, %s to 32780 measured rad",
                      self.target_pub.resolved_name, self.joint_sub.resolved_name)

    def tunnel_cb(self, message):
        try:
            angle = decode_command(message, self.system_id, self.component_id, self.limit_deg)
        except (ValueError, TypeError, struct.error) as exc:
            self.ros.logwarn_throttle(1.0, "Rejected tilt command: %s", exc)
            return
        if angle is not None:
            with self.lock:
                self.last_command_wall = time.monotonic()
                self.command_timed_out = False
                self.target_pub.publish(self.Float64(data=angle))

    def joints_cb(self, message):
        with self.lock:
            self.feedback.accept(message, self.ros.Time.now().to_nsec(), time.monotonic())

    def _publish_feedback(self, report):
        angle, age, flags = report
        message = self.Tunnel()
        message.target_system = self.system_id
        message.target_component = self.autopilot_component
        message.payload_type = FEEDBACK_TYPE
        message.payload_length = FEEDBACK.size
        message.payload = feedback_payload(self.sequence, age, angle, flags)
        self.feedback_pub.publish(message)
        self.sequence = (self.sequence + 1) & 0xffffffff
        self.last_feedback_flags = flags

    def step(self):
        with self.lock:
            now = time.monotonic()
            if (self.last_command_wall is not None and not self.command_timed_out
                    and now - self.last_command_wall >= self.command_timeout):
                self.command_timed_out = True
                self.ros.logwarn("32778 timed out; Gazebo will finish and hold the last bounded target")

            report = self.feedback.next_report(self.ros.Time.now().to_nsec(), now)
            if report is None:
                return
            flags = report[2]
            if flags != VALID:
                if flags == self.last_feedback_flags and now < self.next_fault_wall:
                    return
                self.next_fault_wall = now + 0.2
            else:
                self.next_fault_wall = 0.0
            self._publish_feedback(report)

    def run(self):
        period = 1.0 / self.rate_hz
        # A ROS timer stops with the simulation clock. Use wall time for the watchdog.
        while not self.ros.is_shutdown():
            started = time.monotonic()
            self.step()
            time.sleep(max(0.0, period - (time.monotonic() - started)))


if __name__ == "__main__":
    TiltJointFromTunnel().run()
