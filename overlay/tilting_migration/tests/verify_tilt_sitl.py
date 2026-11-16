#!/usr/bin/env python3
"""Check the disarmed Gazebo tilt bench through the MAVROS TUNNEL topics."""

import argparse
from collections import deque
import json
import math
import struct
import sys
import threading
import time
from typing import NamedTuple


STEPS_DEG = (0, 5, -5, 10, 0)
COMMAND = struct.Struct("<f")
FEEDBACK = struct.Struct("<IIfI")
NANOSECONDS = 1_000_000_000


class JointSample(NamedTuple):
    received_at: float
    stamp_ns: int
    front_deg: float
    rear_deg: float

    @property
    def mean_deg(self):
        return (self.front_deg + self.rear_deg) / 2


class Feedback(NamedTuple):
    received_at: float
    inferred_stamp_ns: int
    sequence: int
    age_us: int
    angle_deg: float


class Bench:
    def __init__(self, ros, tunnel_type, joint_type, args):
        self.ros = ros
        self.args = args
        self.lock = threading.Lock()
        self.commands = deque(maxlen=512)
        self.joints = deque(maxlen=512)
        self.feedback = deque(maxlen=512)
        self.last_sequence = None
        self.invalid_feedback_count = 0
        self.command_sub = ros.Subscriber("/mavros/tunnel/out", tunnel_type, self.on_command,
                                         queue_size=10, tcp_nodelay=True)
        self.feedback_sub = ros.Subscriber("/mavros/tunnel/in", tunnel_type, self.on_feedback,
                                          queue_size=10, tcp_nodelay=True)
        self.joint_sub = ros.Subscriber(args.joint_state_topic, joint_type, self.on_joints,
                                       queue_size=10, tcp_nodelay=True)

    def on_command(self, message):
        if (message.payload_type != 32778 or message.target_system != self.args.system_id
                or message.target_component != 191 or message.payload_length != COMMAND.size
                or len(message.payload) != 128):
            return
        payload = bytes(message.payload[:COMMAND.size])
        angle, = COMMAND.unpack(payload)
        if math.isfinite(angle):
            with self.lock:
                self.commands.append((time.monotonic(), angle, payload.hex()))

    def on_feedback(self, message):
        if (message.payload_type != 32780 or message.target_system != self.args.system_id
                or message.target_component != 1 or message.payload_length != FEEDBACK.size
                or len(message.payload) != 128):
            return
        sequence, age, angle_rad, flags = FEEDBACK.unpack(bytes(message.payload[:FEEDBACK.size]))
        with self.lock:
            if flags != 1 or age > 300_000 or not math.isfinite(angle_rad):
                self.invalid_feedback_count += 1
                return
            advance = (sequence - self.last_sequence) & 0xffffffff if self.last_sequence is not None else 1
            if advance == 0 or advance >= 0x80000000:
                return
            self.last_sequence = sequence
            stamp = self.ros.Time.now().to_nsec() - age * 1000
            self.feedback.append(Feedback(time.monotonic(), stamp, sequence, age, math.degrees(angle_rad)))

    def on_joints(self, message):
        if len(message.name) != len(message.position):
            return
        positions = dict(zip(message.name, message.position))
        if (len(positions) != len(message.name)
                or self.args.front_joint not in positions or self.args.rear_joint not in positions):
            return
        front = math.degrees(self.args.front_sign * (positions[self.args.front_joint] - self.args.front_zero_rad))
        rear = math.degrees(self.args.rear_sign * (positions[self.args.rear_joint] - self.args.rear_zero_rad))
        stamp = message.header.stamp.to_nsec()
        with self.lock:
            if (math.isfinite(front) and math.isfinite(rear) and stamp > 0
                    and (not self.joints or stamp > self.joints[-1].stamp_ns)):
                self.joints.append(JointSample(time.monotonic(), stamp, front, rear))

    def wait_for_step(self, target, previous_target):
        requested_at = time.monotonic()
        print("PX4> mavlink tilt_test {}".format(target), file=sys.stderr, flush=True)
        deadline = requested_at + self.args.step_timeout
        command = None
        last_sequence = None
        witness = None
        settled_since = None
        settled_samples = 0
        matched_samples = 0
        max_skew = 0.0
        last_match = None

        while not self.ros.is_shutdown() and time.monotonic() < deadline:
            with self.lock:
                if command is None:
                    command = next((entry for entry in self.commands
                                    if entry[0] >= requested_at and abs(entry[1] - target) <= 0.01), None)
                if command is None:
                    feedback = []
                    joints = []
                else:
                    feedback = [item for item in self.feedback if item.received_at >= command[0]]
                    joints = [item for item in self.joints if item.received_at >= command[0]]
            for packet in feedback:
                if last_sequence is not None and not 0 < ((packet.sequence - last_sequence) & 0xffffffff) < 0x80000000:
                    continue
                if not joints or joints[-1].stamp_ns + 5_000_000 < packet.inferred_stamp_ns:
                    break
                last_sequence = packet.sequence
                joint = min(joints, key=lambda sample: abs(sample.stamp_ns - packet.inferred_stamp_ns))
                if (abs(joint.stamp_ns - packet.inferred_stamp_ns) > 40_000_000
                        or abs(joint.received_at - packet.received_at) > 0.2
                        or abs(joint.mean_deg - packet.angle_deg) > self.args.match_tolerance_deg):
                    # Match one nearby sample using its measured angle. A second ROS subscriber
                    # can receive /clock and JointState in a different order than the bridge.
                    candidates = (
                        sample for sample in joints
                        if abs(sample.stamp_ns - packet.inferred_stamp_ns) <= 40_000_000
                        and abs(sample.received_at - packet.received_at) <= 0.2
                        and abs(sample.mean_deg - packet.angle_deg) <= self.args.match_tolerance_deg)
                    joint = min(candidates, key=lambda sample: abs(sample.stamp_ns - packet.inferred_stamp_ns),
                                default=None)
                if joint is None:
                    settled_since = None
                    settled_samples = 0
                    continue
                matched_samples += 1
                max_skew = max(max_skew, abs(joint.front_deg - joint.rear_deg))
                if (previous_target is not None and witness is None
                        and abs(joint.mean_deg - previous_target) >= 1.0
                        and abs(joint.mean_deg - target) >= 1.0
                        and (target - previous_target) * (joint.mean_deg - previous_target) > 0):
                    witness = {
                        "gazebo_sample_us": joint.stamp_ns // 1000,
                        "feedback_sequence": packet.sequence,
                        "measured_deg": round(joint.mean_deg, 5),
                        "feedback_deg": round(packet.angle_deg, 5),
                        "target_error_deg": round(abs(target - packet.angle_deg), 5),
                    }
                near_goal = max(abs(value - target) for value in
                                (joint.front_deg, joint.rear_deg, packet.angle_deg))
                if near_goal > self.args.settle_tolerance_deg:
                    settled_since = None
                    settled_samples = 0
                    continue
                if last_match is None or joint.stamp_ns - last_match > 100_000_000:
                    settled_since = None
                    settled_samples = 0
                if settled_since is None:
                    settled_since = joint.stamp_ns
                if last_match != joint.stamp_ns:
                    last_match = joint.stamp_ns
                    settled_samples += 1
                if (settled_samples >= 3 and joint.stamp_ns - settled_since >= 150_000_000
                        and (previous_target is None or witness is not None)):
                    print("Settled at {} deg".format(target), file=sys.stderr, flush=True)
                    return {
                        "command_deg": target,
                        "tunnel_payload_hex": command[2],
                        "matched_samples": matched_samples,
                        "settled_front_deg": round(joint.front_deg, 5),
                        "settled_rear_deg": round(joint.rear_deg, 5),
                        "settled_feedback_deg": round(packet.angle_deg, 5),
                        "feedback_sequence": packet.sequence,
                        "feedback_age_us": packet.age_us,
                        "maximum_joint_skew_deg": round(max_skew, 5),
                        "in_transit_measurement": witness,
                    }
            time.sleep(0.01)
        if command is None:
            raise RuntimeError("did not receive 32778 at {} deg".format(target))
        raise RuntimeError("step {} deg did not pass; matched samples={}, motion witnessed={}".format(
            target, matched_samples, witness is not None))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--joint-state-topic", default="/pitch_decoupled_quad/joint_states")
    parser.add_argument("--system-id", type=int, default=1)
    parser.add_argument("--front-joint", default="front_tilt_joint")
    parser.add_argument("--rear-joint", default="behind_tilt_joint")
    parser.add_argument("--front-sign", type=int, choices=(-1, 1), default=1)
    parser.add_argument("--rear-sign", type=int, choices=(-1, 1), default=1)
    parser.add_argument("--front-zero-rad", type=float, default=0.0)
    parser.add_argument("--rear-zero-rad", type=float, default=0.0)
    parser.add_argument("--step-timeout", type=float, default=25.0)
    parser.add_argument("--settle-tolerance-deg", type=float, default=1.0)
    parser.add_argument("--match-tolerance-deg", type=float, default=0.05)
    args = parser.parse_args()
    if (not 1 <= args.system_id <= 255 or args.front_joint == args.rear_joint
            or not all(math.isfinite(value) for value in (args.front_zero_rad, args.rear_zero_rad))
            or not all(math.isfinite(value) and value > 0 for value in
                       (args.step_timeout, args.settle_tolerance_deg, args.match_tolerance_deg))):
        parser.error("invalid test geometry, target, or timing argument")

    import rospy
    from mavros_msgs.msg import Tunnel
    from sensor_msgs.msg import JointState

    rospy.init_node("verify_tilt_sitl", anonymous=True, log_level=rospy.ERROR)
    results = []
    bench = None
    error = None
    try:
        if not rospy.get_param("/use_sim_time", False):
            raise RuntimeError("/use_sim_time must be true")
        bench = Bench(rospy, Tunnel, JointState, args)
        previous_target = None
        for target in STEPS_DEG:
            results.append(bench.wait_for_step(target, previous_target))
            previous_target = target
    except (RuntimeError, KeyboardInterrupt) as exc:
        error = str(exc) or "interrupted"
        print("Return to neutral in PX4: mavlink tilt_test 0", file=sys.stderr, flush=True)
    print(json.dumps({
        "passed": error is None,
        "observed_at": ["mavros/tunnel/out", args.joint_state_topic, "mavros/tunnel/in"],
        "steps": results,
        "initial_or_fault_feedback_frames": bench.invalid_feedback_count if bench is not None else 0,
        "error": error,
    }, sort_keys=True))
    return 0 if error is None else 1


if __name__ == "__main__":
    sys.exit(main())
