"""ROS-free regression tests for the Gazebo Classic tilt bridge."""

import importlib.util
import math
from pathlib import Path
import struct
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch


script = Path(__file__).resolve().parents[1] / "tilt_joint_from_tunnel.py"
spec = importlib.util.spec_from_file_location("tilt_bridge", script)
bridge = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bridge)

FRONT = "front_tilt_joint"
REAR = "behind_tilt_joint"
SIM_SECOND = 1_000_000_000


def command(angle_deg, **overrides):
    fields = dict(target_system=1, target_component=191, payload_type=32778,
                  payload_length=4, payload=list(struct.pack("<f", angle_deg) + bytes(124)))
    fields.update(overrides)
    return SimpleNamespace(**fields)


def joint_state(stamp, positions, names=(FRONT, REAR)):
    return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(to_nsec=lambda: stamp)),
                           name=names, position=positions)


def new_feedback(signs=(1.0, 1.0), zeros=(0.0, 0.0)):
    return bridge.JointFeedback((FRONT, REAR), signs, zeros, math.radians(35.5),
                                math.radians(2), 0.3, 1.5, 0.06)


class ProtocolTest(unittest.TestCase):
    def test_command_degrees_and_pitch_is_independent(self):
        packet = command(10.0)
        self.assertEqual(bytes(packet.payload[:4]), bytes.fromhex("00002041"))
        self.assertAlmostEqual(bridge.decode_command(packet, 1, 191, 35.0), math.radians(10))
        self.assertIsNone(bridge.decode_command(command(10, payload_type=32779), 1, 191, 35.0))

    def test_rejects_wrong_target_layout_and_angle(self):
        for packet in (command(1, target_system=2), command(1, target_component=1),
                       command(1, payload_length=16), command(1, payload=[0] * 4),
                       command(math.nan), command(math.inf), command(36)):
            with self.subTest(packet=packet):
                with self.assertRaises(ValueError):
                    bridge.decode_command(packet, 1, 191, 35.0)

    def test_feedback_bytes(self):
        payload = bytes(bridge.feedback_payload(1, 5000, math.radians(10), bridge.VALID))
        self.assertEqual(len(payload), 128)
        self.assertEqual(payload[:16].hex(), "0100000088130000c2b8323e01000000")
        self.assertEqual(payload[16:], bytes(112))


class JointFeedbackTest(unittest.TestCase):
    def test_measured_same_frame_calibration_order_and_age(self):
        feedback = new_feedback(signs=(1.0, -1.0), zeros=(0.01, 0.02))
        feedback.accept(joint_state(SIM_SECOND, [0.02 - 0.11, 0.01 + 0.10], (REAR, FRONT)),
                        SIM_SECOND, 0.0)
        angle, age, flags = feedback.next_report(SIM_SECOND + 12_345_678, 0.02)
        self.assertAlmostEqual(angle, 0.105)
        self.assertEqual((age, flags), (12_345, bridge.VALID))

    def test_duplicate_and_old_packets_do_not_refresh_sample(self):
        feedback = new_feedback()
        first = joint_state(SIM_SECOND, [0.1, 0.1])
        feedback.accept(first, SIM_SECOND, 0.0)
        self.assertEqual(feedback.next_report(SIM_SECOND, 0.02), (0.1, 0, bridge.VALID))
        feedback.accept(first, SIM_SECOND, 0.29)
        feedback.accept(joint_state(SIM_SECOND - 10, [0.0, 0.0]), SIM_SECOND, 0.29)
        self.assertIsNone(feedback.next_report(SIM_SECOND, 1.4))
        self.assertEqual(feedback.next_report(SIM_SECOND, 1.51), (0.1, 0, bridge.FAULT))

    def test_slow_lockstep_reports_simulation_age_not_wall_age(self):
        feedback = new_feedback()
        feedback.accept(joint_state(SIM_SECOND, [0.1, 0.1]), SIM_SECOND, 0.0)
        self.assertEqual(feedback.next_report(SIM_SECOND + 7_000_000, 1.0), (0.1, 7000, bridge.VALID))
        self.assertEqual(feedback.next_report(SIM_SECOND + 7_000_000, 1.51), (0.1, 0, bridge.FAULT))

    def test_late_samples_do_not_extend_wall_freshness(self):
        feedback = new_feedback()
        feedback.accept(joint_state(SIM_SECOND, [0.1, 0.1]), SIM_SECOND, 0.0)
        self.assertEqual(feedback.next_report(SIM_SECOND, 0.0), (0.1, 0, bridge.VALID))
        sim_ns = SIM_SECOND + 400_000_000
        feedback.accept(joint_state(SIM_SECOND + 1, [0.2, 0.2]), sim_ns, 1.3)
        self.assertEqual(feedback.next_report(sim_ns, 1.3), (0.1, 0, bridge.FAULT))
        feedback.accept(joint_state(sim_ns, [0.2, 0.2]), sim_ns, 1.4)
        self.assertEqual(feedback.next_report(sim_ns, 1.4), (0.2, 0, bridge.VALID))

    def test_clock_freshness_and_callback_reordering(self):
        feedback = new_feedback()
        stamp = SIM_SECOND + 10_000_000
        feedback.accept(joint_state(stamp, [0.1, 0.1]), SIM_SECOND, 0.0)
        self.assertIsNone(feedback.next_report(SIM_SECOND, 0.0))
        self.assertEqual(feedback.next_report(stamp, 0.01), (0.1, 0, bridge.VALID))
        self.assertEqual(feedback.next_report(stamp + 310_000_000, 0.02), (0.1, 0, bridge.FAULT))

    def test_skew_invalidates_immediately_and_reports_persistent_fault(self):
        feedback = new_feedback()
        feedback.accept(joint_state(SIM_SECOND, [0.1, 0.1]), SIM_SECOND, 0.0)
        self.assertEqual(feedback.next_report(SIM_SECOND, 0.0), (0.1, 0, bridge.VALID))
        feedback.accept(joint_state(SIM_SECOND + 1, [0.10, 0.15]), SIM_SECOND + 1, 0.01)
        self.assertEqual(feedback.next_report(SIM_SECOND + 1, 0.02), (0.1, 0, 0))
        feedback.accept(joint_state(SIM_SECOND + 20_000_001, [0.10, 0.15]), SIM_SECOND + 20_000_001, 0.5)
        self.assertEqual(feedback.next_report(SIM_SECOND + 20_000_001, 0.51), (0.1, 0, 0))
        feedback.accept(joint_state(SIM_SECOND + 60_000_001, [0.10, 0.15]), SIM_SECOND + 60_000_001, 1.0)
        self.assertEqual(feedback.next_report(SIM_SECOND + 60_000_001, 1.01), (0.1, 0, bridge.FAULT))
        feedback.accept(joint_state(SIM_SECOND + 70_000_001, [0.12, 0.12]), SIM_SECOND + 70_000_001, 1.1)
        self.assertEqual(feedback.next_report(SIM_SECOND + 70_000_001, 1.11), (0.12, 0, bridge.VALID))

    def test_missing_duplicate_and_invalid_joint_states(self):
        for message in (joint_state(SIM_SECOND, [0.1], (FRONT,)),
                        joint_state(SIM_SECOND, [0.1, 0.1], (FRONT, FRONT)),
                        joint_state(SIM_SECOND, [0.1]),
                        joint_state(SIM_SECOND, [math.nan, 0.1]),
                        joint_state(SIM_SECOND, [0.7, 0.7])):
            with self.subTest(message=message):
                feedback = new_feedback()
                feedback.accept(message, SIM_SECOND, 0.0)
                self.assertEqual(feedback.next_report(SIM_SECOND, 0.0), (0.0, 0, bridge.FAULT))

    def test_reset_recovers_in_new_sim_epoch(self):
        feedback = new_feedback()
        feedback.accept(joint_state(10 * SIM_SECOND, [0.1, 0.1]), 10 * SIM_SECOND, 0.0)
        self.assertEqual(feedback.next_report(10 * SIM_SECOND, 0.01), (0.1, 0, bridge.VALID))
        self.assertEqual(feedback.next_report(0, 0.02), (0.0, 0, bridge.FAULT))
        feedback.accept(joint_state(SIM_SECOND, [0.2, 0.2]), SIM_SECOND, 0.03)
        self.assertEqual(feedback.next_report(SIM_SECOND, 0.04), (0.2, 0, bridge.VALID))


class PublisherTest(unittest.TestCase):
    def setUp(self):
        node = bridge.TiltJointFromTunnel.__new__(bridge.TiltJointFromTunnel)
        node.ros = Mock()
        node.ros.Time.now.return_value.to_nsec.return_value = SIM_SECOND
        node.Float64 = SimpleNamespace
        node.Tunnel = SimpleNamespace
        node.lock = bridge.threading.Lock()
        node.target_pub = Mock()
        node.feedback_pub = Mock()
        node.feedback = new_feedback()
        node.system_id, node.component_id, node.autopilot_component = 1, 191, 1
        node.limit_deg = 35.0
        node.command_timeout = 0.5
        node.last_command_wall = None
        node.command_timed_out = False
        node.next_fault_wall = 0.0
        node.last_feedback_flags = None
        node.sequence = 0xffffffff
        self.node = node

    def test_target_cannot_be_used_as_feedback_and_sequence_wraps(self):
        node = self.node
        with patch.object(bridge.time, "monotonic", return_value=10.0):
            node.tunnel_cb(command(10))
            node.step()
        target = node.target_pub.publish.call_args[0][0]
        self.assertAlmostEqual(target.data, math.radians(10))
        message = node.feedback_pub.publish.call_args[0][0]
        self.assertEqual((message.target_system, message.target_component, message.payload_length),
                         (1, 1, 16))
        values = bridge.FEEDBACK.unpack(bytes(message.payload[:16]))
        self.assertEqual(values, (0xffffffff, 0, 0.0, bridge.FAULT))
        with patch.object(bridge.time, "monotonic", return_value=10.01):
            node.joints_cb(joint_state(SIM_SECOND, [0.02, 0.02]))
            node.step()
        message = node.feedback_pub.publish.call_args[0][0]
        seq, age, angle, flags = bridge.FEEDBACK.unpack(bytes(message.payload[:16]))
        self.assertEqual((seq, age, flags), (0, 0, bridge.VALID))
        self.assertAlmostEqual(angle, 0.02)

    def test_fault_transition_is_immediate_and_command_timeout_keeps_feedback(self):
        node = self.node
        with patch.object(bridge.time, "monotonic") as monotonic:
            monotonic.return_value = 10.0
            node.tunnel_cb(command(10))
            node.joints_cb(joint_state(SIM_SECOND, [0.1, 0.1]))
            node.step()
            self.assertEqual(node.last_feedback_flags, bridge.VALID)
            for offset, elapsed, positions, flags in (
                    (1, 0.40, [0.1, 0.15], 0),
                    (60_000_001, 0.47, [0.1, 0.15], bridge.FAULT),
                    (70_000_001, 0.52, [0.11, 0.11], bridge.VALID)):
                monotonic.return_value = 10.0 + elapsed
                stamp = SIM_SECOND + offset
                node.ros.Time.now.return_value.to_nsec.return_value = stamp
                node.joints_cb(joint_state(stamp, positions))
                node.step()
                self.assertEqual(node.last_feedback_flags, flags)
            self.assertTrue(node.command_timed_out)
            self.assertEqual(node.target_pub.publish.call_count, 1)
            self.assertEqual(node.feedback_pub.publish.call_count, 4)


if __name__ == "__main__":
    unittest.main()
