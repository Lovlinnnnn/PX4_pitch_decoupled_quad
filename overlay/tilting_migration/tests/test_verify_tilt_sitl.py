"""Check verifier gates without ROS or Gazebo."""

from collections import deque
from contextlib import redirect_stderr
import io
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import verify_tilt_sitl as verifier


class VerificationTest(unittest.TestCase):
    def make_bench(self, target, reported_during_motion):
        bench = verifier.Bench.__new__(verifier.Bench)
        bench.args = SimpleNamespace(step_timeout=0.25, match_tolerance_deg=0.05,
                                     settle_tolerance_deg=1.0)
        bench.ros = Mock()
        bench.ros.is_shutdown.return_value = False
        bench.lock = threading.Lock()
        bench.commands = deque([(1.01, target, verifier.COMMAND.pack(target).hex())])
        bench.joints = deque()
        bench.feedback = deque()
        for index, angle in enumerate((0.0, target * 0.4, target * 0.8) + (target,) * 9):
            stamp = 500_000_000 + index * 20_000_000
            received = 1.02 + index * 0.02
            bench.joints.append(verifier.JointSample(received, stamp, angle, angle))
            measured = angle if reported_during_motion else target
            bench.feedback.append(verifier.Feedback(received + 0.01, stamp, index, 1000, measured))
        return bench

    def run_step(self, bench, target):
        clock = iter(1 + i * 0.05 for i in range(100))
        with patch.object(verifier.time, "monotonic", side_effect=lambda: next(clock)), \
                patch.object(verifier.time, "sleep"), redirect_stderr(io.StringIO()):
            return bench.wait_for_step(target, 0)

    def test_measured_feedback_moving_and_settled(self):
        for target in (-5, 5, 10):
            with self.subTest(target=target):
                result = self.run_step(self.make_bench(target, True), target)
                self.assertEqual(result["settled_feedback_deg"], target)
                self.assertAlmostEqual(result["in_transit_measurement"]["measured_deg"], target * 0.4)

    def test_feedback_echoing_target_does_not_pass(self):
        with self.assertRaisesRegex(RuntimeError, "motion witnessed=False"):
            self.run_step(self.make_bench(5, False), 5)


if __name__ == "__main__":
    unittest.main()
