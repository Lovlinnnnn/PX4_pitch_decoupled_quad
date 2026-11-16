#!/usr/bin/env python3
"""Fly the original keyboard-triggered offboard_pkg eight on local 22012 SITL."""

import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

from verify_shared_tilt_bench import PITCH, check_master
from verify_shared_tilt_flight import FlightTest, TELEMETRY_WALL_TIMEOUT, relative_position, rmse, xyz
from verify_zero_tilt_hover import IN_AIR, MAVROS


EIGHT_NODE = "/offb_node_keyboard_pitch_figure8"
RAW_SETPOINT = MAVROS + "/setpoint_raw/local"
GATE_SERVICE = "/verify_shared_tilt_eight/arm"
EXPECTED_DURATION = 100.0
EXPECTED_X = 1.5
EXPECTED_Y = 1.125
SAMPLE_PERIOD = 0.1


class EightProcess:
    """Run the installed ROS node with its original interactive terminal input."""

    def __init__(self, sim_time):
        import pty

        self.sim_time = sim_time
        self.hover_count = 0
        self.started = None
        self.finished = None
        self.master, slave = pty.openpty()
        try:
            self.process = subprocess.Popen(
                ["rosrun", "offboard_pkg", "eight", "mavros/cmd/arming:=" + GATE_SERVICE],
                stdin=slave, stdout=slave, stderr=slave, start_new_session=True,
            )
        except BaseException:
            os.close(self.master)
            raise
        finally:
            os.close(slave)
        self.reader = threading.Thread(target=self.read_output, daemon=True)
        self.reader.start()

    def read_output(self):
        with os.fdopen(self.master, "rb", closefd=False) as stream:
            while True:
                try:
                    raw = stream.readline()
                except OSError:
                    return  # Linux PTYs report EIO when the last slave closes.
                if not raw:
                    return
                line = raw.decode(errors="replace").rstrip()
                if "Figure eight started;" in line:
                    self.started = self.sim_time()
                elif "Hover: 1=0 deg" in line:
                    self.hover_count += 1
                    if self.started is not None and self.hover_count == 2:
                        self.finished = self.sim_time()
                print("[original eight] " + line, file=sys.stderr, flush=True)

    def assert_running(self):
        code = self.process.poll()
        if code is not None:
            raise RuntimeError("original eight exited before AUTO.LAND (status {})".format(code))

    def send_keys(self, keys):
        self.assert_running()
        os.write(self.master, keys)

    def close(self):
        if self.process.poll() is None:
            for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
                try:
                    os.killpg(self.process.pid, sig)
                except ProcessLookupError:
                    break
                try:
                    self.process.wait(timeout=3.0)
                    break
                except subprocess.TimeoutExpired:
                    continue
        self.reader.join(1.0)
        os.close(self.master)


class EightTest(FlightTest):
    def __init__(self, ros, types, args):
        super().__init__(ros, types, args)
        # The original node owns both command streams through the entire test.
        self.publisher.unregister()
        self.pitch_publisher.unregister()
        self.subscribers.extend((
            ros.Subscriber(RAW_SETPOINT, types["PositionTarget"], self.record_reference, queue_size=5),
            ros.Subscriber(MAVROS + "/tunnel/in", types["Tunnel"], self.record_pitch, queue_size=10),
        ))
        self.eight = None
        self.gate = None
        self.gate_lock = threading.Lock()
        self.gate_error = None
        self.input_error = None
        self.closing = False
        self.world_yaw = 0.0
        self.native_checks = {}

    def record_reference(self, message):
        if message._connection_header.get("callerid") != EIGHT_NODE:
            self.input_error = "unexpected publisher on " + RAW_SETPOINT
            return
        self.record(message, "reference")

    def record_pitch(self, message):
        if message.payload_type != 32779:
            return
        if (message._connection_header.get("callerid") != EIGHT_NODE
                or message.target_system != 1 or message.target_component != 1
                or message.payload_length != PITCH.size):
            self.input_error = "unexpected source or format for the 32779 pitch reference"
            return
        (pitch,) = PITCH.unpack(bytes(message.payload[:PITCH.size]))
        if not math.isfinite(pitch):
            self.input_error = "original eight sent a non-finite pitch reference"
            return
        if not self.args.manual and abs(pitch) > 1e-6:
            self.input_error = "original eight did not retain zero reference pitch"
            return
        self.record(message, "pitch")

    def startup_ready(self):
        state, on_ground = self.ground_state()
        if state is not None and state.armed:
            raise RuntimeError("expected original eight to start from disarm")
        tilt = self.tilt_feedback()
        return (state is not None and state.connected and on_ground and tilt is not None
                and max(map(abs, tilt[1])) <= 1.0 and self.snapshot("pose", TELEMETRY_WALL_TIMEOUT) is not None)

    def gate_arm(self, request):
        from mavros_msgs.srv import CommandBoolResponse

        with self.gate_lock:
            rejected = CommandBoolResponse(success=False, result=1)
            if self.closing or not request.value:
                return rejected
            try:
                self.check_child()
                if (not self.in_mode("OFFBOARD", armed=False) or not self.ground_state()[1]
                        or self.snapshot("reference", TELEMETRY_WALL_TIMEOUT) is None
                        or self.snapshot("pitch", TELEMETRY_WALL_TIMEOUT) is None):
                    raise RuntimeError("original eight arming request lacked disarmed OFFBOARD or fresh references")
                self.px4.command("shared_tilt_control", "enable")
                self.enabled = True
                self.wait_for(lambda: self.controller_state("waiting_arm"), 8.0, "22012 to accept eight's arming")
                if self.closing:
                    return rejected
                self.arm_attempted = True
                result = self.service(MAVROS + "/cmd/arming", "CommandBool", True)
                if not result.success:
                    raise RuntimeError("PX4 rejected original eight's arming request ({})".format(result.result))
                return result
            except (RuntimeError, OSError, subprocess.SubprocessError, self.ros.ROSException) as exc:
                self.gate_error = str(exc)
                return rejected

    def controller_state(self, target):
        status = self.px4.research_status()
        if status.get("state") == "fault":
            raise RuntimeError("22012 controller gate fault: " + status.get("fault", "unknown"))
        return status.get("state") == target

    def check_child(self):
        self.eight.assert_running()
        if self.gate_error or self.input_error:
            raise RuntimeError(self.gate_error or self.input_error)

    def start_eight(self):
        self.verify_endpoints()
        self.wait_for(self.startup_ready, self.args.timeout, "connected disarmed 22012 with valid tilt")
        self.world_origin, _, _ = self.world_pose()
        self.gate = self.ros.Service(GATE_SERVICE, self.types["CommandBool"], self.gate_arm)
        self.eight = EightProcess(self.sim_time)
        self.wait_for(self.observe_hover, self.args.hover_timeout, "original eight's own 1 m Hover gate")
        self.wait_for(lambda: self.controller_state("active"), self.args.timeout, "22012 active")

    def manual_keys(self):
        import select
        import termios
        import tty

        terminal = sys.stdin.fileno()
        saved = termios.tcgetattr(terminal)
        try:
            tty.setcbreak(terminal)
            print("Keyboard active: 1=zero pitch, 2/3=adjust pitch, 8=figure eight, Ctrl+C=AUTO.LAND.",
                  file=sys.stderr, flush=True)
            while not self.ros.is_shutdown():
                self.check_child()
                readable, _, _ = select.select([terminal], [], [], 0.2)
                if readable:
                    data = os.read(terminal, 64)
                    if not data:
                        raise RuntimeError("interactive terminal input closed")
                    keys = bytes(key for key in data if key in b"1238")
                    if keys:
                        self.eight.send_keys(keys)
            raise RuntimeError("ROS shut down during the interactive session")
        except KeyboardInterrupt:
            pass
        finally:
            termios.tcsetattr(terminal, termios.TCSADRAIN, saved)

    def saw_start(self):
        self.check_child()
        return self.eight.started is not None

    def observe_hover(self):
        self.check_child()
        now = self.sim_time()
        reference = self.snapshot("reference", TELEMETRY_WALL_TIMEOUT)
        pose = self.snapshot("pose", TELEMETRY_WALL_TIMEOUT)
        if reference is not None and pose is not None:
            target, actual = xyz(reference.position), xyz(pose.pose.position)
            with self.lock:
                pose_received_at = self.latest["pose"][0]
            self.metrics["last_original_hover_gate_sample"] = {
                "sim_time": now, "reference_enu_m": target, "estimated_enu_m": actual,
                "error_3d_m": math.dist(target, actual),
                "pose_age_sim_s": now - pose.header.stamp.to_sec(),
                "pose_receive_age_wall_s": time.monotonic() - pose_received_at,
                "original_reported_hover": self.eight.hover_count > 0,
            }
        landed = self.snapshot("landed", TELEMETRY_WALL_TIMEOUT)
        if (self.eight.hover_count != 1 or not self.in_mode("OFFBOARD", armed=True)
                or landed is None or landed.landed_state != IN_AIR
                or reference is None or self.snapshot("pitch", TELEMETRY_WALL_TIMEOUT) is None):
            return False
        truth, _ = self.world_sample(now)
        return 0.45 < truth[2] < 1.65

    def collect(self):
        samples = []
        feedback = {}
        references = set()
        next_sample = self.eight.started
        next_message = 25.0

        def sample_until_finished():
            nonlocal next_sample, next_message
            self.check_child()
            now = self.sim_time()
            elapsed = now - self.eight.started
            state = self.snapshot("state", 10.0)
            if state is not None and (not state.connected or not state.armed or state.mode != "OFFBOARD"):
                raise RuntimeError("original eight left armed OFFBOARD before its return to Hover")
            if now < next_sample:
                return False
            next_sample = now + SAMPLE_PERIOD
            reference = self.snapshot("reference", TELEMETRY_WALL_TIMEOUT)
            pose = self.snapshot("pose", TELEMETRY_WALL_TIMEOUT)
            landed = self.snapshot("landed", TELEMETRY_WALL_TIMEOUT)
            if (state is None or reference is None or pose is None or landed is None or landed.landed_state != IN_AIR
                    or self.snapshot("pitch", TELEMETRY_WALL_TIMEOUT) is None
                    or not -0.02 <= now - reference.header.stamp.to_sec() <= 0.25
                    or not -0.02 <= now - pose.header.stamp.to_sec() <= 0.25):
                return False
            if reference.coordinate_frame != reference.FRAME_LOCAL_NED or reference.type_mask != reference.IGNORE_YAW_RATE:
                raise RuntimeError("original eight changed its raw position/velocity/acceleration frame or mask")
            target = relative_position(xyz(reference.position), self.center, 0.0)
            estimated = relative_position(xyz(pose.pose.position), self.center, 0.0)
            truth, pitch = self.world_sample(now)
            actual = (truth[0], truth[1], truth[2] - 1.0)
            if not all(map(math.isfinite, target + estimated + actual + (pitch,))):
                raise RuntimeError("non-finite reference or pose during original eight")
            if abs(actual[0]) > EXPECTED_X + 0.9 or abs(actual[1]) > EXPECTED_Y + 0.9 or abs(actual[2]) > 0.65:
                raise RuntimeError("Gazebo left the original eight trial envelope")
            tilt = self.tilt_feedback()
            if tilt is not None and abs(sum(tilt[1][:2]) * 0.5 - tilt[1][2]) <= 0.8:
                feedback.setdefault(tilt[0], (elapsed, max(map(abs, tilt[1]))))
            samples.append((elapsed, target, estimated, actual, pitch))
            references.add(reference.header.stamp.to_nsec())
            self.metrics["last_eight_sample"] = {"elapsed_sim_s": elapsed, "reference_offset_enu_m": target,
                                                "estimated_offset_enu_m": estimated, "gazebo_offset_enu_m": actual}
            lobe = "positive_x" if target[0] >= EXPECTED_X/2 else "negative_x" if target[0] <= -EXPECTED_X/2 else None
            if lobe and lobe not in self.native_checks:
                native = self.native_allocation(0.0)
                if native:
                    self.native_checks[lobe] = native
            if elapsed >= next_message:
                print("original eight: {:.1f} simulated seconds sampled".format(elapsed), file=sys.stderr, flush=True)
                next_message += 25.0
            if elapsed > EXPECTED_DURATION + 3.0 and self.eight.finished is None:
                raise RuntimeError("original eight did not return to Hover at its 100 s endpoint")
            return self.eight.finished is not None

        self.wait_for(sample_until_finished, self.args.eight_timeout, "full original eight and return to Hover")
        duration = self.eight.finished - self.eight.started
        self.summarize(samples, feedback, references, duration)

    def summarize(self, samples, feedback, references, duration):
        if not samples:
            raise RuntimeError("no synchronized reference and Gazebo samples were captured during original eight")

        def bounds(values):
            return [min(values), max(values)]

        def max_gap(times):
            return max(b - a for a, b in zip([0.0] + times, times + [duration]))

        target_x = bounds([target[0] for _, target, _, _, _ in samples])
        target_y = bounds([target[1] for _, target, _, _, _ in samples])
        actual_x = bounds([actual[0] for _, _, _, actual, _ in samples])
        actual_y = bounds([actual[1] for _, _, _, actual, _ in samples])
        sample_gap = max_gap([sample[0] for sample in samples])
        feedback_gap = max_gap(sorted(m[0] for m in feedback.values()))
        closure = math.hypot(*samples[-1][3][:2])
        gazebo_errors = [math.hypot(actual[0] - target[0], actual[1] - target[1])
                         for _, target, _, actual, _ in samples]
        estimate_errors = [math.hypot(estimate[0] - target[0], estimate[1] - target[1])
                           for _, target, estimate, _, _ in samples]
        self.metrics["original_eight"] = {
            "duration_sim_s": duration, "reference_pitch_deg": 0.0, "position_samples": len(samples),
            "raw_reference_samples": len(references), "32780_samples": len(feedback),
            "position_max_gap_sim_s": sample_gap, "32780_max_gap_sim_s": feedback_gap,
            "target_x_bounds_m": target_x, "target_y_bounds_m": target_y,
            "gazebo_x_bounds_m": actual_x, "gazebo_y_bounds_m": actual_y,
            "gazebo_xy_rmse_m": rmse(gazebo_errors), "gazebo_xy_peak_error_m": max(gazebo_errors),
            "estimated_xy_rmse_m": rmse(estimate_errors),
            "gazebo_altitude_rmse_m": rmse([actual[2] for _, _, _, actual, _ in samples]),
            "gazebo_body_pitch_max_deg": max(abs(pitch) for _, _, _, _, pitch in samples),
            "32780_and_joint_max_abs_deg": max((m[1] for m in feedback.values()), default=None),
            "gazebo_closure_m": closure, "native_allocation_by_lobe": self.native_checks,
        }
        if abs(duration - EXPECTED_DURATION) > 1.0:
            raise RuntimeError("original eight duration differs from 100 simulated seconds")
        if max(sample_gap, feedback_gap) > 1.0:
            raise RuntimeError("original eight did not retain continuous position and 32780 feedback")
        if any(abs(limit - expected) > 0.06
               for limit, expected in zip(target_x + target_y, (-EXPECTED_X, EXPECTED_X, -EXPECTED_Y, EXPECTED_Y))):
            raise RuntimeError("raw references did not cover the original two-lobe geometry")
        if (not actual_x[0] <= -EXPECTED_X / 2 <= EXPECTED_X / 2 <= actual_x[1]
                or not actual_y[0] <= -EXPECTED_Y / 2 <= EXPECTED_Y / 2 <= actual_y[1]
                or closure > 0.45):
            raise RuntimeError("Gazebo did not traverse both lobes and return near the starting point")
        if set(self.native_checks) != {"positive_x", "negative_x"}:
            raise RuntimeError("missing measured-angle native allocation evidence on at least one lobe")

    def run(self):
        if self.args.manual and not sys.stdin.isatty():
            raise RuntimeError("--manual requires an interactive terminal")
        self.start_eight()
        if self.args.manual:
            self.manual_keys()
        else:
            self.center = xyz(self.snapshot("reference", TELEMETRY_WALL_TIMEOUT).position)
            self.eight.send_keys(b"8")
            self.wait_for(self.saw_start, self.args.timeout, "original eight to accept keyboard 8")
            self.collect()
            px4_reference = self.px4.topic("tilting_mc_desired_angles")
            pitch = float(px4_reference["pitch_body"])
            if not math.isfinite(pitch) or abs(pitch) > 1e-6:
                raise RuntimeError("PX4 did not retain the original eight's zero 32779 reference")
        self.land(acceptance=True)
        code = self.eight.process.wait(timeout=20.0)
        if code != 0:
            raise RuntimeError("original eight did not exit normally at AUTO.LAND (status {})".format(code))

    def cleanup(self):
        self.closing = True
        try:
            if self.gate is not None:
                self.gate.shutdown("original eight verification ended")
            with self.gate_lock:
                super().cleanup()
        finally:
            if self.eight is not None:
                self.eight.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manual", action="store_true", help="forward terminal keys to eight and land on Ctrl+C")
    parser.add_argument("--timeout", type=float, default=240.0, help="wall timeout for individual startup checks")
    parser.add_argument("--hover-timeout", type=float, default=500.0, help="wall timeout for eight's own hover gate")
    parser.add_argument("--eight-timeout", type=float, default=1900.0, help="wall timeout for the full 100 s eight")
    parser.add_argument("--land-timeout", type=float, default=400.0, help="wall timeout per landing check")
    parser.add_argument("--px4-bin-dir", type=Path, default=Path("build/px4_sitl_default/bin"))
    args = parser.parse_args()
    if not all(math.isfinite(value) and value > 0
               for value in (args.timeout, args.hover_timeout, args.eight_timeout, args.land_timeout)):
        parser.error("all timeouts must be finite and positive")

    import rospy
    from gazebo_msgs.srv import GetJointProperties, GetLinkState, GetModelProperties
    from geometry_msgs.msg import PoseStamped
    from mavros_msgs.msg import ExtendedState, PositionTarget, State, Tunnel
    from mavros_msgs.srv import CommandBool, SetMode
    from sensor_msgs.msg import JointState

    test = None
    error = cleanup_error = None
    failures = (OSError, RuntimeError, ValueError, KeyError, rospy.ROSException, rospy.ServiceException,
                subprocess.SubprocessError, KeyboardInterrupt)
    try:
        check_master()
        rospy.init_node("verify_shared_tilt_eight", anonymous=True, log_level=rospy.ERROR, disable_signals=True)
        types = {cls.__name__: cls for cls in (GetJointProperties, GetLinkState, GetModelProperties, PoseStamped,
                                              ExtendedState, PositionTarget, State, Tunnel, CommandBool, SetMode,
                                              JointState)}
        test = EightTest(rospy, types, args)
        test.run()
    except failures as exc:
        error = str(exc) or "interrupted"
    finally:
        if test is not None:
            try:
                test.cleanup()
            except failures as exc:
                cleanup_error = str(exc) or "interrupted"
    passed = error is None and cleanup_error is None
    test_name = "interactive-eight" if args.manual else "original-eight"
    print(json.dumps({"passed": passed, "test": test_name, "error": error, "cleanup_error": cleanup_error,
                      "metrics": test.metrics if test is not None else {}}, sort_keys=True, allow_nan=False))
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
