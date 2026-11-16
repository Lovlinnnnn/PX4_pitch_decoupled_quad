#!/usr/bin/env python3
"""Verify 22010 shadow mode and 22011 on the Gazebo Classic fixed-body bench."""

import argparse
import json
import math
import os
from pathlib import Path
import re
import struct
import subprocess
import sys
import threading
import time
from urllib.parse import urlsplit
import xml.etree.ElementTree as ET

from verify_zero_tilt_hover import require_local


MODEL = "pitch_decoupled_quad"
FIXTURE = "shared_tilt_bench_fixture"
BODY = MODEL + "::base_link"
TILT_JOINTS = ("front_tilt_joint", "behind_tilt_joint")
ROTOR_JOINTS = tuple(MODEL + "::rotor_{}_joint".format(i) for i in range(4))
PITCH = struct.Struct("<f")
FEEDBACK = struct.Struct("<IIfI")
GAZEBO_THRUST_COEFFICIENT = 9.84e-6
BENCH_TORQUE_SCALE_NM = 10.0


def wait_for(predicate, timeout, description):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        value = predicate()
        if value:
            return value
        time.sleep(0.04)
    raise RuntimeError("timed out waiting for " + description)


def local_service(ros, name, message_type):
    from rosgraph.network import get_local_addresses

    ros.wait_for_service(name, timeout=8.0)
    code, _, uri = ros.get_master().lookupService(name)
    if code != 1:
        raise RuntimeError("could not locate " + name)
    require_local(urlsplit(uri).hostname, get_local_addresses())
    return ros.ServiceProxy(name, message_type)


def check_master():
    from rosgraph.network import get_local_addresses

    master = urlsplit(os.environ.get("ROS_MASTER_URI", "http://localhost:11311"))
    if master.scheme != "http" or master.port is None:
        raise RuntimeError("expected a local ROS master")
    require_local(master.hostname, get_local_addresses())


def spawn_fixture(ros, path):
    from gazebo_msgs.srv import GetModelProperties, SpawnModel
    from geometry_msgs.msg import Pose

    root = ET.parse(path).getroot()
    model = root.find("model")
    if (root.tag != "sdf" or model is None or model.get("name") != MODEL
            or model.find("link[@name='base_link']") is None
            or model.find("joint[@name='{}']".format(FIXTURE)) is not None):
        raise RuntimeError("expected the unmodified pitch_decoupled_quad model SDF")
    fixed = ET.SubElement(model, "joint", name=FIXTURE, type="fixed")
    ET.SubElement(fixed, "parent").text = "world"
    ET.SubElement(fixed, "child").text = "base_link"
    spawn = local_service(ros, "/gazebo/spawn_sdf_model", SpawnModel)
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = 1.01, 0.98, 0.83
    pose.orientation.w = 1.0
    response = spawn(MODEL, ET.tostring(root, encoding="unicode"), "", pose, "world")
    if not response.success:
        raise RuntimeError("Gazebo did not spawn the fixed model: " + response.status_message)
    properties = local_service(ros, "/gazebo/get_model_properties", GetModelProperties)(MODEL)
    if not properties.success or FIXTURE not in joint_names(properties):
        raise RuntimeError("Gazebo does not report the world-to-fuselage fixture")
    print("Spawned {} with {}".format(MODEL, FIXTURE), flush=True)


def joint_names(properties):
    return {name.rsplit("::", 1)[-1] for name in properties.joint_names}


def array_field(fields, name, minimum_length):
    value = fields[name]
    result = [float(item.strip()) for item in value.strip("[]").split(",")]
    if len(result) < minimum_length or not all(math.isfinite(item) for item in result[:minimum_length]):
        raise RuntimeError("invalid PX4 {}: {}".format(name, value))
    return result


def timestamp(fields):
    return int(fields.get("timestamp", "0").split()[0])


class Px4:
    def __init__(self, bin_dir):
        self.bin_dir = bin_dir.resolve()

    def command(self, module, *arguments, required=True):
        command = [str(self.bin_dir / ("px4-" + module)), *arguments]
        result = subprocess.run(command, check=False, text=True, stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, timeout=5.0)
        if required and result.returncode:
            raise RuntimeError("{} {} exited {}: {}".format(module, " ".join(arguments),
                                                            result.returncode, result.stdout.strip()))
        return result.stdout

    def topic(self, name):
        output = self.command("listener", name, "-n", "1", required=False)
        data = dict(re.findall(r"^\s*([a-z][a-z0-9_]*)\s*:\s*([^\r\n]+)", output, flags=re.MULTILINE))
        return data if "timestamp" in data else {}

    def research_status(self):
        output = self.command("shared_tilt_control", "status")
        return dict(re.findall(r"\b([a-z][a-z0-9_]*)=([^\s,]+)", output.strip()))

    def parameter(self, name):
        return self.command("param", "show", "-q", name).strip()

    def rotor_geometry(self):
        return [tuple(float(self.parameter("CA_ROTOR{}_{}".format(i, key))) for key in ("PX", "PY", "PZ", "KM"))
                for i in range(4)]


def motor_torque(motors, geometry, delta):
    """Reconstruct the moment from the output mapping and measured tilt, in FRD N m."""
    sine, cosine = math.sin(delta), math.cos(delta)
    moments = []
    for motor, (x, y, z, signed_drag) in zip(motors, geometry):
        thrust = GAZEBO_THRUST_COEFFICIENT * (100.0 + 1000.0 * motor) ** 2
        moments.append(tuple(thrust * arm for arm in (-y * cosine - signed_drag * sine,
                                                      x * cosine + z * sine,
                                                      -y * sine + signed_drag * cosine)))
    return [sum(components) for components in zip(*moments)]


class Harness:
    def __init__(self, ros, args):
        from gazebo_msgs.srv import GetJointProperties, GetLinkState, GetModelProperties
        from geometry_msgs.msg import PoseStamped
        from mavros_msgs.msg import State, Tunnel
        from mavros_msgs.srv import CommandBool, SetMode
        from sensor_msgs.msg import JointState

        self.ros = ros
        self.args = args
        self.px4 = Px4(args.px4_bin_dir)
        self.metrics = {}
        self.pitch_rad = 0.0
        self.pose = None
        self.pose_type = PoseStamped
        self.tunnel_type = Tunnel
        self.latest = {}
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.worker = None
        self.stream_error = None
        self.bench_enabled = False
        self.feedback_enabled = True
        self.pitch_pub = ros.Publisher("/mavros/tunnel/in", Tunnel, queue_size=5)
        self.pose_pub = ros.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=5)
        self.subscribers = [
            ros.Subscriber("/mavros/state", State, self.record, "state", queue_size=5),
            ros.Subscriber("/mavros/local_position/pose", PoseStamped, self.record, "pose", queue_size=5),
            ros.Subscriber("/mavros/tunnel/in", Tunnel, self.record_feedback, queue_size=5, tcp_nodelay=True),
            ros.Subscriber("/mavros/tunnel/out", Tunnel, self.record_command, queue_size=5, tcp_nodelay=True),
            ros.Subscriber("/pitch_decoupled_quad/joint_states", JointState, self.record, "joints", queue_size=5),
        ]
        if args.fault_input == "tilt":
            self.subscribers.append(ros.Subscriber(args.relay_feedback_topic, Tunnel, self.relay_feedback,
                                                   queue_size=5, tcp_nodelay=True))
        self.get_model = local_service(ros, "/gazebo/get_model_properties", GetModelProperties)
        if args.run == "bench":
            self.get_joint = local_service(ros, "/gazebo/get_joint_properties", GetJointProperties)
            self.get_link = local_service(ros, "/gazebo/get_link_state", GetLinkState)
            self.arming = local_service(ros, "/mavros/cmd/arming", CommandBool)
            self.set_mode = local_service(ros, "/mavros/set_mode", SetMode)

    def record(self, value, key):
        with self.lock:
            self.latest[key] = (time.monotonic(), value)

    def snapshot(self, key, maximum_age=2.0):
        with self.lock:
            entry = self.latest.get(key)
        return entry[1] if entry and time.monotonic() - entry[0] <= maximum_age else None

    def record_feedback(self, message):
        if (message.payload_type != 32780 or message.target_system != 1 or message.target_component != 1
                or message.payload_length != FEEDBACK.size or len(message.payload) != 128):
            return
        sequence, age_us, alpha, flags = FEEDBACK.unpack(bytes(message.payload[:FEEDBACK.size]))
        self.record((sequence, age_us, alpha), "feedback" if flags == 1 and math.isfinite(alpha) else "fault")

    def relay_feedback(self, message):
        if message.payload_type != 32780:
            return
        self.record(message, "relay_source")
        if self.feedback_enabled:
            self.pitch_pub.publish(message)

    def record_command(self, message):
        if (message.payload_type == 32778 and message.target_system == 1 and message.target_component == 191
                and message.payload_length == PITCH.size and len(message.payload) == 128):
            alpha, = PITCH.unpack(bytes(message.payload[:PITCH.size]))
            self.record(math.radians(alpha), "command")

    def stream(self):
        try:
            while not self.stop.is_set() and not self.ros.is_shutdown():
                pitch = self.pitch_rad
                if pitch is not None:
                    message = self.tunnel_type()
                    message.payload_type, message.target_system, message.target_component = 32779, 1, 1
                    message.payload_length = PITCH.size
                    message.payload = list(PITCH.pack(pitch).ljust(128, b"\0"))
                    self.pitch_pub.publish(message)
                if self.pose is not None:
                    position = self.pose_type()
                    position.header.stamp = self.ros.Time.now()
                    position.header.frame_id = "map"
                    position.pose = self.pose
                    self.pose_pub.publish(position)
                self.stop.wait(0.05)
        except self.ros.ROSException as exc:
            self.stream_error = str(exc)

    def start_stream(self):
        self.worker = threading.Thread(target=self.stream, daemon=True)
        self.worker.start()

    def connected(self, armed, mode=None):
        state = self.snapshot("state", 8.0)
        return state is not None and state.connected and state.armed == armed and (mode is None or state.mode == mode)

    def check_endpoints(self):
        ros = self.ros
        if not ros.get_param("/use_sim_time", False):
            raise RuntimeError("/use_sim_time must be true")
        url = urlsplit(ros.get_param("/mavros/fcu_url", ""))
        if url.scheme != "udp" or url.port != 14580 or url.query != "ids=1,191":
            raise RuntimeError("expected the local MAVROS SITL UDP URL with ids=1,191")
        require_local(url.hostname)
        wait_for(lambda: self.connected(False), self.args.timeout, "a connected, disarmed vehicle")
        expected = 22010 if self.args.run == "shadow" else 22011
        if int(self.px4.parameter("SYS_AUTOSTART")) != expected:
            raise RuntimeError("expected SYS_AUTOSTART={}".format(expected))
        self.px4.command("mc_att_control", "status")
        self.px4.command("mc_rate_control", "status")
        allocator = self.px4.command("control_allocator", "status", required=self.args.run == "shadow")
        if self.args.run == "bench" and "not running" not in allocator:
            raise RuntimeError("stock allocator must be stopped on the research bench")
        model = self.get_model(MODEL)
        joint_set = joint_names(model)
        if (not model.success or not set(TILT_JOINTS) <= joint_set
                or (FIXTURE in joint_set) != (self.args.run == "bench")
                or (self.args.run == "bench" and model.is_static)):
            raise RuntimeError("Gazebo model/fixture does not match the requested test")
        _, _, topics = ros.get_master().getSystemState()
        owners = dict(topics[0]).get("/mavros/setpoint_position/local", ())
        if any(node != ros.get_name() for node in owners):
            raise RuntimeError("another publisher owns the local position reference")
        if self.args.fault_input == "tilt":
            tunnel_owners = dict(topics[0]).get("/mavros/tunnel/in", ())
            if set(tunnel_owners) != {ros.get_name()}:
                raise RuntimeError("tilt-loss test requires the harness to be the only MAVROS TUNNEL publisher")
        wait_for(lambda: self.snapshot("feedback") and self.pitch_pub.get_num_connections(), self.args.timeout,
                 "live 32780 and the 32779 publisher connection")
        self.metrics["airframe"] = expected
        self.metrics["fixed_fixture"] = FIXTURE in joint_set
        self.metrics["native_attitude_running"] = True
        self.metrics["native_rate_pid_running"] = True
        self.metrics["stock_allocator_running"] = self.args.run == "shadow"

    def pitch_received(self, value):
        report = self.px4.topic("tilting_mc_desired_angles")
        return report and abs(float(report["pitch_body"]) - value) < 1e-4

    def shadow(self):
        status = self.px4.research_status()
        command_before = timestamp(self.px4.topic("tilting_servo_setpoint"))
        self.pitch_rad = math.radians(self.args.pitch_deg)
        self.start_stream()
        wait_for(lambda: self.pitch_received(self.pitch_rad), self.args.timeout, "32779 in PX4")

        def sampled():
            current = self.px4.research_status()
            if (int(current.get("shadow_samples", 0)) > int(status.get("shadow_samples", 0))
                    and abs(float(current.get("pitch_ref_rad", "nan")) - self.pitch_rad) < 1e-4):
                return current
            return None

        current = wait_for(sampled, self.args.timeout, "shadow calculation")
        measured = self.px4.topic("tilting_servo_status")
        measured_alpha = float(measured["angle_rad"])
        delta = float(current["tilt_measured_rad"])
        if (measured["valid"].lower() != "true" or measured["fault"].lower() == "true"
                or not all(map(math.isfinite, (delta, measured_alpha)))
                or abs(delta + measured_alpha) > math.radians(0.8)):
            raise RuntimeError("shadow measured angle disagrees with the 32780 uORB")
        command_after = timestamp(self.px4.topic("tilting_servo_setpoint"))
        if current.get("mode") != "shadow" or int(current["actuator_publications"]) or command_before != command_after:
            raise RuntimeError("shadow run changed actuator publication")
        if not self.connected(False):
            raise RuntimeError("vehicle left disarmed state during shadow")
        self.metrics.update(module=current, pitch_ref_rad=self.pitch_rad, tilt_setpoint_timestamp_before=command_before,
                            tilt_setpoint_timestamp_after=command_after, measured_alpha_rad=measured_alpha)

    def fixed_body(self):
        state = self.get_link(BODY, "world")
        if not state.success:
            raise RuntimeError("could not read Gazebo fuselage pose")
        pose = state.link_state.pose
        position = [pose.position.x, pose.position.y, pose.position.z]
        rotation = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        if not all(map(math.isfinite, position + rotation)) or abs(sum(q * q for q in rotation) - 1.0) > 1e-3:
            raise RuntimeError("Gazebo returned an invalid fuselage pose")
        return position, rotation

    def rotor_rpm(self):
        rpm = []
        for joint in ROTOR_JOINTS:
            sample = self.get_joint(joint)
            if not sample.success or not sample.rate or not math.isfinite(sample.rate[0]):
                raise RuntimeError("could not read Gazebo rotor velocity: " + joint)
            rpm.append(abs(sample.rate[0]) * 10.0 * 60.0 / (2.0 * math.pi))
        return rpm

    def servo_angles(self):
        joints = self.snapshot("joints")
        if joints is None:
            return None
        positions = dict(zip(joints.name, joints.position))
        angles = tuple(positions.get(name, math.nan) for name in TILT_JOINTS)
        return angles if all(math.isfinite(angle) for angle in angles) else None

    def bench_step(self, pitch_deg, geometry):
        value = math.radians(pitch_deg)
        direction = math.copysign(1.0, value)
        self.pitch_rad = value

        def settled():
            if not self.connected(True, "OFFBOARD"):
                raise RuntimeError("bench left armed OFFBOARD during pose verification")
            joints = self.servo_angles()
            feedback = self.snapshot("feedback")
            command = self.snapshot("command")
            return (joints, feedback, command) if (joints and feedback and command is not None
                   and max(abs(angle + value) for angle in (*joints, feedback[2], command)) < math.radians(2.0)
                   and abs(sum(joints) * 0.5 - feedback[2]) < math.radians(0.8)) else None

        joints, feedback, command = wait_for(settled, self.args.timeout, "settled {}° servo position".format(pitch_deg))
        received = self.px4.topic("tilting_servo_status")
        raw_pitch = self.px4.topic("tilting_mc_desired_angles")
        torque = array_field(self.px4.topic("vehicle_torque_setpoint"), "xyz", 3)[:3]
        rate = float(self.px4.topic("vehicle_rates_setpoint")["pitch"])
        allocator = self.px4.topic("control_allocator_status")
        residual = array_field(allocator, "unallocated_torque", 3)[:3]
        motors = array_field(self.px4.topic("actuator_motors"), "control", 4)[:4]
        rpm = self.rotor_rpm()
        pitch = float(raw_pitch["pitch_body"])
        alpha = float(received["angle_rad"])
        if (not all(map(math.isfinite, (pitch, alpha))) or abs(pitch - value) >= 1e-4
                or received["valid"].lower() != "true"
                or received["fault"].lower() == "true"
                or abs(alpha - feedback[2]) > math.radians(0.8)):
            raise RuntimeError("PX4 topic disagrees with the live 32779/32780 measurement")
        if (allocator["torque_setpoint_achieved"].lower() != "false"
                or not math.isfinite(rate) or rate * direction <= 0.01
                or torque[1] * direction <= 0.01 or residual[1] * direction <= 0.005):
            raise RuntimeError("expected native PID torque and a matching saturation residual for pitch "
                               "{} deg (rate {}, torque {}, residual {})".format(pitch_deg, rate, torque, residual))
        expected_rpm = [(100.0 + 1000.0 * motor) * 60.0 / (2.0 * math.pi) for motor in motors]
        if min(rpm) < 250 or max(abs(actual - expected) for actual, expected in zip(rpm, expected_rpm)) > 700:
            raise RuntimeError("Gazebo RPM differs from PX4 motor output (rpm {}, expected {})".format(rpm, expected_rpm))
        modeled = motor_torque(motors, geometry, -alpha)
        expected_residual = [request - achieved / BENCH_TORQUE_SCALE_NM for request, achieved in zip(torque, modeled)]
        error = max(abs(actual - expected) for actual, expected in zip(residual, expected_residual))
        if error > 0.02:
            raise RuntimeError("PX4 allocator torque residual differs from expected by {}".format(error))
        landed = self.px4.topic("vehicle_land_detected")
        native_status = self.px4.topic("rate_ctrl_status")
        return {"pitch_deg": pitch_deg, "px4_pitch_rate_sp_rad_s": rate, "px4_torque_sp": torque,
                "px4_unallocated_torque": residual, "max_torque_residual_error": error,
                "gazebo_tilt_deg": list(map(math.degrees, joints)),
                "32778_deg": math.degrees(command), "32780_deg": math.degrees(feedback[2]),
                "32780_sequence": feedback[0], "32780_age_us": feedback[1],
                "32780_px4_sample_us": timestamp({"timestamp": received["timestamp_sample"]}),
                "motor_commands": motors, "measured_rpm": rpm, "model_rpm": expected_rpm,
                "native_pitch_integral": float(native_status["pitchspeed_integ"]),
                "land_detector": {key: landed.get(key) for key in ("landed", "maybe_landed")}}

    def bench(self):
        position, rotation = self.fixed_body()
        geometry = self.px4.rotor_geometry()
        self.pose = wait_for(lambda: self.snapshot("pose"), self.args.timeout, "PX4 local position").pose
        self.start_stream()
        start = self.ros.Time.now().to_sec()
        wait_for(lambda: self.ros.Time.now().to_sec() - start >= 2.0, self.args.timeout, "OFFBOARD prestream")
        if not self.set_mode(custom_mode="OFFBOARD").mode_sent:
            raise RuntimeError("MAVROS failed to request OFFBOARD")
        wait_for(lambda: self.connected(False, "OFFBOARD"), self.args.timeout, "disarmed OFFBOARD")
        wait_for(lambda: self.pitch_received(0.0), self.args.timeout, "zero-pitch 32779")
        self.px4.command("shared_tilt_control", "enable")
        self.bench_enabled = True

        def enabled():
            report = self.px4.research_status()
            if report.get("state") == "fault":
                raise RuntimeError("bench enable failed: {}".format(report.get("fault")))
            return report.get("state") == "waiting_arm"

        wait_for(enabled, self.args.timeout, "fixed bench enable gate")
        arm = self.arming(True)
        if not arm.success:
            raise RuntimeError("normal arming rejected on the fixed bench: result {}".format(arm.result))
        wait_for(lambda: self.connected(True, "OFFBOARD"), self.args.timeout, "armed OFFBOARD")
        initial_allocator = self.px4.topic("control_allocator_status")
        if initial_allocator:
            self.metrics["neutral_px4_unallocated_torque"] = array_field(initial_allocator, "unallocated_torque", 3)[:3]
        self.metrics["steps"] = []
        for pitch_deg in (self.args.pitch_deg, -self.args.pitch_deg):
            step = self.bench_step(pitch_deg, geometry)
            self.metrics["steps"].append(step)
            print("Accepted {}° and matching measured tilt/saturation".format(pitch_deg), file=sys.stderr, flush=True)
        self.metrics["module"] = self.px4.research_status()
        fault = self.args.fault_input
        if fault == "pitch":
            self.pitch_rad = None  # Keep OFFBOARD and 32780 active while withholding only 32779.
        else:
            pitch_before = timestamp(self.px4.topic("tilting_mc_desired_angles"))
            tilt_before = timestamp(self.px4.topic("tilting_servo_status"))
            self.feedback_enabled = False  # Keep the physical source and pilot references active.
        wait_for(lambda: self.connected(False), self.args.timeout, "automatic disarm after {} input stops".format(fault))
        status = self.px4.research_status()
        if status.get("state") != "fault" or status.get("fault") != fault:
            raise RuntimeError("disarm was not associated with the expected {} timeout: {}".format(fault, status))
        if fault == "tilt":
            pitch_after = timestamp(self.px4.topic("tilting_mc_desired_angles"))
            tilt_after = timestamp(self.px4.topic("tilting_servo_status"))
            if not self.snapshot("relay_source") or pitch_after <= pitch_before or tilt_after - tilt_before > 120_000:
                raise RuntimeError("32780 relay cut did not isolate PX4 feedback from the live source and 32779")
            self.metrics.update(continued_pitch_after_tilt_loss=True, tilt_px4_last_timestamp_us=tilt_after)
        cleared = self.px4.topic("control_allocator_status")
        cleared_residual = array_field(cleared, "unallocated_torque", 3)[:3]
        if (cleared["torque_setpoint_achieved"].lower() != "true"
                or max(map(abs, cleared_residual)) > 1e-5):
            raise RuntimeError("saturation feedback remained after the {} timeout".format(fault))

        def stopped():
            rpm = self.rotor_rpm()
            return rpm if max(rpm) < 100 else None

        rpm = wait_for(stopped, self.args.timeout, "physical rotor stop after disarm")
        final_position, final_rotation = self.fixed_body()
        drift = math.sqrt(sum((a - b) ** 2 for a, b in zip(position, final_position)))
        angle = 2.0 * math.acos(min(1.0, abs(sum(a * b for a, b in zip(rotation, final_rotation)))))
        if drift > 0.005 or angle > math.radians(1.0):
            raise RuntimeError("world fixture moved {} m or {} deg".format(drift, math.degrees(angle)))
        self.metrics.update(disarmed_after_input_loss=True, final_rpm=rpm, fuselage_drift_m=drift,
                            fuselage_drift_deg=math.degrees(angle), fault=status.get("fault"),
                            disarmed_px4_unallocated_torque=cleared_residual)

    def cleanup(self):
        if self.bench_enabled:
            self.px4.command("shared_tilt_control", "disable", required=False)
            if not self.connected(False):
                self.arming(False)
                wait_for(lambda: self.connected(False), 8.0, "disarm during cleanup")
        self.stop.set()
        if self.worker is not None:
            self.worker.join(1.0)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--spawn-fixed", type=Path, metavar="SDF")
    mode.add_argument("--run", choices=("shadow", "bench"))
    parser.add_argument("--px4-bin-dir", type=Path, default=Path("build/px4_sitl_default/bin"))
    parser.add_argument("--timeout", type=float, default=65.0, help="wall-time limit per condition")
    parser.add_argument("--pitch-deg", type=float, default=10.0)
    parser.add_argument("--fault-input", choices=("pitch", "tilt"), default="pitch")
    parser.add_argument("--relay-feedback-topic", default="/pitch_decoupled_quad/bench/feedback",
                        help="start the bridge with _feedback_topic set to this topic for a tilt-loss run")
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout <= 0 or not 4.0 <= args.pitch_deg <= 12.0:
        parser.error("timeout must be positive and pitch must be in [4, 12] degrees")
    if args.fault_input == "tilt" and (args.run != "bench" or not args.relay_feedback_topic.startswith("/")
                                       or args.relay_feedback_topic == "/mavros/tunnel/in"):
        parser.error("tilt-loss injection requires bench mode and a separate absolute relay topic")

    import rospy

    harness = None
    error, cleanup_error = None, None
    try:
        check_master()
        rospy.init_node("verify_shared_tilt_bench", anonymous=True, log_level=rospy.ERROR, disable_signals=True)
        if args.spawn_fixed:
            spawn_fixture(rospy, args.spawn_fixed)
            return 0
        harness = Harness(rospy, args)
        harness.check_endpoints()
        if args.run == "shadow":
            harness.shadow()
        else:
            harness.bench()
        if harness.stream_error:
            raise RuntimeError("setpoint publisher: " + harness.stream_error)
    except (OSError, KeyError, ValueError, RuntimeError, ET.ParseError, subprocess.SubprocessError,
            rospy.ROSException, rospy.ServiceException, KeyboardInterrupt) as exc:
        error = str(exc) or "interrupted"
    finally:
        if harness is not None:
            try:
                harness.cleanup()
            except (OSError, RuntimeError, rospy.ROSException, rospy.ServiceException,
                    subprocess.SubprocessError, KeyboardInterrupt) as exc:
                cleanup_error = str(exc) or "interrupted"
    passed = error is None and cleanup_error is None
    print(json.dumps({"passed": passed, "error": error, "cleanup_error": cleanup_error,
                      "metrics": harness.metrics if harness is not None else {}}, sort_keys=True))
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
