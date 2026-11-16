#!/usr/bin/env python3
"""Verify the 22012 shared-tilt free-flight SITL profile and AUTO.LAND."""

import argparse
import json
import math
from pathlib import Path
import re
import subprocess
import sys
import threading
from urllib.parse import urlsplit

from verify_shared_tilt_bench import (BODY, FIXTURE, MODEL, PITCH, ROTOR_JOINTS, Px4,
                                      array_field, check_master, joint_names, timestamp)
from verify_zero_tilt_hover import FEEDBACK, IN_AIR, MAVROS, TILT_JOINTS, HoverTest, require_local


AIRFRAME = 22012
HOVER_HEIGHT = 1.0
HEIGHT_TOLERANCE = 0.20
HORIZONTAL_TOLERANCE = 0.30
ZERO_TILT_TOLERANCE = 1.0
PITCH_RMSE_LIMIT = 2.0
POSITION_RMSE_LIMIT = 0.10
TRUTH_PERIOD = 0.08
SIM_SAMPLE_MAX_GAP = 0.25
TELEMETRY_WALL_TIMEOUT = 6.0
ALLOCATOR_ANGLE_TOLERANCE_DEG = 2.0
ALLOCATOR_SAMPLE_TOLERANCE_US = 200_000
SMALL_TRAJECTORY_SECONDS = 12.0
SMALL_TRAJECTORY_FORWARD_RADIUS_M = 0.25
SMALL_TRAJECTORY_LEFT_RADIUS_M = 0.20
SMALL_TRAJECTORY_MAX_SAMPLE_GAP_S = 0.50


def xyz(point):
    return point.x, point.y, point.z


def attitude(quaternion):
    x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
    if not all(map(math.isfinite, (x, y, z, w))) or abs(x*x + y*y + z*z + w*w - 1.0) > 0.02:
        raise RuntimeError("pose quaternion is not a unit quaternion")
    yaw = math.atan2(2.0 * (w*z + x*y), 1.0 - 2.0 * (y*y + z*z))
    # ROS/Gazebo body axes are FLU. PX4 aircraft pitch is about the opposite Y axis.
    pitch_frd = -math.asin(max(-1.0, min(1.0, 2.0 * (w*y - z*x))))
    return yaw, math.degrees(pitch_frd)


def relative_position(point, origin, heading):
    dx, dy, dz = (end - start for end, start in zip(point, origin))
    c, s = math.cos(heading), math.sin(heading)
    return c*dx + s*dy, -s*dx + c*dy, dz


def smoothstep(ratio):
    return ratio**3 * (10.0 + ratio * (-15.0 + 6.0 * ratio))


def rmse(errors):
    return math.sqrt(sum(error*error for error in errors) / len(errors))


def small_trajectory_target(elapsed):
    ratio = max(0.0, min(1.0, elapsed / SMALL_TRAJECTORY_SECONDS))
    phase = math.tau * smoothstep(ratio)
    return (SMALL_TRAJECTORY_FORWARD_RADIUS_M * math.sin(phase),
            SMALL_TRAJECTORY_LEFT_RADIUS_M * (1.0 - math.cos(phase)), HOVER_HEIGHT, 0.0)


def allocator_matrix(output):
    rows = {int(index): tuple(map(float, row.split(",")))
            for index, row in re.findall(r"\bshared_tilt_motor_([0-3])=([^\r\n]+)", output)}
    if len(rows) != 4 or any(len(row) != 6 or not all(map(math.isfinite, row)) for row in rows.values()):
        raise RuntimeError("native allocator did not report four finite six-axis motor columns")
    return tuple(rows[i] for i in range(4))


def matrix_tilt_degrees(rows, geometry):
    angles = []
    for row, (x, y, z, signed_drag) in zip(rows, geometry):
        roll, pitch, yaw, thrust_x, thrust_y, thrust_z = row
        magnitude = math.hypot(thrust_x, thrust_z)
        if thrust_z >= 0.0 or abs(thrust_y) > 1e-5:
            raise RuntimeError("native allocator motor thrust leaves the expected FRD X-Z plane")
        sine, cosine = thrust_x / magnitude, -thrust_z / magnitude
        expected = (-y*cosine - signed_drag*sine, x*cosine + z*sine, -y*sine + signed_drag*cosine)
        if any(abs(actual / magnitude - wanted) > 2e-4 for actual, wanted in zip((roll, pitch, yaw), expected)):
            raise RuntimeError("native allocator motor moments do not match the measured-tilt rotor geometry")
        angles.append(math.degrees(math.atan2(sine, cosine)))
    if max(angles) - min(angles) > 0.02:
        raise RuntimeError("native allocator motor thrust axes do not share a common tilt")
    return sum(angles) / len(angles)


class FlightTest(HoverTest):
    def __init__(self, ros, types, args):
        super().__init__(ros, types, args)
        self.px4 = Px4(args.px4_bin_dir)
        self.pitch_publisher = ros.Publisher(MAVROS + "/tunnel/in", types["Tunnel"], queue_size=3)
        self.stop_stream = threading.Event()
        self.stream_thread = None
        self.stream_error = None
        self.target = (0.0, 0.0, 0.0, 0.0)  # Home-heading forward/left/up in metres, FRD pitch in degrees.
        self.publish_position = True
        self.enabled = False
        self.truth_read_time = -math.inf
        self.truth = None
        self.rotor_geometry = None

    def verify_endpoints(self):
        if not self.ros.get_param("/use_sim_time", False):
            raise RuntimeError("/use_sim_time must be true")
        fcu = urlsplit(self.ros.get_param(MAVROS + "/fcu_url", ""))
        if fcu.scheme != "udp" or fcu.port != 14580 or fcu.query != "ids=1,191":
            raise RuntimeError("expected local MAVROS UDP SITL with companion IDs 1,191")
        require_local(fcu.hostname)
        if (int(self.ros.get_param(MAVROS + "/target_system_id", 0)),
                int(self.ros.get_param(MAVROS + "/target_component_id", 0))) != (1, 1):
            raise RuntimeError("expected MAVROS target 1.1")
        self.check_service_providers(("/gazebo/get_model_properties", "/gazebo/get_link_state",
                                      "/gazebo/get_joint_properties", MAVROS + "/set_mode", MAVROS + "/cmd/arming"))
        model = self.service("/gazebo/get_model_properties", "GetModelProperties", MODEL)
        joints = joint_names(model)
        if not model.success or model.is_static or FIXTURE in joints or not set(TILT_JOINTS) <= joints:
            raise RuntimeError("expected the free pitch_decoupled_quad model and both moving servo joints")
        if int(self.px4.parameter("SYS_AUTOSTART")) != AIRFRAME:
            raise RuntimeError("expected PX4 SYS_AUTOSTART={}".format(AIRFRAME))
        self.px4.command("mc_att_control", "status")
        self.px4.command("mc_rate_control", "status")
        allocator = self.px4.command("control_allocator", "status")
        if "Method: Sequential desaturation" not in allocator:
            raise RuntimeError("22012 requires the native sequential desaturation allocator")
        if "Configured actuators: 4" not in allocator:
            raise RuntimeError("22012 must allocate the four tilting motors")
        self.rotor_geometry = self.px4.rotor_geometry()
        if self.px4.research_status().get("mode") != "flight":
            raise RuntimeError("shared_tilt_control is not running in flight mode")
        code, _, topic_tree = self.ros.get_master().getSystemState()
        if code != 1:
            raise RuntimeError("could not check ROS setpoint ownership")
        publishers, subscribers, _ = topic_tree
        inputs = {topic for topic, nodes in subscribers if topic.startswith(MAVROS + "/setpoint") and MAVROS in nodes}
        if any(node not in (self.ros.get_name(), MAVROS) for topic, nodes in publishers if topic in inputs for node in nodes):
            raise RuntimeError("another ROS node is publishing to a MAVROS setpoint input")
        self.metrics.update(airframe=AIRFRAME, native_attitude_running=True, native_rate_pid_running=True,
                            stock_allocator_running=True, allocation_method="sequential_desaturation", fixture=False)

    def stream(self):
        try:
            while not self.stop_stream.is_set() and not self.ros.is_shutdown():
                x, y, z, theta = self.target
                tunnel = self.types["Tunnel"]()
                tunnel.payload_type, tunnel.target_system, tunnel.target_component = 32779, 1, 1
                tunnel.payload_length = PITCH.size
                tunnel.payload = list(PITCH.pack(math.radians(theta)).ljust(128, b"\0"))
                self.pitch_publisher.publish(tunnel)
                if self.publish_position:
                    msg = self.types["PoseStamped"]()
                    msg.header.stamp = self.ros.Time.now()
                    msg.header.frame_id = "map"
                    c, s = math.cos(self.yaw), math.sin(self.yaw)
                    msg.pose.position.x = self.origin[0] + c*x - s*y
                    msg.pose.position.y = self.origin[1] + s*x + c*y
                    msg.pose.position.z = self.origin[2] + z
                    msg.pose.orientation.z = math.sin(self.yaw * 0.5)
                    msg.pose.orientation.w = math.cos(self.yaw * 0.5)
                    self.publisher.publish(msg)
                self.stop_stream.wait(0.05)
        except (self.ros.ROSException, ValueError, OSError) as exc:
            self.stream_error = str(exc)

    def stream_ok(self):
        if self.stream_error:
            raise RuntimeError("setpoint publisher: " + self.stream_error)

    def sim_time(self):
        return self.ros.Time.now().to_sec()

    def world_pose(self):
        result = self.service("/gazebo/get_link_state", "GetLinkState", BODY, "world")
        if not result.success:
            raise RuntimeError("Gazebo base_link pose: " + result.status_message)
        pose = result.link_state.pose
        point = xyz(pose.position)
        if not all(map(math.isfinite, point)):
            raise RuntimeError("Gazebo base_link position is non-finite")
        yaw, pitch = attitude(pose.orientation)
        return point, yaw, pitch

    def world_sample(self, now):
        if self.truth is None or now - self.truth_read_time >= TRUTH_PERIOD:
            point, _, pitch = self.world_pose()
            self.truth = relative_position(point, self.world_origin, self.world_yaw), pitch
            self.truth_read_time = now
        return self.truth

    def tilt_feedback(self):
        joints = self.snapshot("joints", TELEMETRY_WALL_TIMEOUT)
        tunnel = self.snapshot("feedback", TELEMETRY_WALL_TIMEOUT)
        if joints is None or tunnel is None or tunnel.target_system != 1 or tunnel.target_component != 1:
            return None
        if tunnel.payload_length != FEEDBACK.size or len(tunnel.payload) != 128:
            return None
        stamp = joints.header.stamp.to_nsec()
        if stamp <= 0 or not -20_000_000 <= self.ros.Time.now().to_nsec() - stamp <= 300_000_000:
            return None
        angles = dict(zip(joints.name, joints.position))
        if len(joints.name) != len(joints.position) or len(angles) != len(joints.name):
            return None
        sequence, age, alpha, flags = FEEDBACK.unpack(bytes(tunnel.payload[:FEEDBACK.size]))
        degrees = tuple(math.degrees(angles.get(name, math.nan)) for name in TILT_JOINTS) + (math.degrees(alpha),)
        if flags == 1 and age <= 300_000 and all(map(math.isfinite, degrees)):
            return sequence, degrees
        return None

    def active_sample(self):
        self.stream_ok()
        state = self.snapshot("state", 10.0)
        if state is None:
            return None
        if not state.connected or not state.armed or state.mode != "OFFBOARD":
            raise RuntimeError("left armed OFFBOARD (connected={}, armed={}, mode={})".format(
                state.connected, state.armed, state.mode))
        now = self.sim_time()
        stamped = self.snapshot("pose", TELEMETRY_WALL_TIMEOUT)
        if stamped is None or abs(now - stamped.header.stamp.to_sec()) > SIM_SAMPLE_MAX_GAP:
            return None
        point, quaternion = stamped.pose.position, stamped.pose.orientation
        if not all(map(math.isfinite, xyz(point))):
            raise RuntimeError("MAVROS local pose is non-finite")
        local = relative_position(xyz(point), self.origin, self.yaw)
        pitch = attitude(quaternion)[1]
        truth, true_pitch = self.world_sample(now)
        tilt = self.tilt_feedback()
        self.metrics["last_active_sample"] = {"sim_time": now, "target_forward_left_up_m": self.target[:3],
                                             "target_pitch_deg": self.target[3], "estimated_position_m": local,
                                             "gazebo_position_m": truth, "estimated_pitch_deg": pitch,
                                             "gazebo_pitch_deg": true_pitch, "tilt_samples_deg": tilt[1] if tilt else None}
        if max(local[2], truth[2]) > 1.65 or math.hypot(*truth[:2]) > 1.25:
            raise RuntimeError("vehicle exceeded the local sortie envelope")
        return now, local, pitch, truth, true_pitch, tilt

    def preflight(self):
        state, landed = self.ground_state()
        if state is not None and state.armed:
            raise RuntimeError("expected the vehicle to start disarmed")
        tilt = self.tilt_feedback()
        return (state is not None and state.connected and landed and tilt is not None
                and max(map(abs, tilt[1])) <= ZERO_TILT_TOLERANCE
                and self.snapshot("pose", 2.0) is not None and self.publisher.get_num_connections() > 0
                and self.pitch_publisher.get_num_connections() > 0)

    def arm(self):
        self.verify_endpoints()
        self.wait_for(self.preflight, self.args.timeout, "connected disarmed 22012, on ground, with fresh zero tilt")
        point, quaternion = self.pose()
        self.origin = xyz(point)
        self.yaw, _ = attitude(quaternion)
        self.world_origin, self.world_yaw, _ = self.world_pose()
        self.stream_thread = threading.Thread(target=self.stream, daemon=True)
        self.stream_thread.start()
        start = self.sim_time()
        self.wait_for(lambda: self.sim_time() - start >= 2.0, self.args.timeout, "2 simulated seconds of OFFBOARD prestream")
        self.stream_ok()
        self.set_mode("OFFBOARD")
        self.wait_for(lambda: self.in_mode("OFFBOARD", armed=False), self.args.timeout, "disarmed OFFBOARD")
        self.px4.command("shared_tilt_control", "enable")
        self.enabled = True

        def gate(expected):
            status = self.px4.research_status()
            if status.get("state") == "fault":
                raise RuntimeError("flight enable/control gate fault: " + status.get("fault", "unknown"))
            return status.get("state") == expected

        self.wait_for(lambda: gate("waiting_arm"), self.args.timeout, "controller waiting_arm")
        self.arm_attempted = True
        armed = self.service(MAVROS + "/cmd/arming", "CommandBool", True)
        if not armed.success:
            raise RuntimeError("PX4 rejected normal arming (result {})".format(armed.result))
        self.wait_for(lambda: self.in_mode("OFFBOARD", armed=True), self.args.timeout, "armed OFFBOARD")
        self.wait_for(lambda: gate("active"), self.args.timeout, "controller active")

    def move(self, target, seconds, label, check_zero_pitch=False):
        print(label, file=sys.stderr, flush=True)
        initial = self.target
        start = self.sim_time()

        def update():
            ratio = max(0.0, min(1.0, (self.sim_time() - start) / seconds))
            weight = smoothstep(ratio)
            self.target = tuple(a + (b - a)*weight for a, b in zip(initial, target))
            sample = self.active_sample()
            if sample is None:
                return False
            _, _, pitch, _, true_pitch, _ = sample
            if check_zero_pitch and not self.args.process_only and max(abs(pitch), abs(true_pitch)) > 3.0:
                raise RuntimeError("zero-pitch translation exceeded 3° body pitch")
            return ratio >= 1.0

        self.wait_for(update, self.args.timeout, label)

    def small_trajectory(self):
        label = "small zero-pitch ellipse"
        print(label, file=sys.stderr, flush=True)
        start = self.sim_time()
        samples = []
        feedback_samples = {}

        def update():
            self.target = small_trajectory_target(self.sim_time() - start)
            sample = self.active_sample()
            if sample is None:
                return False
            now, local, _, truth, true_pitch, tilt = sample
            if truth[2] < 0.45:
                raise RuntimeError("Gazebo showed loss of flight height on the small trajectory")
            if not samples or self.truth_read_time > samples[-1][0]:
                samples.append((self.truth_read_time, local, truth, true_pitch))
            if tilt is not None and abs(sum(tilt[1][:2]) * 0.5 - tilt[1][2]) <= 0.8:
                feedback_samples.setdefault(tilt[0], (now, max(map(abs, tilt[1]))))
            return now - start >= SMALL_TRAJECTORY_SECONDS

        self.wait_for(update, self.args.timeout, label)
        end = start + SMALL_TRAJECTORY_SECONDS
        truth_times = [time for time, *_ in samples]
        feedback_times = [time for time, _ in feedback_samples.values()]
        position_gap = max(b - a for a, b in zip([start] + truth_times, truth_times + [end]))
        feedback_gap = max(b - a for a, b in zip([start] + feedback_times, feedback_times + [end]))
        forwards = [truth[0] for _, _, truth, _ in samples]
        lefts = [truth[1] for _, _, truth, _ in samples]
        forward_span = max(forwards) - min(forwards)
        left_span = max(lefts) - min(lefts)
        references = [small_trajectory_target(time - start) for time in truth_times]
        gazebo_errors = [math.hypot(truth[0] - ref[0], truth[1] - ref[1])
                         for (_, _, truth, _), ref in zip(samples, references)]
        local_errors = [math.hypot(local[0] - ref[0], local[1] - ref[1])
                        for (_, local, _, _), ref in zip(samples, references)]
        summary = {"duration_sim_s": SMALL_TRAJECTORY_SECONDS, "pitch_reference_deg": 0.0,
                   "position_samples": len(samples), "32780_samples": len(feedback_samples),
                   "position_max_gap_sim_s": position_gap, "32780_max_gap_sim_s": feedback_gap,
                   "target_forward_span_m": 2 * SMALL_TRAJECTORY_FORWARD_RADIUS_M,
                   "target_left_span_m": 2 * SMALL_TRAJECTORY_LEFT_RADIUS_M,
                   "gazebo_forward_span_m": forward_span, "gazebo_left_span_m": left_span,
                   "gazebo_xy_rmse_m": rmse(gazebo_errors), "estimated_xy_rmse_m": rmse(local_errors),
                   "gazebo_z_rmse_m": rmse([truth[2] - HOVER_HEIGHT for _, _, truth, _ in samples]),
                   "gazebo_body_pitch_rmse_deg": rmse([pitch for _, _, _, pitch in samples]),
                   "gazebo_body_pitch_max_deg": max(abs(pitch) for _, _, _, pitch in samples),
                   "32780_and_joint_max_abs_deg": max((angle for _, angle in feedback_samples.values()),
                                                    default=None)}
        self.metrics["small_trajectory"] = summary
        if max(position_gap, feedback_gap) > SMALL_TRAJECTORY_MAX_SAMPLE_GAP_S:
            raise RuntimeError("small trajectory did not retain live position and 32780 feedback")
        if forward_span < SMALL_TRAJECTORY_FORWARD_RADIUS_M or left_span < SMALL_TRAJECTORY_LEFT_RADIUS_M:
            raise RuntimeError("Gazebo did not show motion on both axes of the small trajectory")
        print("Accepted " + label, file=sys.stderr, flush=True)

    def hold(self, target, label, zero_tilt=False, maneuver=False):
        x, y, height, theta = target
        self.target = target
        stable = []
        sequences = set()

        def update():
            sample = self.active_sample()
            if sample is None or (stable and sample[0] - stable[-1][0] > SIM_SAMPLE_MAX_GAP):
                stable.clear()
                sequences.clear()
            if sample is None:
                return None
            now, local, pitch, truth, true_pitch, tilt = sample
            landed = self.snapshot("landed", self.args.timeout)
            vertical = max(abs(local[2] - height), abs(truth[2] - height))
            horizontal = max(math.hypot(local[0] - x, local[1] - y), math.hypot(truth[0] - x, truth[1] - y))
            pitch_error = max(abs(pitch - theta), abs(true_pitch - theta))
            tilt_live = tilt is not None and abs(sum(tilt[1][:2])*0.5 - tilt[1][2]) <= 0.8
            window = landed is not None and landed.landed_state == IN_AIR and tilt_live
            if window and not self.args.process_only:
                window = (vertical <= HEIGHT_TOLERANCE and horizontal <= HORIZONTAL_TOLERANCE
                          and pitch_error <= PITCH_RMSE_LIMIT)
                if window and zero_tilt:
                    window = max(map(abs, tilt[1])) <= ZERO_TILT_TOLERANCE
                if window and abs(theta) > 0.0:
                    window = max(abs(alpha + theta) for alpha in tilt[1]) <= 3.0
            if not window:
                stable.clear()
                sequences.clear()
                return None
            if not stable or now != stable[-1][0]:
                stable.append((now, vertical, horizontal, pitch_error, tilt[1]))
                sequences.add(tilt[0])
            if now - stable[0][0] < 3.0 or len(sequences) < 15:
                return None
            tail = [sample for sample in stable if now - sample[0] <= 2.0]
            position_rmse = rmse([math.hypot(sample[1], sample[2]) for sample in tail])
            pitch_rmse = rmse([sample[3] for sample in tail])
            if (maneuver and not self.args.process_only
                    and (position_rmse > POSITION_RMSE_LIMIT or pitch_rmse > PITCH_RMSE_LIMIT)):
                return None
            return {"stage": label, "target_forward_left_up_m": list(target[:3]), "target_pitch_deg": theta,
                    "hold_sim_seconds": now - stable[0][0], "32780_samples": len(sequences),
                    "position_rmse_m": position_rmse, "pitch_rmse_deg": pitch_rmse,
                    "max_vertical_error_m": max(sample[1] for sample in stable),
                    "max_horizontal_error_m": max(sample[2] for sample in stable),
                    "max_absolute_joint_or_feedback_deg": max(abs(angle) for sample in stable for angle in sample[4]),
                    "measured_position_m": list(local), "gazebo_position_m": list(truth),
                    "measured_pitch_deg": pitch, "gazebo_pitch_deg": true_pitch,
                    "joint_angles_deg": list(tilt[1][:2]), "32780_alpha_deg": tilt[1][2]}

        summary = self.wait_for(update, self.args.timeout, label + " (3 simulated seconds of valid flight samples)")
        pitch_rx = float(self.px4.topic("tilting_mc_desired_angles")["pitch_body"])
        tilt_rx = self.px4.topic("tilting_servo_status")
        alpha_rx = math.degrees(float(tilt_rx["angle_rad"]))
        if (not all(map(math.isfinite, (pitch_rx, alpha_rx))) or abs(math.degrees(pitch_rx) - theta) > 0.004
                or tilt_rx.get("valid", "").lower() != "true" or tilt_rx.get("fault", "").lower() != "false"):
            raise RuntimeError("PX4 32779/32780 reception did not match " + label)
        if abs(alpha_rx - summary["32780_alpha_deg"]) > 2.0:
            raise RuntimeError("PX4 and ROS 32780 angles disagree in " + label)
        summary["px4_32779_pitch_deg"] = math.degrees(pitch_rx)
        summary["px4_32780_alpha_deg"] = alpha_rx
        summary["native_allocation"] = self.wait_for(lambda: self.native_allocation(theta), self.args.timeout,
                                                     label + " native measured-tilt allocation")
        print("Accepted " + label, file=sys.stderr, flush=True)
        return summary

    def native_allocation(self, pitch_deg):
        request = self.px4.topic("shared_tilt_allocation_setpoint")
        if not request or int(request["mode"]) != 2:
            return None
        request_tilt = math.degrees(float(request["tilt_rad"]))
        collective = float(request["collective"])
        if not (math.isfinite(request_tilt) and math.isfinite(collective) and collective > 0.0):
            raise RuntimeError("invalid active tilt allocation request")
        output = self.px4.command("control_allocator", "status")
        rows = allocator_matrix(output)
        matrix_angle = matrix_tilt_degrees(rows, self.rotor_geometry)
        native_angle = re.search(r"\bshared_tilt_angle_rad=([^\s]+)", output)
        native_sample = re.search(r"\bshared_tilt_sample_us=(\d+)", output)
        if native_angle is None or native_sample is None:
            raise RuntimeError("native allocator did not report the applied 32780 sample")
        applied_tilt = math.degrees(float(native_angle[1]))
        applied_sample_us = int(native_sample[1])
        request_sample_us = int(request["timestamp_sample"].split()[0])
        if (not math.isfinite(applied_tilt) or abs(matrix_angle - applied_tilt) > 0.02 or applied_sample_us <= 0
                or abs(applied_sample_us - request_sample_us) > ALLOCATOR_SAMPLE_TOLERANCE_US):
            return None
        measured = self.px4.topic("tilting_servo_status")
        if measured.get("valid", "").lower() != "true" or measured.get("fault", "").lower() != "false":
            return None
        measured_tilt = -math.degrees(float(measured["angle_rad"]))
        if not math.isfinite(measured_tilt) or any(abs(matrix_angle - angle) > ALLOCATOR_ANGLE_TOLERANCE_DEG
                                                   for angle in (request_tilt, measured_tilt)):
            return None
        if pitch_deg and matrix_angle * math.copysign(1.0, pitch_deg) < 1.0:
            return None
        motors = self.px4.topic("actuator_motors")
        status = self.px4.topic("control_allocator_status")
        if not motors or not status or timestamp(status) + ALLOCATOR_SAMPLE_TOLERANCE_US < timestamp(request):
            return None
        controls = array_field(motors, "control", 4)[:4]
        if any(not 0.0 <= value <= 1.0 for value in controls):
            raise RuntimeError("native allocator motor command left its normalized bounds")
        return {"matrix_tilt_deg": matrix_angle, "request_tilt_deg": request_tilt,
                "applied_sample_us": applied_sample_us, "request_sample_us": request_sample_us,
                "measured_tilt_deg": measured_tilt, "collective_normalized": collective,
                "motors": controls, "unallocated_torque": array_field(status, "unallocated_torque", 3),
                "unallocated_thrust": array_field(status, "unallocated_thrust", 3),
                "actuator_saturation": array_field(status, "actuator_saturation", 4)[:4]}

    def land(self, acceptance):
        self.target = (*self.target[:3], 0.0)
        state, on_ground = self.ground_state()
        pending = self.pending_arm_request is not None and not self.pending_arm_request.is_set()
        if not acceptance and not pending and state is not None and state.connected and not state.armed and on_ground:
            self.arm_attempted = False
            return
        point, _, _ = self.world_pose()
        starting_height = point[2] - self.world_origin[2]
        print("Requesting AUTO.LAND", file=sys.stderr, flush=True)
        self.set_mode("AUTO.LAND")

        def landing_mode():
            current, grounded = self.ground_state()
            if current is None or not current.connected:
                return False
            return (current.mode == "AUTO.LAND" and (not acceptance or current.armed)
                    or (not acceptance and not pending and not current.armed and grounded))

        self.wait_for(landing_mode, self.args.timeout, "PX4 to enter AUTO.LAND")
        self.publish_position = False

        def touchdown():
            current, grounded = self.ground_state()
            return current if current is not None and current.connected and grounded else False

        self.wait_for(touchdown, self.args.land_timeout, "AUTO.LAND touchdown")
        if pending and not self.pending_arm_request.wait(6.0):
            raise RuntimeError("pending arming command did not complete during cleanup")
        if not acceptance and not self.wait_for(touchdown, self.args.timeout, "ground state for cleanup").armed:
            self.arm_attempted = False
            return
        if not acceptance:
            self.service(MAVROS + "/cmd/arming", "CommandBool", False)

        def disarmed():
            current = touchdown()
            return current and not current.armed

        self.wait_for(disarmed, self.args.land_timeout, "disarm on ground")
        self.arm_attempted = False
        if acceptance:
            point, _, _ = self.world_pose()
            height = point[2] - self.world_origin[2]
            if abs(height) > 0.15 or starting_height - height < 0.45:
                raise RuntimeError("Gazebo did not show the expected AUTO.LAND descent and touchdown")

            def stopped():
                speeds = []
                for joint_name in ROTOR_JOINTS:
                    ret = self.service("/gazebo/get_joint_properties", "GetJointProperties", joint_name)
                    if not ret.success or not ret.rate or not math.isfinite(ret.rate[0]):
                        raise RuntimeError("invalid Gazebo rotor feedback for " + joint_name)
                    speeds.append(abs(ret.rate[0])*600.0/(2.0*math.pi))
                return speeds if max(speeds) < 100.0 else None

            speeds = self.wait_for(stopped, self.args.land_timeout, "four Gazebo rotors to stop")
            self.metrics["landing"] = {"auto_land_observed_armed": True, "landed_disarmed": True,
                                       "gazebo_landing_height_m": height, "gazebo_descent_m": starting_height - height,
                                       "rotor_rpm_when_disarmed": speeds}

    def run(self):
        self.arm()
        home_hover = (0.0, 0.0, HOVER_HEIGHT, 0.0)
        self.move(home_hover, 5.0, "climb to 1 m")
        self.metrics["stages"] = [self.hold(home_hover, "1 m zero-tilt hover", zero_tilt=True)]
        if self.args.run == "small-trajectory":
            self.small_trajectory()
            self.metrics["stages"].append(self.hold(home_hover, "settle at start of small trajectory"))
        if self.args.run == "maneuver":
            for target, label in (((0.5, 0.0, 1.0, 0.0), "forward +0.5 m, zero pitch"),
                                  ((-0.5, 0.0, 1.0, 0.0), "forward -0.5 m, zero pitch"),
                                  ((0.0, 0.5, 1.0, 0.0), "left +0.5 m, zero pitch"),
                                  (home_hover, "return to hover point")):
                self.move(target, 6.0, label, check_zero_pitch=True)
                self.metrics["stages"].append(self.hold(target, label, maneuver=True))
            for pitch in (5.0, -5.0):
                target = (*home_hover[:3], pitch)
                label = "fixed position pitch {:+.0f}°".format(pitch)
                self.move(target, 4.0, label)
                self.metrics["stages"].append(self.hold(target, label, maneuver=True))
            self.move(home_hover, 3.0, "recenter pitch before landing")
        self.land(acceptance=True)
        self.stream_ok()

    def cleanup(self):
        try:
            if self.arm_attempted:
                self.land(acceptance=False)
        finally:
            self.stop_stream.set()
            if self.stream_thread is not None:
                self.stream_thread.join(1.0)
            if self.enabled:
                state, on_ground = self.ground_state()
                if state is not None and state.connected and not state.armed and on_ground:
                    self.px4.command("shared_tilt_control", "disable", required=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", choices=("hover", "maneuver", "small-trajectory"), required=True)
    parser.add_argument("--process-only", action="store_true",
                        help="exercise the flight and feedback sequence, recording tracking errors without gating on them")
    parser.add_argument("--timeout", type=float, default=150.0, help="wall-time limit per flight phase")
    parser.add_argument("--land-timeout", type=float, default=150.0, help="wall-time limit per landing check")
    parser.add_argument("--px4-bin-dir", type=Path, default=Path("build/px4_sitl_default/bin"))
    args = parser.parse_args()
    if not all(math.isfinite(value) and value > 0 for value in (args.timeout, args.land_timeout)):
        parser.error("timeouts must be finite and positive")

    import rospy
    from gazebo_msgs.srv import GetJointProperties, GetLinkState, GetModelProperties
    from geometry_msgs.msg import PoseStamped
    from mavros_msgs.msg import ExtendedState, State, Tunnel
    from mavros_msgs.srv import CommandBool, SetMode
    from sensor_msgs.msg import JointState

    harness = None
    error, cleanup_error = None, None
    failures = (OSError, RuntimeError, ValueError, KeyError, rospy.ROSException, rospy.ServiceException,
                subprocess.SubprocessError, KeyboardInterrupt)
    try:
        check_master()
        rospy.init_node("verify_shared_tilt_flight", anonymous=True, log_level=rospy.ERROR, disable_signals=True)
        types = {cls.__name__: cls for cls in (GetJointProperties, GetLinkState, GetModelProperties,
                                              PoseStamped, ExtendedState, State, Tunnel, CommandBool, SetMode, JointState)}
        harness = FlightTest(rospy, types, args)
        harness.run()
    except failures as exc:
        error = str(exc) or "interrupted"
    finally:
        if harness is not None:
            try:
                harness.cleanup()
            except failures as exc:
                cleanup_error = str(exc) or "interrupted"
    passed = error is None and cleanup_error is None
    print(json.dumps({"passed": passed, "test": args.run, "process_only": args.process_only,
                      "error": error, "cleanup_error": cleanup_error,
                      "metrics": harness.metrics if harness is not None else {}}, sort_keys=True, allow_nan=False))
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
