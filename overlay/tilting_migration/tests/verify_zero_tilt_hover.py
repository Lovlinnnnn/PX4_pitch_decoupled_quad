#!/usr/bin/env python3
"""Run a 1 m hover and landing regression on local, zero-tilt PX4 SITL."""

import argparse
import ipaddress
import json
import math
import os
import socket
import struct
import sys
import threading
import time
from urllib.parse import urlsplit


MAVROS = "/mavros"
SETPOINT_TOPIC = MAVROS + "/setpoint_position/local"
FEEDBACK = struct.Struct("<IIfI")
TILT_JOINTS = ("front_tilt_joint", "behind_tilt_joint")
ON_GROUND = 1
IN_AIR = 2
SERVICE_TIMEOUT = 6.0
TILT_WALL_TIMEOUT = 1.5  # Gazebo lockstep can advance slower than wall time.


def require_local(host, interfaces=()):
    try:
        addresses = {entry[4][0] for entry in socket.getaddrinfo(host, 0, type=socket.SOCK_STREAM)} if host else set()
    except OSError:
        addresses = set()
    if not addresses or any(not ipaddress.ip_address(addr).is_loopback and addr not in interfaces
                            for addr in addresses):
        raise RuntimeError("expected a local endpoint, got {!r}".format(host))


class HoverTest:
    def __init__(self, ros, types, args):
        self.ros = ros
        self.args = args
        self.types = types
        self.lock = threading.Lock()
        self.latest = {}
        self.metrics = {}
        self.pending_arm_request = None
        self.arm_attempted = False
        self.ready_to_use_vehicle = False
        self.publisher = ros.Publisher(SETPOINT_TOPIC, types["PoseStamped"], queue_size=1)
        subscriptions = {
            "state": (MAVROS + "/state", "State"),
            "landed": (MAVROS + "/extended_state", "ExtendedState"),
            "pose": (MAVROS + "/local_position/pose", "PoseStamped"),
            "joints": ("/pitch_decoupled_quad/joint_states", "JointState"),
            "feedback": (MAVROS + "/tunnel/in", "Tunnel"),
        }
        self.subscribers = [
            ros.Subscriber(topic, types[message], self.record, callback_args=key, queue_size=5)
            for key, (topic, message) in subscriptions.items()
        ]

    def record(self, message, key):
        if key == "feedback" and message.payload_type != 32780:
            return
        with self.lock:
            self.latest[key] = time.monotonic(), message

    def snapshot(self, key, max_age, received_after=0.0):
        with self.lock:
            entry = self.latest.get(key)
        if entry is None or entry[0] < received_after or time.monotonic() - entry[0] > max_age:
            return None
        return entry[1]

    def service(self, name, message_type, *args, **kwargs):
        result = {}
        done = threading.Event()

        def call():
            try:
                self.ros.wait_for_service(name, timeout=SERVICE_TIMEOUT)
                proxy = self.ros.ServiceProxy(name, self.types[message_type])
                result["value"] = proxy(*args, **kwargs)
            except (self.ros.ServiceException, self.ros.ROSException, OSError, ValueError) as exc:
                result["error"] = exc
            finally:
                done.set()

        if message_type == "CommandBool" and args == (True,):
            self.pending_arm_request = done
        threading.Thread(target=call, daemon=True).start()
        deadline = time.monotonic() + SERVICE_TIMEOUT
        while not done.is_set() and time.monotonic() < deadline:
            if self.ready_to_use_vehicle:
                state = self.snapshot("state", self.args.timeout)
                if state is not None and state.mode != "AUTO.LAND":
                    self.send(self.command_z)
            done.wait(0.05)
        if not done.is_set():
            raise RuntimeError("{} timed out".format(name))
        if "error" in result:
            raise RuntimeError("{}: {}".format(name, result["error"]))
        if "value" not in result:
            raise RuntimeError("{} ended without a reply".format(name))
        return result["value"]

    def check_service_providers(self, services):
        from rosgraph.network import get_local_addresses

        for service in services:
            self.ros.wait_for_service(service, timeout=SERVICE_TIMEOUT)
            code, _, uri = self.ros.get_master().lookupService(service)
            if code != 1:
                raise RuntimeError("could not locate service {}".format(service))
            require_local(urlsplit(uri).hostname, get_local_addresses())

    def verify_endpoints(self):
        if not self.ros.get_param("/use_sim_time", False):
            raise RuntimeError("/use_sim_time must be true")
        link = self.ros.get_param(MAVROS + "/fcu_url", "")
        fcu = urlsplit(link)
        if fcu.scheme != "udp" or fcu.port != 14580:
            raise RuntimeError("expected local MAVROS UDP connection to PX4 SITL port 14580")
        require_local(fcu.hostname)
        if int(self.ros.get_param(MAVROS + "/target_system_id", 0)) != 1 or int(
                self.ros.get_param(MAVROS + "/target_component_id", 0)) != 1:
            raise RuntimeError("expected target system/component 1.1")
        self.check_service_providers((
            "/gazebo/get_model_properties", MAVROS + "/param/get", MAVROS + "/set_mode",
            MAVROS + "/cmd/arming",
        ))
        model = self.service("/gazebo/get_model_properties", "GetModelProperties", "pitch_decoupled_quad")
        if not model.success or not set(TILT_JOINTS) <= set(model.joint_names):
            raise RuntimeError("expected Gazebo pitch_decoupled_quad model with both tilt joints")

        def known_airframe():
            autostart = self.service(MAVROS + "/param/get", "ParamGet", "SYS_AUTOSTART")
            if not autostart.success:
                time.sleep(0.45)
                return False
            if autostart.value.integer != 22010:
                raise RuntimeError("expected PX4 SITL airframe SYS_AUTOSTART=22010")
            return True

        self.wait_for(known_airframe, self.args.timeout, "SYS_AUTOSTART in the MAVROS parameter cache")
        code, reason, state = self.ros.get_master().getSystemState()
        if code != 1:
            raise RuntimeError("ROS master did not return its publishers: {}".format(reason))
        publishers, subscribers, _ = state
        fcu_setpoints = {
            topic for topic, nodes in subscribers if topic.startswith(MAVROS + "/setpoint") and MAVROS in nodes
        }
        active_setpoints = [node for topic, nodes in publishers if topic in fcu_setpoints
                            for node in nodes if node not in (self.ros.get_name(), MAVROS)]
        if active_setpoints:
            raise RuntimeError("stop the other setpoint publishers: {}".format(", ".join(sorted(set(active_setpoints)))))

    def zero_tilt(self):
        joints = self.snapshot("joints", TILT_WALL_TIMEOUT)
        tunnel = self.snapshot("feedback", TILT_WALL_TIMEOUT)
        if joints is None or tunnel is None or joints.header.stamp.to_nsec() <= 0:
            return None
        now = self.ros.Time.now().to_nsec()
        if not -20_000_000 <= now - joints.header.stamp.to_nsec() <= 300_000_000:
            return None
        if len(joints.name) != len(joints.position) or len(joints.name) != len(set(joints.name)):
            return None
        angles = dict(zip(joints.name, joints.position))
        if (any(name not in angles for name in TILT_JOINTS) or tunnel.target_system != 1
                or tunnel.target_component != 1 or tunnel.payload_length != FEEDBACK.size
                or len(tunnel.payload) != 128):
            return None
        sequence, age, angle, flags = FEEDBACK.unpack(bytes(tunnel.payload[:FEEDBACK.size]))
        degrees = tuple(math.degrees(angles[name]) for name in TILT_JOINTS) + (math.degrees(angle),)
        if flags != 1 or age > 300_000 or not all(math.isfinite(value) for value in degrees):
            return None
        maximum = max(map(abs, degrees))
        if maximum > self.args.tilt_tolerance_deg:
            return None
        return sequence, maximum

    def pose(self):
        stamped = self.snapshot("pose", 2.0)
        if stamped is None:
            raise RuntimeError("MAVROS local pose is unavailable")
        point, rotation = stamped.pose.position, stamped.pose.orientation
        if not all(math.isfinite(value) for value in (point.x, point.y, point.z, rotation.x, rotation.y,
                                                      rotation.z, rotation.w)):
            raise RuntimeError("MAVROS local pose is non-finite")
        return point, rotation

    def send(self, z):
        msg = self.types["PoseStamped"]()
        msg.header.stamp = self.ros.Time.now()
        msg.header.frame_id = "map"
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = self.home_x, self.home_y, z
        msg.pose.orientation.z = math.sin(self.yaw * 0.5)
        msg.pose.orientation.w = math.cos(self.yaw * 0.5)
        self.publisher.publish(msg)
        self.command_z = z

    def check_active(self):
        # MAVROS receives state in PX4's 1 Hz heartbeat, which is paced by simulation time.
        state = self.snapshot("state", self.args.timeout)
        if state is None or not state.connected or not state.armed or state.mode != "OFFBOARD":
            details = "state telemetry unavailable" if state is None else "connected={}, armed={}, mode={}".format(
                state.connected, state.armed, state.mode)
            raise RuntimeError("MAVROS left armed OFFBOARD during the regression ({})".format(details))
        tilt = self.zero_tilt()
        if tilt is None:
            raise RuntimeError("zero-tilt joints or their valid 32780 feedback were lost")
        self.metrics["max_abs_tilt_deg"] = max(self.metrics["max_abs_tilt_deg"], tilt[1])
        return self.pose()[0], tilt[0]

    def wait_for(self, predicate, timeout, label, tick=None):
        deadline = time.monotonic() + timeout
        while not self.ros.is_shutdown() and time.monotonic() < deadline:
            if tick is not None:
                tick()
            result = predicate()
            if result:
                return result
            time.sleep(0.05)
        raise RuntimeError("timed out waiting for {}".format(label))

    def set_mode(self, mode):
        response = self.service(MAVROS + "/set_mode", "SetMode", base_mode=0, custom_mode=mode)
        if not response.mode_sent:
            raise RuntimeError("MAVROS refused mode request {}".format(mode))

    def run(self):
        self.verify_endpoints()

        def preflight():
            state = self.snapshot("state", self.args.timeout)
            landed = self.snapshot("landed", self.args.timeout)
            if state is not None and state.armed:
                raise RuntimeError("expected the vehicle to start disarmed")
            return (state is not None and state.connected and landed is not None
                    and landed.landed_state == ON_GROUND and self.zero_tilt() is not None
                    and self.snapshot("pose", 1.0) is not None
                    and self.publisher.get_num_connections() >= 1)

        self.wait_for(preflight, self.args.timeout, "connected disarmed SITL, zero tilt and valid feedback")
        point, quat = self.pose()
        norm = math.sqrt(quat.w * quat.w + quat.x * quat.x + quat.y * quat.y + quat.z * quat.z)
        if abs(norm - 1) > 0.05:
            raise RuntimeError("local pose orientation is not a unit quaternion")
        self.home_x, self.home_y, self.home_z = point.x, point.y, point.z
        self.command_z = self.home_z
        self.yaw = math.atan2(2 * (quat.w * quat.z + quat.x * quat.y),
                              1 - 2 * (quat.y * quat.y + quat.z * quat.z))
        self.metrics["max_abs_tilt_deg"] = 0.0
        self.ready_to_use_vehicle = True
        start = self.ros.Time.now().to_sec()
        self.wait_for(lambda: self.ros.Time.now().to_sec() - start >= 2.0, self.args.timeout,
                      "2 simulated seconds of OFFBOARD prestream", lambda: self.send(self.home_z))
        self.set_mode("OFFBOARD")
        self.wait_for(lambda: self.in_mode("OFFBOARD"), self.args.timeout, "OFFBOARD",
                      lambda: self.send(self.home_z))
        if self.zero_tilt() is None:
            raise RuntimeError("zero tilt feedback lost before arming")
        self.arm_attempted = True
        armed = self.service(MAVROS + "/cmd/arming", "CommandBool", True)
        if not armed.success:
            raise RuntimeError("PX4 rejected normal arming (result {})".format(armed.result))
        self.wait_for(lambda: self.in_mode("OFFBOARD", armed=True), self.args.timeout, "armed OFFBOARD",
                      lambda: self.send(self.home_z))
        self.climb()
        self.hover()
        self.land()

    def in_mode(self, mode, armed=None):
        state = self.snapshot("state", self.args.timeout)
        return (state is not None and state.connected and state.mode == mode
                and (armed is None or state.armed == armed))

    def climb(self):
        print("Climbing to 1 m in OFFBOARD", file=sys.stderr, flush=True)
        start = self.ros.Time.now().to_sec()

        def update():
            ratio = min(1.0, max(0.0, (self.ros.Time.now().to_sec() - start) / 4.0))
            height = ratio * ratio * (3.0 - 2.0 * ratio)
            self.send(self.home_z + height)
            point, _ = self.check_active()
            if point.z - self.home_z > 1.6:
                raise RuntimeError("excessive altitude during climb")
            return ratio >= 1.0

        self.wait_for(update, self.args.timeout, "1 m climb command")

    def hover(self):
        print("Checking altitude and zero tilt", file=sys.stderr, flush=True)
        target = self.home_z + 1.0
        stable_start = None
        peak_vertical_error = 0.0
        peak_horizontal_error = 0.0
        sequences = set()

        def update():
            nonlocal stable_start, peak_vertical_error, peak_horizontal_error
            self.send(target)
            point, sequence = self.check_active()
            landed = self.snapshot("landed", self.args.timeout)
            vertical = abs(point.z - target)
            horizontal = math.hypot(point.x - self.home_x, point.y - self.home_y)
            if (vertical > self.args.altitude_tolerance or horizontal > self.args.horizontal_tolerance
                    or landed is None or landed.landed_state != IN_AIR):
                stable_start, peak_vertical_error, peak_horizontal_error = None, 0.0, 0.0
                sequences.clear()
                return False
            now = self.ros.Time.now().to_sec()
            if stable_start is None:
                stable_start = now
            sequences.add(sequence)
            peak_vertical_error = max(peak_vertical_error, vertical)
            peak_horizontal_error = max(peak_horizontal_error, horizontal)
            if now - stable_start < 3.0:
                return False
            self.metrics.update(hover_seconds=round(now - stable_start, 3), hover_samples=len(sequences),
                                hover_altitude_m=round(point.z - self.home_z, 4),
                                peak_hover_altitude_error_m=round(peak_vertical_error, 4),
                                peak_hover_horizontal_error_m=round(peak_horizontal_error, 4))
            return True

        self.wait_for(update, self.args.timeout, "3 simulated seconds at 1 m within tolerances")

    def ground_state(self, received_after=0.0):
        state = self.snapshot("state", self.args.timeout, received_after)
        landed = self.snapshot("landed", self.args.timeout)
        return (state, landed is not None and landed.landed_state == ON_GROUND)

    def land(self):
        state_after = 0.0
        pending_arm = self.pending_arm_request is not None and not self.pending_arm_request.is_set()

        def disarmed_on_ground():
            state, on_ground = self.ground_state(state_after)
            return state is not None and state.connected and not state.armed and on_ground

        if not pending_arm and disarmed_on_ground():
            self.metrics["landed_disarmed"] = True
            self.arm_attempted = False
            return

        print("Requesting AUTO.LAND", file=sys.stderr, flush=True)
        self.set_mode("AUTO.LAND")

        def descending():
            state, on_ground = self.ground_state()
            return state is not None and state.connected and (state.mode == "AUTO.LAND"
                                                               or (on_ground and not state.armed))

        def keep_offboard_alive():
            if self.in_mode("OFFBOARD", armed=True):
                self.send(self.command_z)

        self.wait_for(descending, self.args.timeout, "AUTO.LAND", keep_offboard_alive)

        def touchdown():
            state, on_ground = self.ground_state()
            return state is not None and state.connected and on_ground

        self.wait_for(touchdown, self.args.land_timeout, "landed telemetry")
        if pending_arm and not self.pending_arm_request.wait(SERVICE_TIMEOUT):
            raise RuntimeError("arming request completion could not be confirmed")
        if pending_arm:
            state_after = time.monotonic()
        state = self.wait_for(lambda: self.snapshot("state", 2.0, state_after),
                              self.args.timeout, "final arming state on the ground")
        if state is not None and state.armed:
            # PX4 normally auto-disarms. Request a regular disarm only on confirmed ground.
            response = self.service(MAVROS + "/cmd/arming", "CommandBool", False)
            if not response.success and not disarmed_on_ground():
                raise RuntimeError("PX4 rejected disarming on ground")

        self.wait_for(disarmed_on_ground, self.args.timeout, "disarmed landed SITL")
        self.metrics["landed_disarmed"] = True
        self.arm_attempted = False


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", action="store_true", help="run against local Gazebo and arm the SITL vehicle")
    parser.add_argument("--timeout", type=float, default=30.0, help="per-phase wall-time limit in seconds")
    parser.add_argument("--land-timeout", type=float, default=30.0, help="touchdown wall-time limit in seconds")
    parser.add_argument("--altitude-tolerance", type=float, default=0.20, help="absolute altitude tolerance in m")
    parser.add_argument("--horizontal-tolerance", type=float, default=0.30, help="horizontal error tolerance in m")
    parser.add_argument("--tilt-tolerance-deg", type=float, default=1.0, help="zero-tilt tolerance for both joints")
    args = parser.parse_args()
    if not args.run:
        parser.error("explicit --run is required")
    if not all(math.isfinite(value) and value > 0 for value in (args.timeout, args.land_timeout,
               args.altitude_tolerance, args.horizontal_tolerance, args.tilt_tolerance_deg)):
        parser.error("timeouts and tolerances must be finite and positive")

    import rospy
    from gazebo_msgs.srv import GetModelProperties
    from geometry_msgs.msg import PoseStamped
    from mavros_msgs.msg import ExtendedState, State, Tunnel
    from mavros_msgs.srv import CommandBool, ParamGet, SetMode
    from sensor_msgs.msg import JointState

    test = None
    error, cleanup_error = None, None
    try:
        from rosgraph.network import get_local_addresses

        master = urlsplit(os.environ.get("ROS_MASTER_URI", "http://localhost:11311"))
        if master.scheme != "http" or master.port is None:
            raise RuntimeError("expected a local HTTP ROS master")
        require_local(master.hostname, get_local_addresses())
        rospy.init_node("verify_zero_tilt_hover", anonymous=True, log_level=rospy.ERROR, disable_signals=True)
        ros_types = {
            cls.__name__: cls for cls in
            (GetModelProperties, PoseStamped, ExtendedState, State, Tunnel, CommandBool, ParamGet,
             SetMode, JointState)
        }
        test = HoverTest(rospy, ros_types, args)
        test.run()
    except (RuntimeError, rospy.ROSException, rospy.ServiceException, KeyboardInterrupt, ValueError, OSError) as exc:
        error = str(exc) or "interrupted"
        if test is not None and test.arm_attempted:
            try:
                test.land()
            except (RuntimeError, rospy.ROSException, rospy.ServiceException, KeyboardInterrupt, OSError) as cleanup:
                cleanup_error = str(cleanup) or "interrupted"
                print("Landing was not confirmed; check local PX4 SITL console", file=sys.stderr, flush=True)
    print(json.dumps({
        "passed": error is None, "metrics": test.metrics if test is not None else {},
        "error": error, "cleanup_error": cleanup_error,
    }, sort_keys=True))
    return 0 if error is None else 1


if __name__ == "__main__":
    sys.exit(main())
