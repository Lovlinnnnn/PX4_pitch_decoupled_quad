# Pitch-decoupled quadrotor on PX4 v1.16.0

This project adds a quadrotor with a shared tilt command for its front and rear motor assemblies. The flight mode accepts position and body-pitch references and uses the measured servo angle to compute the thrust direction and actuator effectiveness. PX4's native multicopter attitude controller, angular-rate PID and sequential desaturation remain in the control loop.

## Changes from PX4 v1.16.0

- [Airframes](overlay/ROMFS/px4fmu_common/init.d-posix/airframes) define 22010 for the baseline and shadow runs, 22011 for the fixed-body bench, and 22012 for free flight.
- [Shared-tilt control](overlay/src/modules/shared_tilt_control) converts position and velocity references into a desired force. It uses the requested body pitch and the measured tilt to build the attitude and thrust setpoints. The [control allocator](overlay/src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessSharedTilt.cpp) builds the motor effectiveness matrix from the measured tilt.
- The [Gazebo Classic model](overlay/Tools/simulation/gazebo-classic/models/pitch_decoupled_quad) has two tilt joints. The [servo plugin](overlay/Tools/simulation/gazebo-classic/tilt_servo_plugin) closes a position PID on each joint, limits target slew, and publishes measured joint positions. The [ROS bridge](overlay/tilting_migration/tilt_joint_from_tunnel.py) forwards angle commands and reports measured feedback.
- The [SITL launcher](patches/px4-v1.16.0.patch) starts the model with ROS Noetic. The VirtualBox flight profile puts the Gazebo server and PX4 on one vCPU and limits LLVMpipe rendering workers to reduce contention.
- The [ROS package](overlay/tilting_migration/ros1/offboard_pkg) contains the keyboard-driven `eight` node. The [verification scripts](overlay/tilting_migration/tests) exercise servo feedback, fixed-body operation, hover and the figure-eight mission.

## MAVLink TUNNEL messages

Payload fields use little-endian byte order. In the SITL setup, PX4 uses system/component `1/1` and MAVROS uses `1/191`.

| Payload type | Direction | Payload |
| --- | --- | --- |
| `32778` | PX4 → servo bridge | One float32 common tilt command in degrees |
| `32779` | ROS mission → PX4 | One float32 body-pitch reference in radians |
| `32780` | Servo bridge → PX4 | uint32 sequence, uint32 sample age in microseconds, float32 measured common tilt in radians, uint32 flags |

The command gateway sends `32778` to the simulated position PID. The simulator publishes both joint positions. The bridge compares the two measurements and encodes their mean in `32780`. Flag bit 0 marks valid feedback and bit 1 marks a fault. See the [packet definitions](overlay/src/modules/mavlink/tilting_tunnel.hpp) for the receiver checks and the [bridge](overlay/tilting_migration/tilt_joint_from_tunnel.py) for servo calibration and freshness handling. The transport boundary lets a hardware bridge supply feedback from physical angle sensors.

## Build

This repository contains the changes and additional source for the project. Apply them to the official PX4 v1.16.0 checkout. The installer checks the exact upstream commit and applies the patch and source overlay.

Use a Linux host with the PX4 v1.16.0 SITL build dependencies, Gazebo Classic 11, ROS Noetic, and MAVROS with the TUNNEL plugin. Run the following commands from this repository.

```bash
git clone --branch v1.16.0 --depth 1 https://github.com/PX4/PX4-Autopilot.git ../PX4-Autopilot-v1.16.0
git -C ../PX4-Autopilot-v1.16.0 submodule update --init --recursive
./apply.sh ../PX4-Autopilot-v1.16.0
cd ../PX4-Autopilot-v1.16.0
```

The [PX4 patch](patches/px4-v1.16.0.patch) contains changes to upstream files. The [source overlay](overlay) contains the new modules, model, message definitions and ROS package.

From the PX4 repository root, build the included mission node in a Catkin workspace.

```bash
source /opt/ros/noetic/setup.bash
px4_source="$PWD"
mkdir -p "$HOME/px4_tilt_ws/src"
ln -s "$px4_source/tilting_migration/ros1/offboard_pkg" "$HOME/px4_tilt_ws/src/offboard_pkg"
cd "$HOME/px4_tilt_ws"
catkin_make
```

## Run the 22012 figure eight

Run each block in its own terminal. Start with the ROS master.

```bash
source /opt/ros/noetic/setup.bash
roscore
```

From the PX4 repository root in the Gazebo terminal, start the flight profile.

```bash
source /opt/ros/noetic/setup.bash
rosparam set /use_sim_time true
export GAZEBO_PLUGIN_PATH="/opt/ros/noetic/lib${GAZEBO_PLUGIN_PATH:+:$GAZEBO_PLUGIN_PATH}"
PX4_SYS_AUTOSTART=22012 make px4_sitl gazebo_pitch_decoupled_quad
```

At the PX4 `pxh>` prompt, configure the instance-0 Onboard streams.

```text
mavlink stream -u 14580 -s TUNNEL_TILTING_SERVO -r 50
mavlink stream -u 14580 -s HEARTBEAT -r 5
mavlink stream -u 14580 -s LOCAL_POSITION_NED -r 60
```

Start MAVROS and wait for its PX4 heartbeat.

```bash
source /opt/ros/noetic/setup.bash
roslaunch mavros px4.launch \
  fcu_url:="udp://:14540@127.0.0.1:14580/?ids=1,191" fcu_protocol:=v2.0
```

From the PX4 repository root in another terminal, start the servo bridge.

```bash
source /opt/ros/noetic/setup.bash
python3 -B -u tilting_migration/tilt_joint_from_tunnel.py \
  _angle_limit_deg:=55.0 _feedback_angle_limit_deg:=60.0 \
  _feedback_wall_timeout:=5.0 _command_timeout:=5.0
```

From the PX4 repository root in an interactive terminal, run the mission wrapper.

```bash
source /opt/ros/noetic/setup.bash
source "$HOME/px4_tilt_ws/devel/setup.bash"
python3 -B -u tilting_migration/tests/verify_shared_tilt_eight.py --manual
```

The wrapper starts `offboard_pkg/eight` and enables keyboard input after the vehicle reaches its hover point 1 m above its starting position. Press `8` to follow the figure eight for 100 simulation seconds. Press `1` to set body pitch to zero, or use `2` and `3` to adjust it by 5° per press up to ±45°. Press `Ctrl+C` in the mission terminal to request `AUTO.LAND`.
