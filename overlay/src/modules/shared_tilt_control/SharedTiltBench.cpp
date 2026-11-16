// SPDX-License-Identifier: BSD-3-Clause

#include <SharedTiltControl.hpp>
#include "FlightOuterLoop.hpp"
#include <drivers/drv_hrt.h>
#include <parameters/param.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/control_allocator_status.h>
#include <uORB/topics/shared_tilt_allocation_setpoint.h>
#include <uORB/topics/tilting_mc_desired_angles.h>
#include <uORB/topics/tilting_servo_sp.h>
#include <uORB/topics/tilting_servo_status.h>
#include <uORB/topics/takeoff_status.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_torque_setpoint.h>

#include <algorithm>
#include <atomic>
#include <cfloat>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iterator>
#include <mutex>

using namespace time_literals;
using namespace shared_tilt_control;

namespace
{
constexpr int32_t shadow_airframe = 22010;
constexpr int32_t bench_airframe = 22011;
constexpr int32_t flight_airframe = 22012;
constexpr uint32_t bench_interval_us = 4_ms;
constexpr uint64_t arm_timeout_us = 10_s;
constexpr uint64_t run_timeout_us = 20_s;
constexpr uint64_t tilt_timeout_us = 120_ms;
constexpr uint64_t pitch_timeout_us = 300_ms;
constexpr uint64_t land_timeout_us = 1500_ms; // Source republishes every second plus scheduling delay.
constexpr float max_pitch_rad = 0.2617994f; // 15 degrees.
constexpr float max_tilt_rad = 0.34906585f; // Includes the servo tracking transient.
constexpr float pitch_rate_rad_s = 1.f;
constexpr float collective_n = 5.f;
constexpr float bench_max_thrust_n = 3.f;
constexpr float gazebo_kt = 9.84e-6f;
constexpr float gazebo_idle_rad_s = 100.f;
constexpr float gazebo_command_rad_s = 1000.f;
constexpr float gazebo_max_rad_s = gazebo_idle_rad_s + gazebo_command_rad_s;
constexpr float gazebo_min_thrust_n = gazebo_kt * gazebo_idle_rad_s * gazebo_idle_rad_s;
constexpr float gazebo_max_thrust_n = gazebo_kt * gazebo_max_rad_s * gazebo_max_rad_s;
constexpr float gazebo_thrust_model = gazebo_command_rad_s / (gazebo_max_rad_s + gazebo_idle_rad_s);
constexpr float torque_scale_nm = 10.f; // Deliberately exposes saturation on the fixed fixture.
constexpr float residual_tolerance = 1e-3f; // Native allocator normalized tolerance.
constexpr ReferenceConfig reference_config{1.f, 1.f, .1f, .1f, .1f, .1f, max_pitch_rad, max_tilt_rad};
constexpr px4::wq_config_t bench_work_queue{"wq:shared_tilt_bench", 5000, -19};

bool fresh(uint64_t now, uint64_t stamp, uint64_t maximum_age)
{
	return stamp != 0 && stamp <= now && now - stamp <= maximum_age;
}

float gazeboControl(float thrust_n)
{
	return (std::sqrt(thrust_n / gazebo_kt) - gazebo_idle_rad_s) / gazebo_command_rad_s;
}

bool readVehicle(bool bench, VehicleConfig &vehicle)
{
	vehicle.mass_kg = 1.54166f;

	for (size_t motor = 0; motor < vehicle.rotors.size(); motor++) {
		float values[4]{};
		constexpr const char *fields[]{"PX", "PY", "PZ", "KM"};

		for (size_t i = 0; i < 4; i++) {
			char name[16];
			snprintf(name, sizeof(name), "CA_ROTOR%u_%s", static_cast<unsigned>(motor), fields[i]);

			if (param_get(param_find(name), &values[i]) != 0 || !std::isfinite(values[i])) {
				return false;
			}
		}

		vehicle.rotors[motor] = {matrix::Vector3f{values[0], values[1], values[2]}, std::fabs(values[3]),
					values[3] > 0.f ? 1 : -1, gazebo_min_thrust_n,
					bench ? bench_max_thrust_n : gazebo_max_thrust_n};
	}

	AllocationResult validation;
	return allocate(vehicle, 0.f, collective_n, {}, validation) == Status::Ok;
}

class SharedTiltBench : public ModuleBase<SharedTiltBench>, public px4::ScheduledWorkItem
{
public:
	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);
	int print_status() override;

private:
	enum class Command {None, Enable, Disable};
	enum class State {Idle, WaitingArm, Active, Fault};

	struct Report {
		State state{State::Idle};
		const char *fault{"none"};
		unsigned shadow_samples{};
		unsigned allocation_updates{};
		unsigned actuator_publications{};
		unsigned positive_pitch_residuals{};
		unsigned negative_pitch_residuals{};
		unsigned saturated_allocations{};
		unsigned native_torque_samples{};
		float pitch_ref_rad{};
		float tilt_measured_rad{};
		float tilt_target_rad{};
		float collective_n{};
		float strong_retention{};
		float weak_retention{};
		float pitch_residual{};
	};

	SharedTiltBench(bool bench, bool flight, const VehicleConfig &vehicle) :
		ScheduledWorkItem(MODULE_NAME, bench_work_queue), _bench(bench), _flight(flight), _vehicle(vehicle) {}

	void Run() override;
	void evaluate();
	void evaluateFlight();
	void publishAttitude(uint64_t now, const DesiredReference &desired, float thrust_body_z);
	void publishTilt(uint64_t now, float delta_rad);
	void publishAllocation(uint64_t now, uint64_t sample, const AllocationResult &allocation,
			       const matrix::Vector3f &torque_normalized);
	void disarmAndInhibit(uint64_t now, bool armed);
	void requestFlightLanding(uint64_t now, const vehicle_status_s &status, const char *reason);
	void holdFlightActuators(uint64_t now);
	void publishFlightAllocation(uint64_t now, uint8_t mode, const TiltSample &tilt = {}, float collective = 0.f);
	void publishTakeoff(uint64_t now, uint8_t takeoff_state);
	void setFault(const char *fault);
	static const char *stateName(State state);

	const bool _bench;
	const bool _flight;
	const VehicleConfig _vehicle;
	FlightOuterLoop _flight_outer;
	std::atomic<Command> _command{Command::None};
	std::mutex _snapshot_mutex;
	Report _report;
	Report _snapshot;
	uint64_t _wait_started{};
	uint64_t _run_started{};
	uint64_t _last_disarm_request{};
	uint64_t _stop_disarmed_since{};
	uint64_t _previous_run{};
	uint64_t _last_land_request{};
	uint64_t _flight_input_loss_since{};
	bool _flight_land_requested{};
	TiltSample _last_flight_tilt{};
	bool _flight_reference_warning{};
	float _pitch_command{};
	float _yaw_command{};
	bool _inhibited{};

	uORB::Subscription _pitch_sub{ORB_ID(tilting_mc_desired_angles)};
	uORB::Subscription _tilt_sub{ORB_ID(tilting_servo_status)};
	uORB::Subscription _attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _land_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _torque_sub{ORB_ID(vehicle_torque_setpoint)};
	uORB::Subscription _position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _trajectory_sub{ORB_ID(trajectory_setpoint)};
	uORB::Publication<actuator_motors_s> _motors_pub{ORB_ID(actuator_motors)};
	uORB::Publication<control_allocator_status_s> _allocation_pub{ORB_ID(control_allocator_status)};
	uORB::Publication<shared_tilt_allocation_setpoint_s> _flight_allocation_pub{ORB_ID(shared_tilt_allocation_setpoint)};
	uORB::Publication<tilting_servo_sp_s> _tilt_pub{ORB_ID(tilting_servo_setpoint)};
	uORB::Publication<vehicle_attitude_setpoint_s> _attitude_pub{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Publication<vehicle_command_s> _command_pub{ORB_ID(vehicle_command)};
	uORB::Publication<takeoff_status_s> _takeoff_pub{ORB_ID(takeoff_status)};
};

void SharedTiltBench::Run()
{
	const uint64_t now = hrt_absolute_time();

	if (should_exit()) {
		if (_flight) {
			vehicle_status_s status{};
			_status_sub.copy(&status);

			if (status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
				_flight_land_requested = true;
				evaluateFlight();
				_previous_run = hrt_absolute_time();
				return;
			}

			_report.state = State::Idle;
			disarmAndInhibit(now, false);

			if (_stop_disarmed_since == 0 || now < _stop_disarmed_since) {
				_stop_disarmed_since = now;
			}

			if (now - _stop_disarmed_since < 80_ms) {
				return;
			}
		}

		if (_bench) {
			vehicle_status_s status{};
			_status_sub.copy(&status);
			const bool armed = status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
			_report.state = State::Idle;
			disarmAndInhibit(now, armed);

			if (armed) {
				_stop_disarmed_since = 0;
				return;
			}

			if (_stop_disarmed_since == 0 || now < _stop_disarmed_since) {
				_stop_disarmed_since = now;
			}

			if (now - _stop_disarmed_since < 80_ms) {
				return;
			}
		}

		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	if (_flight) {
		evaluateFlight();

	} else {
		evaluate();
	}
	_previous_run = hrt_absolute_time();
	std::lock_guard<std::mutex> lock(_snapshot_mutex);
	_snapshot = _report;
}

void SharedTiltBench::evaluate()
{
	vehicle_status_s status{};
	vehicle_control_mode_s mode{};
	vehicle_land_detected_s land{};
	vehicle_attitude_s attitude{};
	vehicle_torque_setpoint_s torque{};
	tilting_mc_desired_angles_s pitch{};
	tilting_servo_status_s tilt{};
	_status_sub.copy(&status);
	_mode_sub.copy(&mode);
	_land_sub.copy(&land);
	_attitude_sub.copy(&attitude);
	_torque_sub.copy(&torque);
	_pitch_sub.copy(&pitch);
	_tilt_sub.copy(&tilt);

	const uint64_t now = hrt_absolute_time();
	const bool armed = status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
	const bool offboard = fresh(now, status.timestamp, 1_s) && fresh(now, mode.timestamp, 1_s)
			      && status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD && !status.failsafe
			      && mode.flag_control_offboard_enabled && mode.flag_control_position_enabled
			      && mode.flag_control_attitude_enabled && mode.flag_control_rates_enabled;
	const matrix::Quatf q{attitude.q};
	const bool attitude_ok = fresh(now, attitude.timestamp_sample, 100_ms) && q.isAllFinite()
				 && std::fabs(q.norm_squared() - 1.f) < 1e-3f;
	const TiltSample measured{tilt.timestamp_sample, -tilt.angle_rad};
	const bool tilt_ok = tilt.valid && !tilt.fault && fresh(now, tilt.timestamp, tilt_timeout_us)
			     && isTiltSampleUsable(measured, now, tilt_timeout_us, max_tilt_rad);
	const bool pitch_ok = fresh(now, pitch.timestamp, pitch_timeout_us) && std::isfinite(pitch.pitch_body)
			      && std::fabs(pitch.pitch_body) <= max_pitch_rad;
	const matrix::Vector3f native_torque{torque.xyz};
	const bool torque_ok = fresh(now, torque.timestamp_sample, 100_ms) && native_torque.isAllFinite();

	if (pitch_ok) {
		_report.pitch_ref_rad = pitch.pitch_body;
	}

	if (tilt_ok) {
		_report.tilt_measured_rad = measured.delta_rad;
	}

	if (_bench) {
		const Command command = _command.exchange(Command::None);

		if (command == Command::Disable) {
			_report.state = State::Idle;
			_report.fault = "none";
		}

		if (command == Command::Enable && !armed && (_report.state == State::Idle || _report.state == State::Fault)) {
			const bool landed = fresh(now, land.timestamp, land_timeout_us) && land.landed;

			if (landed && offboard && attitude_ok && tilt_ok && pitch_ok) {
				_report.state = State::WaitingArm;
				_report.fault = "none";
				_wait_started = now;
				_pitch_command = 0.f;
				_yaw_command = matrix::Eulerf{q}.psi();

			} else {
				setFault(!landed ? "enable_land" : !offboard ? "enable_offboard" : !attitude_ok ? "enable_attitude" :
					 !tilt_ok ? "enable_tilt" : "enable_pitch");
			}
		}

		if (_report.state == State::WaitingArm && !fresh(now, _wait_started, arm_timeout_us)) {
			setFault("waiting_arm_timeout");
		}

		if (_report.state == State::WaitingArm && armed) {
			if (fresh(now, land.timestamp, land_timeout_us) && land.landed) {
				_report.state = State::Active;
				_run_started = now;

			} else {
				setFault("arming_not_landed");
			}
		}

		if (_report.state == State::Active && !armed) {
			_report.state = State::Idle;
		}

		const bool running = _report.state == State::Active || _report.state == State::WaitingArm;

		if (running && (!offboard || !attitude_ok || !tilt_ok || !pitch_ok)) {
			setFault(!offboard ? "offboard" : !attitude_ok ? "attitude" : !tilt_ok ? "tilt" : "pitch");
		}

		if (_report.state == State::Active) {
			if (!fresh(now, _run_started, run_timeout_us)) {
				setFault("run_timeout");

			} else if (!torque_ok) {
				setFault("native_torque");
			}
		}

		if (_report.state == State::Idle || _report.state == State::Fault) {
			disarmAndInhibit(now, armed);
			return;
		}

	} else if (!attitude_ok || !tilt_ok || !pitch_ok) {
		return;
	}

	const float dt = fresh(now, _previous_run, 100_ms) ? static_cast<float>(now - _previous_run) * 1e-6f : 0.f;
	const float pitch_step = pitch_rate_rad_s * dt;
	_pitch_command = _bench ? _pitch_command + std::max(-pitch_step, std::min(pitch_step, pitch.pitch_body - _pitch_command))
			 : pitch.pitch_body;
	const float yaw = _bench ? _yaw_command : matrix::Eulerf{q}.psi();
	DesiredReference desired;

	if (solveAttitudeReference(reference_config, {0.f, 0.f, -collective_n}, _pitch_command, yaw, desired) != Status::Ok) {
		if (_bench) {
			setFault("reference");
			disarmAndInhibit(now, armed);
		}

		return;
	}

	_report.tilt_target_rad = desired.tilt_rad;

	if (_bench) {
		publishAttitude(now, desired, _report.state == State::Active ? -gazeboControl(collective_n / 4.f) : 0.f);
		publishTilt(now, desired.tilt_rad);

		if (_report.state == State::WaitingArm) {
			disarmAndInhibit(now, false);
			return;
		}
	}

	const matrix::Vector3f requested = torque_ok ? native_torque : matrix::Vector3f{};
	ActuatorRequest request;
	AllocationResult allocation;

	if (computeActuatorRequest(_vehicle, desired, matrix::Dcmf{q}, measured, now, tilt_timeout_us, max_tilt_rad,
				   requested * torque_scale_nm, request, allocation) != Status::Ok) {
		if (_bench) {
			setFault("allocation");
			disarmAndInhibit(now, armed);
		}

		return;
	}

	_report.allocation_updates++;
	_report.native_torque_samples += torque_ok;
	_report.collective_n = allocation.collective_n;
	_report.strong_retention = allocation.strong_retention;
	_report.weak_retention = allocation.weak_retention;

	if (_bench) {
		publishAllocation(now, torque.timestamp_sample, allocation, requested);

	} else {
		_report.shadow_samples++;
	}
}

void SharedTiltBench::evaluateFlight()
{
	vehicle_status_s status{};
	vehicle_control_mode_s mode{};
	vehicle_land_detected_s land{};
	vehicle_attitude_s attitude{};
	vehicle_torque_setpoint_s torque{};
	vehicle_local_position_s position{};
	trajectory_setpoint_s trajectory{};
	tilting_mc_desired_angles_s pitch{};
	tilting_servo_status_s tilt{};
	_status_sub.copy(&status);
	_mode_sub.copy(&mode);
	_land_sub.copy(&land);
	_attitude_sub.copy(&attitude);
	_torque_sub.copy(&torque);
	_position_sub.copy(&position);
	_trajectory_sub.copy(&trajectory);
	_pitch_sub.copy(&pitch);
	_tilt_sub.copy(&tilt);
	const uint64_t now = hrt_absolute_time();
	const bool armed = status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
	const bool landed = fresh(now, land.timestamp, land_timeout_us) && land.landed;
	const bool offboard = status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD
			      && mode.flag_control_offboard_enabled && mode.flag_control_position_enabled && !status.failsafe;
	const bool landing = status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LAND
			     || status.nav_state == vehicle_status_s::NAVIGATION_STATE_DESCEND;
	const bool native_control = fresh(now, mode.timestamp, 1_s) && fresh(now, status.timestamp, 1_s)
				    && mode.flag_control_attitude_enabled && mode.flag_control_rates_enabled;
	const bool trajectory_ok = fresh(now, trajectory.timestamp, 500_ms);
	const matrix::Quatf q{attitude.q};
	const bool attitude_ok = fresh(now, attitude.timestamp_sample, 100_ms) && q.isAllFinite()
				 && std::fabs(q.norm_squared() - 1.f) < 1e-3f;
	const bool position_ok = fresh(now, position.timestamp_sample, 300_ms) && position.xy_valid && position.z_valid
				 && position.v_xy_valid && position.v_z_valid;
	const TiltSample measured{tilt.timestamp_sample, -tilt.angle_rad};
	const bool tilt_ok = tilt.valid && !tilt.fault && fresh(now, tilt.timestamp, tilt_timeout_us)
			     && isTiltSampleUsable(measured, now, tilt_timeout_us, FlightOuterLoop::max_measured_tilt_rad);
	const bool pitch_ok = fresh(now, pitch.timestamp, pitch_timeout_us) && std::isfinite(pitch.pitch_body)
			      && std::fabs(pitch.pitch_body) <= FlightOuterLoop::max_pitch_rad;
	const matrix::Vector3f native_torque{torque.xyz};
	const bool torque_ok = fresh(now, torque.timestamp_sample, 100_ms) && native_torque.isAllFinite();
	const float dt = fresh(now, _previous_run, 100_ms) ? static_cast<float>(now - _previous_run) * 1e-6f : 0.f;

	if (tilt_ok) {
		_last_flight_tilt = measured;
		_report.tilt_measured_rad = measured.delta_rad;
	}

	if (pitch_ok) {
		_report.pitch_ref_rad = pitch.pitch_body;
	}

	const Command command = _command.exchange(Command::None);

	if (command == Command::Disable) {
		if (armed && _report.state == State::Active) {
			_flight_land_requested = true;

		} else {
			_report.state = State::Idle;
			_report.fault = "none";
		}
	}

	if (!armed && _report.state == State::Active) {
		_report.state = State::Idle;
		_report.fault = "none";
	}

	if (!armed && command == Command::Enable && (_report.state == State::Idle || _report.state == State::Fault)) {
		if (landed && offboard && native_control && trajectory_ok && attitude_ok && position_ok && tilt_ok && pitch_ok) {
			_report.state = State::WaitingArm;
			_report.fault = "none";
			_wait_started = now;
			_flight_land_requested = false;
			_flight_reference_warning = false;
			_last_land_request = 0;
			_flight_input_loss_since = 0;

		} else {
			setFault(!landed ? "enable_land" : !offboard || !native_control ? "enable_offboard" :
				 !position_ok || !trajectory_ok ? "enable_position" : !attitude_ok ? "enable_attitude" :
				 !tilt_ok ? "enable_tilt" : "enable_pitch");
		}
	}

	if (_report.state == State::WaitingArm) {
		if (!fresh(now, _wait_started, arm_timeout_us) || !offboard || !position_ok || !attitude_ok || !native_control
		    || !trajectory_ok || !tilt_ok || !pitch_ok) {
			setFault("arming_inputs");

		} else if (armed && landed) {
			_report.state = State::Active;

		} else if (armed) {
			setFault("arming_not_landed");
		}
	}

	if (_report.state == State::Idle || _report.state == State::Fault) {
		FlightOuterOutput inactive;
		FlightOuterInput input{};
		input.now_us = now;
		input.position = position;
		input.landed = landed;
		_flight_outer.update(_vehicle, input, inactive);
		publishTakeoff(now, takeoff_status_s::TAKEOFF_STATE_DISARMED);
		disarmAndInhibit(now, armed && landed);
		return;
	}

	const bool core_ok = attitude_ok && position_ok && native_control && (!armed || torque_ok);
	const char *input_loss = !attitude_ok ? "attitude" : !position_ok ? "position" : !native_control ? "native_mode" :
				 (armed && !torque_ok) ? "native_torque" : !tilt_ok ? "32780" :
				 (!landing && !offboard) ? "offboard" : (!landing && !pitch_ok) ? "32779" :
				 (!landing && !trajectory_ok) ? "trajectory" : nullptr;

	if (armed && input_loss) {
		if (_flight_input_loss_since == 0 || now < _flight_input_loss_since) {
			_flight_input_loss_since = now;
		}

		if (now - _flight_input_loss_since < 200_ms && !_flight_land_requested) {
			holdFlightActuators(now);
			return;
		}

		_flight_land_requested = true;
		requestFlightLanding(now, status, input_loss);

	} else {
		_flight_input_loss_since = 0;
	}

	if (armed && _flight_land_requested && !landing) {
		requestFlightLanding(now, status, "stop_request");
	}

	if (!core_ok) {
		if (armed && _flight_input_loss_since != 0 && now - _flight_input_loss_since > 1_s) {
			disarmAndInhibit(now, false);

		} else {
			holdFlightActuators(now);
		}

		return;
	}

	if (!trajectory_ok || status.nav_state == vehicle_status_s::NAVIGATION_STATE_DESCEND) {
		// Bridge the mode/trajectory handover with a bounded local descent reference.
		trajectory.position[0] = position.x;
		trajectory.position[1] = position.y;
		trajectory.position[2] = NAN;
		trajectory.velocity[0] = 0.f;
		trajectory.velocity[1] = 0.f;
		trajectory.velocity[2] = .4f;
		std::fill(std::begin(trajectory.acceleration), std::end(trajectory.acceleration), 0.f);
		trajectory.yaw = NAN;
	}

	FlightOuterInput input{};
	input.now_us = now;
	input.dt_s = dt;
	input.armed = armed;
	input.landed = landed;
	input.ground_contact = fresh(now, land.timestamp, land_timeout_us) && land.ground_contact;
	input.landing = landing || _flight_land_requested;
	input.pitch_rad = pitch_ok ? pitch.pitch_body : 0.f;
	input.yaw_rad = matrix::Eulerf{q}.psi();
	input.position = position;
	input.trajectory = trajectory;
	FlightOuterOutput output;
	const Status outer_status = _flight_outer.update(_vehicle, input, output);

	if (outer_status != Status::Ok) {
		if (!_flight_reference_warning) {
			PX4_WARN("Flight reference outside envelope (%d)", static_cast<int>(outer_status));
			_flight_reference_warning = true;
		}

		if (armed) {
			_flight_land_requested = true;
			requestFlightLanding(now, status, "reference");
		}

		holdFlightActuators(now);
		return;
	}

	publishTakeoff(now, output.takeoff_state);
	_report.tilt_target_rad = output.desired.tilt_rad;

	if (!armed || !output.motors_enabled) {
		publishAttitude(now, output.desired, 0.f);
		disarmAndInhibit(now, false);
		return;
	}

	if (!torque_ok || _last_flight_tilt.timestamp_sample_us == 0) {
		requestFlightLanding(now, status, "allocator_inputs");
		holdFlightActuators(now);
		return;
	}

	const matrix::Dcmf attitude_body_to_ned{q};
	const TiltSample &used_tilt = tilt_ok ? measured : _last_flight_tilt;
	// Only the recovery descent may use the last angle while returning the servo to neutral.
	const matrix::Vector3f axis = thrustAxis(used_tilt.delta_rad);
	const float collective = axis.dot(attitude_body_to_ned.transpose() * output.desired.force_ned_n);

	if (!std::isfinite(collective)) {
		requestFlightLanding(now, status, "allocation");
		holdFlightActuators(now);
		return;
	}

	const float rotor_count = static_cast<float>(_vehicle.rotors.size());
	const float motor_thrust_n = std::min(std::max(collective / rotor_count, gazebo_min_thrust_n), gazebo_max_thrust_n);
	const float normalized_collective = (motor_thrust_n - gazebo_min_thrust_n)
					    / (gazebo_max_thrust_n - gazebo_min_thrust_n);
	// Preserve the existing Z-only throttle indicator for the native land detector.
	const float thrust_body_z = -std::max(0.f, gazeboControl(-axis(2) * motor_thrust_n));
	publishAttitude(now, output.desired, thrust_body_z);
	publishTilt(now, tilt_ok ? output.desired.tilt_rad : 0.f);
	publishFlightAllocation(now, shared_tilt_allocation_setpoint_s::ACTIVE, used_tilt, normalized_collective);
	_report.native_torque_samples++;
	_report.collective_n = rotor_count * motor_thrust_n;
}

void SharedTiltBench::requestFlightLanding(uint64_t now, const vehicle_status_s &status, const char *reason)
{
	if (status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LAND
	    || status.nav_state == vehicle_status_s::NAVIGATION_STATE_DESCEND || fresh(now, _last_land_request, 500_ms)) {
		return;
	}

	if (_last_land_request == 0) {
		PX4_WARN("Requesting SITL flight landing: %s", reason);
	}

	vehicle_command_s command{};
	command.timestamp = now;
	command.command = vehicle_command_s::VEHICLE_CMD_NAV_LAND;
	command.param4 = NAN;
	command.param5 = NAN;
	command.param6 = NAN;
	command.param7 = NAN;
	command.target_system = status.system_id;
	command.target_component = status.component_id;
	command.source_system = status.system_id;
	command.source_component = status.component_id;
	_command_pub.publish(command);
	_last_land_request = now;
}

void SharedTiltBench::holdFlightActuators(uint64_t now)
{
	publishFlightAllocation(now, shared_tilt_allocation_setpoint_s::HOLD);
}

void SharedTiltBench::publishFlightAllocation(uint64_t now, uint8_t mode, const TiltSample &tilt, float collective)
{
	shared_tilt_allocation_setpoint_s request{};
	request.timestamp = now;
	request.timestamp_sample = tilt.timestamp_sample_us;
	request.tilt_rad = tilt.delta_rad;
	request.collective = collective;
	request.mode = mode;
	_flight_allocation_pub.publish(request);
}

void SharedTiltBench::publishTakeoff(uint64_t now, uint8_t takeoff_state)
{
	takeoff_status_s status{};
	status.timestamp = now;
	status.takeoff_state = takeoff_state;
	status.tilt_limit = .30f;
	_takeoff_pub.publish(status);
}

void SharedTiltBench::publishAttitude(uint64_t now, const DesiredReference &desired, float thrust_body_z)
{
	vehicle_attitude_setpoint_s setpoint{};
	setpoint.timestamp = now;
	setpoint.yaw_sp_move_rate = NAN;
	matrix::Quatf{desired.attitude_body_to_ned}.copyTo(setpoint.q_d);
	setpoint.thrust_body[2] = thrust_body_z;
	_attitude_pub.publish(setpoint);
}

void SharedTiltBench::publishTilt(uint64_t now, float delta_rad)
{
	tilting_servo_sp_s setpoint{};
	setpoint.timestamp = now;
	setpoint.angle[0] = -delta_rad; // Existing 32778 uses the opposite sign, represented in deg on the wire.
	_tilt_pub.publish(setpoint);
}

void SharedTiltBench::publishAllocation(uint64_t now, uint64_t sample, const AllocationResult &allocation,
				       const matrix::Vector3f &torque_normalized)
{
	actuator_motors_s motors{};
	motors.timestamp = now;
	motors.timestamp_sample = sample;
	std::fill(std::begin(motors.control), std::end(motors.control), NAN);
	control_allocator_status_s feedback{};
	feedback.timestamp = now;
	feedback.thrust_setpoint_achieved = !allocation.collective_limited;

	for (size_t i = 0; i < _vehicle.rotors.size(); i++) {
		const float thrust = allocation.thrust_n(i);
		motors.control[i] = std::max(0.f, gazeboControl(thrust));

		if (thrust >= _vehicle.rotors[i].max_thrust_n - 1e-4f) {
			feedback.actuator_saturation[i] = control_allocator_status_s::ACTUATOR_SATURATION_UPPER;

		} else if (thrust <= _vehicle.rotors[i].min_thrust_n + 1e-4f) {
			feedback.actuator_saturation[i] = control_allocator_status_s::ACTUATOR_SATURATION_LOWER;
		}
	}

	matrix::Vector3f residual = torque_normalized - allocation.torque_frd_nm / torque_scale_nm;
	feedback.torque_setpoint_achieved = residual.norm_squared() < residual_tolerance * residual_tolerance;

	if (feedback.torque_setpoint_achieved) {
		residual.setZero();
	}

	residual.copyTo(feedback.unallocated_torque);
	_report.pitch_residual = residual(1);
	_report.positive_pitch_residuals += residual(1) > residual_tolerance;
	_report.negative_pitch_residuals += residual(1) < -residual_tolerance;
	_report.saturated_allocations += !feedback.torque_setpoint_achieved;
	_allocation_pub.publish(feedback);
	_motors_pub.publish(motors);
	_report.actuator_publications++;
	_inhibited = false;
}

void SharedTiltBench::disarmAndInhibit(uint64_t now, bool armed)
{
	if (_flight) {
		publishFlightAllocation(now, shared_tilt_allocation_setpoint_s::INHIBIT);

	} else {
		actuator_motors_s motors{};
		motors.timestamp = now;
		motors.timestamp_sample = now;
		std::fill(std::begin(motors.control), std::end(motors.control), NAN);
		_motors_pub.publish(motors); // Refresh HIL output even after the last active command.
		_report.actuator_publications++;

		if (!_inhibited) {
			control_allocator_status_s feedback{};
			feedback.timestamp = now;
			feedback.torque_setpoint_achieved = true;
			feedback.thrust_setpoint_achieved = true;
			_allocation_pub.publish(feedback);
			_inhibited = true;
		}
	}

	if (_report.state != State::WaitingArm) {
		publishTilt(now, 0.f);
	}

	if (armed && !fresh(now, _last_disarm_request, 200_ms)) {
		vehicle_status_s status{};
		_status_sub.copy(&status);
		vehicle_command_s command{};
		command.timestamp = now;
		command.command = vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM;
		command.param1 = vehicle_command_s::ARMING_ACTION_DISARM;
		command.param2 = _bench ? 21196.f : 0.f; // Forced disarm is exclusive to the fixed fixture.
		command.target_system = status.system_id;
		command.target_component = status.component_id;
		command.source_system = status.system_id;
		command.source_component = status.component_id;
		_command_pub.publish(command);
		_last_disarm_request = now;
	}
}

void SharedTiltBench::setFault(const char *fault)
{
	_report.state = State::Fault;
	_report.fault = fault;
	PX4_WARN("%s inhibited: %s", _flight ? "SITL flight profile" : "Fixed bench", fault);
}

const char *SharedTiltBench::stateName(State state)
{
	switch (state) {
	case State::Idle: return "idle";
	case State::WaitingArm: return "waiting_arm";
	case State::Active: return "active";
	case State::Fault: return "fault";
	}

	return "unknown";
}

int SharedTiltBench::print_status()
{
	Report report;

	{
		std::lock_guard<std::mutex> lock(_snapshot_mutex);
		report = _snapshot;
	}

	PX4_INFO_RAW("mode=%s state=%s fault=%s\n", _flight ? "flight" : _bench ? "bench" : "shadow", stateName(report.state),
		     report.fault);
	PX4_INFO_RAW("shadow_samples=%u allocation_updates=%u actuator_publications=%u native_torque_samples=%u\n",
		     report.shadow_samples, report.allocation_updates, report.actuator_publications, report.native_torque_samples);
	PX4_INFO_RAW("positive_pitch_residuals=%u negative_pitch_residuals=%u saturated_allocations=%u\n",
		     report.positive_pitch_residuals, report.negative_pitch_residuals, report.saturated_allocations);
	PX4_INFO_RAW("pitch_ref_rad=%.6f tilt_measured_rad=%.6f tilt_target_rad=%.6f\n",
		     static_cast<double>(report.pitch_ref_rad), static_cast<double>(report.tilt_measured_rad),
		     static_cast<double>(report.tilt_target_rad));
	PX4_INFO_RAW("collective_n=%.6f strong_retention=%.6f weak_retention=%.6f pitch_residual=%.6f\n",
		     static_cast<double>(report.collective_n), static_cast<double>(report.strong_retention),
		     static_cast<double>(report.weak_retention), static_cast<double>(report.pitch_residual));
	return PX4_OK;
}

int SharedTiltBench::task_spawn(int argc, char *argv[])
{
	if (argc != 1) {
		return print_usage("unexpected start arguments");
	}

	int32_t airframe{};
	param_get(param_find("SYS_AUTOSTART"), &airframe);
	const char *model = std::getenv("PX4_SIM_MODEL");
	const char *fixture = std::getenv("PX4_SHARED_TILT_FIXED_BENCH");
	const char *flight_profile = std::getenv("PX4_SHARED_TILT_FLIGHT");
	const bool bench = airframe == bench_airframe;
	const bool flight = airframe == flight_airframe;

	if ((airframe != shadow_airframe && !bench && !flight) || !model || strcmp(model, "gazebo-classic_pitch_decoupled_quad") != 0
	    || (bench && (!fixture || strcmp(fixture, "1") != 0))
	    || (flight && (!flight_profile || strcmp(flight_profile, "1") != 0))) {
		PX4_ERR("Requires a pitch_decoupled_quad SITL shadow, fixed fixture, or flight launcher");
		return PX4_ERROR;
	}

	VehicleConfig vehicle;
	float thrust_model{};
	const float required_thrust_model = flight ? gazebo_thrust_model : 0.f;

	if (!readVehicle(bench, vehicle) || ((bench || flight) && (param_get(param_find("THR_MDL_FAC"), &thrust_model) != 0
						      || !std::isfinite(thrust_model)
						      || std::fabs(thrust_model - required_thrust_model) > FLT_EPSILON))) {
		PX4_ERR("Invalid propulsion geometry or THR_MDL_FAC for this profile");
		return PX4_ERROR;
	}

	float spoolup = 1.f;
	float ramp = 2.f;
	float landing_speed = .45f;

	if (flight && (param_get(param_find("COM_SPOOLUP_TIME"), &spoolup) != 0
		       || param_get(param_find("MPC_TKO_RAMP_T"), &ramp) != 0
		       || param_get(param_find("MPC_LAND_SPEED"), &landing_speed) != 0)) {
		PX4_ERR("Invalid flight takeoff configuration");
		return PX4_ERROR;
	}

	SharedTiltBench *instance = new SharedTiltBench(bench, flight, vehicle);

	if (!instance) {
		return PX4_ERROR;
	}

	if (flight) {
		instance->_flight_outer.configure(spoolup, ramp, landing_speed);
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;
	instance->ScheduleOnInterval(bench || flight ? bench_interval_us : 20_ms);
	return PX4_OK;
}

int SharedTiltBench::custom_command(int argc, char *argv[])
{
	SharedTiltBench *instance = get_instance();

	if (argc != 1 || !instance || (!instance->_bench && !instance->_flight)) {
		PX4_ERR("Enable/disable require the 22011 bench or 22012 flight profile");
		return PX4_ERROR;
	}

	if (strcmp(argv[0], "enable") == 0) {
		instance->_command.store(Command::Enable);

	} else if (strcmp(argv[0], "disable") == 0) {
		instance->_command.store(Command::Disable);

	} else {
		return print_usage("unknown command");
	}

	return PX4_OK;
}

int SharedTiltBench::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION("Read-only 22010 shadow and fixed 22011 SITL bench for shared-tilt allocation.");
	PRINT_MODULE_USAGE_NAME("shared_tilt_control", "controller");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start mode selected by SYS_AUTOSTART");
	PRINT_MODULE_USAGE_COMMAND_DESCR("enable", "Prepare fixed 22011 bench while disarmed and landed in Offboard");
	PRINT_MODULE_USAGE_COMMAND_DESCR("disable", "Disarm and inhibit fixed bench outputs");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return reason ? PX4_ERROR : PX4_OK;
}

} // namespace

extern "C" __EXPORT int shared_tilt_control_main(int argc, char *argv[])
{
	return SharedTiltBench::main(argc, argv);
}
