// SPDX-License-Identifier: BSD-3-Clause

#include "FlightOuterLoop.hpp"

#include <lib/geo/geo.h>
#include <mathlib/mathlib.h>

#include <algorithm>
#include <cmath>

namespace shared_tilt_control
{
namespace
{

constexpr float max_dt_s = 0.04f;
constexpr float max_roll_rad = 0.26179939f;
// The lateral acceleration limit adds at most atan(0.14) to the 45-degree pitch target.
constexpr float max_tilt_rad = 0.9599310885968813f; // 55 degrees.
constexpr float pitch_slew_rad_s = 0.17453293f;
constexpr float yaw_slew_rad_s = 0.52359878f;
constexpr float horizontal_position_gain = 1.35f;
constexpr float vertical_position_gain = 1.4f;
constexpr float horizontal_velocity_gain = 2.2f;
constexpr float vertical_velocity_gain = 3.f;
constexpr float horizontal_velocity_integral_gain = 1.f;
constexpr float vertical_velocity_integral_gain = 1.2f;
constexpr float horizontal_integral_limit_m_s2 = 0.50f;
constexpr float vertical_integral_limit_m_s2 = 0.60f;
constexpr float max_horizontal_speed_m_s = 0.65f;
constexpr float max_horizontal_accel_m_s2 = 1.35f;
constexpr float max_climb_speed_m_s = 0.8f;
constexpr float max_descent_speed_m_s = 0.9f;
constexpr float touchdown_height_m = 0.10f;
constexpr float touchdown_speed_m_s = 0.20f;
constexpr float touchdown_detection_s = 0.35f;
constexpr float takeoff_height_m = 0.15f;
constexpr float minimum_specific_thrust = 0.10f;
constexpr ReferenceConfig reference_config{1.f, 1.f, 0.01f, 0.01f, 0.01f,
		max_roll_rad, FlightOuterLoop::max_pitch_rad, max_tilt_rad};

bool usablePosition(const vehicle_local_position_s &position)
{
	return position.xy_valid && position.z_valid && position.v_xy_valid && position.v_z_valid
	       && matrix::Vector3f{position.x, position.y, position.z}.isAllFinite()
	       && matrix::Vector3f{position.vx, position.vy, position.vz}.isAllFinite();
}

bool usableRange(const vehicle_local_position_s &position)
{
	return position.dist_bottom_valid && std::isfinite(position.dist_bottom) && position.dist_bottom >= 0.f;
}

float slew(float value, float target, float max_step)
{
	return value + math::constrain(target - value, -max_step, max_step);
}

float finiteOr(float value, float fallback)
{
	return std::isfinite(value) ? value : fallback;
}

void limitNorm(matrix::Vector2f &value, float limit)
{
	const float norm = value.norm();

	if (norm > limit) {
		value *= limit / norm;
	}
}

} // namespace

void FlightOuterLoop::configure(float spool_up_s, float ramp_s, float landing_speed_m_s)
{
	_takeoff.setSpoolupTime(math::constrain(finiteOr(spool_up_s, 1.f), 0.f, 10.f));
	_takeoff.setTakeoffRampTime(math::constrain(finiteOr(ramp_s, 3.f), 0.1f, 10.f));
	_takeoff.generateInitialRampValue(vertical_velocity_gain);
	_landing_speed_m_s = math::constrain(finiteOr(landing_speed_m_s, 0.7f), 0.3f, max_descent_speed_m_s);
}

void FlightOuterLoop::resetVelocityIntegral()
{
	_velocity_integral_ne.setZero();
	_velocity_integral_d = 0.f;
}

void FlightOuterLoop::updateGroundReference(const FlightOuterInput &input)
{
	const auto &position = input.position;

	if (!input.armed || (!_ground_reference_valid && input.landed && !_takeoff_height_reached)) {
		if (input.landed) {
			_ground_z = position.z;
			_ground_range = position.dist_bottom;
			_ground_range_valid = usableRange(position);
			_z_reset_counter = position.z_reset_counter;
			_ground_reference_valid = true;
		}

	} else if (_ground_reference_valid && _z_reset_counter != position.z_reset_counter) {
		const auto resets = static_cast<uint8_t>(position.z_reset_counter - _z_reset_counter);

		if (resets == 1 && std::isfinite(position.delta_z)) {
			_ground_z += position.delta_z;

		} else {
			_ground_reference_valid = false;
		}

		_z_reset_counter = position.z_reset_counter;
	}
}

float FlightOuterLoop::heightAboveTakeoff(const vehicle_local_position_s &position) const
{
	float height_m = _ground_reference_valid ? _ground_z - position.z : INFINITY;

	if (_ground_range_valid && usableRange(position)) {
		height_m = std::max(height_m, position.dist_bottom - _ground_range);
	}

	return height_m;
}

void FlightOuterLoop::updateTouchdown(const FlightOuterInput &input, float height_m, float dt_s, bool descending)
{
	if (!input.landing) {
		_stopped_near_ground_s = 0.f;
		return;
	}

	if (input.ground_contact || input.landed) {
		_touchdown = true;

	} else if (descending && height_m <= touchdown_height_m && std::fabs(input.position.vz) <= touchdown_speed_m_s) {
		// The native detector also needs low thrust. At observed contact, release thrust so it can latch landed.
		_stopped_near_ground_s += dt_s;
		_touchdown |= _stopped_near_ground_s >= touchdown_detection_s;

	} else {
		_stopped_near_ground_s = 0.f;
	}
}

Status FlightOuterLoop::update(const VehicleConfig &vehicle, const FlightOuterInput &input, FlightOuterOutput &output)
{
	output = {};

	if (!std::isfinite(input.dt_s) || input.dt_s < 0.f || !std::isfinite(input.yaw_rad)
	    || !std::isfinite(vehicle.mass_kg) || vehicle.mass_kg <= 0.f) {
		return Status::InvalidInput;
	}

	const float dt_s = std::min(input.dt_s, max_dt_s);
	const auto &position = input.position;
	const auto &trajectory = input.trajectory;

	if (!input.armed) {
		_takeoff.updateTakeoffState(false, input.landed, false, max_climb_speed_m_s, false, input.now_us);
		resetVelocityIntegral();
		_yaw = input.yaw_rad;
		_pitch = 0.f;
		_touchdown = false;
		_landing_started = false;
		_takeoff_height_reached = false;
		_stopped_near_ground_s = 0.f;

		if (usablePosition(position)) {
			updateGroundReference(input);
		}

		output.desired.attitude_body_to_ned = matrix::Dcmf{matrix::Eulerf{0.f, 0.f, _yaw}};
		return Status::Ok;
	}

	if (!usablePosition(position) || (!input.landing && !std::isfinite(input.pitch_rad))) {
		return Status::InvalidInput;
	}

	updateGroundReference(input);
	const float height_m = heightAboveTakeoff(position);
	_takeoff_height_reached |= _ground_reference_valid && height_m >= takeoff_height_m;

	if (input.landing && !_landing_started) {
		_landing_hold_ne = {position.x, position.y};
	}

	_landing_started = input.landing;
	const bool want_takeoff = !input.landing && (finiteOr(trajectory.position[2], position.z) < position.z - 0.035f
				   || finiteOr(trajectory.velocity[2], 0.f) < -0.05f);
	_takeoff.updateTakeoffState(true, input.landed, want_takeoff, max_climb_speed_m_s, false, input.now_us);
	const TakeoffState takeoff_state = _takeoff.getTakeoffState();
	const float climb_limit = _takeoff.updateRamp(dt_s, max_climb_speed_m_s);
	const bool flying = takeoff_state == TakeoffState::flight;
	const bool integral_enabled = flying && !input.landing && !input.landed;

	if (!integral_enabled) {
		resetVelocityIntegral();
	}

	output.takeoff_state = static_cast<uint8_t>(input.landing && _takeoff_height_reached ? TakeoffState::flight : takeoff_state);

	const float yaw_error = matrix::wrap_pi(finiteOr(trajectory.yaw, _yaw) - _yaw);
	_yaw = matrix::wrap_pi(_yaw + math::constrain(yaw_error, -yaw_slew_rad_s * dt_s, yaw_slew_rad_s * dt_s));
	const float pitch_target = flying && !input.landing ? math::constrain(input.pitch_rad, -max_pitch_rad, max_pitch_rad) : 0.f;
	_pitch = slew(_pitch, pitch_target, pitch_slew_rad_s * dt_s);

	const bool native_descent = input.landing && std::isfinite(trajectory.velocity[2]) && trajectory.velocity[2] > 0.05f;
	updateTouchdown(input, height_m, dt_s, native_descent);

	if (takeoff_state < TakeoffState::rampup || _touchdown) {
		output.desired.attitude_body_to_ned = matrix::Dcmf{matrix::Eulerf{0.f, 0.f, _yaw}};
		return Status::Ok;
	}

	// In AUTO.LAND the native FlightTask supplies XY and a positive-down velocity. Its Z position is unused.
	const matrix::Vector2f p_xy{position.x, position.y};
	const matrix::Vector2f hold_xy = input.landing ? _landing_hold_ne : p_xy;
	const matrix::Vector2f target_xy{finiteOr(trajectory.position[0], hold_xy(0)), finiteOr(trajectory.position[1], hold_xy(1))};
	const matrix::Vector2f requested_velocity{finiteOr(trajectory.velocity[0], 0.f), finiteOr(trajectory.velocity[1], 0.f)};
	const matrix::Vector2f actual_velocity{position.vx, position.vy};
	matrix::Vector2f velocity_xy = (target_xy - p_xy) * horizontal_position_gain + requested_velocity;
	limitNorm(velocity_xy, max_horizontal_speed_m_s);
	const matrix::Vector2f velocity_error_xy = velocity_xy - actual_velocity;
	const matrix::Vector2f requested_accel_xy = velocity_error_xy * horizontal_velocity_gain + _velocity_integral_ne
			+ matrix::Vector2f{finiteOr(trajectory.acceleration[0], 0.f), finiteOr(trajectory.acceleration[1], 0.f)};

	float velocity_z = native_descent ? std::min(trajectory.velocity[2], _landing_speed_m_s) : 0.f;
	float accel_feedforward_z = 0.f;

	if (!input.landing) {
		velocity_z = (finiteOr(trajectory.position[2], position.z) - position.z) * vertical_position_gain
			     + finiteOr(trajectory.velocity[2], 0.f);
		accel_feedforward_z = flying ? finiteOr(trajectory.acceleration[2], 0.f) : 0.f;
	}

	const float limited_vz = math::constrain(velocity_z, -climb_limit, max_descent_speed_m_s + std::max(-climb_limit, 0.f));
	const float velocity_error_z = limited_vz - position.vz;
	const float accel_z = velocity_error_z * vertical_velocity_gain + _velocity_integral_d + accel_feedforward_z;
	const float requested_specific_thrust = CONSTANTS_ONE_G - accel_z;
	const float specific_thrust = math::constrain(requested_specific_thrust, 0.f, 1.35f * CONSTANTS_ONE_G);

	if (specific_thrust <= minimum_specific_thrust) {
		output.desired.attitude_body_to_ned = matrix::Dcmf{matrix::Eulerf{0.f, 0.f, _yaw}};
		return Status::Ok;
	}

	// Limit lateral reference tilt while vertical force is being ramped from zero.
	matrix::Vector2f accel_xy = requested_accel_xy;
	limitNorm(accel_xy, std::min(max_horizontal_accel_m_s2, 0.14f * specific_thrust));
	const matrix::Vector3f force_ned = matrix::Vector3f{accel_xy(0), accel_xy(1), -specific_thrust} * vehicle.mass_kg;
	const Status result = solveAttitudeReference(reference_config, force_ned, _pitch, _yaw, output.desired);
	output.motors_enabled = result == Status::Ok;

	if (output.motors_enabled && integral_enabled) {
		matrix::Vector2f increment_xy = velocity_error_xy * (horizontal_velocity_integral_gain * dt_s);
		const matrix::Vector2f clipped_accel = requested_accel_xy - accel_xy;
		const float clipped_squared = clipped_accel.norm_squared();
		const float outward_increment = increment_xy.dot(clipped_accel);

		// Keep corrections along the saturated acceleration boundary and allow unwinding.
		if (outward_increment > 0.f && clipped_squared > 0.f) {
			increment_xy -= clipped_accel * (outward_increment / clipped_squared);
		}

		_velocity_integral_ne += increment_xy;
		limitNorm(_velocity_integral_ne, horizontal_integral_limit_m_s2);

		const float clipped_accel_d = specific_thrust - requested_specific_thrust;

		if (velocity_error_z * clipped_accel_d <= 0.f) {
			_velocity_integral_d += velocity_error_z * (vertical_velocity_integral_gain * dt_s);
			_velocity_integral_d = math::constrain(_velocity_integral_d, -vertical_integral_limit_m_s2,
							     vertical_integral_limit_m_s2);
		}
	}

	return result;
}

} // namespace shared_tilt_control
