// SPDX-License-Identifier: BSD-3-Clause

#include "SharedTiltControl.hpp"

#include <cmath>

namespace shared_tilt_control
{
namespace
{

constexpr float kGravityM_S2 = 9.80665f; // Standard gravity, as in PX4's CONSTANTS_ONE_G.
constexpr float kHalfPi = 1.5707963267948966f;

bool positiveFinite(float value)
{
	return std::isfinite(value) && value > 0.f;
}

bool validAngleLimit(float value)
{
	return positiveFinite(value) && value < kHalfPi;
}

bool isRotation(const matrix::Dcmf &attitude)
{
	if (!attitude.isAllFinite()) {
		return false;
	}

	const matrix::Vector3f x(attitude.col(0));
	const matrix::Vector3f y(attitude.col(1));
	const matrix::Vector3f z(attitude.col(2));
	constexpr float tolerance = 1e-3f;
	return std::fabs(x.norm_squared() - 1.f) <= tolerance
	       && std::fabs(y.norm_squared() - 1.f) <= tolerance
	       && std::fabs(z.norm_squared() - 1.f) <= tolerance
	       && (x.cross(y) - z).norm() <= tolerance;
}

} // namespace

bool isTiltSampleUsable(const TiltSample &sample, uint64_t now_us, uint64_t max_age_us, float max_abs_delta_rad)
{
	return sample.timestamp_sample_us > 0 && sample.timestamp_sample_us <= now_us && max_age_us > 0
	       && now_us - sample.timestamp_sample_us <= max_age_us && validAngleLimit(max_abs_delta_rad)
	       && std::isfinite(sample.delta_rad) && std::fabs(sample.delta_rad) <= max_abs_delta_rad;
}

matrix::Vector3f thrustAxis(float delta_rad)
{
	return {std::sin(delta_rad), 0.f, -std::cos(delta_rad)};
}

Status buildEffectiveness(const VehicleConfig &vehicle, float delta_rad,
			  matrix::SquareMatrix<float, 4> &effectiveness)
{
	effectiveness.setZero();

	if (!std::isfinite(delta_rad)) {
		return Status::InvalidInput;
	}

	if (std::fabs(delta_rad) >= kHalfPi) {
		return Status::OutsideEnvelope;
	}

	const matrix::Vector3f axis = thrustAxis(delta_rad);
	matrix::SquareMatrix<float, 4> candidate;

	for (size_t i = 0; i < vehicle.rotors.size(); ++i) {
		const Rotor &rotor = vehicle.rotors[i];

		if (!rotor.position_frd_m.isAllFinite() || !positiveFinite(rotor.drag_torque_per_thrust_m)
		    || (rotor.spin_sign != 1 && rotor.spin_sign != -1)) {
			return Status::InvalidInput;
		}

		const matrix::Vector3f moment = rotor.position_frd_m.cross(axis)
						- axis * (rotor.drag_torque_per_thrust_m * rotor.spin_sign);
		candidate(0, i) = 1.f;

		for (size_t j = 0; j < 3; ++j) {
			candidate(j + 1, i) = moment(j);
		}
	}

	if (!candidate.isAllFinite()) {
		return Status::NumericalFailure;
	}

	effectiveness = candidate;
	return Status::Ok;
}

Status computeReference(const VehicleConfig &vehicle, const ReferenceConfig &config,
			const PositionState &state, const PositionReference &reference, DesiredReference &desired)
{
	desired = {};

	if (!positiveFinite(vehicle.mass_kg) || !positiveFinite(config.position_gain)
	    || !positiveFinite(config.velocity_gain) || !state.position_ned_m.isAllFinite()
	    || !state.velocity_ned_m_s.isAllFinite() || !reference.position_ned_m.isAllFinite()
	    || !reference.velocity_ned_m_s.isAllFinite() || !reference.acceleration_ned_m_s2.isAllFinite()) {
		return Status::InvalidInput;
	}

	const matrix::Vector3f error_position = state.position_ned_m - reference.position_ned_m;
	const matrix::Vector3f error_velocity = state.velocity_ned_m_s - reference.velocity_ned_m_s;
	const matrix::Vector3f gravity_ned{0.f, 0.f, kGravityM_S2};
	const matrix::Vector3f force_ned = (reference.acceleration_ned_m_s2 - gravity_ned
					   - error_position * config.position_gain - error_velocity * config.velocity_gain) * vehicle.mass_kg;
	if (!force_ned.isAllFinite()) {
		return Status::NumericalFailure;
	}

	return solveAttitudeReference(config, force_ned, reference.pitch_rad, reference.yaw_rad, desired);
}

Status solveAttitudeReference(const ReferenceConfig &config, matrix::Vector3f force_ned_n,
			      float pitch_rad, float yaw_rad, DesiredReference &desired)
{
	desired = {};

	if (!positiveFinite(config.min_force_n) || !positiveFinite(config.min_upward_force_n)
	    || !positiveFinite(config.min_transverse_force_n) || !validAngleLimit(config.max_roll_rad)
	    || !validAngleLimit(config.max_pitch_rad) || !validAngleLimit(config.max_tilt_rad)
	    || !force_ned_n.isAllFinite() || !std::isfinite(pitch_rad) || !std::isfinite(yaw_rad)) {
		return Status::InvalidInput;
	}

	if (std::fabs(pitch_rad) > config.max_pitch_rad) {
		return Status::OutsideEnvelope;
	}

	const matrix::Dcmf yaw_pitch{matrix::Eulerf{0.f, pitch_rad, yaw_rad}};
	const matrix::Vector3f force_yaw_pitch = yaw_pitch.transpose() * force_ned_n;
	const float transverse = std::hypot(force_yaw_pitch(1), force_yaw_pitch(2));
	const float magnitude = std::hypot(force_yaw_pitch(0), transverse);

	if (!std::isfinite(magnitude)) {
		return Status::NumericalFailure;
	}

	if (magnitude < config.min_force_n || transverse < config.min_transverse_force_n
	    || -force_yaw_pitch(2) < config.min_upward_force_n) {
		return Status::OutsideEnvelope;
	}

	const float roll = std::atan2(force_yaw_pitch(1), -force_yaw_pitch(2));
	const float tilt = std::atan2(force_yaw_pitch(0), transverse);

	if (std::fabs(roll) > config.max_roll_rad || std::fabs(tilt) > config.max_tilt_rad) {
		return Status::OutsideEnvelope;
	}

	desired.force_ned_n = force_ned_n;
	desired.attitude_body_to_ned = matrix::Dcmf{matrix::Eulerf{roll, pitch_rad, yaw_rad}};
	desired.roll_rad = roll;
	desired.tilt_rad = tilt;
	return Status::Ok;
}

Status projectThrust(const matrix::Vector3f &force_ned_n, const matrix::Dcmf &attitude_body_to_ned,
		     const TiltSample &tilt, uint64_t now_us, uint64_t max_age_us, float max_abs_delta_rad,
		     float &collective_n)
{
	collective_n = 0.f;

	if (!isTiltSampleUsable(tilt, now_us, max_age_us, max_abs_delta_rad)
	    || !force_ned_n.isAllFinite() || !isRotation(attitude_body_to_ned)) {
		return Status::InvalidInput;
	}

	const float projected = thrustAxis(tilt.delta_rad).dot(attitude_body_to_ned.transpose() * force_ned_n);

	if (!std::isfinite(projected)) {
		return Status::NumericalFailure;
	}

	collective_n = projected;
	return Status::Ok;
}

Status computeActuatorRequest(const VehicleConfig &vehicle, const DesiredReference &desired,
			      const matrix::Dcmf &attitude_body_to_ned, const TiltSample &tilt,
			      uint64_t now_us, uint64_t max_age_us, float max_abs_delta_rad,
			      const matrix::Vector3f &torque_frd_nm, ActuatorRequest &request, AllocationResult &allocation)
{
	request = {};
	allocation = {};

	if (!std::isfinite(desired.tilt_rad) || !validAngleLimit(max_abs_delta_rad)) {
		return Status::InvalidInput;
	}

	if (std::fabs(desired.tilt_rad) > max_abs_delta_rad) {
		return Status::OutsideEnvelope;
	}

	float collective_n;
	const Status projection_status = projectThrust(desired.force_ned_n, attitude_body_to_ned, tilt, now_us,
					 max_age_us, max_abs_delta_rad, collective_n);

	if (projection_status != Status::Ok) {
		return projection_status;
	}

	const Status allocation_status = allocate(vehicle, tilt.delta_rad, collective_n, torque_frd_nm, allocation);

	if (allocation_status == Status::Ok) {
		request.timestamp_sample_us = tilt.timestamp_sample_us;
		request.thrust_n = allocation.thrust_n;
		request.tilt_rad = desired.tilt_rad;
	}

	return allocation_status;
}

} // namespace shared_tilt_control
