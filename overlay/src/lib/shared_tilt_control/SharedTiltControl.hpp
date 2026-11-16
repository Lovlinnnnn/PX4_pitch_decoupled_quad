// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <array>
#include <cstdint>
#include <matrix/matrix/math.hpp>

namespace shared_tilt_control
{

using MotorThrust = matrix::Vector4f;

enum class Status {
	Ok,
	InvalidInput,
	OutsideEnvelope,
	SingularGeometry,
	NumericalFailure
};

struct Rotor {
	matrix::Vector3f position_frd_m{}; // Effective thrust-line point relative to the center of mass.
	float drag_torque_per_thrust_m{};
	int spin_sign{}; // Sign of the drag moment about FRD +Z at zero tilt, either +1 or -1.
	float min_thrust_n{};
	float max_thrust_n{};
};

struct VehicleConfig {
	float mass_kg{};
	std::array<Rotor, 4> rotors{}; // Logical Motor 1 through Motor 4.
};

struct TiltSample {
	uint64_t timestamp_sample_us{}; // Sample time in the same monotonic clock as the control cycle.
	float delta_rad{}; // Positive when thrust tilts forward in FRD.
};

struct PositionState {
	matrix::Vector3f position_ned_m{};
	matrix::Vector3f velocity_ned_m_s{};
};

struct PositionReference {
	matrix::Vector3f position_ned_m{};
	matrix::Vector3f velocity_ned_m_s{};
	matrix::Vector3f acceleration_ned_m_s2{};
	float pitch_rad{};
	float yaw_rad{};
};

struct ReferenceConfig {
	float position_gain{}; // 1/s^2
	float velocity_gain{}; // 1/s
	float min_force_n{};
	float min_upward_force_n{}; // Lower bound on -g_z in the requested yaw/pitch frame.
	float min_transverse_force_n{}; // Lower bound on hypot(g_y, g_z) in that frame.
	float max_roll_rad{};
	float max_pitch_rad{};
	float max_tilt_rad{};
};

struct DesiredReference {
	matrix::Vector3f force_ned_n{};
	matrix::Dcmf attitude_body_to_ned{};
	float roll_rad{};
	float tilt_rad{};
};

struct AllocationResult {
	MotorThrust thrust_n{};
	float collective_n{};
	matrix::Vector3f force_frd_n{};
	matrix::Vector3f torque_frd_nm{};
	float strong_retention{}; // Torque perpendicular to the thrust axis, relative to feasible trim.
	float weak_retention{}; // Torque along the thrust axis, relative to feasible trim.
	bool collective_limited{};
};

struct ActuatorRequest {
	uint64_t timestamp_sample_us{}; // Measured tilt sample used in this allocation.
	MotorThrust thrust_n{};
	float tilt_rad{}; // Common target angle in the same convention as TiltSample.
};

bool isTiltSampleUsable(const TiltSample &sample, uint64_t now_us, uint64_t max_age_us, float max_abs_delta_rad);

matrix::Vector3f thrustAxis(float delta_rad);

// Rows map rotor thrusts in N to [collective in N; body torque in N*m].
Status buildEffectiveness(const VehicleConfig &vehicle, float delta_rad, matrix::SquareMatrix<float, 4> &effectiveness);

Status computeReference(const VehicleConfig &vehicle, const ReferenceConfig &config,
			const PositionState &state, const PositionReference &reference, DesiredReference &desired);

// Maps an externally computed NED force to the requested pitch/yaw, roll, and tilt.
Status solveAttitudeReference(const ReferenceConfig &config, matrix::Vector3f force_ned_n,
			      float pitch_rad, float yaw_rad, DesiredReference &desired);

// The projection is signed. Only allocation applies the motor thrust bounds.
Status projectThrust(const matrix::Vector3f &force_ned_n, const matrix::Dcmf &attitude_body_to_ned,
		     const TiltSample &tilt, uint64_t now_us, uint64_t max_age_us, float max_abs_delta_rad,
		     float &collective_n);

Status allocate(const VehicleConfig &vehicle, float delta_rad, float collective_n,
		const matrix::Vector3f &torque_frd_nm, AllocationResult &result);

// The flight controller provides torque from its existing PID, with the actuator scale applied upstream.
Status computeActuatorRequest(const VehicleConfig &vehicle, const DesiredReference &desired,
			      const matrix::Dcmf &attitude_body_to_ned, const TiltSample &tilt,
			      uint64_t now_us, uint64_t max_age_us, float max_abs_delta_rad,
			      const matrix::Vector3f &torque_frd_nm, ActuatorRequest &request, AllocationResult &allocation);

} // namespace shared_tilt_control
