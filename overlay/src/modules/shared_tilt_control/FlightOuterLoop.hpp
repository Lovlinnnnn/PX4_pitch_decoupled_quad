// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <SharedTiltControl.hpp>
#include <Takeoff.hpp>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_local_position.h>

namespace shared_tilt_control
{

struct FlightOuterInput {
	uint64_t now_us{};
	float dt_s{};
	bool armed{};
	bool landed{};
	bool ground_contact{};
	bool landing{};
	float pitch_rad{};
	float yaw_rad{}; // Estimated heading, for initialization and fallback.
	vehicle_local_position_s position{};
	trajectory_setpoint_s trajectory{};
};

struct FlightOuterOutput {
	DesiredReference desired{};
	uint8_t takeoff_state{takeoff_status_s::TAKEOFF_STATE_DISARMED};
	bool motors_enabled{};
};

// Supplies targets to the native attitude/rate loop. The caller owns all uORB publications.
class FlightOuterLoop
{
public:
	static constexpr float max_pitch_rad = 0.7853981633974483f; // 45 degrees.
	static constexpr float max_measured_tilt_rad = 1.0471975511965976f; // 60 degrees, including tracking transients.

	void configure(float spool_up_s, float ramp_s, float landing_speed_m_s);
	Status update(const VehicleConfig &vehicle, const FlightOuterInput &input, FlightOuterOutput &output);

private:
	void resetVelocityIntegral();
	void updateGroundReference(const FlightOuterInput &input);
	float heightAboveTakeoff(const vehicle_local_position_s &position) const;
	void updateTouchdown(const FlightOuterInput &input, float height_m, float dt_s, bool descending);

	TakeoffHandling _takeoff;
	matrix::Vector2f _landing_hold_ne{};
	matrix::Vector2f _velocity_integral_ne{};
	float _velocity_integral_d{};
	float _landing_speed_m_s{0.7f};
	float _ground_z{};
	float _ground_range{};
	float _yaw{};
	float _pitch{};
	float _stopped_near_ground_s{};
	uint8_t _z_reset_counter{};
	bool _ground_reference_valid{};
	bool _ground_range_valid{};
	bool _takeoff_height_reached{};
	bool _landing_started{};
	bool _touchdown{};
};

} // namespace shared_tilt_control
