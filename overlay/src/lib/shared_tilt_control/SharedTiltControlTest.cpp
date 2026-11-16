// SPDX-License-Identifier: BSD-3-Clause

#include "SharedTiltControl.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace
{
using namespace shared_tilt_control;
using matrix::Dcmf;
using matrix::Eulerf;
using matrix::Vector3f;

constexpr float gravity = 9.80665f;
constexpr float tolerance = 1e-4f;
constexpr float arm_x = .375f;
constexpr float arm_y = .25f;
constexpr float drag = .0625f;

VehicleConfig planarQuad(float max_thrust = 30.f)
{
	VehicleConfig config{};
	config.mass_kg = 6.f;
	config.rotors = {{
			{Vector3f{arm_x, arm_y, 0.f}, drag, 1, 0.f, max_thrust},
			{Vector3f{-arm_x, -arm_y, 0.f}, drag, 1, 0.f, max_thrust},
			{Vector3f{arm_x, -arm_y, 0.f}, drag, -1, 0.f, max_thrust},
			{Vector3f{-arm_x, arm_y, 0.f}, drag, -1, 0.f, max_thrust},
		}};
	return config;
}

VehicleConfig gazeboQuad()
{
	// The 22010 airframe records the effective thrust-line coordinates from the SDF.
	VehicleConfig config{};
	config.mass_kg = 1.54166f;
	config.rotors = {{
			{Vector3f{.235554551f, .243601002f, .006249418f}, .09f, 1, .0984f, 11.9064f},
			{Vector3f{-.245547724f, -.237003998f, .006275652f}, .09f, 1, .0984f, 11.9064f},
			{Vector3f{.235552276f, -.237098998f, .006247652f}, .09f, -1, .0984f, 11.9064f},
			{Vector3f{-.245521449f, .243696002f, .006277418f}, .09f, -1, .0984f, 11.9064f},
		}};
	return config;
}

ReferenceConfig referenceConfig()
{
	return {1.f, 2.f, 1.f, 1.f, 1.f, .7f, .45f, .6f};
}

// Scalar expansion of r x n - c*s*n, independent of the allocator matrix construction.
Vector3f rotorTorque(const Rotor &rotor, float angle, float force)
{
	const Vector3f &r = rotor.position_frd_m;
	const float s = std::sin(angle);
	const float c = std::cos(angle);
	const float reaction = rotor.drag_torque_per_thrust_m * rotor.spin_sign;
	return Vector3f{-r(1) * c - reaction * s, r(0) * c + r(2) * s,
		               -r(1) * s + reaction * c} * force;
}

Vector3f netTorque(const VehicleConfig &config, float angle, const MotorThrust &thrust)
{
	Vector3f torque{};

	for (size_t i = 0; i < config.rotors.size(); ++i) {
		torque += rotorTorque(config.rotors[i], angle, thrust(i));
	}

	return torque;
}

template<size_t N>
void expectVectorNear(const matrix::Vector<float, N> &actual, const matrix::Vector<float, N> &expected)
{
	for (size_t i = 0; i < N; ++i) {
		EXPECT_NEAR(actual(i), expected(i), tolerance) << "component " << i;
	}
}

void expectWithinLimits(const VehicleConfig &config, const MotorThrust &thrust)
{
	for (size_t i = 0; i < config.rotors.size(); ++i) {
		EXPECT_GE(thrust(i), config.rotors[i].min_thrust_n) << "motor " << i + 1;
		EXPECT_LE(thrust(i), config.rotors[i].max_thrust_n) << "motor " << i + 1;
	}
}
} // namespace

TEST(SharedTiltControl, SixKilogramPlanarHover)
{
	const VehicleConfig config = planarQuad();
	DesiredReference desired{};
	ASSERT_EQ(computeReference(config, referenceConfig(), PositionState{}, PositionReference{}, desired), Status::Ok);
	const float weight = 6.f * gravity;
	expectVectorNear(desired.force_ned_n, Vector3f{0.f, 0.f, -weight});
	EXPECT_FLOAT_EQ(desired.roll_rad, 0.f);
	EXPECT_FLOAT_EQ(desired.tilt_rad, 0.f);

	float collective = 0.f;
	ASSERT_EQ(projectThrust(desired.force_ned_n, Dcmf{}, TiltSample{1000, 0.f}, 1000, 100, .6f, collective), Status::Ok);
	EXPECT_NEAR(collective, 58.8399f, tolerance);

	AllocationResult allocated{};
	ASSERT_EQ(allocate(config, 0.f, collective, Vector3f{}, allocated), Status::Ok);
	expectVectorNear(allocated.thrust_n, MotorThrust{weight / 4.f, weight / 4.f, weight / 4.f, weight / 4.f});
	expectVectorNear(allocated.force_frd_n, desired.force_ned_n);
	expectVectorNear(allocated.torque_frd_nm, Vector3f{});
	EXPECT_FALSE(allocated.collective_limited);
	EXPECT_NEAR(allocated.strong_retention, 1.f, tolerance);
	EXPECT_NEAR(allocated.weak_retention, 1.f, tolerance);
}

TEST(SharedTiltControl, PitchedHoverUsesSameSignForAircraftPitchAndDesiredTilt)
{
	for (const float pitch : {-.18f, .18f}) {
		SCOPED_TRACE(pitch);
		PositionReference reference{};
		reference.pitch_rad = pitch;
		reference.yaw_rad = .4f;
		DesiredReference desired{};
		ASSERT_EQ(computeReference(planarQuad(), referenceConfig(), PositionState{}, reference, desired), Status::Ok);
		EXPECT_NEAR(desired.roll_rad, 0.f, tolerance);
		EXPECT_NEAR(desired.tilt_rad, pitch, tolerance);
		const Eulerf euler{desired.attitude_body_to_ned};
		EXPECT_NEAR(euler.theta(), pitch, tolerance);
		EXPECT_NEAR(euler.psi(), reference.yaw_rad, tolerance);
		const Vector3f axis_ned = desired.attitude_body_to_ned * thrustAxis(desired.tilt_rad);
		expectVectorNear(axis_ned, Vector3f{0.f, 0.f, -1.f});
	}
}

TEST(SharedTiltControl, PositionVelocityAndAccelerationInNed)
{
	PositionState state{};
	state.position_ned_m = Vector3f{0.f, -.2f, 0.f};
	state.velocity_ned_m_s = Vector3f{-.1f, 0.f, 0.f};
	PositionReference reference{};
	reference.position_ned_m = Vector3f{.3f, 0.f, 0.f};
	reference.velocity_ned_m_s = Vector3f{.2f, .1f, 0.f};
	reference.acceleration_ned_m_s2 = Vector3f{.1f, .2f, .3f};

	DesiredReference desired{};
	ASSERT_EQ(computeReference(planarQuad(), referenceConfig(), state, reference, desired), Status::Ok);
	const Vector3f expected_force{6.f, 3.6f, 6.f * (.3f - gravity)};
	expectVectorNear(desired.force_ned_n, expected_force);
	EXPECT_GT(desired.roll_rad, 0.f);
	EXPECT_GT(desired.tilt_rad, 0.f);
	const Vector3f axis_ned = desired.attitude_body_to_ned * thrustAxis(desired.tilt_rad);
	expectVectorNear(axis_ned, expected_force.unit());
}

TEST(SharedTiltControl, ExternalForceRespectsYawAndProjectsToDesiredAxis)
{
	constexpr float half_pi = 1.57079632679f;
	const Vector3f force_ned{0.f, 6.f, -20.f};
	DesiredReference desired{};
	ASSERT_EQ(solveAttitudeReference(referenceConfig(), force_ned, 0.f, half_pi, desired), Status::Ok);
	EXPECT_NEAR(desired.roll_rad, 0.f, tolerance);
	EXPECT_NEAR(desired.tilt_rad, std::atan2(6.f, 20.f), tolerance);
	const Vector3f axis_ned = desired.attitude_body_to_ned * thrustAxis(desired.tilt_rad);
	expectVectorNear(axis_ned, force_ned.unit());
}

TEST(SharedTiltControl, ReferenceRejectsInvalidInputsAndOutsideEnvelope)
{
	const VehicleConfig vehicle = planarQuad();
	const ReferenceConfig config = referenceConfig();
	PositionReference reference{};
	DesiredReference desired{};

	reference.acceleration_ned_m_s2(0) = std::numeric_limits<float>::quiet_NaN();
	EXPECT_EQ(computeReference(vehicle, config, PositionState{}, reference, desired), Status::InvalidInput);
	reference.acceleration_ned_m_s2 = Vector3f{0.f, 0.f, gravity};
	EXPECT_EQ(computeReference(vehicle, config, PositionState{}, reference, desired), Status::OutsideEnvelope);
	reference.acceleration_ned_m_s2 = Vector3f{0.f, 0.f, gravity + 2.f};
	EXPECT_EQ(computeReference(vehicle, config, PositionState{}, reference, desired), Status::OutsideEnvelope);
	reference.acceleration_ned_m_s2 = Vector3f{30.f, 0.f, 0.f};
	EXPECT_EQ(computeReference(vehicle, config, PositionState{}, reference, desired), Status::OutsideEnvelope);
	reference.acceleration_ned_m_s2 = Vector3f{};
	reference.pitch_rad = config.max_pitch_rad + .01f;
	EXPECT_EQ(computeReference(vehicle, config, PositionState{}, reference, desired), Status::OutsideEnvelope);
}

TEST(SharedTiltControl, ProjectionUsesMeasuredTiltAndActualAttitude)
{
	PositionReference reference{};
	reference.pitch_rad = .2f;
	DesiredReference desired{};
	ASSERT_EQ(computeReference(planarQuad(), referenceConfig(), PositionState{}, reference, desired), Status::Ok);
	const Dcmf actual_attitude{Eulerf{0.f, -.08f, 0.f}};
	const TiltSample actual_tilt{950, -.31f};
	float projected = 0.f;
	ASSERT_EQ(projectThrust(desired.force_ned_n, actual_attitude, actual_tilt, 1000, 100, .6f, projected), Status::Ok);
	EXPECT_NEAR(projected, 6.f * gravity * std::cos(-.31f + .08f), tolerance);
	EXPECT_GT(std::fabs(projected - 6.f * gravity), 1.f);

	ASSERT_EQ(projectThrust(Vector3f{0.f, 0.f, 5.f}, Dcmf{}, TiltSample{1000, 0.f}, 1000, 100, .6f, projected),
		  Status::Ok);
	EXPECT_NEAR(projected, -5.f, tolerance);
}

TEST(SharedTiltControl, TimestampAndMeasuredAngleAreSufficientForFeedback)
{
	constexpr uint64_t now = 1'000'000;
	constexpr uint64_t max_age = 120'000;
	EXPECT_TRUE(isTiltSampleUsable(TiltSample{now, 0.f}, now, max_age, .6f));
	EXPECT_TRUE(isTiltSampleUsable(TiltSample{now - max_age, .6f}, now, max_age, .6f));
	EXPECT_FALSE(isTiltSampleUsable(TiltSample{now - max_age - 1, 0.f}, now, max_age, .6f));
	EXPECT_FALSE(isTiltSampleUsable(TiltSample{now + 1, 0.f}, now, max_age, .6f));
	EXPECT_FALSE(isTiltSampleUsable(TiltSample{0, 0.f}, now, max_age, .6f));
	EXPECT_FALSE(isTiltSampleUsable(TiltSample{now, -.61f}, now, max_age, .6f));
	EXPECT_FALSE(isTiltSampleUsable(TiltSample{now, std::numeric_limits<float>::infinity()}, now, max_age, .6f));

	float collective = 0.f;
	EXPECT_EQ(projectThrust(Vector3f{0.f, 0.f, -4.f}, Dcmf{}, TiltSample{now - max_age - 1, 0.f},
				now, max_age, .6f, collective), Status::InvalidInput);
}

TEST(SharedTiltControl, RejectsInvalidForceAndReflectedAttitudeForProjection)
{
	const TiltSample sample{1000, 0.f};
	float collective = 1.f;
	const Vector3f invalid_force{std::numeric_limits<float>::quiet_NaN(), 0.f, -5.f};
	EXPECT_EQ(projectThrust(invalid_force, Dcmf{}, sample, 1000, 100, .6f, collective), Status::InvalidInput);
	EXPECT_FLOAT_EQ(collective, 0.f);
	Dcmf reflection{};
	reflection(2, 2) = -1.f;
	EXPECT_EQ(projectThrust(Vector3f{0.f, 0.f, -5.f}, reflection, sample, 1000, 100, .6f, collective),
		  Status::InvalidInput);
}

TEST(SharedTiltControl, ActuatorRequestUsesMeasuredGeometryAndClearsStaleOutputs)
{
	const VehicleConfig config = planarQuad();
	PositionReference reference{};
	reference.pitch_rad = .25f;
	DesiredReference desired{};
	ASSERT_EQ(computeReference(config, referenceConfig(), PositionState{}, reference, desired), Status::Ok);

	constexpr float pitch = -.08f;
	const Dcmf actual_attitude{Eulerf{0.f, pitch, 0.f}};
	const TiltSample measured{950, -.31f};
	const float collective = 6.f * gravity * std::cos(measured.delta_rad - pitch);
	const float trim = collective / 4.f;
	const MotorThrust expected{trim + 1.f, trim - 1.f, trim + 2.f, trim - 2.f};
	const Vector3f requested_torque = netTorque(config, measured.delta_rad, expected);
	ActuatorRequest request{};
	AllocationResult allocation{};
	ASSERT_EQ(computeActuatorRequest(config, desired, actual_attitude, measured, 1000, 100, .6f,
					 requested_torque, request, allocation), Status::Ok);
	EXPECT_EQ(request.timestamp_sample_us, measured.timestamp_sample_us);
	EXPECT_NEAR(request.tilt_rad, reference.pitch_rad, tolerance);
	expectVectorNear(request.thrust_n, expected);
	expectVectorNear(allocation.thrust_n, expected);
	expectVectorNear(allocation.torque_frd_nm, requested_torque);
	EXPECT_NEAR(allocation.collective_n, collective, tolerance);

	EXPECT_EQ(computeActuatorRequest(config, desired, actual_attitude, measured, 1051, 100, .6f,
					requested_torque, request, allocation), Status::InvalidInput);
	EXPECT_EQ(request.timestamp_sample_us, 0u);
	EXPECT_FLOAT_EQ(request.tilt_rad, 0.f);
	expectVectorNear(request.thrust_n, MotorThrust{});
	expectVectorNear(allocation.thrust_n, MotorThrust{});
	expectVectorNear(allocation.torque_frd_nm, Vector3f{});
	EXPECT_FLOAT_EQ(allocation.collective_n, 0.f);
}

TEST(SharedTiltControl, IdealGeometryMatchesPaperRotationAndMotorOrder)
{
	const VehicleConfig config = planarQuad();
	for (const float angle : {-.48f, 0.f, .48f}) {
		SCOPED_TRACE(angle);
		matrix::SquareMatrix<float, 4> matrix{};
		ASSERT_EQ(buildEffectiveness(config, angle, matrix), Status::Ok);
		const float cos_delta = std::cos(angle);
		const float sin_delta = std::sin(angle);
		const float roll[] {-arm_y, arm_y, arm_y, -arm_y};
		const float pitch[] {arm_x, -arm_x, arm_x, -arm_x};
		const float yaw[] {drag, drag, -drag, -drag};

		for (size_t motor = 0; motor < config.rotors.size(); ++motor) {
			SCOPED_TRACE(motor);
			EXPECT_NEAR(matrix(0, motor), 1.f, tolerance);
			EXPECT_NEAR(matrix(1, motor), cos_delta * roll[motor] - sin_delta * yaw[motor], tolerance);
			EXPECT_NEAR(matrix(2, motor), cos_delta * pitch[motor], tolerance);
			EXPECT_NEAR(matrix(3, motor), sin_delta * roll[motor] + cos_delta * yaw[motor], tolerance);
		}
	}
}

TEST(SharedTiltControl, IdealAndGazeboGeometryRoundTrip)
{
	for (const auto &config : {planarQuad(), gazeboQuad()}) {
		SCOPED_TRACE(config.mass_kg);
		for (const float angle : {-.55f, 0.f, .55f}) {
			SCOPED_TRACE(angle);
			const MotorThrust expected{1.4f, 2.7f, 4.4f, 5.6f};
			const float collective = 14.1f;
			const Vector3f torque = netTorque(config, angle, expected);
			AllocationResult result{};
			ASSERT_EQ(allocate(config, angle, collective, torque, result), Status::Ok);
			expectVectorNear(result.thrust_n, expected);
			expectVectorNear(result.torque_frd_nm, torque);
			expectVectorNear(result.force_frd_n, Vector3f{collective * std::sin(angle), 0.f, -collective * std::cos(angle)});
			EXPECT_NEAR(result.collective_n, collective, tolerance);
			EXPECT_FALSE(result.collective_limited);
			EXPECT_NEAR(result.strong_retention, 1.f, tolerance);
			EXPECT_NEAR(result.weak_retention, 1.f, tolerance);
		}
	}
}

TEST(SharedTiltControl, LimitsWeakTorqueAlongTiltedThrustAxisFirst)
{
	const VehicleConfig config = planarQuad(8.f);
	constexpr float angle = .4f;
	const MotorThrust unconstrained{9.5f, 6.5f, .5f, -.5f};
	const Vector3f requested = netTorque(config, angle, unconstrained);
	AllocationResult result{};
	ASSERT_EQ(allocate(config, angle, 16.f, requested, result), Status::Ok);
	expectWithinLimits(config, result.thrust_n);
	EXPECT_NEAR(result.collective_n, 16.f, tolerance);
	EXPECT_NEAR(result.strong_retention, 1.f, tolerance);
	EXPECT_NEAR(result.weak_retention, .625f, tolerance);
	const MotorThrust expected{8.f, 5.f, 2.f, 1.f};
	expectVectorNear(result.thrust_n, expected);
	expectVectorNear(result.torque_frd_nm, netTorque(config, angle, expected));
	const Vector3f lost = requested - result.torque_frd_nm;
	EXPECT_NEAR(lost.cross(thrustAxis(angle)).norm(), 0.f, tolerance);
}

TEST(SharedTiltControl, FeasibleWeakTorqueIntervalCanExcludeBothEndpoints)
{
	const VehicleConfig config = planarQuad(6.f);
	// f(k)=(4+2k, 4+2k, 7-2k, 1-2k); only k=0.5 is feasible.
	const Vector3f requested = netTorque(config, 0.f, MotorThrust{6.f, 6.f, 5.f, -1.f});
	AllocationResult result{};
	ASSERT_EQ(allocate(config, 0.f, 16.f, requested, result), Status::Ok);
	expectWithinLimits(config, result.thrust_n);
	EXPECT_NEAR(result.strong_retention, 1.f, tolerance);
	EXPECT_NEAR(result.weak_retention, .5f, tolerance);
	expectVectorNear(result.thrust_n, MotorThrust{5.f, 5.f, 6.f, 0.f});
}

TEST(SharedTiltControl, ReducesStrongTorqueWhenFullRetentionIsInfeasible)
{
	const VehicleConfig config = planarQuad(6.f);
	const Vector3f requested = netTorque(config, 0.f, MotorThrust{6.f, 6.f, 9.f, -5.f});
	AllocationResult result{};
	ASSERT_EQ(allocate(config, 0.f, 16.f, requested, result), Status::Ok);
	expectWithinLimits(config, result.thrust_n);
	EXPECT_NEAR(result.collective_n, 16.f, tolerance);
	EXPECT_NEAR(result.strong_retention, 3.f / 7.f, tolerance);
	EXPECT_NEAR(result.weak_retention, .5f, tolerance);
	expectVectorNear(result.thrust_n, MotorThrust{5.f, 5.f, 6.f, 0.f});
}

TEST(SharedTiltControl, ClipsCollectiveAtUnequalMotorLimitsAndReconstructsWrench)
{
	VehicleConfig config = gazeboQuad();
	config.rotors[2].min_thrust_n = .3f;
	config.rotors[3].max_thrust_n = 8.f;
	constexpr float angle = .4f;
	for (const float requested : {-10.f, 100.f}) {
		SCOPED_TRACE(requested);
		AllocationResult result{};
		ASSERT_EQ(allocate(config, angle, requested, Vector3f{}, result), Status::Ok);
		EXPECT_TRUE(result.collective_limited);
		expectWithinLimits(config, result.thrust_n);
		float collective = 0.f;

		for (size_t i = 0; i < config.rotors.size(); ++i) {
			const auto &rotor = config.rotors[i];
			const float expected = requested < 0.f ? rotor.min_thrust_n : rotor.max_thrust_n;
			EXPECT_NEAR(result.thrust_n(i), expected, tolerance);
			collective += expected;
		}

		EXPECT_NEAR(result.collective_n, collective, tolerance);
		expectVectorNear(result.torque_frd_nm, netTorque(config, angle, result.thrust_n));
	}
}

TEST(SharedTiltControl, RejectsSingularGeometryAndInvalidThrustBounds)
{
	VehicleConfig config = planarQuad();
	AllocationResult result{};
	config.rotors[3] = config.rotors[0];
	EXPECT_EQ(allocate(config, 0.f, 16.f, Vector3f{}, result), Status::SingularGeometry);
	config = planarQuad();
	config.rotors[0].min_thrust_n = 31.f;
	EXPECT_EQ(allocate(config, 0.f, 16.f, Vector3f{}, result), Status::InvalidInput);
	config = planarQuad();
	EXPECT_EQ(allocate(config, 0.f, std::numeric_limits<float>::quiet_NaN(), Vector3f{}, result), Status::InvalidInput);
}
