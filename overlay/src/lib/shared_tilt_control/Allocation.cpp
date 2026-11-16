// SPDX-License-Identifier: BSD-3-Clause

#include "SharedTiltControl.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace shared_tilt_control
{
namespace
{
using Vector4d = matrix::Vector<double, 4>;
using Vector3d = matrix::Vector3<double>;
using Matrix4d = matrix::SquareMatrix<double, 4>;

constexpr size_t motor_count = 4;
constexpr double roundoff = 64.0 * std::numeric_limits<double>::epsilon();
constexpr double max_scaled_condition = 1e5;

Vector4d wrench(double collective, const Vector3d &torque)
{
	Vector4d result;
	result(0) = collective;

	for (size_t i = 0; i < 3; ++i) {
		result(i + 1) = torque(i);
	}

	return result;
}

bool invertEffectiveness(const matrix::SquareMatrix<float, 4> &effectiveness, Matrix4d &inverse)
{
	// Normalize the rows before assessing conditioning: the collective and moment rows have different units.
	Matrix4d scaled;
	Vector4d row_scale;

	for (size_t row = 0; row < motor_count; ++row) {
		double row_total = 0.0;

		for (size_t col = 0; col < motor_count; ++col) {
			row_total += std::fabs(static_cast<double>(effectiveness(row, col)));
		}

		if (!std::isfinite(row_total) || row_total <= 0.0) {
			return false;
		}

		row_scale(row) = row_total;

		for (size_t col = 0; col < motor_count; ++col) {
			scaled(row, col) = static_cast<double>(effectiveness(row, col)) / row_total;
		}
	}

	if (!matrix::inv(scaled, inverse)) {
		return false;
	}

	for (size_t row = 0; row < motor_count; ++row) {
		double row_total = 0.0;

		for (size_t col = 0; col < motor_count; ++col) {
			row_total += std::fabs(inverse(row, col));
			inverse(row, col) /= row_scale(col);
		}

		// Each normalized input row has sum of absolute values 1.
		if (!std::isfinite(row_total) || row_total > max_scaled_condition) {
			return false;
		}
	}

	return inverse.isAllFinite();
}

bool withinBounds(const Vector4d &values, const Vector4d &lower, const Vector4d &upper)
{
	if (!values.isAllFinite()) {
		return false;
	}

	for (size_t i = 0; i < motor_count; ++i) {
		const double tolerance = roundoff * std::max(1.0, upper(i));

		if (values(i) < lower(i) - tolerance || values(i) > upper(i) + tolerance) {
			return false;
		}
	}

	return true;
}

struct Retention {
	double strong;
	double weak;
};

struct Constraint {
	double strong;
	double weak;
	double limit;
};

Retention maximizeRetention(const Vector4d &trim, const Vector4d &strong, const Vector4d &weak,
			    const Vector4d &lower, const Vector4d &upper)
{
	// Each motor adds two half-planes to the unit square. A lexicographic optimum is a vertex.
	std::array<Constraint, 4 + 2 * motor_count> edges{{{1.0, 0.0, 1.0}, {-1.0, 0.0, 0.0},
			{0.0, 1.0, 1.0}, {0.0, -1.0, 0.0}}};

	for (size_t i = 0; i < motor_count; ++i) {
		edges[4 + 2 * i] = {strong(i), weak(i), upper(i) - trim(i)};
		edges[5 + 2 * i] = {-strong(i), -weak(i), trim(i) - lower(i)};
	}

	Retention best{0.0, 0.0}; // The trim point is feasible by construction.

	for (size_t i = 0; i < edges.size(); ++i) {
		const Constraint &a = edges[i];

		for (size_t j = i + 1; j < edges.size(); ++j) {
			const Constraint &b = edges[j];
			const double determinant = a.strong * b.weak - a.weak * b.strong;

			if (std::fabs(determinant) < std::numeric_limits<double>::min()) {
				continue;
			}

			const double lambda = (a.limit * b.weak - a.weak * b.limit) / determinant;
			const double kappa = (a.strong * b.limit - a.limit * b.strong) / determinant;

			if (!std::isfinite(lambda) || !std::isfinite(kappa) || lambda < -roundoff || lambda > 1.0 + roundoff
			    || kappa < -roundoff || kappa > 1.0 + roundoff) {
				continue;
			}

			const Retention candidate{std::min(std::max(lambda, 0.0), 1.0), std::min(std::max(kappa, 0.0), 1.0)};
			const bool better = candidate.strong > best.strong + roundoff
					    || (std::fabs(candidate.strong - best.strong) <= roundoff && candidate.weak > best.weak);

			if (better && withinBounds(trim + strong * candidate.strong + weak * candidate.weak, lower, upper)) {
				best = candidate;
			}
		}
	}

	return best;
}
} // namespace

Status allocate(const VehicleConfig &config, float delta_rad, float collective_n, const matrix::Vector3f &torque_frd_nm,
		AllocationResult &result)
{
	result = {};

	if (!std::isfinite(collective_n) || !torque_frd_nm.isAllFinite()) {
		return Status::InvalidInput;
	}

	matrix::SquareMatrix<float, 4> effectiveness;
	const Status geometry_status = buildEffectiveness(config, delta_rad, effectiveness);

	if (geometry_status != Status::Ok) {
		return geometry_status;
	}

	Matrix4d inverse;

	if (!invertEffectiveness(effectiveness, inverse)) {
		return Status::SingularGeometry;
	}

	Vector4d lower;
	Vector4d upper;
	double minimum_collective = 0.0;
	double maximum_collective = 0.0;

	for (size_t i = 0; i < motor_count; ++i) {
		const Rotor &rotor = config.rotors[i];

		if (!std::isfinite(rotor.min_thrust_n) || !std::isfinite(rotor.max_thrust_n) || rotor.min_thrust_n < 0.0f
		    || rotor.max_thrust_n < rotor.min_thrust_n) {
			return Status::InvalidInput;
		}

		lower(i) = rotor.min_thrust_n;
		upper(i) = rotor.max_thrust_n;
		minimum_collective += lower(i);
		maximum_collective += upper(i);
	}

	const double collective = std::min(std::max(static_cast<double>(collective_n), minimum_collective), maximum_collective);
	const Vector3d requested_torque{torque_frd_nm(0), torque_frd_nm(1), torque_frd_nm(2)};
	const matrix::Vector3f axis = thrustAxis(delta_rad);
	Vector4d thrust = inverse * wrench(collective, requested_torque);
	Retention retention{1.0, 1.0};

	if (!withinBounds(thrust, lower, upper)) {
		const double available = maximum_collective - minimum_collective;
		const double fraction = available > 0.0 ? (collective - minimum_collective) / available : 0.0;
		const Vector4d trim = lower + (upper - lower) * fraction;
		Vector3d trim_torque;

		for (size_t i = 0; i < motor_count; ++i) {
			for (size_t j = 0; j < 3; ++j) {
				trim_torque(j) += static_cast<double>(effectiveness(j + 1, i)) * trim(i);
			}
		}

		const Vector3d weak_axis = Vector3d{axis(0), axis(1), axis(2)}.unit();
		const Vector3d torque_delta = requested_torque - trim_torque;
		const Vector3d weak_torque = weak_axis * torque_delta.dot(weak_axis);
		const Vector4d strong = inverse * wrench(0.0, torque_delta - weak_torque);
		const Vector4d weak = inverse * wrench(0.0, weak_torque);

		if (!strong.isAllFinite() || !weak.isAllFinite()) {
			return Status::NumericalFailure;
		}

		retention = maximizeRetention(trim, strong, weak, lower, upper);
		thrust = trim + strong * retention.strong + weak * retention.weak;
	}

	if (!withinBounds(thrust, lower, upper)) {
		return Status::NumericalFailure;
	}

	AllocationResult allocation{};
	Vector4d achieved;

	for (size_t i = 0; i < motor_count; ++i) {
		allocation.thrust_n(i) = static_cast<float>(std::min(std::max(thrust(i), lower(i)), upper(i)));

		for (size_t row = 0; row < motor_count; ++row) {
			achieved(row) += static_cast<double>(effectiveness(row, i)) * static_cast<double>(allocation.thrust_n(i));
		}
	}

	const double collective_tolerance = 8.0 * static_cast<double>(std::numeric_limits<float>::epsilon())
					    * std::max(1.0, collective);
	if (std::fabs(achieved(0) - collective) > collective_tolerance) {
		return Status::NumericalFailure;
	}

	allocation.collective_n = static_cast<float>(achieved(0));
	allocation.force_frd_n = axis * allocation.collective_n;
	allocation.torque_frd_nm = matrix::Vector3f{static_cast<float>(achieved(1)), static_cast<float>(achieved(2)),
						   static_cast<float>(achieved(3))};
	allocation.strong_retention = static_cast<float>(retention.strong);
	allocation.weak_retention = static_cast<float>(retention.weak);
	allocation.collective_limited = static_cast<double>(collective_n) < minimum_collective
					|| static_cast<double>(collective_n) > maximum_collective;

	if (!allocation.force_frd_n.isAllFinite() || !allocation.torque_frd_nm.isAllFinite()) {
		return Status::NumericalFailure;
	}

	result = allocation;
	return Status::Ok;
}
} // namespace shared_tilt_control
