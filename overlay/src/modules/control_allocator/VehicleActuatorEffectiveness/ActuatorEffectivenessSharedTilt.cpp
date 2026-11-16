// SPDX-License-Identifier: BSD-3-Clause

#include "ActuatorEffectivenessSharedTilt.hpp"

#include <px4_platform_common/defines.h>

matrix::Vector3f
ActuatorEffectivenessSharedTilt::thrustAxis(float tilt_rad)
{
	return {sinf(tilt_rad), 0.f, -cosf(tilt_rad)};
}

bool
ActuatorEffectivenessSharedTilt::getEffectivenessMatrix(Configuration &configuration, EffectivenessUpdateReason reason)
{
	// Normalize PX4's mixer at zero tilt. Subsequent measured-angle updates keep those scales.
	const float tilt = reason == EffectivenessUpdateReason::CONFIGURATION_UPDATE ? 0.f : _requested_tilt_rad;

	if (reason == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE && fabsf(tilt - _matrix_tilt_rad) <= FLT_EPSILON) {
		return false;
	}

	auto geometry = _mc_rotors.geometry();
	_matrix_tilt_rad = NAN;

	if (geometry.num_rotors != 4) {
		return false;
	}

	const matrix::Vector3f axis = thrustAxis(tilt);

	for (int i = 0; i < geometry.num_rotors; ++i) {
		auto &rotor = geometry.rotors[i];

		if (!rotor.position.isAllFinite() || !PX4_ISFINITE(rotor.thrust_coef) || rotor.thrust_coef <= FLT_EPSILON
		    || !PX4_ISFINITE(rotor.moment_ratio)) {
			return false;
		}

		rotor.axis = axis;
	}

	const int count = ActuatorEffectivenessRotors::computeEffectivenessMatrix(geometry,
			  configuration.effectiveness_matrices[0]);
	configuration.actuatorsAdded(ActuatorType::MOTORS, count);
	_matrix_tilt_rad = tilt;
	return true;
}
