// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include "ActuatorEffectivenessMultirotor.hpp"

class ActuatorEffectivenessSharedTilt : public ActuatorEffectivenessMultirotor
{
public:
	using ActuatorEffectivenessMultirotor::ActuatorEffectivenessMultirotor;

	bool getEffectivenessMatrix(Configuration &configuration, EffectivenessUpdateReason reason) override;
	const char *name() const override { return "Shared-tilt multirotor"; }

	static matrix::Vector3f thrustAxis(float tilt_rad);
	void setTilt(float tilt_rad) { _requested_tilt_rad = tilt_rad; }
	float matrixTilt() const { return _matrix_tilt_rad; }

private:
	float _requested_tilt_rad{0.f};
	float _matrix_tilt_rad{NAN};
};
