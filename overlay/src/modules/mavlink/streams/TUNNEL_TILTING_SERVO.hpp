/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

#include "../tilting_tunnel.hpp"

#include <lib/mathlib/mathlib.h>
#include <parameters/param.h>
#include <uORB/topics/tilting_servo_sp.h>

class MavlinkStreamTunnelTiltingServo : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamTunnelTiltingServo(mavlink); }
	static constexpr const char *get_name_static() { return "TUNNEL_TILTING_SERVO"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_TUNNEL; }
	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _setpoint_sub.advertised() ? MAVLINK_MSG_ID_TUNNEL_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamTunnelTiltingServo(Mavlink *mavlink) : MavlinkStream(mavlink),
		_command_limit_deg(commandLimitDeg()) {}

	static float commandLimitDeg()
	{
		int32_t autostart{};
		const bool shared_tilt_flight = param_get(param_find("SYS_AUTOSTART"), &autostart) == PX4_OK && autostart == 22012;
		return shared_tilt_flight ? 55.f : tilting_tunnel::command_limit_deg;
	}

	uORB::Subscription _setpoint_sub{ORB_ID(tilting_servo_setpoint)};
	const float _command_limit_deg;

	bool send() override
	{
		if (_mavlink->get_status()->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
			return false;
		}

		tilting_servo_sp_s setpoint{};

		if (!_setpoint_sub.update(&setpoint) || !PX4_ISFINITE(setpoint.angle[0])) {
			return false;
		}

		const float angle_deg = math::degrees(setpoint.angle[0]);

		if (!PX4_ISFINITE(angle_deg)) {
			return false;
		}

		mavlink_tunnel_t tunnel{};
		tunnel.target_system = _mavlink->get_system_id();
		tunnel.target_component = MAV_COMP_ID_ONBOARD_COMPUTER;
		tunnel.payload_type = tilting_tunnel::command_type;
		tunnel.payload_length = tilting_tunnel::angle_size;
		tilting_tunnel::writeAngle(math::constrain(angle_deg, -_command_limit_deg, _command_limit_deg), tunnel.payload);
		mavlink_msg_tunnel_send_struct(_mavlink->get_channel(), &tunnel);
		return true;
	}
};
