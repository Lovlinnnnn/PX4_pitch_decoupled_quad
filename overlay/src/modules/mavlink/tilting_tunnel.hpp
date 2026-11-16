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

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>

namespace tilting_tunnel
{

static constexpr uint16_t command_type = 32778;
static constexpr uint16_t pitch_type = 32779;
static constexpr uint16_t feedback_type = 32780;
static constexpr uint8_t angle_size = 4;
static constexpr uint8_t feedback_size = 16;
static constexpr uint32_t valid_flag = 1;
static constexpr uint32_t fault_flag = 2;
static constexpr float command_limit_deg = 35.f;
static constexpr float pitch_limit_rad = 1.57079632679f;
static constexpr uint32_t sample_timeout_us = 300000;
static constexpr uint32_t session_timeout_us = 1000000;

static_assert(sizeof(float) == sizeof(uint32_t) && std::numeric_limits<float>::is_iec559,
	      "Tilting TUNNEL requires IEEE-754 float32");

inline uint32_t readUint32(const uint8_t *bytes)
{
	return static_cast<uint32_t>(bytes[0])
	       | (static_cast<uint32_t>(bytes[1]) << 8)
	       | (static_cast<uint32_t>(bytes[2]) << 16)
	       | (static_cast<uint32_t>(bytes[3]) << 24);
}

inline float readAngle(const uint8_t *bytes)
{
	const uint32_t bits = readUint32(bytes);
	float angle;
	std::memcpy(&angle, &bits, sizeof(angle));
	return angle;
}

inline void writeAngle(float angle, uint8_t *bytes)
{
	uint32_t bits;
	std::memcpy(&bits, &angle, sizeof(bits));

	for (unsigned i = 0; i < sizeof(bits); ++i) {
		bytes[i] = static_cast<uint8_t>(bits >> (8 * i));
	}
}

inline bool decodePitch(const uint8_t *bytes, uint8_t length, float &pitch_rad)
{
	if (length != angle_size) {
		return false;
	}

	pitch_rad = readAngle(bytes);
	return std::isfinite(pitch_rad) && std::fabs(pitch_rad) <= pitch_limit_rad;
}

struct Feedback {
	uint64_t timestamp{};
	uint64_t timestamp_sample{};
	float angle_rad{std::numeric_limits<float>::quiet_NaN()};
	uint32_t sequence{};
	bool valid{};
	bool fault{};
};

inline bool updateFeedback(const uint8_t *bytes, uint8_t length, uint64_t receipt_time, Feedback &feedback)
{
	if (length != feedback_size || receipt_time == 0) {
		return false;
	}

	const uint32_t sequence = readUint32(bytes);
	const uint32_t age = readUint32(bytes + 4);
	const float angle = readAngle(bytes + 8);
	const uint32_t flags = readUint32(bytes + 12);
	const bool fault = (flags & fault_flag) != 0;
	const bool valid = (flags & valid_flag) != 0 && !fault;

	if ((flags & ~(valid_flag | fault_flag)) != 0 || !std::isfinite(angle)
	    || std::fabs(angle) >= pitch_limit_rad || age > session_timeout_us || age > receipt_time
	    || (valid && age > sample_timeout_us)) {
		return false;
	}

	// The simulator can reset HRT. Start a new session rather than preserving data from the old epoch.
	const bool clock_reset = receipt_time < feedback.timestamp;

	if (feedback.timestamp != 0 && !clock_reset && receipt_time - feedback.timestamp <= session_timeout_us) {
		const uint32_t advance = sequence - feedback.sequence;

		if (advance == 0 || advance >= 0x80000000u) {
			return false;
		}
	}

	const uint64_t sample_time = receipt_time - age;

	if (valid && (sample_time == 0 || (!clock_reset && sample_time <= feedback.timestamp_sample))) {
		return false;
	}

	if (clock_reset) {
		feedback = Feedback{};
	}

	feedback.timestamp = receipt_time;
	feedback.sequence = sequence;
	feedback.valid = valid;
	feedback.fault = fault;

	if (valid) {
		feedback.timestamp_sample = sample_time;
		feedback.angle_rad = angle;
	}

	return true;
}

} // namespace tilting_tunnel
