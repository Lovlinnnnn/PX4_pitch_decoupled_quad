// SPDX-License-Identifier: BSD-3-Clause

#include "tilting_tunnel.hpp"

#include <gtest/gtest.h>

namespace
{

using Payload = uint8_t[tilting_tunnel::feedback_size];

void makeFeedback(Payload &payload, uint32_t sequence, uint32_t age, float angle, uint32_t flags)
{
	for (unsigned i = 0; i < sizeof(uint32_t); ++i) {
		payload[i] = static_cast<uint8_t>(sequence >> (8 * i));
		payload[i + 4] = static_cast<uint8_t>(age >> (8 * i));
		payload[i + 12] = static_cast<uint8_t>(flags >> (8 * i));
	}

	tilting_tunnel::writeAngle(angle, payload + 8);
}

} // namespace

TEST(TiltingTunnel, WireAngles)
{
	uint8_t command[tilting_tunnel::angle_size]{};
	tilting_tunnel::writeAngle(10.f, command);
	EXPECT_EQ(command[0], 0x00);
	EXPECT_EQ(command[1], 0x00);
	EXPECT_EQ(command[2], 0x20);
	EXPECT_EQ(command[3], 0x41);

	uint8_t pitch[] {0xc2, 0xb8, 0x32, 0x3e};
	float pitch_rad = 0.f;
	EXPECT_TRUE(tilting_tunnel::decodePitch(pitch, sizeof(pitch), pitch_rad));
	EXPECT_FLOAT_EQ(pitch_rad, 0.174532925f);
	EXPECT_FALSE(tilting_tunnel::decodePitch(pitch, sizeof(pitch) - 1, pitch_rad));

	tilting_tunnel::writeAngle(10.f, pitch);
	EXPECT_FALSE(tilting_tunnel::decodePitch(pitch, sizeof(pitch), pitch_rad));
	tilting_tunnel::writeAngle(std::numeric_limits<float>::quiet_NaN(), pitch);
	EXPECT_FALSE(tilting_tunnel::decodePitch(pitch, sizeof(pitch), pitch_rad));
}

TEST(TiltingTunnel, FeedbackFaultRetainsPreviousSample)
{
	tilting_tunnel::Feedback status;
	const uint8_t valid[] {0x09, 0, 0, 0, 0x20, 0x4e, 0, 0, 0xc2, 0xb8, 0x32, 0x3e, 1, 0, 0, 0};
	ASSERT_TRUE(tilting_tunnel::updateFeedback(valid, sizeof(valid), 500000, status));
	EXPECT_TRUE(status.valid);
	EXPECT_FALSE(status.fault);
	EXPECT_EQ(status.sequence, 9u);
	EXPECT_EQ(status.timestamp_sample, 480000u);
	EXPECT_FLOAT_EQ(status.angle_rad, 0.174532925f);

	Payload invalid{};
	makeFeedback(invalid, 10, 0, 0.f, tilting_tunnel::fault_flag);
	ASSERT_TRUE(tilting_tunnel::updateFeedback(invalid, sizeof(invalid), 520000, status));
	EXPECT_FALSE(status.valid);
	EXPECT_TRUE(status.fault);
	EXPECT_EQ(status.timestamp, 520000u);
	EXPECT_EQ(status.timestamp_sample, 480000u);
	EXPECT_FLOAT_EQ(status.angle_rad, 0.174532925f);
}

TEST(TiltingTunnel, FeedbackRejectsMalformedStaleAndReordered)
{
	tilting_tunnel::Feedback status;
	Payload payload{};
	makeFeedback(payload, 20, 0, 0.1f, tilting_tunnel::valid_flag);
	EXPECT_FALSE(tilting_tunnel::updateFeedback(payload, sizeof(payload) - 1, 600000, status));
	ASSERT_TRUE(tilting_tunnel::updateFeedback(payload, sizeof(payload), 600000, status));
	EXPECT_FALSE(tilting_tunnel::updateFeedback(payload, sizeof(payload), 620000, status));

	makeFeedback(payload, 19, 0, 0.1f, tilting_tunnel::valid_flag);
	EXPECT_FALSE(tilting_tunnel::updateFeedback(payload, sizeof(payload), 620000, status));
	makeFeedback(payload, 21, 20000, 0.1f, tilting_tunnel::valid_flag);
	EXPECT_FALSE(tilting_tunnel::updateFeedback(payload, sizeof(payload), 620000, status));
	makeFeedback(payload, 21, 300001, 0.1f, tilting_tunnel::valid_flag);
	EXPECT_FALSE(tilting_tunnel::updateFeedback(payload, sizeof(payload), 1000000, status));
	makeFeedback(payload, 21, 0, 0.1f, 4);
	EXPECT_FALSE(tilting_tunnel::updateFeedback(payload, sizeof(payload), 620000, status));
	makeFeedback(payload, 21, 0, 2.f, tilting_tunnel::valid_flag);
	EXPECT_FALSE(tilting_tunnel::updateFeedback(payload, sizeof(payload), 620000, status));
	makeFeedback(payload, 21, 0, std::numeric_limits<float>::infinity(), tilting_tunnel::valid_flag);
	EXPECT_FALSE(tilting_tunnel::updateFeedback(payload, sizeof(payload), 620000, status));
	EXPECT_EQ(status.sequence, 20u);
}

TEST(TiltingTunnel, SequenceWrapAndSessionReset)
{
	tilting_tunnel::Feedback status;
	Payload payload{};
	makeFeedback(payload, UINT32_MAX, 0, 0.1f, tilting_tunnel::valid_flag);
	ASSERT_TRUE(tilting_tunnel::updateFeedback(payload, sizeof(payload), 700000, status));
	makeFeedback(payload, 0, 0, 0.1f, tilting_tunnel::valid_flag);
	ASSERT_TRUE(tilting_tunnel::updateFeedback(payload, sizeof(payload), 720000, status));
	EXPECT_FALSE(tilting_tunnel::updateFeedback(payload, sizeof(payload), 740000, status));
	EXPECT_TRUE(tilting_tunnel::updateFeedback(payload, sizeof(payload), 1720001, status));

	makeFeedback(payload, 0, 0, 0.f, tilting_tunnel::fault_flag);
	EXPECT_TRUE(tilting_tunnel::updateFeedback(payload, sizeof(payload), 1000, status));
	EXPECT_EQ(status.timestamp_sample, 0u);
	EXPECT_TRUE(std::isnan(status.angle_rad));
	EXPECT_FALSE(status.valid);
}
