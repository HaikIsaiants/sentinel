#include <sentinel/protocol/framing.hpp>
#include <sentinel/v1/sentinel.pb.h>

#include <gtest/gtest.h>

#include <sstream>
#include <stdexcept>

TEST(Framing, RoundTripsEnvelopeMetadata) {
    sentinel::v1::Envelope expected;
    expected.set_schema_version(1);
    expected.set_sequence(9);
    expected.set_simulation_time_ms(250);
    expected.set_sender_id("sim");
    expected.set_recipient_id("agent-a");

    std::stringstream stream(std::ios::in | std::ios::out | std::ios::binary);
    sentinel::protocol::write_frame(stream, expected);

    sentinel::v1::Envelope actual;
    ASSERT_TRUE(sentinel::protocol::read_frame(stream, actual));
    EXPECT_EQ(sentinel::protocol::deterministic_bytes(expected),
              sentinel::protocol::deterministic_bytes(actual));
    EXPECT_FALSE(sentinel::protocol::read_frame(stream, actual));
}

TEST(Framing, RejectsIncompleteHeader) {
    std::stringstream stream(std::string("\0\0\0", 3), std::ios::in | std::ios::binary);
    sentinel::v1::Envelope message;
    EXPECT_THROW(sentinel::protocol::read_frame(stream, message), std::runtime_error);
}

TEST(Framing, RejectsIncompletePayload) {
    std::string bytes{"\0\0\0\4", 4};
    bytes += "abc";
    std::stringstream stream(bytes, std::ios::in | std::ios::binary);
    sentinel::v1::Envelope message;
    EXPECT_THROW(sentinel::protocol::read_frame(stream, message), std::runtime_error);
}
