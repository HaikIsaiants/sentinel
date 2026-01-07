#include <sentinel/core/rng.hpp>

#include <gtest/gtest.h>

TEST(Rng, RepeatsAStreamExactly) {
    sentinel::core::DeterministicRng first(73);
    sentinel::core::DeterministicRng second(73);
    for (int index = 0; index < 32; ++index) {
        EXPECT_EQ(first.next(), second.next());
    }
}

TEST(Rng, NamesCreateIndependentStreams) {
    sentinel::core::RngStreams streams(91);
    const auto first = streams.stream("events").next();
    const auto second = streams.stream("network").next();
    EXPECT_NE(first, second);
    EXPECT_EQ(streams.states().size(), 2U);
}

TEST(Rng, UniformIncludesBothEndpoints) {
    sentinel::core::DeterministicRng rng(11);
    for (int index = 0; index < 100; ++index) {
        const auto value = rng.uniform(-4, 7);
        EXPECT_GE(value, -4);
        EXPECT_LE(value, 7);
    }
}
