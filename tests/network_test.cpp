#include <sentinel/core/network.hpp>

#include <gtest/gtest.h>

namespace {

sentinel::v1::NetworkMessage message(std::uint64_t version) {
    sentinel::v1::NetworkMessage result;
    result.set_sender_id("agent-a");
    result.set_recipient_id("agent-b");
    result.set_version(version);
    result.set_payload("allocation-state");
    return result;
}

}

TEST(Network, DeliversOnlyAfterTheConfiguredLatency) {
    sentinel::core::NetworkEmulator network(17, 100);
    sentinel::v1::NetworkProfile profile;
    profile.set_id("delayed");
    profile.set_latency_ticks(2);
    network.set_profile(profile);

    const auto queued = network.step(0, {message(1)});
    ASSERT_EQ(queued.outcomes.size(), 1U);
    EXPECT_EQ(queued.outcomes[0].disposition(), sentinel::v1::NETWORK_DISPOSITION_QUEUED);
    EXPECT_TRUE(network.step(1, {}).delivered.empty());
    EXPECT_TRUE(network.step(2, {}).delivered.empty());
    const auto delivered = network.step(3, {});
    ASSERT_EQ(delivered.delivered.size(), 1U);
    EXPECT_EQ(delivered.delivered[0].version(), 1);
}

TEST(Network, OrdersSimultaneousDeliveryCanonically) {
    sentinel::core::NetworkEmulator network(21, 100);
    sentinel::v1::NetworkProfile profile;
    profile.set_id("local");
    network.set_profile(profile);
    auto second = message(2);
    second.set_sender_id("agent-z");
    auto first = message(1);
    first.set_sender_id("agent-a");
    network.step(0, {second, first});
    const auto delivered = network.step(1, {});
    ASSERT_EQ(delivered.delivered.size(), 2U);
    EXPECT_EQ(delivered.delivered[0].sender_id(), "agent-a");
    EXPECT_EQ(network.communication_messages(), 2);
    EXPECT_EQ(network.delivered_messages(), 2);
}

TEST(Network, IncludesQueuedStateInTheHash) {
    sentinel::core::NetworkEmulator first(9, 100);
    sentinel::core::NetworkEmulator second(9, 100);
    sentinel::v1::NetworkProfile profile;
    profile.set_id("delayed");
    profile.set_latency_ticks(4);
    first.set_profile(profile);
    second.set_profile(profile);
    first.step(0, {message(1)});
    second.step(0, {message(1)});
    sentinel::core::HashBuilder left;
    sentinel::core::HashBuilder right;
    first.append_hash(left);
    second.append_hash(right);
    EXPECT_EQ(left.finish(), right.finish());
}

TEST(Network, RejectsAZeroDurationStep) {
    EXPECT_THROW(
        sentinel::core::NetworkEmulator(9, 0),
        std::invalid_argument);
}

TEST(Network, RequiresAProfileIdentifier) {
    sentinel::core::NetworkEmulator network(9, 100);
    sentinel::v1::NetworkProfile profile;
    EXPECT_THROW(network.set_profile(profile), std::invalid_argument);
}

TEST(Network, RequiresAMessageSender) {
    sentinel::core::NetworkEmulator network(9, 100);
    sentinel::v1::NetworkProfile profile;
    profile.set_id("local");
    network.set_profile(profile);
    auto invalid = message(1);
    invalid.clear_sender_id();
    EXPECT_THROW(network.step(0, {invalid}), std::invalid_argument);
}

TEST(Network, RequiresAMessageRecipient) {
    sentinel::core::NetworkEmulator network(9, 100);
    sentinel::v1::NetworkProfile profile;
    profile.set_id("local");
    network.set_profile(profile);
    auto invalid = message(1);
    invalid.clear_recipient_id();
    EXPECT_THROW(network.step(0, {invalid}), std::invalid_argument);
}

TEST(Network, RejectsLoopbackMessages) {
    sentinel::core::NetworkEmulator network(9, 100);
    sentinel::v1::NetworkProfile profile;
    profile.set_id("local");
    network.set_profile(profile);
    auto invalid = message(1);
    invalid.set_recipient_id("agent-a");
    EXPECT_THROW(network.step(0, {invalid}), std::invalid_argument);
}

TEST(Network, EmitsCompleteQueueOutcomeMetadata) {
    sentinel::core::NetworkEmulator network(31, 100);
    sentinel::v1::NetworkProfile profile;
    profile.set_id("delayed");
    profile.set_latency_ticks(2);
    network.set_profile(profile);

    const auto step = network.step(4, {message(8)});
    ASSERT_EQ(step.outcomes.size(), 1U);
    const auto& outcome = step.outcomes.front();
    EXPECT_EQ(outcome.sequence(), 1);
    EXPECT_EQ(
        outcome.disposition(),
        sentinel::v1::NETWORK_DISPOSITION_QUEUED);
    EXPECT_EQ(outcome.tick(), 4);
    EXPECT_EQ(outcome.enqueue_tick(), 4);
    EXPECT_EQ(outcome.transmit_start_tick(), 4);
    EXPECT_EQ(outcome.transmit_end_tick(), 4);
    EXPECT_EQ(outcome.delivery_tick(), 7);
    EXPECT_EQ(outcome.profile_id(), "delayed");
    EXPECT_EQ(outcome.message().version(), 8);
    EXPECT_GT(outcome.serialized_bytes(), 0);
}

TEST(Network, EmitsDeliveryOutcomeMetadata) {
    sentinel::core::NetworkEmulator network(31, 100);
    sentinel::v1::NetworkProfile profile;
    profile.set_id("local");
    network.set_profile(profile);
    network.step(0, {message(3)});

    const auto step = network.step(1, {});
    ASSERT_EQ(step.outcomes.size(), 1U);
    const auto& outcome = step.outcomes.front();
    EXPECT_EQ(
        outcome.disposition(),
        sentinel::v1::NETWORK_DISPOSITION_DELIVERED);
    EXPECT_EQ(outcome.tick(), 1);
    EXPECT_EQ(outcome.enqueue_tick(), 0);
    EXPECT_EQ(outcome.delivery_tick(), 1);
    EXPECT_EQ(outcome.message().version(), 3);
}

TEST(Network, AssignsSequencesInAdmissionOrder) {
    sentinel::core::NetworkEmulator network(31, 100);
    sentinel::v1::NetworkProfile profile;
    profile.set_id("local");
    network.set_profile(profile);
    const auto first = network.step(0, {message(1), message(2)});
    const auto second = network.step(1, {message(3)});

    ASSERT_EQ(first.outcomes.size(), 2U);
    ASSERT_EQ(second.outcomes.size(), 3U);
    EXPECT_EQ(first.outcomes[0].sequence(), 1);
    EXPECT_EQ(first.outcomes[1].sequence(), 2);
    EXPECT_EQ(second.outcomes[0].sequence(), 3);
}

TEST(Network, AccumulatesSerializedTrafficCounters) {
    sentinel::core::NetworkEmulator network(31, 100);
    sentinel::v1::NetworkProfile profile;
    profile.set_id("local");
    network.set_profile(profile);
    const auto first = network.step(0, {message(1)});
    const auto bytes = first.outcomes.front().serialized_bytes();
    network.step(1, {message(2)});

    EXPECT_EQ(network.communication_messages(), 2);
    EXPECT_EQ(network.communication_bytes(), bytes * 2);
    EXPECT_EQ(network.delivered_messages(), 1);
    network.step(2, {});
    EXPECT_EQ(network.delivered_messages(), 2);
}

TEST(Network, SortsByRecipientBeforeSenderAndVersion) {
    sentinel::core::NetworkEmulator network(31, 100);
    sentinel::v1::NetworkProfile profile;
    profile.set_id("local");
    network.set_profile(profile);
    auto z_to_b = message(3);
    z_to_b.set_sender_id("agent-z");
    auto a_to_c = message(2);
    a_to_c.set_recipient_id("agent-c");
    auto a_to_b = message(4);
    network.step(0, {z_to_b, a_to_c, a_to_b});

    const auto delivered = network.step(1, {}).delivered;
    ASSERT_EQ(delivered.size(), 3U);
    EXPECT_EQ(delivered[0].recipient_id(), "agent-b");
    EXPECT_EQ(delivered[0].sender_id(), "agent-a");
    EXPECT_EQ(delivered[1].recipient_id(), "agent-b");
    EXPECT_EQ(delivered[1].sender_id(), "agent-z");
    EXPECT_EQ(delivered[2].recipient_id(), "agent-c");
}

TEST(Network, DoesNotDeliverNewTrafficInItsAdmissionTick) {
    sentinel::core::NetworkEmulator network(31, 100);
    sentinel::v1::NetworkProfile profile;
    profile.set_id("local");
    network.set_profile(profile);
    const auto admitted = network.step(7, {message(1)});
    EXPECT_TRUE(admitted.delivered.empty());
    EXPECT_TRUE(network.step(7, {}).delivered.empty());
    EXPECT_EQ(network.step(8, {}).delivered.size(), 1U);
}

TEST(Network, HashChangesWhenQueuedTrafficChanges) {
    sentinel::core::NetworkEmulator first(9, 100);
    sentinel::core::NetworkEmulator second(9, 100);
    sentinel::v1::NetworkProfile profile;
    profile.set_id("delayed");
    profile.set_latency_ticks(4);
    first.set_profile(profile);
    second.set_profile(profile);
    first.step(0, {message(1)});
    second.step(0, {message(2)});
    sentinel::core::HashBuilder left;
    sentinel::core::HashBuilder right;
    first.append_hash(left);
    second.append_hash(right);
    EXPECT_NE(left.finish(), right.finish());
}

TEST(Network, HashChangesWithTheProfile) {
    sentinel::core::NetworkEmulator first(9, 100);
    sentinel::core::NetworkEmulator second(9, 100);
    sentinel::v1::NetworkProfile local;
    local.set_id("local");
    sentinel::v1::NetworkProfile delayed;
    delayed.set_id("delayed");
    delayed.set_latency_ticks(2);
    first.set_profile(local);
    second.set_profile(delayed);
    sentinel::core::HashBuilder left;
    sentinel::core::HashBuilder right;
    first.append_hash(left);
    second.append_hash(right);
    EXPECT_NE(left.finish(), right.finish());
}
