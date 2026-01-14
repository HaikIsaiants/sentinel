#include <sentinel/core/network.hpp>

#include <gtest/gtest.h>

#include <string>
#include <utility>

namespace {

sentinel::v1::NetworkProfile profile(
    std::uint64_t latency_ticks = 0) {
    sentinel::v1::NetworkProfile result;
    result.set_id(
        latency_ticks == 0 ? "local" : "delayed");
    result.set_latency_ticks(latency_ticks);
    return result;
}

sentinel::v1::NetworkMessage message(
    std::string sender, std::string recipient,
    std::uint64_t version) {
    sentinel::v1::NetworkMessage result;
    result.set_sender_id(std::move(sender));
    result.set_recipient_id(std::move(recipient));
    result.set_version(version);
    result.set_payload("allocation-state");
    return result;
}

sentinel::v1::NetworkMessage message(
    std::uint64_t version) {
    return message("agent-a", "agent-b", version);
}

}

TEST(Network, RequiresASelectedProfileBeforeStepping) {
    sentinel::core::NetworkEmulator network(17, 100);
    EXPECT_THROW(network.step(0, {}), std::logic_error);
}

TEST(Network, DeliversAfterTheSelectedLatency) {
    sentinel::core::NetworkEmulator network(17, 100);
    network.set_profile(profile(2));
    const auto admitted = network.step(0, {message(1)});
    ASSERT_EQ(admitted.outcomes.size(), 1U);
    EXPECT_EQ(
        admitted.outcomes.front().disposition(),
        sentinel::v1::NETWORK_DISPOSITION_QUEUED);
    EXPECT_TRUE(network.step(1, {}).delivered.empty());
    EXPECT_TRUE(network.step(2, {}).delivered.empty());
    const auto delivered = network.step(3, {});
    ASSERT_EQ(delivered.delivered.size(), 1U);
    EXPECT_EQ(delivered.delivered.front().version(), 1);
    ASSERT_EQ(delivered.outcomes.size(), 1U);
    EXPECT_EQ(
        delivered.outcomes.front().disposition(),
        sentinel::v1::NETWORK_DISPOSITION_DELIVERED);
}

TEST(Network, ReleasesDueTrafficBeforeRecordingNewTraffic) {
    sentinel::core::NetworkEmulator network(21, 100);
    network.set_profile(profile());
    network.step(0, {message(1)});
    const auto step = network.step(1, {message(2)});
    ASSERT_EQ(step.outcomes.size(), 2U);
    EXPECT_EQ(
        step.outcomes[0].disposition(),
        sentinel::v1::NETWORK_DISPOSITION_DELIVERED);
    EXPECT_EQ(step.outcomes[0].message().version(), 1);
    EXPECT_EQ(
        step.outcomes[1].disposition(),
        sentinel::v1::NETWORK_DISPOSITION_QUEUED);
    EXPECT_EQ(step.outcomes[1].message().version(), 2);
}

TEST(Network, RejectsTheWholeBatchBeforeMutatingCounters) {
    sentinel::core::NetworkEmulator network(21, 100);
    network.set_profile(profile());
    auto invalid = message(2);
    invalid.clear_recipient_id();
    EXPECT_THROW(
        network.step(0, {message(1), invalid}),
        std::invalid_argument);
    EXPECT_EQ(network.communication_messages(), 0);
    EXPECT_EQ(network.communication_bytes(), 0);
    EXPECT_EQ(network.delivered_messages(), 0);
    const auto accepted = network.step(0, {message(3)});
    EXPECT_EQ(accepted.outcomes.front().sequence(), 1);
}

TEST(Network, RejectsDuplicateMessagesWithinABatch) {
    sentinel::core::NetworkEmulator network(21, 100);
    network.set_profile(profile());
    EXPECT_THROW(
        network.step(0, {message(1), message(1)}),
        std::invalid_argument);
    EXPECT_EQ(network.communication_messages(), 0);
}

TEST(Network, AllowsDistinctVersionsBetweenTheSamePeers) {
    sentinel::core::NetworkEmulator network(21, 100);
    network.set_profile(profile());
    const auto accepted =
        network.step(0, {message(1), message(2)});
    ASSERT_EQ(accepted.outcomes.size(), 2U);
    EXPECT_EQ(accepted.outcomes[0].sequence(), 1);
    EXPECT_EQ(accepted.outcomes[1].sequence(), 2);
}

TEST(Network, OrdersDeliveriesByRecipientSenderAndVersion) {
    sentinel::core::NetworkEmulator network(21, 100);
    network.set_profile(profile());
    network.step(
        0,
        {
            message("agent-z", "agent-b", 3),
            message("agent-a", "agent-c", 2),
            message("agent-a", "agent-b", 4),
            message("agent-a", "agent-b", 1),
        });
    const auto delivered = network.step(1, {}).delivered;
    ASSERT_EQ(delivered.size(), 4U);
    EXPECT_EQ(delivered[0].recipient_id(), "agent-b");
    EXPECT_EQ(delivered[0].sender_id(), "agent-a");
    EXPECT_EQ(delivered[0].version(), 1);
    EXPECT_EQ(delivered[1].version(), 4);
    EXPECT_EQ(delivered[2].sender_id(), "agent-z");
    EXPECT_EQ(delivered[3].recipient_id(), "agent-c");
}

TEST(Network, AccountsForAcceptedSerializedTraffic) {
    sentinel::core::NetworkEmulator network(21, 100);
    network.set_profile(profile());
    const auto first = network.step(0, {message(1)});
    const auto bytes =
        first.outcomes.front().serialized_bytes();
    network.step(1, {message(2)});
    EXPECT_EQ(network.communication_messages(), 2);
    EXPECT_EQ(network.communication_bytes(), bytes * 2);
    EXPECT_EQ(network.delivered_messages(), 1);
    network.step(2, {});
    EXPECT_EQ(network.delivered_messages(), 2);
}

TEST(Network, OutcomeCapturesTheAdmissionAndDeliveryTicks) {
    sentinel::core::NetworkEmulator network(21, 100);
    network.set_profile(profile(3));
    const auto queued = network.step(4, {message(7)});
    ASSERT_EQ(queued.outcomes.size(), 1U);
    EXPECT_EQ(queued.outcomes[0].enqueue_tick(), 4);
    EXPECT_EQ(queued.outcomes[0].delivery_tick(), 8);
    EXPECT_EQ(queued.outcomes[0].profile_id(), "delayed");
    const auto delivered = network.step(8, {});
    ASSERT_EQ(delivered.outcomes.size(), 1U);
    EXPECT_EQ(delivered.outcomes[0].tick(), 8);
    EXPECT_EQ(delivered.outcomes[0].enqueue_tick(), 4);
    EXPECT_EQ(delivered.outcomes[0].delivery_tick(), 8);
}

TEST(Network, ProducesAStableHashForEquivalentQueues) {
    sentinel::core::NetworkEmulator first(9, 100);
    sentinel::core::NetworkEmulator second(9, 100);
    first.set_profile(profile(4));
    second.set_profile(profile(4));
    first.step(0, {message(1), message(2)});
    second.step(0, {message(1), message(2)});
    sentinel::core::HashBuilder left;
    sentinel::core::HashBuilder right;
    first.append_hash(left);
    second.append_hash(right);
    EXPECT_EQ(left.finish(), right.finish());
}

TEST(Network, IncludesQueuedPayloadInTheHash) {
    sentinel::core::NetworkEmulator first(9, 100);
    sentinel::core::NetworkEmulator second(9, 100);
    first.set_profile(profile(4));
    second.set_profile(profile(4));
    auto changed = message(1);
    changed.set_payload("different-state");
    first.step(0, {message(1)});
    second.step(0, {changed});
    sentinel::core::HashBuilder left;
    sentinel::core::HashBuilder right;
    first.append_hash(left);
    second.append_hash(right);
    EXPECT_NE(left.finish(), right.finish());
}
