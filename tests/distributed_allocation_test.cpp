#include <sentinel/agent/allocation.hpp>

#include "test_helpers.hpp"

#include <gtest/gtest.h>

#include <string>

namespace {

sentinel::v1::AgentObservation observation(
    const std::string& self_id, const std::string& peer_id,
    std::int64_t x_mm, std::uint64_t tick) {
    const auto scenario = sentinel::test::baseline_scenario();
    sentinel::v1::AgentObservation result;
    result.set_tick(tick);
    result.set_step_ms(scenario.step_ms());
    result.set_allocation_epoch(1);
    result.set_allocation_policy(
        sentinel::v1::ALLOCATION_POLICY_NEAREST_CAPABLE);
    result.add_peer_ids(peer_id);
    result.mutable_world()->CopyFrom(scenario.world());
    auto* self = result.mutable_self();
    self->set_id(self_id);
    self->set_active(true);
    self->mutable_position()->set_x_mm(x_mm);
    self->set_max_speed_mm_s(1000);
    self->set_energy_mj(100000);
    self->set_energy_capacity_mj(100000);
    self->set_energy_cost_mj_per_meter(100);
    self->set_payload_grams(1000);
    self->add_capabilities(sentinel::v1::CAPABILITY_SEARCH);
    auto* task = result.add_available_tasks();
    task->set_id("task-a");
    task->set_required_capability(sentinel::v1::CAPABILITY_SEARCH);
    task->mutable_target()->set_x_mm(2000);
    task->set_deadline_tick(30);
    task->set_service_ticks(2);
    task->set_status(sentinel::v1::TASK_STATUS_PENDING);
    result.add_known_tasks()->CopyFrom(*task);
    return result;
}

void deliver(
    sentinel::v1::AgentObservation& target,
    const sentinel::agent::AllocationResult& source) {
    for (const auto& message : source.outgoing_messages) {
        target.add_delivered_messages()->CopyFrom(message);
    }
}

}

TEST(DistributedAllocation, ExchangesViewsBeforeCommittingAContestedTask) {
    sentinel::agent::Allocator first_allocator("agent-a");
    sentinel::agent::Allocator second_allocator("agent-b");

    auto first_view = observation("agent-a", "agent-b", 0, 0);
    auto second_view = observation("agent-b", "agent-a", 5000, 0);
    const auto first_round_a = first_allocator.update(first_view);
    const auto first_round_b = second_allocator.update(second_view);
    EXPECT_TRUE(first_round_a.commits.empty());
    EXPECT_TRUE(first_round_b.commits.empty());
    ASSERT_EQ(first_round_a.outgoing_messages.size(), 1U);
    ASSERT_EQ(first_round_b.outgoing_messages.size(), 1U);

    first_view.set_tick(1);
    second_view.set_tick(1);
    deliver(first_view, first_round_b);
    deliver(second_view, first_round_a);
    const auto second_round_a = first_allocator.update(first_view);
    const auto second_round_b = second_allocator.update(second_view);
    EXPECT_TRUE(second_round_a.commits.empty());
    EXPECT_TRUE(second_round_b.commits.empty());
    ASSERT_EQ(second_round_b.state.winners_size(), 1);
    EXPECT_EQ(second_round_b.state.winners(0).bidder_id(), "agent-a");
    ASSERT_EQ(second_round_b.outgoing_messages.size(), 1U);

    first_view.clear_delivered_messages();
    first_view.set_tick(2);
    deliver(first_view, second_round_b);
    const auto converged = first_allocator.update(first_view);
    ASSERT_EQ(converged.commits.size(), 1U);
    EXPECT_EQ(converged.commits[0].task_id(), "task-a");
    EXPECT_EQ(converged.commits[0].agent_id(), "agent-a");
    EXPECT_TRUE(converged.coordinated);
}

TEST(DistributedAllocation, IgnoresStateFromAnUnknownSender) {
    sentinel::agent::Allocator allocator("agent-a");
    auto view = observation("agent-a", "agent-b", 0, 0);
    sentinel::v1::AllocationState forged;
    forged.set_epoch(1);
    forged.set_version(9);
    forged.set_sender_id("outsider");
    forged.set_map_version(view.world().map_version());
    auto* message = view.add_delivered_messages();
    message->set_sender_id("outsider");
    message->set_recipient_id("agent-a");
    message->set_version(9);
    message->set_payload(forged.SerializeAsString());

    const auto result = allocator.update(view);
    ASSERT_EQ(result.state.winners_size(), 1);
    EXPECT_EQ(result.state.winners(0).bidder_id(), "agent-a");
    EXPECT_TRUE(result.commits.empty());
}
