#include <sentinel/agent/allocation.hpp>

#include "test_helpers.hpp"

#include <gtest/gtest.h>

namespace {

sentinel::v1::AgentObservation allocation_observation() {
    const auto scenario = sentinel::test::baseline_scenario();
    sentinel::v1::AgentObservation result;
    result.set_tick(4);
    result.set_step_ms(scenario.step_ms());
    result.set_allocation_epoch(1);
    result.set_allocation_policy(sentinel::v1::ALLOCATION_POLICY_NEAREST_CAPABLE);
    result.mutable_world()->CopyFrom(scenario.world());
    auto* self = result.mutable_self();
    self->set_id("agent-a");
    self->set_active(true);
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
    return result;
}

}

TEST(Allocation, ClaimsAFeasibleTaskForTheLocalAgent) {
    sentinel::agent::Allocator allocator("agent-a");
    const auto result = allocator.update(allocation_observation());
    ASSERT_EQ(result.commits.size(), 1U);
    EXPECT_EQ(result.commits[0].task_id(), "task-a");
    EXPECT_EQ(result.commits[0].agent_id(), "agent-a");
    EXPECT_EQ(result.commits[0].distance_mm(), 2000);
    EXPECT_TRUE(result.pending);
}

TEST(Allocation, SkipsTasksThatNeedAnotherCapability) {
    sentinel::agent::Allocator allocator("agent-a");
    auto observation = allocation_observation();
    observation.mutable_available_tasks(0)->set_required_capability(
        sentinel::v1::CAPABILITY_INSPECTION);
    const auto result = allocator.update(observation);
    EXPECT_TRUE(result.commits.empty());
    EXPECT_FALSE(result.pending);
}

TEST(Allocation, VersionsSuccessiveClaimsMonotonically) {
    sentinel::agent::Allocator allocator("agent-a");
    const auto first = allocator.update(allocation_observation());
    const auto second = allocator.update(allocation_observation());
    ASSERT_EQ(first.commits.size(), 1U);
    ASSERT_EQ(second.commits.size(), 1U);
    EXPECT_LT(first.commits[0].version(), second.commits[0].version());
}

TEST(Allocation, DoesNothingForScriptedMissions) {
    sentinel::agent::Allocator allocator("agent-a");
    auto observation = allocation_observation();
    observation.set_allocation_policy(sentinel::v1::ALLOCATION_POLICY_SCRIPTED);
    EXPECT_TRUE(allocator.update(observation).commits.empty());
}
