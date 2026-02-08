#include <sentinel/agent/autonomy.hpp>

#include "test_helpers.hpp"

#include <gtest/gtest.h>

namespace {

sentinel::v1::Envelope observation(std::int64_t x, std::int64_t y) {
    auto scenario = sentinel::test::baseline_scenario();
    sentinel::v1::Envelope result;
    result.set_schema_version(1);
    result.set_sequence(4);
    result.set_sender_id("sim");
    result.set_recipient_id("agent-a");
    auto* value = result.mutable_observation();
    value->set_tick(3);
    value->set_step_ms(scenario.step_ms());
    value->mutable_world()->CopyFrom(scenario.world());
    value->mutable_self()->set_id("agent-a");
    value->mutable_self()->set_max_speed_mm_s(1000);
    value->mutable_self()->mutable_position()->set_x_mm(x);
    value->mutable_self()->mutable_position()->set_y_mm(y);
    auto* task = value->add_assigned_tasks();
    task->set_id("task-a");
    task->mutable_target()->set_x_mm(1000);
    task->mutable_target()->set_y_mm(0);
    task->set_deadline_tick(30);
    task->set_completion_radius_mm(100);
    task->set_service_ticks(2);
    return result;
}

}

TEST(Autonomy, NavigatesTowardTheEarliestTask) {
    sentinel::agent::Controller controller("agent-a");
    auto input = observation(2000, 0);
    auto* urgent = input.mutable_observation()->add_assigned_tasks();
    urgent->set_id("urgent");
    urgent->mutable_target()->set_x_mm(0);
    urgent->set_deadline_tick(10);
    urgent->set_completion_radius_mm(100);
    urgent->set_service_ticks(1);
    const auto action = controller.act(input);
    ASSERT_TRUE(action.has_action());
    EXPECT_EQ(action.action().behavior_mode(), sentinel::v1::BEHAVIOR_MODE_NAVIGATING);
    EXPECT_EQ(action.action().velocity_x_mm_s(), -1000);
}

TEST(Autonomy, ReportsWorkOnlyInsideTheCompletionRadius) {
    sentinel::agent::Controller controller("agent-a");
    const auto action = controller.act(observation(1000, 0));
    EXPECT_EQ(action.action().behavior_mode(), sentinel::v1::BEHAVIOR_MODE_EXECUTING);
    ASSERT_EQ(action.action().task_reports_size(), 1);
    EXPECT_EQ(action.action().task_reports(0).task_id(), "task-a");
}

TEST(Autonomy, RejectsAnObservationForAnotherAgent) {
    sentinel::agent::Controller controller("agent-a");
    auto input = observation(0, 0);
    input.set_recipient_id("agent-b");
    EXPECT_THROW(controller.act(input), std::invalid_argument);
}

TEST(Autonomy, DivertsToAChargerBeforeEnergyRunsOut) {
    sentinel::agent::Controller controller("agent-a");
    auto input = observation(0, 0);
    auto* self = input.mutable_observation()->mutable_self();
    self->set_energy_capacity_mj(10000);
    self->set_energy_mj(1000);
    auto* charger = input.mutable_observation()->mutable_world()->add_locations();
    charger->set_id("charger");
    charger->set_kind(sentinel::v1::LOCATION_KIND_CHARGING);
    charger->mutable_position()->set_x_mm(500);
    charger->set_radius_mm(100);
    charger->set_charge_mj_per_tick(500);
    const auto action = controller.act(input);
    EXPECT_EQ(action.action().behavior_mode(), sentinel::v1::BEHAVIOR_MODE_NAVIGATING);
    EXPECT_GT(action.action().velocity_x_mm_s(), 0);
    EXPECT_TRUE(action.action().replanned());
}

TEST(Autonomy, RequestsChargeOnlyInsideTheServiceRadius) {
    sentinel::agent::Controller controller("agent-a");
    auto input = observation(500, 0);
    auto* self = input.mutable_observation()->mutable_self();
    self->set_energy_capacity_mj(10000);
    self->set_energy_mj(1000);
    auto* charger = input.mutable_observation()->mutable_world()->add_locations();
    charger->set_id("charger");
    charger->set_kind(sentinel::v1::LOCATION_KIND_CHARGING);
    charger->mutable_position()->set_x_mm(500);
    charger->set_radius_mm(100);
    charger->set_charge_mj_per_tick(500);
    const auto action = controller.act(input);
    EXPECT_EQ(action.action().behavior_mode(), sentinel::v1::BEHAVIOR_MODE_CHARGING);
    EXPECT_EQ(action.action().charge_location_id(), "charger");
}

TEST(Autonomy, ReevaluatesBehaviorPriorityOnEveryTick) {
    sentinel::agent::Controller controller("agent-a");
    auto charging = observation(500, 0);
    charging.mutable_observation()->mutable_self()->set_energy_capacity_mj(10000);
    charging.mutable_observation()->mutable_self()->set_energy_mj(1000);
    auto* charger = charging.mutable_observation()->mutable_world()->add_locations();
    charger->set_id("charger");
    charger->set_kind(sentinel::v1::LOCATION_KIND_CHARGING);
    charger->mutable_position()->set_x_mm(500);
    charger->set_radius_mm(100);
    charger->set_charge_mj_per_tick(500);
    EXPECT_EQ(
        controller.act(charging).action().behavior_mode(),
        sentinel::v1::BEHAVIOR_MODE_CHARGING);

    auto mission = observation(0, 0);
    mission.set_sequence(5);
    mission.mutable_observation()->set_tick(4);
    const auto resumed = controller.act(mission);
    EXPECT_EQ(resumed.action().behavior_mode(), sentinel::v1::BEHAVIOR_MODE_NAVIGATING);
    EXPECT_EQ(resumed.action().velocity_x_mm_s(), 1000);
    EXPECT_TRUE(resumed.action().charge_location_id().empty());
}

TEST(Autonomy, WaitsAfterProposingAnUnownedTask) {
    sentinel::agent::Controller controller("agent-a");
    auto input = observation(0, 0);
    auto* value = input.mutable_observation();
    value->clear_assigned_tasks();
    value->set_allocation_epoch(1);
    value->set_allocation_policy(sentinel::v1::ALLOCATION_POLICY_NEAREST_CAPABLE);
    value->mutable_self()->set_active(true);
    value->mutable_self()->set_energy_mj(100000);
    value->mutable_self()->set_energy_capacity_mj(100000);
    value->mutable_self()->set_energy_cost_mj_per_meter(100);
    value->mutable_self()->add_capabilities(sentinel::v1::CAPABILITY_SEARCH);
    auto* task = value->add_available_tasks();
    task->set_id("task-a");
    task->set_required_capability(sentinel::v1::CAPABILITY_SEARCH);
    task->mutable_target()->set_x_mm(1000);
    task->set_deadline_tick(30);
    task->set_service_ticks(2);
    const auto result = controller.act(input);
    EXPECT_EQ(result.action().behavior_mode(), sentinel::v1::BEHAVIOR_MODE_WAITING);
    ASSERT_EQ(result.action().allocation_commits_size(), 1);
    EXPECT_EQ(result.action().allocation_commits(0).agent_id(), "agent-a");
}

TEST(Autonomy, ReturnsHomeWhenNoMissionBranchIsReady) {
    sentinel::agent::Controller controller("agent-a");
    auto input = observation(0, 0);
    input.mutable_observation()->clear_assigned_tasks();
    input.mutable_observation()->mutable_self()->set_return_location_id("base");
    auto* home = input.mutable_observation()->mutable_world()->add_locations();
    home->set_id("base");
    home->set_kind(sentinel::v1::LOCATION_KIND_RETURN);
    home->mutable_position()->set_x_mm(500);
    home->set_radius_mm(100);
    const auto result = controller.act(input);
    EXPECT_EQ(result.action().behavior_mode(), sentinel::v1::BEHAVIOR_MODE_RETURNING);
    EXPECT_GT(result.action().velocity_x_mm_s(), 0);
}

TEST(Autonomy, RoutesAroundAnObstacleWithAStablePlanVersion) {
    sentinel::agent::Controller controller("agent-a");
    auto input = observation(0, 0);
    auto* world = input.mutable_observation()->mutable_world();
    world->set_map_version(1);
    auto* obstacle = world->add_regions();
    obstacle->set_id("blocked-cell");
    obstacle->set_kind(sentinel::v1::REGION_KIND_OBSTACLE);
    obstacle->mutable_minimum()->set_x_mm(1000);
    obstacle->mutable_minimum()->set_y_mm(0);
    obstacle->mutable_maximum()->set_x_mm(1000);
    obstacle->mutable_maximum()->set_y_mm(0);
    input.mutable_observation()
        ->mutable_assigned_tasks(0)
        ->mutable_target()
        ->set_x_mm(2000);

    const auto first = controller.act(input);
    EXPECT_EQ(
        first.action().behavior_mode(),
        sentinel::v1::BEHAVIOR_MODE_NAVIGATING);
    EXPECT_EQ(first.action().velocity_x_mm_s(), 0);
    EXPECT_GT(first.action().velocity_y_mm_s(), 0);
    EXPECT_TRUE(first.action().replanned());
    ASSERT_GT(first.action().route_plan().waypoints_size(), 3);

    const auto second = controller.act(input);
    EXPECT_EQ(
        second.action().route_version(),
        first.action().route_version());
    EXPECT_FALSE(second.action().replanned());
}
