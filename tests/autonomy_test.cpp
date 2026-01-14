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
    const auto action = controller.act(observation(0, 0));
    ASSERT_TRUE(action.has_action());
    EXPECT_EQ(action.action().behavior_mode(), sentinel::v1::BEHAVIOR_MODE_NAVIGATING);
    EXPECT_EQ(action.action().velocity_x_mm_s(), 1000);
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
