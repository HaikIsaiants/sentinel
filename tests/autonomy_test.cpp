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
    value->mutable_self()->set_active(true);
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

TEST(Autonomy, PreservesEnvelopeMetadataInTheAction) {
    sentinel::agent::Controller controller("agent-a");
    auto input = observation(0, 0);
    input.set_sequence(19);
    input.set_simulation_time_ms(300);
    const auto output = controller.act(input);
    EXPECT_EQ(output.schema_version(), 1);
    EXPECT_EQ(output.sequence(), 19);
    EXPECT_EQ(output.simulation_time_ms(), 300);
    EXPECT_EQ(output.sender_id(), "agent-a");
    EXPECT_EQ(output.recipient_id(), "sim");
    EXPECT_EQ(output.action().tick(), 3);
}

TEST(Autonomy, WaitsWhenTheVehicleIsInactive) {
    sentinel::agent::Controller controller("agent-a");
    auto input = observation(0, 0);
    input.mutable_observation()->mutable_self()->set_active(false);
    const auto output = controller.act(input);
    EXPECT_EQ(
        output.action().behavior_mode(),
        sentinel::v1::BEHAVIOR_MODE_WAITING);
    EXPECT_EQ(output.action().velocity_x_mm_s(), 0);
    EXPECT_EQ(output.action().task_reports_size(), 0);
}

TEST(Autonomy, IdlesWithoutAnAssignedTask) {
    sentinel::agent::Controller controller("agent-a");
    auto input = observation(0, 0);
    input.mutable_observation()->clear_assigned_tasks();
    const auto output = controller.act(input);
    EXPECT_EQ(
        output.action().behavior_mode(),
        sentinel::v1::BEHAVIOR_MODE_IDLE);
    EXPECT_EQ(output.action().velocity_x_mm_s(), 0);
    EXPECT_EQ(output.action().velocity_y_mm_s(), 0);
}

TEST(Autonomy, NavigatesAlongTheDominantAxis) {
    sentinel::agent::Controller controller("agent-a");
    auto input = observation(0, 0);
    input.mutable_observation()
        ->mutable_assigned_tasks(0)
        ->mutable_target()
        ->set_x_mm(200);
    input.mutable_observation()
        ->mutable_assigned_tasks(0)
        ->mutable_target()
        ->set_y_mm(900);
    const auto output = controller.act(input);
    EXPECT_EQ(output.action().velocity_x_mm_s(), 0);
    EXPECT_EQ(output.action().velocity_y_mm_s(), 900);
}

TEST(Autonomy, NavigatesInTheNegativeDirection) {
    sentinel::agent::Controller controller("agent-a");
    auto input = observation(900, 0);
    input.mutable_observation()
        ->mutable_assigned_tasks(0)
        ->mutable_target()
        ->set_x_mm(100);
    const auto output = controller.act(input);
    EXPECT_EQ(output.action().velocity_x_mm_s(), -800);
    EXPECT_EQ(output.action().velocity_y_mm_s(), 0);
}

TEST(Autonomy, ChoosesTheTaskWithTheEarlierDeadline) {
    sentinel::agent::Controller controller("agent-a");
    auto input = observation(0, 0);
    input.mutable_observation()
        ->mutable_assigned_tasks(0)
        ->set_deadline_tick(50);
    auto* urgent =
        input.mutable_observation()->add_assigned_tasks();
    urgent->set_id("task-urgent");
    urgent->mutable_target()->set_x_mm(0);
    urgent->mutable_target()->set_y_mm(700);
    urgent->set_deadline_tick(10);
    urgent->set_completion_radius_mm(100);
    urgent->set_service_ticks(1);
    const auto output = controller.act(input);
    EXPECT_EQ(output.action().velocity_x_mm_s(), 0);
    EXPECT_EQ(output.action().velocity_y_mm_s(), 700);
}

TEST(Autonomy, BreaksDeadlineTiesByTravelDistance) {
    sentinel::agent::Controller controller("agent-a");
    auto input = observation(0, 0);
    input.mutable_observation()
        ->mutable_assigned_tasks(0)
        ->mutable_target()
        ->set_x_mm(900);
    auto* nearby =
        input.mutable_observation()->add_assigned_tasks();
    nearby->set_id("task-near");
    nearby->mutable_target()->set_x_mm(200);
    nearby->set_deadline_tick(30);
    nearby->set_completion_radius_mm(50);
    nearby->set_service_ticks(1);
    const auto output = controller.act(input);
    EXPECT_EQ(output.action().velocity_x_mm_s(), 200);
}

TEST(Autonomy, ContinuesTheSelectedTaskWhileItRemainsAssigned) {
    sentinel::agent::Controller controller("agent-a");
    auto first = observation(0, 0);
    first.mutable_observation()
        ->mutable_assigned_tasks(0)
        ->set_deadline_tick(20);
    const auto first_output = controller.act(first);
    EXPECT_EQ(first_output.action().velocity_x_mm_s(), 1000);

    auto second = observation(100, 0);
    second.mutable_observation()->set_tick(4);
    second.mutable_observation()
        ->mutable_assigned_tasks(0)
        ->set_deadline_tick(20);
    auto* new_task =
        second.mutable_observation()->add_assigned_tasks();
    new_task->set_id("task-new");
    new_task->mutable_target()->set_y_mm(100);
    new_task->set_deadline_tick(5);
    new_task->set_completion_radius_mm(20);
    new_task->set_service_ticks(1);
    const auto second_output = controller.act(second);
    EXPECT_GT(second_output.action().velocity_x_mm_s(), 0);
    EXPECT_EQ(second_output.action().velocity_y_mm_s(), 0);
}

TEST(Autonomy, RejectsAnEnvelopeWithoutAnObservation) {
    sentinel::agent::Controller controller("agent-a");
    sentinel::v1::Envelope input;
    input.set_sender_id("sim");
    input.set_recipient_id("agent-a");
    EXPECT_THROW(controller.act(input), std::invalid_argument);
}

TEST(Autonomy, RejectsAnObservationFromAnotherSender) {
    sentinel::agent::Controller controller("agent-a");
    auto input = observation(0, 0);
    input.set_sender_id("agent-b");
    EXPECT_THROW(controller.act(input), std::invalid_argument);
}

TEST(Autonomy, RejectsAnObservationWithAnotherIdentity) {
    sentinel::agent::Controller controller("agent-a");
    auto input = observation(0, 0);
    input.mutable_observation()->mutable_self()->set_id("agent-b");
    EXPECT_THROW(controller.act(input), std::invalid_argument);
}

TEST(Autonomy, RejectsARepeatedObservationTick) {
    sentinel::agent::Controller controller("agent-a");
    auto input = observation(0, 0);
    controller.act(input);
    EXPECT_THROW(controller.act(input), std::invalid_argument);
}
