#include <sentinel/core/simulator.hpp>

#include "test_helpers.hpp"

#include <gtest/gtest.h>

TEST(Simulator, AdvancesAtTheConfiguredFixedStep) {
    sentinel::core::Simulator simulator(sentinel::test::baseline_scenario());
    for (std::uint64_t tick = 0; tick < 10; ++tick) {
        simulator.step(sentinel::test::action(tick, 1000));
    }
    const auto observation = simulator.observe();
    ASSERT_EQ(observation.observations_size(), 1);
    EXPECT_EQ(observation.observations(0).observation().self().position().x_mm(), 1000);
}

TEST(Simulator, RequiresContinuousServiceAtTheTarget) {
    sentinel::core::Simulator simulator(sentinel::test::baseline_scenario());
    for (std::uint64_t tick = 0; tick < 10; ++tick) {
        simulator.step(sentinel::test::action(tick, 1000));
    }
    simulator.step(sentinel::test::action(10, 0, true));
    EXPECT_FALSE(simulator.finished());
    simulator.step(sentinel::test::action(11, 0, true));
    EXPECT_TRUE(simulator.finished());
    EXPECT_TRUE(simulator.summary().success());
}

TEST(Simulator, RejectsAnActionForTheWrongTick) {
    sentinel::core::Simulator simulator(sentinel::test::baseline_scenario());
    EXPECT_THROW(simulator.step(sentinel::test::action(3, 0)), std::invalid_argument);
}

TEST(Simulator, ProducesTheSameHashForTheSameActions) {
    sentinel::core::Simulator first(sentinel::test::baseline_scenario());
    sentinel::core::Simulator second(sentinel::test::baseline_scenario());
    for (std::uint64_t tick = 0; tick < 8; ++tick) {
        const auto actions = sentinel::test::action(tick, 1000);
        first.step(actions);
        second.step(actions);
        EXPECT_EQ(first.state_hash(), second.state_hash());
    }
}

TEST(Simulator, BuildsAnObservationForEachVehicle) {
    auto scenario = sentinel::test::baseline_scenario();
    auto* second = scenario.add_vehicles();
    second->CopyFrom(scenario.vehicles(0));
    second->set_id("agent-b");
    second->mutable_initial_position()->set_y_mm(500);
    sentinel::core::Simulator simulator(scenario);

    const auto observations = simulator.observe();
    ASSERT_EQ(observations.observations_size(), 2);
    EXPECT_EQ(observations.tick(), 0);
    EXPECT_FALSE(observations.finished());
    EXPECT_EQ(observations.observations(0).sender_id(), "sim");
    EXPECT_EQ(observations.observations(0).recipient_id(), "agent-a");
    EXPECT_EQ(observations.observations(1).recipient_id(), "agent-b");
    EXPECT_EQ(
        observations.observations(0).observation().assigned_tasks_size(), 1);
    EXPECT_EQ(
        observations.observations(1).observation().assigned_tasks_size(), 0);
}

TEST(Simulator, StopsAnAgentThatDoesNotSubmitACommand) {
    auto scenario = sentinel::test::baseline_scenario();
    auto* second = scenario.add_vehicles();
    second->CopyFrom(scenario.vehicles(0));
    second->set_id("agent-b");
    second->mutable_initial_position()->set_y_mm(500);
    sentinel::core::Simulator simulator(scenario);

    simulator.step(sentinel::test::action(0, 1000));
    auto empty = sentinel::v1::ActionBatch{};
    empty.set_tick(1);
    simulator.step(empty);

    const auto observation = simulator.observe();
    EXPECT_EQ(
        observation.observations(0).observation().self().velocity_x_mm_s(), 0);
    EXPECT_EQ(
        observation.observations(1).observation().self().velocity_x_mm_s(), 0);
}

TEST(Simulator, ClampsVelocityToTheVehicleLimit) {
    sentinel::core::Simulator simulator(sentinel::test::baseline_scenario());
    simulator.step(sentinel::test::action(0, 9000));
    const auto self =
        simulator.observe().observations(0).observation().self();
    EXPECT_EQ(self.velocity_x_mm_s(), 1000);
    EXPECT_EQ(self.position().x_mm(), 100);
}

TEST(Simulator, KeepsVehiclesInsideTheWorld) {
    auto scenario = sentinel::test::baseline_scenario();
    scenario.mutable_vehicles(0)->mutable_initial_position()->set_x_mm(9950);
    sentinel::core::Simulator simulator(scenario);
    simulator.step(sentinel::test::action(0, 1000));
    EXPECT_EQ(
        simulator.observe()
            .observations(0)
            .observation()
            .self()
            .position()
            .x_mm(),
        10000);
}

TEST(Simulator, ConsumesEnergyFromActualDistanceMoved) {
    auto scenario = sentinel::test::baseline_scenario();
    scenario.mutable_vehicles(0)->set_initial_energy_mj(25);
    scenario.mutable_vehicles(0)->set_energy_cost_mj_per_meter(100);
    sentinel::core::Simulator simulator(scenario);
    simulator.step(sentinel::test::action(0, 1000));

    const auto self =
        simulator.observe().observations(0).observation().self();
    EXPECT_EQ(self.position().x_mm(), 100);
    EXPECT_EQ(self.energy_mj(), 15);
    EXPECT_TRUE(self.active());
}

TEST(Simulator, StopsMovingAfterEnergyIsExhausted) {
    auto scenario = sentinel::test::baseline_scenario();
    scenario.mutable_vehicles(0)->set_initial_energy_mj(5);
    scenario.mutable_vehicles(0)->set_energy_cost_mj_per_meter(100);
    sentinel::core::Simulator simulator(scenario);
    simulator.step(sentinel::test::action(0, 1000));
    simulator.step(sentinel::test::action(1, 1000));

    const auto self =
        simulator.observe().observations(0).observation().self();
    EXPECT_EQ(self.position().x_mm(), 100);
    EXPECT_EQ(self.energy_mj(), 0);
    EXPECT_FALSE(self.active());
    EXPECT_EQ(self.velocity_x_mm_s(), 0);
}

TEST(Simulator, RejectsDuplicateActionsFromOneVehicle) {
    sentinel::core::Simulator simulator(sentinel::test::baseline_scenario());
    auto actions = sentinel::test::action(0, 0);
    actions.add_actions()->CopyFrom(actions.actions(0));
    EXPECT_THROW(simulator.step(actions), std::invalid_argument);
}

TEST(Simulator, RejectsAnActionWithoutABody) {
    sentinel::core::Simulator simulator(sentinel::test::baseline_scenario());
    sentinel::v1::ActionBatch actions;
    actions.set_tick(0);
    actions.add_actions()->set_sender_id("agent-a");
    EXPECT_THROW(simulator.step(actions), std::invalid_argument);
}

TEST(Simulator, RejectsAnActionAddressedElsewhere) {
    sentinel::core::Simulator simulator(sentinel::test::baseline_scenario());
    auto actions = sentinel::test::action(0, 0);
    actions.mutable_actions(0)->set_recipient_id("agent-b");
    EXPECT_THROW(simulator.step(actions), std::invalid_argument);
}

TEST(Simulator, RejectsAnUnknownActionSender) {
    sentinel::core::Simulator simulator(sentinel::test::baseline_scenario());
    auto actions = sentinel::test::action(0, 0);
    actions.mutable_actions(0)->set_sender_id("missing");
    EXPECT_THROW(simulator.step(actions), std::invalid_argument);
}

TEST(Simulator, ResetsInterruptedServiceProgress) {
    auto scenario = sentinel::test::baseline_scenario();
    scenario.mutable_tasks(0)->mutable_target()->set_x_mm(0);
    scenario.mutable_tasks(0)->set_service_ticks(2);
    sentinel::core::Simulator simulator(scenario);

    simulator.step(sentinel::test::action(0, 0, true));
    simulator.step(sentinel::test::action(1, 0, false));
    simulator.step(sentinel::test::action(2, 0, true));
    EXPECT_FALSE(simulator.finished());
    simulator.step(sentinel::test::action(3, 0, true));
    EXPECT_TRUE(simulator.summary().success());
}

TEST(Simulator, RejectsWorkFromAnIncapableVehicle) {
    auto scenario = sentinel::test::baseline_scenario();
    scenario.mutable_tasks(0)->mutable_target()->set_x_mm(0);
    scenario.mutable_tasks(0)->set_required_capability(
        sentinel::v1::CAPABILITY_INSPECTION);
    scenario.mutable_tasks(0)->set_deadline_tick(3);
    sentinel::core::Simulator simulator(scenario);

    simulator.step(sentinel::test::action(0, 0, true));
    simulator.step(sentinel::test::action(1, 0, true));
    EXPECT_FALSE(simulator.finished());
    simulator.step(sentinel::test::action(2, 0, true));
    EXPECT_TRUE(simulator.finished());
    EXPECT_FALSE(simulator.summary().success());
}

TEST(Simulator, RejectsWorkBeyondPayloadCapacity) {
    auto scenario = sentinel::test::baseline_scenario();
    scenario.mutable_tasks(0)->mutable_target()->set_x_mm(0);
    scenario.mutable_tasks(0)->set_payload_required_grams(2000);
    scenario.mutable_tasks(0)->set_deadline_tick(2);
    sentinel::core::Simulator simulator(scenario);

    simulator.step(sentinel::test::action(0, 0, true));
    simulator.step(sentinel::test::action(1, 0, true));
    EXPECT_TRUE(simulator.finished());
    EXPECT_EQ(simulator.summary().completed_tasks(), 0);
}

TEST(Simulator, MarksAnOverdueTaskMissed) {
    auto scenario = sentinel::test::baseline_scenario();
    scenario.mutable_tasks(0)->set_deadline_tick(2);
    sentinel::core::Simulator simulator(scenario);

    simulator.step(sentinel::test::action(0, 0));
    simulator.step(sentinel::test::action(1, 0));
    EXPECT_TRUE(simulator.finished());
    EXPECT_FALSE(simulator.summary().success());
    EXPECT_EQ(simulator.summary().ticks(), 2);
    EXPECT_EQ(simulator.summary().completed_tasks(), 0);
    EXPECT_EQ(simulator.summary().total_tasks(), 1);
}

TEST(Simulator, IncludesEveryStateHashInTheSummary) {
    auto scenario = sentinel::test::baseline_scenario();
    scenario.set_max_ticks(3);
    sentinel::core::Simulator simulator(scenario);
    for (std::uint64_t tick = 0; tick < 3; ++tick) {
        simulator.step(sentinel::test::action(tick, 0));
    }

    const auto summary = simulator.summary();
    ASSERT_EQ(summary.tick_hashes_size(), 4);
    EXPECT_EQ(summary.tick_hashes(3), summary.terminal_hash());
    EXPECT_EQ(summary.terminal_hash(), simulator.state_hash());
}

TEST(Simulator, RefusesToAdvanceAfterTheRunFinishes) {
    auto scenario = sentinel::test::baseline_scenario();
    scenario.set_max_ticks(1);
    sentinel::core::Simulator simulator(scenario);
    simulator.step(sentinel::test::action(0, 0));
    EXPECT_THROW(
        simulator.step(sentinel::test::action(1, 0)),
        std::logic_error);
}
