#include <sentinel/core/simulator.hpp>

#include "test_helpers.hpp"

#include <gtest/gtest.h>

namespace {

sentinel::v1::Scenario networked_scenario() {
    auto scenario = sentinel::test::baseline_scenario();
    scenario.mutable_network_profiles(0)->set_latency_ticks(1);
    auto* second = scenario.add_vehicles();
    second->CopyFrom(scenario.vehicles(0));
    second->set_id("agent-b");
    second->mutable_initial_position()->set_y_mm(1000);
    return scenario;
}

}

TEST(Simulator, AdvancesAtTheConfiguredFixedStep) {
    auto scenario = sentinel::test::baseline_scenario();
    sentinel::core::Simulator simulator(scenario);
    for (std::uint64_t tick = 0; tick < 10; ++tick) {
        simulator.step(sentinel::test::action(tick, 1000));
    }
    const auto observation = simulator.observe();
    ASSERT_EQ(observation.observations_size(), 1);
    EXPECT_EQ(observation.observations(0).observation().self().position().x_mm(), 1000);
}

TEST(Simulator, RequiresContinuousServiceAtTheTarget) {
    auto scenario = sentinel::test::baseline_scenario();
    sentinel::core::Simulator simulator(scenario);
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
    auto scenario = sentinel::test::baseline_scenario();
    sentinel::core::Simulator simulator(scenario);
    EXPECT_THROW(simulator.step(sentinel::test::action(3, 0)), std::invalid_argument);
}

TEST(Simulator, ProducesTheSameHashForTheSameActions) {
    auto scenario = sentinel::test::baseline_scenario();
    sentinel::core::Simulator first(scenario);
    sentinel::core::Simulator second(scenario);
    for (std::uint64_t tick = 0; tick < 8; ++tick) {
        const auto actions = sentinel::test::action(tick, 1000);
        first.step(actions);
        second.step(actions);
        EXPECT_EQ(first.state_hash(), second.state_hash());
    }
}

TEST(Simulator, DeliversAgentMessagesAfterNetworkLatency) {
    sentinel::core::Simulator simulator(networked_scenario());
    auto actions = sentinel::test::action(0, 0);
    auto* message = actions.mutable_actions(0)->mutable_action()->add_outgoing_messages();
    message->set_sender_id("agent-a");
    message->set_recipient_id("agent-b");
    message->set_version(1);
    message->set_payload("hello");
    auto* second = actions.add_actions();
    second->set_schema_version(1);
    second->set_sender_id("agent-b");
    second->set_recipient_id("sim");
    second->mutable_action()->set_tick(0);
    simulator.step(actions);

    for (std::uint64_t tick = 1; tick <= 2; ++tick) {
        auto idle = sentinel::test::action(tick, 0);
        auto* agent_b = idle.add_actions();
        agent_b->set_schema_version(1);
        agent_b->set_sender_id("agent-b");
        agent_b->set_recipient_id("sim");
        agent_b->mutable_action()->set_tick(tick);
        simulator.step(idle);
    }
    const auto observations = simulator.observe();
    const auto recipient = std::find_if(
        observations.observations().begin(), observations.observations().end(),
        [](const auto& envelope) { return envelope.recipient_id() == "agent-b"; });
    ASSERT_NE(recipient, observations.observations().end());
    ASSERT_EQ(recipient->observation().delivered_messages_size(), 1);
    EXPECT_EQ(recipient->observation().delivered_messages(0).payload(), "hello");
    EXPECT_EQ(simulator.summary().communication_messages(), 1);
}

TEST(Simulator, ReleasesADeferredTaskAtItsRecordedTick) {
    auto scenario = networked_scenario();
    scenario.mutable_tasks(0)->set_released(false);
    auto* event = scenario.add_events();
    event->set_id("release-task-a");
    event->set_tick(2);
    event->set_kind(sentinel::v1::TAPE_EVENT_KIND_RELEASE_TASK);
    event->set_target_id("task-a");
    sentinel::core::Simulator simulator(scenario);
    EXPECT_EQ(simulator.observe().observations(0).observation().assigned_tasks_size(), 0);
    for (std::uint64_t tick = 0; tick <= 2; ++tick) {
        auto actions = sentinel::test::action(tick, 0);
        auto* second = actions.add_actions();
        second->set_sender_id("agent-b");
        second->set_recipient_id("sim");
        second->mutable_action()->set_tick(tick);
        simulator.step(actions);
    }
    EXPECT_EQ(simulator.observe().observations(0).observation().assigned_tasks_size(), 1);
}

TEST(Simulator, AppliesDeterministicEnergyEvents) {
    auto scenario = networked_scenario();
    auto* event = scenario.add_events();
    event->set_id("drain-agent-a");
    event->set_tick(0);
    event->set_kind(sentinel::v1::TAPE_EVENT_KIND_ENERGY_DELTA);
    event->set_target_id("agent-a");
    event->set_value_min(-500);
    event->set_value_max(-500);
    sentinel::core::Simulator simulator(scenario);
    auto actions = sentinel::test::action(0, 0);
    auto* second = actions.add_actions();
    second->set_sender_id("agent-b");
    second->set_recipient_id("sim");
    second->mutable_action()->set_tick(0);
    const auto observations = simulator.step(actions);
    const auto agent = std::find_if(
        observations.observations().begin(), observations.observations().end(),
        [](const auto& envelope) { return envelope.recipient_id() == "agent-a"; });
    ASSERT_NE(agent, observations.observations().end());
    EXPECT_EQ(agent->observation().self().energy_mj(), 99500);
}

TEST(Simulator, ChargesOnlyAtAKnownLocation) {
    auto scenario = networked_scenario();
    auto* charger = scenario.mutable_world()->add_locations();
    charger->set_id("charger");
    charger->set_kind(sentinel::v1::LOCATION_KIND_CHARGING);
    charger->mutable_position()->set_x_mm(0);
    charger->mutable_position()->set_y_mm(0);
    charger->set_radius_mm(100);
    charger->set_charge_mj_per_tick(500);
    scenario.mutable_vehicles(0)->set_initial_energy_mj(1000);
    sentinel::core::Simulator simulator(scenario);
    auto actions = sentinel::test::action(0, 0);
    actions.mutable_actions(0)->mutable_action()->set_charge_location_id("charger");
    actions.mutable_actions(0)->mutable_action()->set_behavior_mode(
        sentinel::v1::BEHAVIOR_MODE_CHARGING);
    auto* second = actions.add_actions();
    second->set_sender_id("agent-b");
    second->set_recipient_id("sim");
    second->mutable_action()->set_tick(0);
    const auto observations = simulator.step(actions);
    const auto agent = std::find_if(
        observations.observations().begin(), observations.observations().end(),
        [](const auto& envelope) { return envelope.recipient_id() == "agent-a"; });
    ASSERT_NE(agent, observations.observations().end());
    EXPECT_EQ(agent->observation().self().energy_mj(), 1000);
    EXPECT_EQ(simulator.summary().recharge_ticks(), 1);
}

TEST(Simulator, ChoosesTheCloserValidAllocationCommit) {
    auto scenario = networked_scenario();
    scenario.set_allocation_policy(sentinel::v1::ALLOCATION_POLICY_NEAREST_CAPABLE);
    scenario.mutable_tasks(0)->clear_assigned_agent_id();
    sentinel::core::Simulator simulator(scenario);
    sentinel::v1::ActionBatch actions;
    actions.set_tick(0);
    for (const auto& id : {"agent-b", "agent-a"}) {
        auto* envelope = actions.add_actions();
        envelope->set_sender_id(id);
        envelope->set_recipient_id("sim");
        auto* action = envelope->mutable_action();
        action->set_tick(0);
        auto* commit = action->add_allocation_commits();
        commit->set_epoch(1);
        commit->set_version(1);
        commit->set_task_id("task-a");
        commit->set_agent_id(id);
        commit->set_distance_mm(id == std::string_view("agent-a") ? 1000 : 2000);
        commit->set_score(-commit->distance_mm());
    }
    const auto observations = simulator.step(actions);
    const auto agent_a = std::find_if(
        observations.observations().begin(), observations.observations().end(),
        [](const auto& envelope) { return envelope.recipient_id() == "agent-a"; });
    ASSERT_NE(agent_a, observations.observations().end());
    ASSERT_EQ(agent_a->observation().assigned_tasks_size(), 1);
    EXPECT_EQ(agent_a->observation().assigned_tasks(0).assigned_agent_id(), "agent-a");
}
