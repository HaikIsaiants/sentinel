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
    scenario.set_network_profile("local");
    scenario.add_network_profiles()->set_id("local");
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
    scenario.set_network_profile("local");
    scenario.add_network_profiles()->set_id("local");
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
    scenario.set_network_profile("local");
    scenario.add_network_profiles()->set_id("local");
    sentinel::core::Simulator simulator(scenario);
    EXPECT_THROW(simulator.step(sentinel::test::action(3, 0)), std::invalid_argument);
}

TEST(Simulator, ProducesTheSameHashForTheSameActions) {
    auto scenario = sentinel::test::baseline_scenario();
    scenario.set_network_profile("local");
    scenario.add_network_profiles()->set_id("local");
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
