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

sentinel::v1::ActionBatch idle_actions(
    std::uint64_t tick, bool include_first = true) {
    sentinel::v1::ActionBatch actions;
    actions.set_tick(tick);
    if (include_first) {
        auto* first = actions.add_actions();
        first->set_schema_version(1);
        first->set_sender_id("agent-a");
        first->set_recipient_id("sim");
        first->mutable_action()->set_tick(tick);
    }
    auto* second = actions.add_actions();
    second->set_schema_version(1);
    second->set_sender_id("agent-b");
    second->set_recipient_id("sim");
    second->mutable_action()->set_tick(tick);
    return actions;
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

TEST(Simulator, DisablesAVehicleAtTheRecordedTick) {
    auto scenario = networked_scenario();
    auto* event = scenario.add_events();
    event->set_id("disable-agent-a");
    event->set_tick(0);
    event->set_kind(
        sentinel::v1::TAPE_EVENT_KIND_DISABLE_VEHICLE);
    event->set_target_id("agent-a");
    sentinel::core::Simulator simulator(scenario);

    const auto observations =
        simulator.step(idle_actions(0, false));
    ASSERT_EQ(observations.observations_size(), 1);
    EXPECT_EQ(
        observations.observations(0).recipient_id(),
        "agent-b");
    EXPECT_EQ(simulator.summary().active_agents(), 1);
}

TEST(Simulator, RejectsAnActionFromAVehicleDisabledThatTick) {
    auto scenario = networked_scenario();
    auto* event = scenario.add_events();
    event->set_id("disable-agent-a");
    event->set_tick(0);
    event->set_kind(
        sentinel::v1::TAPE_EVENT_KIND_DISABLE_VEHICLE);
    event->set_target_id("agent-a");
    sentinel::core::Simulator simulator(scenario);
    EXPECT_THROW(
        simulator.step(idle_actions(0)),
        std::invalid_argument);
}

TEST(Simulator, SwitchesNetworkProfileAtTheRecordedTick) {
    auto scenario = networked_scenario();
    auto* disrupted = scenario.add_network_profiles();
    disrupted->set_id("disrupted");
    disrupted->set_latency_ticks(5);
    auto* event = scenario.add_events();
    event->set_id("switch-network");
    event->set_tick(0);
    event->set_kind(
        sentinel::v1::TAPE_EVENT_KIND_SET_NETWORK_PROFILE);
    event->set_text_value("disrupted");
    sentinel::core::Simulator simulator(scenario);

    const auto observations =
        simulator.step(idle_actions(0));
    ASSERT_EQ(observations.observations_size(), 2);
    EXPECT_EQ(
        observations.observations(0)
            .observation()
            .network_profile(),
        "disrupted");
    EXPECT_EQ(simulator.scenario().network_profile(), "disrupted");
}

TEST(Simulator, RejectsAnUnknownEventNetworkProfile) {
    auto scenario = networked_scenario();
    auto* event = scenario.add_events();
    event->set_id("switch-network");
    event->set_tick(0);
    event->set_kind(
        sentinel::v1::TAPE_EVENT_KIND_SET_NETWORK_PROFILE);
    event->set_text_value("missing");
    sentinel::core::Simulator simulator(scenario);
    EXPECT_THROW(
        simulator.step(idle_actions(0)),
        std::invalid_argument);
}

TEST(Simulator, ClosesARegionAndAdvancesTheMapVersion) {
    auto scenario = networked_scenario();
    auto* region = scenario.mutable_world()->add_regions();
    region->set_id("corridor");
    region->set_kind(sentinel::v1::REGION_KIND_RESTRICTED);
    region->mutable_minimum()->set_x_mm(100);
    region->mutable_minimum()->set_y_mm(100);
    region->mutable_maximum()->set_x_mm(200);
    region->mutable_maximum()->set_y_mm(200);
    const auto initial_version = scenario.world().map_version();
    auto* event = scenario.add_events();
    event->set_id("close-corridor");
    event->set_tick(0);
    event->set_kind(
        sentinel::v1::TAPE_EVENT_KIND_SET_REGION_CLOSED);
    event->set_target_id("corridor");
    event->set_bool_value(true);
    sentinel::core::Simulator simulator(scenario);

    simulator.step(idle_actions(0));
    ASSERT_EQ(simulator.scenario().world().regions_size(), 1);
    EXPECT_TRUE(
        simulator.scenario().world().regions(0).closed());
    EXPECT_EQ(
        simulator.scenario().world().map_version(),
        initial_version + 1);
}

TEST(Simulator, LeavesMapVersionStableForAnIdempotentRegionEvent) {
    auto scenario = networked_scenario();
    auto* region = scenario.mutable_world()->add_regions();
    region->set_id("corridor");
    region->set_kind(sentinel::v1::REGION_KIND_RESTRICTED);
    region->mutable_minimum()->set_x_mm(100);
    region->mutable_minimum()->set_y_mm(100);
    region->mutable_maximum()->set_x_mm(200);
    region->mutable_maximum()->set_y_mm(200);
    region->set_closed(true);
    const auto initial_version = scenario.world().map_version();
    auto* event = scenario.add_events();
    event->set_id("keep-corridor-closed");
    event->set_tick(0);
    event->set_kind(
        sentinel::v1::TAPE_EVENT_KIND_SET_REGION_CLOSED);
    event->set_target_id("corridor");
    event->set_bool_value(true);
    sentinel::core::Simulator simulator(scenario);

    simulator.step(idle_actions(0));
    EXPECT_EQ(
        simulator.scenario().world().map_version(),
        initial_version);
}

TEST(Simulator, RejectsARegionEventWithAnUnknownTarget) {
    auto scenario = networked_scenario();
    auto* event = scenario.add_events();
    event->set_id("close-missing");
    event->set_tick(0);
    event->set_kind(
        sentinel::v1::TAPE_EVENT_KIND_SET_REGION_CLOSED);
    event->set_target_id("missing");
    event->set_bool_value(true);
    sentinel::core::Simulator simulator(scenario);
    EXPECT_THROW(
        simulator.step(idle_actions(0)),
        std::invalid_argument);
}

TEST(Simulator, ClampsPositiveEnergyEventsToCapacity) {
    auto scenario = networked_scenario();
    scenario.mutable_vehicles(0)->set_initial_energy_mj(1000);
    auto* event = scenario.add_events();
    event->set_id("boost-agent-a");
    event->set_tick(0);
    event->set_kind(
        sentinel::v1::TAPE_EVENT_KIND_ENERGY_DELTA);
    event->set_target_id("agent-a");
    event->set_value_min(500);
    event->set_value_max(500);
    sentinel::core::Simulator simulator(scenario);

    const auto observations =
        simulator.step(idle_actions(0));
    const auto agent = std::find_if(
        observations.observations().begin(),
        observations.observations().end(),
        [](const auto& envelope) {
            return envelope.recipient_id() == "agent-a";
        });
    ASSERT_NE(agent, observations.observations().end());
    EXPECT_EQ(agent->observation().self().energy_mj(), 1000);
}

TEST(Simulator, ResolvesRangedEnergyEventsRepeatably) {
    auto scenario = networked_scenario();
    auto* event = scenario.add_events();
    event->set_id("vary-agent-a");
    event->set_tick(0);
    event->set_kind(
        sentinel::v1::TAPE_EVENT_KIND_ENERGY_DELTA);
    event->set_target_id("agent-a");
    event->set_rng_stream("energy-events");
    event->set_value_min(-500);
    event->set_value_max(-100);
    sentinel::core::Simulator first(scenario);
    sentinel::core::Simulator second(scenario);

    first.step(idle_actions(0));
    second.step(idle_actions(0));
    EXPECT_EQ(first.state_hash(), second.state_hash());
}

TEST(Simulator, RejectsAReleaseForAnUnknownTask) {
    auto scenario = networked_scenario();
    auto* event = scenario.add_events();
    event->set_id("release-missing");
    event->set_tick(0);
    event->set_kind(
        sentinel::v1::TAPE_EVENT_KIND_RELEASE_TASK);
    event->set_target_id("missing");
    sentinel::core::Simulator simulator(scenario);
    EXPECT_THROW(
        simulator.step(idle_actions(0)),
        std::invalid_argument);
}

TEST(Simulator, RecordsBehaviorCountersFromAcceptedActions) {
    sentinel::core::Simulator simulator(networked_scenario());
    auto actions = idle_actions(0);
    actions.mutable_actions(0)
        ->mutable_action()
        ->set_behavior_mode(
            sentinel::v1::BEHAVIOR_MODE_WAITING);
    actions.mutable_actions(1)
        ->mutable_action()
        ->set_behavior_mode(
            sentinel::v1::BEHAVIOR_MODE_RETURNING);
    actions.mutable_actions(1)
        ->mutable_action()
        ->set_replanned(true);
    simulator.step(actions);

    const auto summary = simulator.summary();
    EXPECT_EQ(summary.wait_ticks(), 1);
    EXPECT_EQ(summary.return_ticks(), 1);
    EXPECT_EQ(summary.replan_count(), 1);
}
