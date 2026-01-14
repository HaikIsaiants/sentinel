#include <sentinel/agent/autonomy.hpp>
#include <sentinel/core/simulator.hpp>

#include "test_helpers.hpp"

#include <gtest/gtest.h>

TEST(MissionExecution, ControllerCompletesAScriptedTask) {
    sentinel::core::Simulator simulator(sentinel::test::baseline_scenario());
    sentinel::agent::Controller controller("agent-a");
    while (!simulator.finished()) {
        const auto observations = simulator.observe();
        sentinel::v1::ActionBatch actions;
        actions.set_tick(simulator.tick());
        ASSERT_EQ(observations.observations_size(), 1);
        actions.add_actions()->CopyFrom(controller.act(observations.observations(0)));
        simulator.step(actions);
    }
    EXPECT_TRUE(simulator.summary().success());
    EXPECT_LT(simulator.summary().ticks(), simulator.scenario().max_ticks());
}

TEST(MissionExecution, RepeatedRunsProduceTheSameTerminalHash) {
    auto run = [] {
        sentinel::core::Simulator simulator(sentinel::test::baseline_scenario());
        sentinel::agent::Controller controller("agent-a");
        while (!simulator.finished()) {
            const auto observations = simulator.observe();
            sentinel::v1::ActionBatch actions;
            actions.set_tick(simulator.tick());
            actions.add_actions()->CopyFrom(controller.act(observations.observations(0)));
            simulator.step(actions);
        }
        return simulator.state_hash();
    };
    EXPECT_EQ(run(), run());
}

TEST(MissionExecution, ServiceConsumesConfiguredEnergy) {
    auto scenario = sentinel::test::baseline_scenario();
    scenario.mutable_tasks(0)->set_service_energy_mj_per_tick(50);
    sentinel::core::Simulator simulator(scenario);
    sentinel::agent::Controller controller("agent-a");
    while (!simulator.finished()) {
        const auto observations = simulator.observe();
        sentinel::v1::ActionBatch actions;
        actions.set_tick(simulator.tick());
        actions.add_actions()->CopyFrom(controller.act(observations.observations(0)));
        simulator.step(actions);
    }
    EXPECT_TRUE(simulator.summary().success());
    EXPECT_GE(simulator.summary().energy_consumed_mj(), 100);
}
