#include <sentinel/core/event_log.hpp>
#include <sentinel/core/simulator.hpp>

#include "test_helpers.hpp"

#include <gtest/gtest.h>

#include <filesystem>

namespace {

std::filesystem::path temporary(std::string_view name) {
    return std::filesystem::temp_directory_path() / std::string(name);
}

}

TEST(EventLog, ReplaysActionsToTheSameTerminalHash) {
    const auto path = temporary("sentinel-early-event-log.bin");
    auto scenario = sentinel::test::baseline_scenario();
    sentinel::core::Simulator simulator(scenario);
    sentinel::core::EventLogWriter writer(path, scenario);
    while (!simulator.finished()) {
        const auto tick = simulator.tick();
        const auto working = tick >= 10;
        const auto actions = sentinel::test::action(tick, tick < 10 ? 1000 : 0, working);
        simulator.step(actions);
        writer.append(tick, actions, {}, simulator.state_hash());
    }
    writer.finish(simulator.summary());
    const auto replay = sentinel::core::replay_event_log(scenario, path);
    EXPECT_EQ(replay.terminal_hash(), simulator.summary().terminal_hash());
    std::filesystem::remove(path);
}

TEST(EventLog, RejectsARecordWithTheWrongScenario) {
    const auto path = temporary("sentinel-early-event-log-mismatch.bin");
    auto scenario = sentinel::test::baseline_scenario();
    sentinel::core::Simulator simulator(scenario);
    sentinel::core::EventLogWriter writer(path, scenario);
    writer.finish(simulator.summary());
    scenario.set_seed(scenario.seed() + 1);
    EXPECT_THROW(sentinel::core::replay_event_log(scenario, path), std::runtime_error);
    std::filesystem::remove(path);
}
