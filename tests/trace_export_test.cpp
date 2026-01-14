#include <sentinel/core/event_log.hpp>
#include <sentinel/core/simulator.hpp>

#include "test_helpers.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <iterator>

TEST(TraceExport, WritesReplayPositionsAsJson) {
    const auto directory = std::filesystem::temp_directory_path();
    const auto log = directory / "sentinel-early-trace.bin";
    const auto trace = directory / "sentinel-early-trace.json";
    auto scenario = sentinel::test::baseline_scenario();
    sentinel::core::Simulator simulator(scenario);
    sentinel::core::EventLogWriter writer(log, scenario);
    while (!simulator.finished()) {
        const auto tick = simulator.tick();
        const auto actions = sentinel::test::action(tick, tick < 10 ? 1000 : 0, tick >= 10);
        simulator.step(actions);
        writer.append(tick, actions, {}, simulator.state_hash());
    }
    writer.finish(simulator.summary());
    const auto summary = sentinel::core::export_replay_trace(scenario, log, trace);
    std::ifstream input(trace);
    const std::string text{std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
    EXPECT_NE(text.find("\"scenario\":\"baseline\""), std::string::npos);
    EXPECT_NE(text.find(summary.terminal_hash()), std::string::npos);
    EXPECT_NE(text.find("\"agent\":\"agent-a\""), std::string::npos);
    std::filesystem::remove(log);
    std::filesystem::remove(trace);
}
