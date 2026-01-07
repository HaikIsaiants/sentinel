#include <sentinel/core/scenario.hpp>

#include "test_helpers.hpp"

#include <gtest/gtest.h>

TEST(Scenario, NormalizesIdentifiersBeforeHashing) {
    auto first = sentinel::test::baseline_scenario();
    auto second = first;
    auto* extra = second.add_vehicles();
    extra->CopyFrom(second.vehicles(0));
    extra->set_id("agent-0");
    first.add_vehicles()->CopyFrom(*extra);
    sentinel::core::normalize_scenario(first);
    sentinel::core::normalize_scenario(second);
    EXPECT_EQ(sentinel::core::scenario_hash(first), sentinel::core::scenario_hash(second));
}

TEST(Scenario, RejectsMissingTaskOwner) {
    auto scenario = sentinel::test::baseline_scenario();
    scenario.mutable_tasks(0)->set_assigned_agent_id("missing");
    EXPECT_THROW(sentinel::core::validate_scenario(scenario), std::invalid_argument);
}

TEST(Scenario, RejectsDuplicateVehicleIdentifiers) {
    auto scenario = sentinel::test::baseline_scenario();
    scenario.add_vehicles()->CopyFrom(scenario.vehicles(0));
    EXPECT_THROW(sentinel::core::validate_scenario(scenario), std::invalid_argument);
}
