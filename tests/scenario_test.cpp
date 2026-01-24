#include <sentinel/core/scenario.hpp>

#include "test_helpers.hpp"

#include <gtest/gtest.h>

#include <algorithm>

TEST(Scenario, HashCanonicalizesOrderInsensitiveCollections) {
    auto first = sentinel::test::baseline_scenario();
    auto* second_profile = first.add_network_profiles();
    second_profile->set_id("backup");
    second_profile->set_latency_ticks(2);
    first.mutable_vehicles(0)->add_capabilities(
        sentinel::v1::CAPABILITY_INSPECTION);
    first.mutable_vehicles(0)->add_terrain_access("paved");
    first.mutable_vehicles(0)->add_terrain_access("gravel");

    auto reordered = first;
    std::reverse(reordered.mutable_network_profiles()->begin(),
                 reordered.mutable_network_profiles()->end());
    std::reverse(reordered.mutable_vehicles(0)->mutable_capabilities()->begin(),
                 reordered.mutable_vehicles(0)->mutable_capabilities()->end());
    std::reverse(reordered.mutable_vehicles(0)->mutable_terrain_access()->begin(),
                 reordered.mutable_vehicles(0)->mutable_terrain_access()->end());

    EXPECT_EQ(sentinel::core::scenario_hash(first),
              sentinel::core::scenario_hash(reordered));
}

TEST(Scenario, RejectsMissingAndInfeasibleTaskOwners) {
    auto scenario = sentinel::test::baseline_scenario();
    scenario.mutable_tasks(0)->set_assigned_agent_id("missing");
    EXPECT_THROW(sentinel::core::validate_scenario(scenario),
                 std::invalid_argument);

    scenario = sentinel::test::baseline_scenario();
    scenario.mutable_tasks(0)->set_required_capability(
        sentinel::v1::CAPABILITY_DELIVERY);
    EXPECT_THROW(sentinel::core::validate_scenario(scenario),
                 std::invalid_argument);

    scenario = sentinel::test::baseline_scenario();
    scenario.mutable_tasks(0)->set_payload_required_grams(1001);
    EXPECT_THROW(sentinel::core::validate_scenario(scenario),
                 std::invalid_argument);
}

TEST(Scenario, AcceptsUnassignedTaskWhenACapableVehicleExists) {
    auto scenario = sentinel::test::baseline_scenario();
    scenario.set_allocation_policy(
        sentinel::v1::ALLOCATION_POLICY_NEAREST_CAPABLE);
    scenario.mutable_tasks(0)->clear_assigned_agent_id();
    EXPECT_NO_THROW(sentinel::core::validate_scenario(scenario));

    scenario.mutable_tasks(0)->set_required_capability(
        sentinel::v1::CAPABILITY_DELIVERY);
    EXPECT_THROW(sentinel::core::validate_scenario(scenario),
                 std::invalid_argument);
}

TEST(Scenario, RejectsDuplicateIdentifiersAcrossScenarioCollections) {
    auto scenario = sentinel::test::baseline_scenario();
    scenario.add_vehicles()->CopyFrom(scenario.vehicles(0));
    EXPECT_THROW(sentinel::core::validate_scenario(scenario),
                 std::invalid_argument);

    scenario = sentinel::test::baseline_scenario();
    scenario.add_network_profiles()->CopyFrom(scenario.network_profiles(0));
    EXPECT_THROW(sentinel::core::validate_scenario(scenario),
                 std::invalid_argument);

    scenario = sentinel::test::baseline_scenario();
    scenario.add_tasks()->CopyFrom(scenario.tasks(0));
    EXPECT_THROW(sentinel::core::validate_scenario(scenario),
                 std::invalid_argument);
}

TEST(Scenario, RejectsCoordinatesOutsideTheWorldOrOffTheGrid) {
    auto scenario = sentinel::test::baseline_scenario();
    scenario.mutable_vehicles(0)->mutable_initial_position()->set_x_mm(-1);
    EXPECT_THROW(sentinel::core::validate_scenario(scenario),
                 std::invalid_argument);

    scenario = sentinel::test::baseline_scenario();
    scenario.mutable_tasks(0)->mutable_target()->set_x_mm(10'001);
    EXPECT_THROW(sentinel::core::validate_scenario(scenario),
                 std::invalid_argument);

    scenario = sentinel::test::baseline_scenario();
    scenario.mutable_tasks(0)->mutable_target()->set_x_mm(500);
    EXPECT_THROW(sentinel::core::validate_scenario(scenario),
                 std::invalid_argument);
}

TEST(Scenario, RejectsInvalidWorldAndClockBounds) {
    auto scenario = sentinel::test::baseline_scenario();
    scenario.mutable_world()->set_grid_cell_mm(3);
    EXPECT_THROW(sentinel::core::validate_scenario(scenario),
                 std::invalid_argument);

    scenario = sentinel::test::baseline_scenario();
    scenario.set_step_ms(60'001);
    EXPECT_THROW(sentinel::core::validate_scenario(scenario),
                 std::invalid_argument);

    scenario = sentinel::test::baseline_scenario();
    scenario.mutable_world()->set_width_mm(1'000'000'001);
    EXPECT_THROW(sentinel::core::validate_scenario(scenario),
                 std::invalid_argument);
}

TEST(Scenario, RejectsUnknownServiceAndEventReferences) {
    auto scenario = sentinel::test::baseline_scenario();
    scenario.mutable_vehicles(0)->set_return_location_id("home");
    EXPECT_THROW(sentinel::core::validate_scenario(scenario),
                 std::invalid_argument);

    scenario = sentinel::test::baseline_scenario();
    auto* event = scenario.add_events();
    event->set_id("release");
    event->set_tick(2);
    event->set_kind(sentinel::v1::TAPE_EVENT_KIND_RELEASE_TASK);
    event->set_target_id("missing");
    EXPECT_THROW(sentinel::core::validate_scenario(scenario),
                 std::invalid_argument);

    scenario = sentinel::test::baseline_scenario();
    event = scenario.add_events();
    event->set_id("switch-profile");
    event->set_tick(2);
    event->set_kind(sentinel::v1::TAPE_EVENT_KIND_SET_NETWORK_PROFILE);
    event->set_text_value("missing");
    EXPECT_THROW(sentinel::core::validate_scenario(scenario),
                 std::invalid_argument);
}

TEST(Scenario, RejectsUnknownEnumsAndDuplicateCapabilities) {
    auto scenario = sentinel::test::baseline_scenario();
    scenario.mutable_vehicles(0)->set_capabilities(
        0, static_cast<sentinel::v1::Capability>(99));
    scenario.mutable_tasks(0)->set_required_capability(
        static_cast<sentinel::v1::Capability>(99));
    EXPECT_THROW(sentinel::core::validate_scenario(scenario),
                 std::invalid_argument);

    scenario = sentinel::test::baseline_scenario();
    scenario.mutable_vehicles(0)->add_capabilities(
        sentinel::v1::CAPABILITY_SEARCH);
    EXPECT_THROW(sentinel::core::validate_scenario(scenario),
                 std::invalid_argument);
}
