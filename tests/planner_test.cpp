#include <sentinel/planning/planner.hpp>

#include "test_helpers.hpp"

#include <gtest/gtest.h>

TEST(Planner, BuildsARepeatableManhattanRoute) {
    const auto scenario = sentinel::test::baseline_scenario();
    sentinel::v1::VehicleState vehicle;
    vehicle.set_max_speed_mm_s(1000);
    vehicle.set_energy_cost_mj_per_meter(100);
    const sentinel::planning::Coordinate start{0, 0};
    const sentinel::planning::Coordinate goal{3000, 2000};
    const auto route = sentinel::planning::route_from(scenario.world(), vehicle, start, goal, 100);
    ASSERT_TRUE(route);
    EXPECT_EQ(route->distance_mm, 5000);
    EXPECT_EQ(route->points.front(), start);
    EXPECT_EQ(route->points.back(), goal);
}

TEST(Planner, RejectsEndpointsInsideAnObstacle) {
    auto scenario = sentinel::test::baseline_scenario();
    auto* obstacle = scenario.mutable_world()->add_regions();
    obstacle->set_id("wall");
    obstacle->set_kind(sentinel::v1::REGION_KIND_OBSTACLE);
    obstacle->mutable_minimum()->set_x_mm(1000);
    obstacle->mutable_minimum()->set_y_mm(0);
    obstacle->mutable_maximum()->set_x_mm(2000);
    obstacle->mutable_maximum()->set_y_mm(2000);
    sentinel::v1::VehicleState vehicle;
    vehicle.set_max_speed_mm_s(1000);
    const auto route = sentinel::planning::route_from(
        scenario.world(), vehicle, {0, 0}, {1500, 1000}, 100);
    EXPECT_FALSE(route);
}
