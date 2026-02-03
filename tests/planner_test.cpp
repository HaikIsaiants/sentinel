#include <sentinel/planning/planner.hpp>

#include "test_helpers.hpp"

#include <gtest/gtest.h>

namespace {

sentinel::v1::VehicleState bundle_vehicle() {
    sentinel::v1::VehicleState vehicle;
    vehicle.set_id("agent-a");
    vehicle.set_active(true);
    vehicle.set_max_speed_mm_s(1000);
    vehicle.set_energy_mj(100000);
    vehicle.set_energy_capacity_mj(100000);
    vehicle.set_energy_cost_mj_per_meter(100);
    vehicle.set_payload_grams(1000);
    vehicle.add_capabilities(sentinel::v1::CAPABILITY_SEARCH);
    return vehicle;
}

sentinel::v1::TaskState bundle_task(
    const char* id, std::int64_t x_mm, std::uint32_t priority) {
    sentinel::v1::TaskState task;
    task.set_id(id);
    task.set_status(sentinel::v1::TASK_STATUS_PENDING);
    task.set_required_capability(sentinel::v1::CAPABILITY_SEARCH);
    task.mutable_target()->set_x_mm(x_mm);
    task.set_deadline_tick(80);
    task.set_service_ticks(1);
    task.set_service_energy_mj_per_tick(10);
    task.set_priority(priority);
    return task;
}

}

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

TEST(Planner, EvaluatesAnOrderedTaskBundle) {
    const auto scenario = sentinel::test::baseline_scenario();
    const auto vehicle = bundle_vehicle();
    const std::vector tasks{
        bundle_task("near", 1000, 5),
        bundle_task("far", 3000, 5)};
    const auto evaluated = sentinel::planning::evaluate_bundle(
        scenario.world(), vehicle, tasks, 0, 1000);
    ASSERT_TRUE(evaluated);
    EXPECT_EQ(evaluated->distance_mm, 3000);
    EXPECT_EQ(evaluated->task_completion_ticks.size(), 2U);
    EXPECT_LT(
        evaluated->task_completion_ticks[0],
        evaluated->task_completion_ticks[1]);
}

TEST(Planner, ScoresPriorityBeforeMarginalDistance) {
    const auto scenario = sentinel::test::baseline_scenario();
    const auto vehicle = bundle_vehicle();
    sentinel::planning::BundleEvaluator evaluator(
        scenario.world(), vehicle, 0, 1000);
    const auto low = evaluator.best_insertion(
        {}, bundle_task("near", 1000, 2));
    const auto high = evaluator.best_insertion(
        {}, bundle_task("far", 3000, 20));
    ASSERT_TRUE(low);
    ASSERT_TRUE(high);
    EXPECT_GT(high->score, low->score);
}
