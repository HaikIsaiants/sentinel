#pragma once

#include <sentinel/v1/sentinel.pb.h>

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace sentinel::planning {

using Coordinate = Eigen::Matrix<std::int64_t, 2, 1>;

struct Route {
    std::vector<Coordinate> points;
    std::int64_t distance_mm{};
    std::int64_t energy_mj{};
    std::uint64_t travel_ticks{};
};

bool segment_allowed(const v1::World& world, const v1::VehicleState& vehicle,
                     const Coordinate& from, const Coordinate& to);
std::optional<Route> route_from(const v1::World& world, const v1::VehicleState& vehicle,
                                const Coordinate& start, const Coordinate& goal,
                                std::uint64_t step_ms);
std::int64_t reserve_energy(const v1::VehicleState& vehicle);
std::optional<Route> feasible_route(
    const v1::World& world, const v1::VehicleState& vehicle,
    const v1::TaskState& task, std::uint64_t tick, std::uint64_t step_ms);
v1::AllocationClaim make_claim(
    const v1::World& world, const v1::VehicleState& vehicle,
    const v1::TaskState& task, std::uint64_t tick, std::uint64_t step_ms,
    std::uint64_t epoch, std::uint64_t version);
std::optional<v1::AllocationClaim> nearest_capable(
    const v1::World& world, const std::vector<v1::VehicleState>& vehicles,
    const v1::TaskState& task, std::uint64_t tick, std::uint64_t step_ms,
    std::uint64_t epoch, std::uint64_t version);

struct BundleEvaluation {
    std::int64_t distance_mm{};
    std::int64_t energy_mj{};
    std::uint64_t completion_tick{};
    std::vector<std::uint64_t> task_completion_ticks;
};

struct BundleInsertion {
    std::size_t index{};
    std::int64_t score{};
    std::int64_t distance_mm{};
    std::int64_t energy_mj{};
    std::uint64_t completion_tick{};
};

class BundleEvaluator {
public:
    BundleEvaluator(
        const v1::World& world, const v1::VehicleState& vehicle,
        std::uint64_t tick, std::uint64_t step_ms);
    ~BundleEvaluator();
    BundleEvaluator(const BundleEvaluator&) = delete;
    BundleEvaluator& operator=(const BundleEvaluator&) = delete;
    std::optional<BundleEvaluation> evaluate(
        const std::vector<v1::TaskState>& ordered_tasks);
    std::optional<BundleInsertion> best_insertion(
        const std::vector<v1::TaskState>& ordered_tasks,
        const v1::TaskState& candidate, std::size_t minimum_index = 0);

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

std::optional<BundleEvaluation> evaluate_bundle(
    const v1::World& world, const v1::VehicleState& vehicle,
    const std::vector<v1::TaskState>& ordered_tasks,
    std::uint64_t tick, std::uint64_t step_ms);
std::optional<BundleInsertion> best_insertion(
    const v1::World& world, const v1::VehicleState& vehicle,
    const std::vector<v1::TaskState>& ordered_tasks,
    const v1::TaskState& candidate, std::uint64_t tick,
    std::uint64_t step_ms, std::size_t minimum_index = 0);

}
