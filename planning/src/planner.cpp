#include <sentinel/planning/planner.hpp>

#include <algorithm>
#include <cstdlib>
#include <utility>

namespace sentinel::planning {

namespace {

bool contains(const v1::Region& region, const Coordinate& point) {
    return point.x() >= region.minimum().x_mm() && point.x() <= region.maximum().x_mm()
           && point.y() >= region.minimum().y_mm() && point.y() <= region.maximum().y_mm();
}

std::int64_t manhattan(const Coordinate& from, const Coordinate& to) {
    return std::llabs(to.x() - from.x()) + std::llabs(to.y() - from.y());
}

}

bool segment_allowed(const v1::World& world, const v1::VehicleState&,
                     const Coordinate& from, const Coordinate& to) {
    if (from.x() < 0 || from.y() < 0 || to.x() < 0 || to.y() < 0
        || from.x() > world.width_mm() || to.x() > world.width_mm()
        || from.y() > world.height_mm() || to.y() > world.height_mm()) {
        return false;
    }
    for (const auto& region : world.regions()) {
        if ((region.kind() == v1::REGION_KIND_OBSTACLE || region.kind() == v1::REGION_KIND_RESTRICTED)
            && (contains(region, from) || contains(region, to))) {
            return false;
        }
    }
    return true;
}

std::optional<Route> route_from(const v1::World& world, const v1::VehicleState& vehicle,
                                const Coordinate& start, const Coordinate& goal,
                                std::uint64_t step_ms) {
    const Coordinate corner{goal.x(), start.y()};
    const auto corner_allowed = segment_allowed(world, vehicle, start, corner)
                                && segment_allowed(world, vehicle, corner, goal);
    const Coordinate alternate{start.x(), goal.y()};
    const auto alternate_allowed = segment_allowed(world, vehicle, start, alternate)
                                   && segment_allowed(world, vehicle, alternate, goal);
    if (!corner_allowed && !alternate_allowed) {
        return std::nullopt;
    }
    Route route;
    route.points.push_back(start);
    const auto middle = corner_allowed ? corner : alternate;
    if (middle != start && middle != goal) {
        route.points.push_back(middle);
    }
    if (goal != start) {
        route.points.push_back(goal);
    }
    route.distance_mm = manhattan(start, goal);
    route.energy_mj = (route.distance_mm * vehicle.energy_cost_mj_per_meter() + 999) / 1000;
    const auto distance_per_tick = std::max<std::int64_t>(
        1, (vehicle.max_speed_mm_s() * static_cast<std::int64_t>(step_ms)) / 1000);
    route.travel_ticks = static_cast<std::uint64_t>((route.distance_mm + distance_per_tick - 1) / distance_per_tick);
    return route;
}

std::int64_t reserve_energy(const v1::VehicleState& vehicle) {
    return std::max<std::int64_t>(0, vehicle.energy_capacity_mj() / 10);
}

std::optional<Route> feasible_route(
    const v1::World& world, const v1::VehicleState& vehicle,
    const v1::TaskState& task, std::uint64_t tick, std::uint64_t step_ms) {
    const auto capable = std::find(
        vehicle.capabilities().begin(), vehicle.capabilities().end(),
        task.required_capability()) != vehicle.capabilities().end();
    if (!vehicle.active() || !capable
        || vehicle.payload_grams() < task.payload_required_grams()) {
        return std::nullopt;
    }
    const Coordinate start{
        vehicle.position().x_mm(), vehicle.position().y_mm()};
    const Coordinate target{task.target().x_mm(), task.target().y_mm()};
    auto route = route_from(world, vehicle, start, target, step_ms);
    const auto service_energy =
        task.service_energy_mj_per_tick() * static_cast<std::int64_t>(task.service_ticks());
    if (!route || route->energy_mj + service_energy + reserve_energy(vehicle)
                      > vehicle.energy_mj()) {
        return std::nullopt;
    }
    if (tick + route->travel_ticks + task.service_ticks() >= task.deadline_tick()) {
        return std::nullopt;
    }
    return route;
}

v1::AllocationClaim make_claim(
    const v1::World& world, const v1::VehicleState& vehicle,
    const v1::TaskState& task, std::uint64_t tick, std::uint64_t step_ms,
    std::uint64_t epoch, std::uint64_t version) {
    v1::AllocationClaim result;
    result.set_epoch(epoch);
    result.set_version(version);
    result.set_task_id(task.id());
    result.set_agent_id(vehicle.id());
    const auto route = feasible_route(world, vehicle, task, tick, step_ms);
    result.set_feasible(route.has_value());
    if (route) {
        result.set_distance_mm(route->distance_mm);
    }
    return result;
}

std::optional<v1::AllocationClaim> nearest_capable(
    const v1::World& world, const std::vector<v1::VehicleState>& vehicles,
    const v1::TaskState& task, std::uint64_t tick, std::uint64_t step_ms,
    std::uint64_t epoch, std::uint64_t version) {
    std::optional<v1::AllocationClaim> winner;
    for (const auto& vehicle : vehicles) {
        auto claim = make_claim(
            world, vehicle, task, tick, step_ms, epoch, version);
        if (!claim.feasible()) {
            continue;
        }
        if (!winner || claim.distance_mm() < winner->distance_mm()
            || (claim.distance_mm() == winner->distance_mm()
                && claim.agent_id() < winner->agent_id())) {
            winner = std::move(claim);
        }
    }
    return winner;
}

}
