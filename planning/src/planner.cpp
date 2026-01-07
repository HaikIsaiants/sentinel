#include <sentinel/planning/planner.hpp>

#include <algorithm>
#include <cstdlib>

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

}
