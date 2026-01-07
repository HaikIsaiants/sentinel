#pragma once

#include <sentinel/v1/sentinel.pb.h>

#include <Eigen/Core>

#include <cstdint>
#include <optional>
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

}
