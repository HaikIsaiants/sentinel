#include <sentinel/planning/planner.hpp>

#include <algorithm>
#include <cstdlib>
#include <map>
#include <memory>
#include <tuple>
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

class BundleEvaluator::Impl {
public:
    Impl(
        const v1::World& world, const v1::VehicleState& vehicle,
        std::uint64_t tick, std::uint64_t step_ms)
        : world_(world), vehicle_(vehicle), tick_(tick), step_ms_(step_ms) {}

    std::optional<BundleEvaluation> evaluate(
        const std::vector<v1::TaskState>& ordered_tasks) {
        if (!vehicle_.active() || step_ms_ == 0) {
            return std::nullopt;
        }
        BundleEvaluation result;
        result.completion_tick = tick_;
        result.task_completion_ticks.reserve(ordered_tasks.size());
        Coordinate position{
            vehicle_.position().x_mm(), vehicle_.position().y_mm()};
        auto payload = vehicle_.payload_grams();
        const auto available_energy =
            vehicle_.energy_mj() - reserve_energy(vehicle_);
        if (available_energy < 0) {
            return std::nullopt;
        }
        for (const auto& task : ordered_tasks) {
            const auto capable =
                std::find(
                    vehicle_.capabilities().begin(),
                    vehicle_.capabilities().end(),
                    task.required_capability())
                != vehicle_.capabilities().end();
            if (task.status() != v1::TASK_STATUS_PENDING || !capable
                || payload < task.payload_required_grams()
                || result.completion_tick > task.deadline_tick()) {
                return std::nullopt;
            }
            const Coordinate target{
                task.target().x_mm(), task.target().y_mm()};
            const auto& route = route_to(position, target);
            if (!route) {
                return std::nullopt;
            }
            const auto service_ticks =
                task.service_ticks() > task.progress_ticks()
                    ? task.service_ticks() - task.progress_ticks()
                    : 0;
            const auto service_energy =
                static_cast<std::int64_t>(service_ticks)
                * task.service_energy_mj_per_tick();
            result.distance_mm += route->distance_mm;
            result.energy_mj += route->energy_mj + service_energy;
            result.completion_tick += route->travel_ticks + service_ticks;
            if (service_energy < 0 || result.energy_mj > available_energy
                || result.completion_tick > task.deadline_tick()) {
                return std::nullopt;
            }
            result.task_completion_ticks.push_back(result.completion_tick);
            if (task.required_capability() == v1::CAPABILITY_DELIVERY) {
                payload -= task.payload_required_grams();
            }
            position = target;
        }
        return result;
    }

    std::optional<BundleInsertion> best_insertion(
        const std::vector<v1::TaskState>& ordered_tasks,
        const v1::TaskState& candidate, std::size_t minimum_index) {
        if (minimum_index > ordered_tasks.size()
            || std::any_of(
                ordered_tasks.begin(), ordered_tasks.end(),
                [&candidate](const auto& task) {
                    return task.id() == candidate.id();
                })) {
            return std::nullopt;
        }
        const auto current = evaluate(ordered_tasks);
        if (!current) {
            return std::nullopt;
        }
        std::optional<BundleInsertion> best;
        for (std::size_t index = minimum_index;
             index <= ordered_tasks.size(); ++index) {
            auto tasks = ordered_tasks;
            tasks.insert(
                tasks.begin() + static_cast<std::ptrdiff_t>(index),
                candidate);
            const auto evaluated = evaluate(tasks);
            if (!evaluated) {
                continue;
            }
            std::uint64_t delay_ticks = 0;
            for (std::size_t task_index = 0;
                 task_index < ordered_tasks.size(); ++task_index) {
                const auto shifted =
                    task_index < index ? task_index : task_index + 1;
                if (evaluated->task_completion_ticks[shifted]
                    > current->task_completion_ticks[task_index]) {
                    delay_ticks +=
                        evaluated->task_completion_ticks[shifted]
                        - current->task_completion_ticks[task_index];
                }
            }
            const auto distance =
                evaluated->distance_mm - current->distance_mm;
            const auto energy =
                evaluated->energy_mj - current->energy_mj;
            const auto completion =
                evaluated->task_completion_ticks[index];
            const auto slack =
                candidate.deadline_tick() - completion;
            const auto score =
                static_cast<std::int64_t>(candidate.priority())
                    * 1'000'000'000'000LL
                + static_cast<std::int64_t>(slack) * 1'000'000LL
                - static_cast<std::int64_t>(delay_ticks) * 1'000'000LL
                - distance - energy;
            const BundleInsertion insertion{
                index, score, distance, energy, completion};
            if (!best || insertion.score > best->score
                || (insertion.score == best->score
                    && std::tuple{
                           insertion.distance_mm,
                           insertion.energy_mj,
                           insertion.completion_tick,
                           insertion.index}
                           < std::tuple{
                                 best->distance_mm,
                                 best->energy_mj,
                                 best->completion_tick,
                                 best->index})) {
                best = insertion;
            }
        }
        return best;
    }

private:
    const std::optional<Route>& route_to(
        const Coordinate& start, const Coordinate& goal) {
        const auto key =
            std::tuple{start.x(), start.y(), goal.x(), goal.y()};
        const auto current = routes_.find(key);
        if (current != routes_.end()) {
            return current->second;
        }
        return routes_
            .emplace(
                key,
                route_from(world_, vehicle_, start, goal, step_ms_))
            .first->second;
    }

    const v1::World& world_;
    const v1::VehicleState& vehicle_;
    std::uint64_t tick_{};
    std::uint64_t step_ms_{};
    std::map<
        std::tuple<
            std::int64_t, std::int64_t,
            std::int64_t, std::int64_t>,
        std::optional<Route>>
        routes_;
};

BundleEvaluator::BundleEvaluator(
    const v1::World& world, const v1::VehicleState& vehicle,
    std::uint64_t tick, std::uint64_t step_ms)
    : impl_(std::make_unique<Impl>(
          world, vehicle, tick, step_ms)) {}

BundleEvaluator::~BundleEvaluator() = default;

std::optional<BundleEvaluation> BundleEvaluator::evaluate(
    const std::vector<v1::TaskState>& ordered_tasks) {
    return impl_->evaluate(ordered_tasks);
}

std::optional<BundleInsertion> BundleEvaluator::best_insertion(
    const std::vector<v1::TaskState>& ordered_tasks,
    const v1::TaskState& candidate, std::size_t minimum_index) {
    return impl_->best_insertion(
        ordered_tasks, candidate, minimum_index);
}

std::optional<BundleEvaluation> evaluate_bundle(
    const v1::World& world, const v1::VehicleState& vehicle,
    const std::vector<v1::TaskState>& ordered_tasks,
    std::uint64_t tick, std::uint64_t step_ms) {
    return BundleEvaluator(world, vehicle, tick, step_ms)
        .evaluate(ordered_tasks);
}

std::optional<BundleInsertion> best_insertion(
    const v1::World& world, const v1::VehicleState& vehicle,
    const std::vector<v1::TaskState>& ordered_tasks,
    const v1::TaskState& candidate, std::uint64_t tick,
    std::uint64_t step_ms, std::size_t minimum_index) {
    return BundleEvaluator(world, vehicle, tick, step_ms)
        .best_insertion(
            ordered_tasks, candidate, minimum_index);
}

}
