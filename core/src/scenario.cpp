#include <sentinel/core/scenario.hpp>

#include <sentinel/core/hash.hpp>
#include <sentinel/protocol/framing.hpp>

#include <google/protobuf/text_format.h>

#include <algorithm>
#include <cstdint>
#include <fstream>
#include <iterator>
#include <set>
#include <stdexcept>
#include <string_view>

namespace sentinel::core {
namespace {

constexpr std::int64_t max_world_mm = 1'000'000'000;
constexpr std::uint64_t max_step_ms = 60'000;
constexpr std::uint64_t max_ticks = 10'000'000;
constexpr std::int64_t max_speed_mm_s = 1'000'000;
constexpr std::int64_t max_energy_mj = 1'000'000'000'000'000;
constexpr std::int64_t max_payload_grams = 1'000'000'000;
constexpr std::int64_t max_grid_cells = 1'000'000;

bool valid_id(std::string_view value) {
    return !value.empty()
           && value.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_")
                  == std::string_view::npos;
}

bool valid_capability(sentinel::v1::Capability value) {
    return value == sentinel::v1::CAPABILITY_SEARCH
           || value == sentinel::v1::CAPABILITY_INSPECTION
           || value == sentinel::v1::CAPABILITY_RELAY
           || value == sentinel::v1::CAPABILITY_DELIVERY;
}

bool valid_region_kind(sentinel::v1::RegionKind value) {
    return value == sentinel::v1::REGION_KIND_OBSTACLE
           || value == sentinel::v1::REGION_KIND_RESTRICTED
           || value == sentinel::v1::REGION_KIND_TERRAIN;
}

bool valid_location_kind(sentinel::v1::LocationKind value) {
    return value == sentinel::v1::LOCATION_KIND_CHARGING
           || value == sentinel::v1::LOCATION_KIND_RETURN;
}

bool valid_event_kind(sentinel::v1::TapeEventKind value) {
    return value == sentinel::v1::TAPE_EVENT_KIND_RELEASE_TASK
           || value == sentinel::v1::TAPE_EVENT_KIND_DISABLE_VEHICLE
           || value == sentinel::v1::TAPE_EVENT_KIND_ENERGY_DELTA
           || value == sentinel::v1::TAPE_EVENT_KIND_SET_NETWORK_PROFILE
           || value == sentinel::v1::TAPE_EVENT_KIND_SET_REGION_CLOSED;
}

bool point_in_world(const sentinel::v1::Scenario& scenario, const sentinel::v1::Point& point) {
    return point.x_mm() >= 0 && point.y_mm() >= 0
           && point.x_mm() <= scenario.world().width_mm()
           && point.y_mm() <= scenario.world().height_mm();
}

bool grid_aligned(const sentinel::v1::Scenario& scenario, const sentinel::v1::Point& point) {
    return point.x_mm() % scenario.world().grid_cell_mm() == 0
           && point.y_mm() % scenario.world().grid_cell_mm() == 0;
}

bool has_capability(const sentinel::v1::VehicleSpec& vehicle,
                    sentinel::v1::Capability capability) {
    return std::find(vehicle.capabilities().begin(), vehicle.capabilities().end(), capability)
           != vehicle.capabilities().end();
}

const sentinel::v1::VehicleSpec* vehicle_by_id(const sentinel::v1::Scenario& scenario,
                                               std::string_view id) {
    const auto position = std::find_if(
        scenario.vehicles().begin(), scenario.vehicles().end(),
        [id](const auto& vehicle) { return vehicle.id() == id; });
    return position == scenario.vehicles().end() ? nullptr : &*position;
}

const sentinel::v1::ServiceLocation* location_by_id(const sentinel::v1::Scenario& scenario,
                                                    std::string_view id) {
    const auto position = std::find_if(
        scenario.world().locations().begin(), scenario.world().locations().end(),
        [id](const auto& location) { return location.id() == id; });
    return position == scenario.world().locations().end() ? nullptr : &*position;
}

bool task_exists(const sentinel::v1::Scenario& scenario, std::string_view id) {
    return std::any_of(scenario.tasks().begin(), scenario.tasks().end(),
                       [id](const auto& task) { return task.id() == id; });
}

bool region_exists(const sentinel::v1::Scenario& scenario, std::string_view id) {
    return std::any_of(
        scenario.world().regions().begin(), scenario.world().regions().end(),
        [id](const auto& region) { return region.id() == id; });
}

bool network_profile_exists(const sentinel::v1::Scenario& scenario, std::string_view id) {
    return std::any_of(
        scenario.network_profiles().begin(), scenario.network_profiles().end(),
        [id](const auto& profile) { return profile.id() == id; });
}

template <typename Range, typename Selector>
void require_unique_ids(const Range& values, Selector selector, std::string_view label) {
    std::set<std::string> identifiers;
    for (const auto& value : values) {
        const auto& id = selector(value);
        if (!valid_id(id) || !identifiers.insert(id).second) {
            throw std::invalid_argument(std::string("invalid ") + std::string(label));
        }
    }
}

}

sentinel::v1::Scenario load_scenario(const std::filesystem::path& path) {
    std::ifstream input(path, std::ios::binary);
    if (!input) {
        throw std::runtime_error("failed to open scenario");
    }
    const std::string text{std::istreambuf_iterator<char>(input),
                           std::istreambuf_iterator<char>()};
    sentinel::v1::Scenario scenario;
    if (!google::protobuf::TextFormat::ParseFromString(text, &scenario)) {
        throw std::runtime_error("invalid scenario textproto");
    }
    normalize_scenario(scenario);
    validate_scenario(scenario);
    return scenario;
}

void normalize_scenario(sentinel::v1::Scenario& scenario) {
    std::sort(scenario.mutable_network_profiles()->begin(),
              scenario.mutable_network_profiles()->end(),
              [](const auto& left, const auto& right) {
                  return left.id() < right.id();
              });
    std::sort(scenario.mutable_world()->mutable_regions()->begin(),
              scenario.mutable_world()->mutable_regions()->end(),
              [](const auto& left, const auto& right) {
                  return left.id() < right.id();
              });
    std::sort(scenario.mutable_world()->mutable_locations()->begin(),
              scenario.mutable_world()->mutable_locations()->end(),
              [](const auto& left, const auto& right) {
                  return left.id() < right.id();
              });
    std::sort(scenario.mutable_vehicles()->begin(), scenario.mutable_vehicles()->end(),
              [](const auto& left, const auto& right) {
                  return left.id() < right.id();
              });
    for (auto& vehicle : *scenario.mutable_vehicles()) {
        std::sort(vehicle.mutable_capabilities()->begin(),
                  vehicle.mutable_capabilities()->end());
        std::sort(vehicle.mutable_terrain_access()->begin(),
                  vehicle.mutable_terrain_access()->end());
    }
    std::sort(scenario.mutable_tasks()->begin(), scenario.mutable_tasks()->end(),
              [](const auto& left, const auto& right) {
                  return left.id() < right.id();
              });
    std::sort(scenario.mutable_events()->begin(), scenario.mutable_events()->end(),
              [](const auto& left, const auto& right) {
                  if (left.tick() != right.tick()) {
                      return left.tick() < right.tick();
                  }
                  return left.id() < right.id();
              });
}

void validate_scenario(const sentinel::v1::Scenario& scenario) {
    if (scenario.schema_version() != 1 || !valid_id(scenario.name())) {
        throw std::invalid_argument("invalid scenario identity");
    }
    if (scenario.step_ms() == 0 || scenario.step_ms() > max_step_ms
        || scenario.max_ticks() == 0 || scenario.max_ticks() > max_ticks) {
        throw std::invalid_argument("invalid scenario clock");
    }
    if (scenario.world().width_mm() <= 0
        || scenario.world().width_mm() > max_world_mm
        || scenario.world().height_mm() <= 0
        || scenario.world().height_mm() > max_world_mm
        || scenario.world().grid_cell_mm() <= 0
        || scenario.world().grid_cell_mm() > max_world_mm
        || scenario.world().width_mm() % scenario.world().grid_cell_mm() != 0
        || scenario.world().height_mm() % scenario.world().grid_cell_mm() != 0) {
        throw std::invalid_argument("invalid world bounds");
    }
    const auto grid_width =
        scenario.world().width_mm() / scenario.world().grid_cell_mm() + 1;
    const auto grid_height =
        scenario.world().height_mm() / scenario.world().grid_cell_mm() + 1;
    if (grid_width > max_grid_cells || grid_height > max_grid_cells
        || grid_width * grid_height > max_grid_cells) {
        throw std::invalid_argument("world grid is too large");
    }

    const auto policy = scenario.allocation_policy();
    if (policy != sentinel::v1::ALLOCATION_POLICY_SCRIPTED
        && policy != sentinel::v1::ALLOCATION_POLICY_NEAREST_CAPABLE) {
        throw std::invalid_argument("unsupported allocation policy");
    }

    require_unique_ids(
        scenario.network_profiles(), [](const auto& value) -> const auto& {
            return value.id();
        },
        "network profile");
    if (!valid_id(scenario.network_profile())
        || !network_profile_exists(scenario, scenario.network_profile())) {
        throw std::invalid_argument("active network profile is missing");
    }
    for (const auto& profile : scenario.network_profiles()) {
        if (profile.latency_ticks() > scenario.max_ticks()) {
            throw std::invalid_argument("invalid network profile");
        }
    }

    require_unique_ids(
        scenario.world().regions(), [](const auto& value) -> const auto& {
            return value.id();
        },
        "region");
    for (const auto& region : scenario.world().regions()) {
        if (!valid_region_kind(region.kind())
            || region.minimum().x_mm() >= region.maximum().x_mm()
            || region.minimum().y_mm() >= region.maximum().y_mm()
            || !point_in_world(scenario, region.minimum())
            || !point_in_world(scenario, region.maximum())
            || (region.kind() == sentinel::v1::REGION_KIND_TERRAIN
                && !valid_id(region.terrain()))) {
            throw std::invalid_argument("invalid region");
        }
    }

    require_unique_ids(
        scenario.world().locations(), [](const auto& value) -> const auto& {
            return value.id();
        },
        "service location");
    for (const auto& location : scenario.world().locations()) {
        if (!valid_location_kind(location.kind()) || location.radius_mm() <= 0
            || location.radius_mm() > max_world_mm
            || !point_in_world(scenario, location.position())
            || !grid_aligned(scenario, location.position())
            || location.charge_mj_per_tick() < 0
            || location.charge_mj_per_tick() > max_energy_mj
            || (location.kind() == sentinel::v1::LOCATION_KIND_CHARGING
                && location.charge_mj_per_tick() == 0)
            || (location.kind() == sentinel::v1::LOCATION_KIND_RETURN
                && location.charge_mj_per_tick() != 0)) {
            throw std::invalid_argument("invalid service location");
        }
    }

    require_unique_ids(
        scenario.vehicles(), [](const auto& value) -> const auto& {
            return value.id();
        },
        "vehicle");
    if (scenario.vehicles().empty()) {
        throw std::invalid_argument("scenario has no vehicles");
    }
    for (const auto& vehicle : scenario.vehicles()) {
        if (vehicle.kind().empty() || vehicle.max_speed_mm_s() <= 0
            || vehicle.max_speed_mm_s() > max_speed_mm_s
            || vehicle.initial_energy_mj() <= 0
            || vehicle.initial_energy_mj() > max_energy_mj
            || vehicle.energy_cost_mj_per_meter() < 0
            || vehicle.payload_grams() < 0
            || vehicle.payload_grams() > max_payload_grams
            || vehicle.capabilities().empty()
            || !point_in_world(scenario, vehicle.initial_position())
            || !grid_aligned(scenario, vehicle.initial_position())) {
            throw std::invalid_argument("invalid vehicle");
        }
        std::set<int> capabilities;
        for (const auto capability : vehicle.capabilities()) {
            if (!valid_capability(
                    static_cast<sentinel::v1::Capability>(capability))
                || !capabilities.insert(static_cast<int>(capability)).second) {
                throw std::invalid_argument("invalid vehicle capability");
            }
        }
        std::set<std::string> terrain_access;
        for (const auto& terrain : vehicle.terrain_access()) {
            if (!valid_id(terrain) || !terrain_access.insert(terrain).second) {
                throw std::invalid_argument("invalid terrain access");
            }
        }
        if (!vehicle.return_location_id().empty()) {
            const auto* location =
                location_by_id(scenario, vehicle.return_location_id());
            if (!location
                || location->kind() != sentinel::v1::LOCATION_KIND_RETURN) {
                throw std::invalid_argument("invalid return location");
            }
        }
    }

    require_unique_ids(
        scenario.tasks(), [](const auto& value) -> const auto& {
            return value.id();
        },
        "task");
    if (scenario.tasks().empty()) {
        throw std::invalid_argument("scenario has no tasks");
    }
    for (const auto& task : scenario.tasks()) {
        if (task.kind().empty() || !valid_capability(task.required_capability())
            || task.payload_required_grams() < 0
            || task.payload_required_grams() > max_payload_grams
            || task.deadline_tick() == 0
            || task.deadline_tick() > scenario.max_ticks()
            || task.completion_radius_mm() <= 0
            || task.completion_radius_mm() > max_world_mm
            || task.service_ticks() == 0
            || task.service_ticks() > scenario.max_ticks()
            || task.service_energy_mj_per_tick() < 0
            || !point_in_world(scenario, task.target())
            || !grid_aligned(scenario, task.target())) {
            throw std::invalid_argument("invalid task");
        }
        if (policy == sentinel::v1::ALLOCATION_POLICY_SCRIPTED
            && task.assigned_agent_id().empty()) {
            throw std::invalid_argument("scripted task has no owner");
        }
        if (!task.assigned_agent_id().empty()) {
            const auto* vehicle =
                vehicle_by_id(scenario, task.assigned_agent_id());
            if (!vehicle
                || !has_capability(*vehicle, task.required_capability())
                || vehicle->payload_grams() < task.payload_required_grams()) {
                throw std::invalid_argument("task assignment is infeasible");
            }
        } else if (!std::any_of(
                       scenario.vehicles().begin(), scenario.vehicles().end(),
                       [&task](const auto& vehicle) {
                           return has_capability(
                                      vehicle, task.required_capability())
                                  && vehicle.payload_grams()
                                         >= task.payload_required_grams();
                       })) {
            throw std::invalid_argument("task has no capable vehicle");
        }
    }

    require_unique_ids(
        scenario.events(), [](const auto& value) -> const auto& {
            return value.id();
        },
        "tape event");
    for (const auto& event : scenario.events()) {
        if (event.tick() >= scenario.max_ticks()
            || !valid_event_kind(event.kind())
            || event.value_min() > event.value_max()) {
            throw std::invalid_argument("invalid tape event");
        }
        if (event.kind() == sentinel::v1::TAPE_EVENT_KIND_RELEASE_TASK
            && !task_exists(scenario, event.target_id())) {
            throw std::invalid_argument("release event targets unknown task");
        }
        if ((event.kind() == sentinel::v1::TAPE_EVENT_KIND_DISABLE_VEHICLE
             || event.kind() == sentinel::v1::TAPE_EVENT_KIND_ENERGY_DELTA)
            && !vehicle_by_id(scenario, event.target_id())) {
            throw std::invalid_argument("vehicle event targets unknown vehicle");
        }
        if (event.kind() == sentinel::v1::TAPE_EVENT_KIND_SET_NETWORK_PROFILE
            && !network_profile_exists(scenario, event.text_value())) {
            throw std::invalid_argument("network event targets unknown profile");
        }
        if (event.kind() == sentinel::v1::TAPE_EVENT_KIND_SET_REGION_CLOSED
            && !region_exists(scenario, event.target_id())) {
            throw std::invalid_argument("region event targets unknown region");
        }
    }
}

std::string scenario_hash(const sentinel::v1::Scenario& scenario) {
    auto normalized = scenario;
    normalize_scenario(normalized);
    return hash_bytes(sentinel::protocol::deterministic_bytes(normalized));
}

}
