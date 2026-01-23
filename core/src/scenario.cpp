#include <sentinel/core/scenario.hpp>

#include <sentinel/core/hash.hpp>
#include <sentinel/protocol/framing.hpp>

#include <google/protobuf/text_format.h>

#include <algorithm>
#include <fstream>
#include <iterator>
#include <set>
#include <stdexcept>

namespace sentinel::core {

sentinel::v1::Scenario load_scenario(const std::filesystem::path& path) {
    std::ifstream input(path, std::ios::binary);
    if (!input) {
        throw std::runtime_error("failed to open scenario");
    }
    const std::string text{std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
    sentinel::v1::Scenario scenario;
    if (!google::protobuf::TextFormat::ParseFromString(text, &scenario)) {
        throw std::runtime_error("invalid scenario textproto");
    }
    normalize_scenario(scenario);
    validate_scenario(scenario);
    return scenario;
}

void normalize_scenario(sentinel::v1::Scenario& scenario) {
    std::sort(scenario.mutable_vehicles()->begin(), scenario.mutable_vehicles()->end(),
              [](const auto& left, const auto& right) { return left.id() < right.id(); });
    std::sort(scenario.mutable_tasks()->begin(), scenario.mutable_tasks()->end(),
              [](const auto& left, const auto& right) { return left.id() < right.id(); });
    std::sort(scenario.mutable_events()->begin(), scenario.mutable_events()->end(),
              [](const auto& left, const auto& right) {
                  if (left.tick() != right.tick()) {
                      return left.tick() < right.tick();
                  }
                  return left.id() < right.id();
              });
}

void validate_scenario(const sentinel::v1::Scenario& scenario) {
    if (scenario.schema_version() != 1 || scenario.name().empty()) {
        throw std::invalid_argument("unsupported scenario");
    }
    if (scenario.step_ms() == 0 || scenario.max_ticks() == 0) {
        throw std::invalid_argument("invalid scenario clock");
    }
    if (scenario.world().width_mm() <= 0 || scenario.world().height_mm() <= 0
        || scenario.world().grid_cell_mm() <= 0) {
        throw std::invalid_argument("invalid world bounds");
    }
    const auto policy = scenario.allocation_policy();
    if (policy != sentinel::v1::ALLOCATION_POLICY_SCRIPTED
        && policy != sentinel::v1::ALLOCATION_POLICY_NEAREST_CAPABLE) {
        throw std::invalid_argument("unsupported allocation policy");
    }
    std::set<std::string> vehicles;
    for (const auto& vehicle : scenario.vehicles()) {
        if (vehicle.id().empty() || !vehicles.insert(vehicle.id()).second || vehicle.max_speed_mm_s() <= 0) {
            throw std::invalid_argument("invalid vehicle");
        }
    }
    std::set<std::string> tasks;
    for (const auto& task : scenario.tasks()) {
        const auto has_known_owner = task.assigned_agent_id().empty()
            || vehicles.contains(task.assigned_agent_id());
        const auto has_required_owner = policy != sentinel::v1::ALLOCATION_POLICY_SCRIPTED
            || !task.assigned_agent_id().empty();
        if (task.id().empty() || !tasks.insert(task.id()).second || task.deadline_tick() == 0
            || task.service_ticks() == 0 || !has_known_owner || !has_required_owner) {
            throw std::invalid_argument("invalid task");
        }
    }
    std::set<std::string> profiles;
    for (const auto& profile : scenario.network_profiles()) {
        if (profile.id().empty() || !profiles.insert(profile.id()).second) {
            throw std::invalid_argument("invalid network profile");
        }
    }
    if (!profiles.contains(scenario.network_profile())) {
        throw std::invalid_argument("active network profile is missing");
    }
    std::set<std::string> locations;
    for (const auto& location : scenario.world().locations()) {
        if (location.id().empty() || !locations.insert(location.id()).second
            || location.radius_mm() <= 0) {
            throw std::invalid_argument("invalid service location");
        }
    }
    std::set<std::string> events;
    for (const auto& event : scenario.events()) {
        if (event.id().empty() || !events.insert(event.id()).second
            || event.tick() >= scenario.max_ticks() || event.value_min() > event.value_max()) {
            throw std::invalid_argument("invalid tape event");
        }
    }
}

std::string scenario_hash(const sentinel::v1::Scenario& scenario) {
    return hash_bytes(sentinel::protocol::deterministic_bytes(scenario));
}

}
