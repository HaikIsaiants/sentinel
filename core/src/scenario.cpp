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
    if (scenario.allocation_policy() != sentinel::v1::ALLOCATION_POLICY_SCRIPTED) {
        throw std::invalid_argument("only scripted allocation is available");
    }
    std::set<std::string> vehicles;
    for (const auto& vehicle : scenario.vehicles()) {
        if (vehicle.id().empty() || !vehicles.insert(vehicle.id()).second || vehicle.max_speed_mm_s() <= 0) {
            throw std::invalid_argument("invalid vehicle");
        }
    }
    std::set<std::string> tasks;
    for (const auto& task : scenario.tasks()) {
        if (task.id().empty() || !tasks.insert(task.id()).second || task.deadline_tick() == 0
            || task.service_ticks() == 0 || !vehicles.contains(task.assigned_agent_id())) {
            throw std::invalid_argument("invalid task");
        }
    }
}

std::string scenario_hash(const sentinel::v1::Scenario& scenario) {
    return hash_bytes(sentinel::protocol::deterministic_bytes(scenario));
}

}
