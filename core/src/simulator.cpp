#include <sentinel/core/simulator.hpp>

#include <sentinel/core/hash.hpp>
#include <sentinel/core/scenario.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace sentinel::core {

namespace {

std::int64_t clamp_velocity(std::int64_t value, std::int64_t maximum) {
    return std::clamp(value, -maximum, maximum);
}

}

Simulator::Simulator(sentinel::v1::Scenario scenario) : scenario_(std::move(scenario)) {
    normalize_scenario(scenario_);
    validate_scenario(scenario_);
    initialize_vehicles();
    initialize_tasks();
    record_hash();
}

void Simulator::initialize_vehicles() {
    vehicles_.reserve(static_cast<std::size_t>(scenario_.vehicles_size()));
    for (const auto& spec : scenario_.vehicles()) {
        Vehicle current;
        current.spec = spec;
        current.x_mm = spec.initial_position().x_mm();
        current.y_mm = spec.initial_position().y_mm();
        current.energy_mj = spec.initial_energy_mj();
        vehicles_.push_back(std::move(current));
    }
}

void Simulator::initialize_tasks() {
    tasks_.reserve(static_cast<std::size_t>(scenario_.tasks_size()));
    for (const auto& spec : scenario_.tasks()) {
        Task current;
        current.spec = spec;
        tasks_.push_back(std::move(current));
    }
}

sentinel::v1::ObservationBatch Simulator::observe() const {
    sentinel::v1::ObservationBatch result;
    result.set_tick(tick_);
    result.set_finished(finished());
    for (const auto& current : vehicles_) {
        result.add_observations()->CopyFrom(observation_for(current));
    }
    if (result.finished()) {
        result.mutable_summary()->CopyFrom(summary());
    }
    return result;
}

sentinel::v1::Envelope Simulator::observation_for(const Vehicle& current) const {
    sentinel::v1::Envelope envelope;
    envelope.set_schema_version(1);
    envelope.set_sequence(tick_ + 1);
    envelope.set_simulation_time_ms(tick_ * scenario_.step_ms());
    envelope.set_sender_id("sim");
    envelope.set_recipient_id(current.spec.id());
    auto* observation = envelope.mutable_observation();
    observation->set_tick(tick_);
    observation->set_step_ms(scenario_.step_ms());
    observation->mutable_self()->CopyFrom(vehicle_state(current));
    observation->mutable_world()->CopyFrom(scenario_.world());
    for (const auto& current_task : tasks_) {
        if (current_task.status != sentinel::v1::TASK_STATUS_PENDING) {
            continue;
        }
        if (current_task.spec.assigned_agent_id() != current.spec.id()) {
            continue;
        }
        observation->add_assigned_tasks()->CopyFrom(task_state(current_task));
    }
    return envelope;
}

sentinel::v1::ObservationBatch Simulator::step(const sentinel::v1::ActionBatch& actions) {
    if (finished()) {
        throw std::logic_error("simulation already finished");
    }
    if (actions.tick() != tick_) {
        throw std::invalid_argument("action batch tick mismatch");
    }
    const auto commands = read_commands(actions);
    apply_commands(commands);
    advance_motion(commands);
    advance_tasks(commands);
    ++tick_;
    record_hash();
    return observe();
}

sentinel::v1::SimulationSummary Simulator::summary() const {
    sentinel::v1::SimulationSummary result;
    const auto completed = completed_task_count();
    result.set_success(completed == static_cast<std::uint32_t>(tasks_.size()));
    result.set_ticks(tick_);
    result.set_terminal_hash(state_hash_);
    for (const auto& value : tick_hashes_) {
        result.add_tick_hashes(value);
    }
    result.set_completed_tasks(completed);
    result.set_total_tasks(static_cast<std::uint32_t>(tasks_.size()));
    result.set_active_agents(active_vehicle_count());
    return result;
}

std::uint32_t Simulator::completed_task_count() const {
    return static_cast<std::uint32_t>(std::count_if(
        tasks_.begin(), tasks_.end(),
        [](const Task& current) {
            return current.status == sentinel::v1::TASK_STATUS_COMPLETED;
        }));
}

std::uint32_t Simulator::active_vehicle_count() const {
    return static_cast<std::uint32_t>(std::count_if(
        vehicles_.begin(), vehicles_.end(),
        [](const Vehicle& current) {
            return current.energy_mj > 0;
        }));
}

std::string Simulator::state_hash() const {
    return state_hash_;
}

bool Simulator::finished() const {
    return all_tasks_terminal() || tick_ >= scenario_.max_ticks();
}

bool Simulator::all_tasks_terminal() const {
    return std::all_of(
        tasks_.begin(), tasks_.end(),
        [](const Task& current) {
            return current.status != sentinel::v1::TASK_STATUS_PENDING;
        });
}

std::uint64_t Simulator::tick() const {
    return tick_;
}

const sentinel::v1::Scenario& Simulator::scenario() const {
    return scenario_;
}

Simulator::Vehicle& Simulator::vehicle(std::string_view id) {
    const auto current = std::find_if(vehicles_.begin(), vehicles_.end(),
                                      [&](const Vehicle& value) { return value.spec.id() == id; });
    if (current == vehicles_.end()) {
        throw std::invalid_argument("unknown vehicle");
    }
    return *current;
}

const Simulator::Vehicle& Simulator::vehicle(std::string_view id) const {
    const auto current = std::find_if(
        vehicles_.begin(), vehicles_.end(),
        [&](const Vehicle& value) { return value.spec.id() == id; });
    if (current == vehicles_.end()) {
        throw std::invalid_argument("unknown vehicle");
    }
    return *current;
}

sentinel::v1::VehicleState Simulator::vehicle_state(const Vehicle& current) const {
    sentinel::v1::VehicleState result;
    result.set_id(current.spec.id());
    result.set_kind(current.spec.kind());
    result.mutable_position()->set_x_mm(current.x_mm);
    result.mutable_position()->set_y_mm(current.y_mm);
    result.set_velocity_x_mm_s(current.velocity_x_mm_s);
    result.set_velocity_y_mm_s(current.velocity_y_mm_s);
    result.set_max_speed_mm_s(current.spec.max_speed_mm_s());
    result.set_energy_mj(current.energy_mj);
    result.set_payload_grams(current.spec.payload_grams());
    for (const auto capability : current.spec.capabilities()) {
        result.add_capabilities(static_cast<sentinel::v1::Capability>(capability));
    }
    result.set_active(current.energy_mj > 0);
    result.set_initial_energy_mj(current.spec.initial_energy_mj());
    result.set_energy_cost_mj_per_meter(current.spec.energy_cost_mj_per_meter());
    return result;
}

sentinel::v1::TaskState Simulator::task_state(const Task& current) const {
    sentinel::v1::TaskState result;
    result.set_id(current.spec.id());
    result.set_kind(current.spec.kind());
    result.mutable_target()->CopyFrom(current.spec.target());
    result.set_required_capability(current.spec.required_capability());
    result.set_payload_required_grams(current.spec.payload_required_grams());
    result.set_deadline_tick(current.spec.deadline_tick());
    result.set_completion_radius_mm(current.spec.completion_radius_mm());
    result.set_assigned_agent_id(current.spec.assigned_agent_id());
    result.set_status(current.status);
    result.set_service_ticks(current.spec.service_ticks());
    result.set_progress_ticks(current.progress_ticks);
    return result;
}

std::vector<Simulator::Command> Simulator::read_commands(
    const sentinel::v1::ActionBatch& actions) const {
    std::vector<Command> commands;
    commands.reserve(static_cast<std::size_t>(actions.actions_size()));
    for (const auto& envelope : actions.actions()) {
        if (!envelope.has_action()) {
            throw std::invalid_argument("action envelope does not contain an action");
        }
        if (envelope.sender_id().empty()) {
            throw std::invalid_argument("action sender is required");
        }
        if (envelope.recipient_id() != "sim") {
            throw std::invalid_argument("action must be addressed to the simulator");
        }
        if (envelope.action().tick() != tick_) {
            throw std::invalid_argument("action tick mismatch");
        }
        vehicle(envelope.sender_id());
        const auto duplicate = std::find_if(
            commands.begin(), commands.end(),
            [&](const Command& command) {
                return command.agent_id == envelope.sender_id();
            });
        if (duplicate != commands.end()) {
            throw std::invalid_argument("duplicate action sender");
        }
        Command command;
        command.agent_id = envelope.sender_id();
        command.velocity_x_mm_s = envelope.action().velocity_x_mm_s();
        command.velocity_y_mm_s = envelope.action().velocity_y_mm_s();
        command.reports.assign(
            envelope.action().task_reports().begin(),
            envelope.action().task_reports().end());
        commands.push_back(std::move(command));
    }
    return commands;
}

const Simulator::Command* Simulator::command_for(
    const std::vector<Command>& commands, std::string_view agent_id) const {
    const auto found = std::find_if(
        commands.begin(), commands.end(),
        [&](const Command& command) {
            return command.agent_id == agent_id;
        });
    return found == commands.end() ? nullptr : &*found;
}

void Simulator::apply_commands(const std::vector<Command>& commands) {
    for (auto& current : vehicles_) {
        const auto* command = command_for(commands, current.spec.id());
        if (command == nullptr) {
            current.velocity_x_mm_s = 0;
            current.velocity_y_mm_s = 0;
            continue;
        }
        apply_command(current, *command);
    }
}

void Simulator::apply_command(Vehicle& current, const Command& command) {
    if (current.energy_mj <= 0) {
        current.velocity_x_mm_s = 0;
        current.velocity_y_mm_s = 0;
        return;
    }
    current.velocity_x_mm_s = clamp_velocity(
        command.velocity_x_mm_s, current.spec.max_speed_mm_s());
    current.velocity_y_mm_s = clamp_velocity(
        command.velocity_y_mm_s, current.spec.max_speed_mm_s());
}

void Simulator::advance_motion(const std::vector<Command>& commands) {
    for (auto& current : vehicles_) {
        if (command_for(commands, current.spec.id()) == nullptr) {
            continue;
        }
        advance_vehicle(current);
    }
}

void Simulator::advance_vehicle(Vehicle& current) {
    if (current.energy_mj <= 0) {
        return;
    }
    const auto previous_x = current.x_mm;
    const auto previous_y = current.y_mm;
    const auto step_ms = static_cast<std::int64_t>(scenario_.step_ms());
    const auto requested_x = current.x_mm + current.velocity_x_mm_s * step_ms / 1000;
    const auto requested_y = current.y_mm + current.velocity_y_mm_s * step_ms / 1000;
    current.x_mm = std::clamp(
        requested_x, std::int64_t{0}, scenario_.world().width_mm());
    current.y_mm = std::clamp(
        requested_y, std::int64_t{0}, scenario_.world().height_mm());
    const auto dx = static_cast<long double>(current.x_mm - previous_x);
    const auto dy = static_cast<long double>(current.y_mm - previous_y);
    consume_motion_energy(
        current, static_cast<std::int64_t>(std::llround(std::hypotl(dx, dy))));
}

void Simulator::consume_motion_energy(Vehicle& current, std::int64_t distance_mm) {
    if (distance_mm <= 0) {
        return;
    }
    const auto numerator = distance_mm * current.spec.energy_cost_mj_per_meter();
    const auto consumed = (numerator + 999) / 1000;
    current.energy_mj = std::max<std::int64_t>(0, current.energy_mj - consumed);
    if (current.energy_mj == 0) {
        current.velocity_x_mm_s = 0;
        current.velocity_y_mm_s = 0;
    }
}

void Simulator::advance_tasks(const std::vector<Command>& commands) {
    for (auto& current : tasks_) {
        advance_task(current, commands);
    }
}

void Simulator::advance_task(
    Task& current, const std::vector<Command>& commands) {
    if (current.status != sentinel::v1::TASK_STATUS_PENDING) {
        return;
    }
    if (deadline_elapsed(current)) {
        current.status = sentinel::v1::TASK_STATUS_MISSED;
        current.progress_ticks = 0;
        return;
    }
    const auto& owner = vehicle(current.spec.assigned_agent_id());
    const auto* command = command_for(commands, owner.spec.id());
    if (!vehicle_can_service(owner, current)
        || !vehicle_is_at_task(owner, current)
        || !command_reports_work(command, current.spec.id())) {
        current.progress_ticks = 0;
        return;
    }
    ++current.progress_ticks;
    if (current.progress_ticks >= current.spec.service_ticks()) {
        current.status = sentinel::v1::TASK_STATUS_COMPLETED;
    }
}

bool Simulator::command_reports_work(
    const Command* command, std::string_view task_id) const {
    if (command == nullptr) {
        return false;
    }
    return std::any_of(
        command->reports.begin(), command->reports.end(),
        [&](const sentinel::v1::TaskReport& report) {
            return report.task_id() == task_id
                   && report.kind() == sentinel::v1::TASK_REPORT_KIND_WORKING;
        });
}

bool Simulator::vehicle_can_service(
    const Vehicle& current, const Task& current_task) const {
    if (current.energy_mj <= 0
        || current.spec.payload_grams() < current_task.spec.payload_required_grams()) {
        return false;
    }
    return std::find(
               current.spec.capabilities().begin(),
               current.spec.capabilities().end(),
               current_task.spec.required_capability())
           != current.spec.capabilities().end();
}

bool Simulator::vehicle_is_at_task(
    const Vehicle& current, const Task& current_task) const {
    const auto dx = current.x_mm - current_task.spec.target().x_mm();
    const auto dy = current.y_mm - current_task.spec.target().y_mm();
    const auto radius = current_task.spec.completion_radius_mm();
    return dx * dx + dy * dy <= radius * radius;
}

bool Simulator::deadline_elapsed(const Task& current) const {
    return tick_ + 1 >= current.spec.deadline_tick();
}

void Simulator::append_vehicle_hash(
    HashBuilder& hash, const Vehicle& current) const {
    hash.text(current.spec.id());
    hash.signed_integer(current.x_mm);
    hash.signed_integer(current.y_mm);
    hash.signed_integer(current.velocity_x_mm_s);
    hash.signed_integer(current.velocity_y_mm_s);
    hash.signed_integer(current.energy_mj);
}

void Simulator::append_task_hash(
    HashBuilder& hash, const Task& current) const {
    hash.text(current.spec.id());
    hash.unsigned_integer(current.status);
    hash.unsigned_integer(current.progress_ticks);
}

void Simulator::record_hash() {
    HashBuilder hash;
    hash.unsigned_integer(tick_);
    for (const auto& current : vehicles_) {
        append_vehicle_hash(hash, current);
    }
    for (const auto& current : tasks_) {
        append_task_hash(hash, current);
    }
    state_hash_ = hash.finish();
    tick_hashes_.push_back(state_hash_);
}

}
