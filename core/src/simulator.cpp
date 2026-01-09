#include <sentinel/core/simulator.hpp>

#include <sentinel/core/hash.hpp>
#include <sentinel/core/scenario.hpp>

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace sentinel::core {

namespace {

std::int64_t clamp_velocity(std::int64_t value, std::int64_t maximum) {
    return std::clamp(value, -maximum, maximum);
}

std::int64_t squared_distance(
    std::int64_t ax, std::int64_t ay, std::int64_t bx, std::int64_t by) {
    const auto dx = ax - bx;
    const auto dy = ay - by;
    return dx * dx + dy * dy;
}

}

Simulator::Simulator(sentinel::v1::Scenario scenario)
    : scenario_(std::move(scenario)), network_(scenario_.seed(), scenario_.step_ms()) {
    normalize_scenario(scenario_);
    validate_scenario(scenario_);
    const auto profile = std::find_if(
        scenario_.network_profiles().begin(), scenario_.network_profiles().end(),
        [&](const auto& value) { return value.id() == scenario_.network_profile(); });
    if (profile == scenario_.network_profiles().end()) {
        throw std::invalid_argument("active network profile is missing");
    }
    network_.set_profile(*profile);
    for (const auto& spec : scenario_.vehicles()) {
        vehicles_.push_back(Vehicle{
            spec,
            spec.initial_position().x_mm(),
            spec.initial_position().y_mm(),
            0,
            0,
            spec.initial_energy_mj()
        });
    }
    for (const auto& spec : scenario_.tasks()) {
        tasks_.push_back(Task{spec});
    }
    record_hash();
}

sentinel::v1::ObservationBatch Simulator::observe() const {
    sentinel::v1::ObservationBatch result;
    result.set_tick(tick_);
    result.set_finished(finished());
    for (const auto& current : vehicles_) {
        auto* envelope = result.add_observations();
        envelope->set_schema_version(1);
        envelope->set_sequence(tick_ + 1);
        envelope->set_simulation_time_ms(tick_ * scenario_.step_ms());
        envelope->set_sender_id("sim");
        envelope->set_recipient_id(current.spec.id());
        auto* observation = envelope->mutable_observation();
        observation->set_tick(tick_);
        observation->set_step_ms(scenario_.step_ms());
        observation->set_network_profile(scenario_.network_profile());
        observation->mutable_self()->CopyFrom(vehicle_state(current));
        observation->mutable_world()->CopyFrom(scenario_.world());
        for (const auto& peer : vehicles_) {
            if (peer.spec.id() != current.spec.id()) {
                observation->add_peer_ids(peer.spec.id());
            }
        }
        for (const auto& message : delivered_messages_) {
            if (message.recipient_id() == current.spec.id()) {
                observation->add_delivered_messages()->CopyFrom(message);
            }
        }
        for (const auto& task : tasks_) {
            if (task.spec.assigned_agent_id() == current.spec.id()
                && task.status == sentinel::v1::TASK_STATUS_PENDING) {
                observation->add_assigned_tasks()->CopyFrom(task_state(task));
            }
        }
    }
    if (result.finished()) {
        result.mutable_summary()->CopyFrom(summary());
    }
    return result;
}

sentinel::v1::ObservationBatch Simulator::step(const sentinel::v1::ActionBatch& actions) {
    if (finished()) {
        throw std::logic_error("simulation already finished");
    }
    if (actions.tick() != tick_) {
        throw std::invalid_argument("action batch tick mismatch");
    }
    const auto outgoing = apply_actions(actions);
    const auto network = network_.step(tick_, outgoing);
    delivered_messages_ = network.delivered;
    advance_motion();
    advance_tasks(actions);
    ++tick_;
    record_hash();
    return observe();
}

sentinel::v1::SimulationSummary Simulator::summary() const {
    sentinel::v1::SimulationSummary result;
    const auto completed = static_cast<std::uint32_t>(std::count_if(
        tasks_.begin(), tasks_.end(),
        [](const Task& task) { return task.status == sentinel::v1::TASK_STATUS_COMPLETED; }));
    result.set_success(completed == tasks_.size());
    result.set_ticks(tick_);
    result.set_terminal_hash(state_hash_);
    for (const auto& value : tick_hashes_) {
        result.add_tick_hashes(value);
    }
    result.set_completed_tasks(completed);
    result.set_total_tasks(static_cast<std::uint32_t>(tasks_.size()));
    result.set_active_agents(static_cast<std::uint32_t>(vehicles_.size()));
    result.set_communication_bytes(network_.communication_bytes());
    result.set_communication_messages(network_.communication_messages());
    result.set_delivered_messages(network_.delivered_messages());
    return result;
}

std::string Simulator::state_hash() const {
    return state_hash_;
}

bool Simulator::finished() const {
    const auto all_complete = std::all_of(
        tasks_.begin(), tasks_.end(),
        [](const Task& task) { return task.status != sentinel::v1::TASK_STATUS_PENDING; });
    return all_complete || tick_ >= scenario_.max_ticks();
}

std::uint64_t Simulator::tick() const {
    return tick_;
}

const sentinel::v1::Scenario& Simulator::scenario() const {
    return scenario_;
}

Simulator::Vehicle& Simulator::vehicle(std::string_view id) {
    const auto current = std::find_if(
        vehicles_.begin(), vehicles_.end(),
        [&](const Vehicle& value) { return value.spec.id() == id; });
    if (current == vehicles_.end()) {
        throw std::invalid_argument("unknown vehicle");
    }
    return *current;
}

const Simulator::Vehicle& Simulator::vehicle(std::string_view id) const {
    return const_cast<Simulator*>(this)->vehicle(id);
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
    result.set_active(true);
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

std::vector<sentinel::v1::NetworkMessage> Simulator::apply_actions(
    const sentinel::v1::ActionBatch& actions) {
    std::vector<std::string> seen;
    std::vector<sentinel::v1::NetworkMessage> outgoing;
    for (const auto& envelope : actions.actions()) {
        if (!envelope.has_action() || envelope.sender_id().empty()
            || std::find(seen.begin(), seen.end(), envelope.sender_id()) != seen.end()) {
            throw std::invalid_argument("invalid or duplicate action");
        }
        seen.push_back(envelope.sender_id());
        auto& current = vehicle(envelope.sender_id());
        current.velocity_x_mm_s = clamp_velocity(
            envelope.action().velocity_x_mm_s(), current.spec.max_speed_mm_s());
        current.velocity_y_mm_s = clamp_velocity(
            envelope.action().velocity_y_mm_s(), current.spec.max_speed_mm_s());
        for (const auto& message : envelope.action().outgoing_messages()) {
            if (message.sender_id() != envelope.sender_id()) {
                throw std::invalid_argument("network sender does not match action");
            }
            outgoing.push_back(message);
        }
    }
    std::sort(outgoing.begin(), outgoing.end(), [](const auto& left, const auto& right) {
        if (left.sender_id() != right.sender_id()) {
            return left.sender_id() < right.sender_id();
        }
        if (left.recipient_id() != right.recipient_id()) {
            return left.recipient_id() < right.recipient_id();
        }
        return left.version() < right.version();
    });
    return outgoing;
}

void Simulator::advance_motion() {
    for (auto& current : vehicles_) {
        const auto dx =
            (current.velocity_x_mm_s * static_cast<std::int64_t>(scenario_.step_ms())) / 1000;
        const auto dy =
            (current.velocity_y_mm_s * static_cast<std::int64_t>(scenario_.step_ms())) / 1000;
        current.x_mm = std::clamp(
            current.x_mm + dx, std::int64_t{0}, scenario_.world().width_mm());
        current.y_mm = std::clamp(
            current.y_mm + dy, std::int64_t{0}, scenario_.world().height_mm());
    }
}

void Simulator::advance_tasks(const sentinel::v1::ActionBatch& actions) {
    for (auto& task : tasks_) {
        if (task.status != sentinel::v1::TASK_STATUS_PENDING) {
            continue;
        }
        if (tick_ + 1 >= task.spec.deadline_tick()) {
            task.status = sentinel::v1::TASK_STATUS_MISSED;
            continue;
        }
        const auto action = std::find_if(
            actions.actions().begin(), actions.actions().end(),
            [&](const auto& envelope) {
                return envelope.sender_id() == task.spec.assigned_agent_id();
            });
        const auto& owner = vehicle(task.spec.assigned_agent_id());
        const auto radius = task.spec.completion_radius_mm();
        const auto close = squared_distance(
                               owner.x_mm, owner.y_mm,
                               task.spec.target().x_mm(), task.spec.target().y_mm())
                           <= radius * radius;
        const auto working =
            action != actions.actions().end()
            && std::any_of(
                action->action().task_reports().begin(),
                action->action().task_reports().end(),
                [&](const auto& report) {
                    return report.task_id() == task.spec.id()
                           && report.kind() == sentinel::v1::TASK_REPORT_KIND_WORKING;
                });
        if (close && working) {
            ++task.progress_ticks;
            if (task.progress_ticks >= task.spec.service_ticks()) {
                task.status = sentinel::v1::TASK_STATUS_COMPLETED;
            }
        } else {
            task.progress_ticks = 0;
        }
    }
}

void Simulator::record_hash() {
    HashBuilder hash;
    hash.unsigned_integer(tick_);
    for (const auto& current : vehicles_) {
        hash.text(current.spec.id());
        hash.signed_integer(current.x_mm);
        hash.signed_integer(current.y_mm);
        hash.signed_integer(current.velocity_x_mm_s);
        hash.signed_integer(current.velocity_y_mm_s);
    }
    for (const auto& task : tasks_) {
        hash.text(task.spec.id());
        hash.unsigned_integer(task.status);
        hash.unsigned_integer(task.progress_ticks);
    }
    network_.append_hash(hash);
    state_hash_ = hash.finish();
    tick_hashes_.push_back(state_hash_);
}

}
