#include <sentinel/core/simulator.hpp>

#include <sentinel/core/hash.hpp>
#include <sentinel/core/scenario.hpp>

#include <algorithm>
#include <cstdlib>
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
    : scenario_(std::move(scenario)),
      rng_streams_(scenario_.seed()),
      network_(scenario_.seed(), scenario_.step_ms()) {
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
            spec.initial_energy_mj(),
            spec.initial_energy_mj(),
            true
        });
    }
    for (const auto& spec : scenario_.tasks()) {
        tasks_.push_back(Task{spec, spec.released()});
    }
    record_hash();
}

sentinel::v1::ObservationBatch Simulator::observe() const {
    sentinel::v1::ObservationBatch result;
    result.set_tick(tick_);
    result.set_finished(finished());
    for (const auto& current : vehicles_) {
        if (!current.active) {
            continue;
        }
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
            if (task.released && task.spec.assigned_agent_id() == current.spec.id()
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
    apply_events();
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
    result.set_active_agents(static_cast<std::uint32_t>(std::count_if(
        vehicles_.begin(), vehicles_.end(),
        [](const Vehicle& vehicle) { return vehicle.active; })));
    result.set_wait_ticks(wait_ticks_);
    result.set_replan_count(replan_count_);
    result.set_recharge_ticks(recharge_ticks_);
    result.set_return_ticks(return_ticks_);
    result.set_travel_distance_mm(travel_distance_mm_);
    result.set_energy_consumed_mj(energy_consumed_mj_);
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
    for (const auto& terrain : current.spec.terrain_access()) {
        result.add_terrain_access(terrain);
    }
    result.set_active(current.active);
    result.set_initial_energy_mj(current.spec.initial_energy_mj());
    result.set_energy_cost_mj_per_meter(current.spec.energy_cost_mj_per_meter());
    result.set_return_location_id(current.spec.return_location_id());
    result.set_energy_capacity_mj(current.energy_capacity_mj);
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
    result.set_service_energy_mj_per_tick(current.spec.service_energy_mj_per_tick());
    result.set_progress_ticks(current.progress_ticks);
    result.set_priority(current.spec.priority());
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
        if (!current.active) {
            throw std::invalid_argument("inactive vehicle produced an action");
        }
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
        if (envelope.action().replanned()) {
            ++replan_count_;
        }
        if (envelope.action().behavior_mode() == sentinel::v1::BEHAVIOR_MODE_WAITING) {
            ++wait_ticks_;
        }
        if (envelope.action().behavior_mode() == sentinel::v1::BEHAVIOR_MODE_RETURNING) {
            ++return_ticks_;
        }
        if (!envelope.action().charge_location_id().empty()) {
            const auto location = std::find_if(
                scenario_.world().locations().begin(), scenario_.world().locations().end(),
                [&](const auto& value) {
                    return value.id() == envelope.action().charge_location_id()
                           && value.kind() == sentinel::v1::LOCATION_KIND_CHARGING;
                });
            if (location == scenario_.world().locations().end()) {
                throw std::invalid_argument("unknown charging location");
            }
            const auto dx = current.x_mm - location->position().x_mm();
            const auto dy = current.y_mm - location->position().y_mm();
            if (dx * dx + dy * dy > location->radius_mm() * location->radius_mm()) {
                throw std::invalid_argument("vehicle is outside charging location");
            }
            current.energy_mj = std::min(
                current.energy_capacity_mj,
                current.energy_mj + location->charge_mj_per_tick());
            ++recharge_ticks_;
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
        if (!current.active) {
            continue;
        }
        const auto dx =
            (current.velocity_x_mm_s * static_cast<std::int64_t>(scenario_.step_ms())) / 1000;
        const auto dy =
            (current.velocity_y_mm_s * static_cast<std::int64_t>(scenario_.step_ms())) / 1000;
        const auto next_x = std::clamp(
            current.x_mm + dx, std::int64_t{0}, scenario_.world().width_mm());
        const auto next_y = std::clamp(
            current.y_mm + dy, std::int64_t{0}, scenario_.world().height_mm());
        const auto distance = std::llabs(next_x - current.x_mm) + std::llabs(next_y - current.y_mm);
        const auto energy = (distance * current.spec.energy_cost_mj_per_meter() + 999) / 1000;
        if (energy > current.energy_mj) {
            current.velocity_x_mm_s = 0;
            current.velocity_y_mm_s = 0;
            continue;
        }
        current.x_mm = next_x;
        current.y_mm = next_y;
        current.energy_mj -= energy;
        travel_distance_mm_ += static_cast<std::uint64_t>(distance);
        energy_consumed_mj_ += static_cast<std::uint64_t>(energy);
    }
}

void Simulator::advance_tasks(const sentinel::v1::ActionBatch& actions) {
    for (auto& task : tasks_) {
        if (!task.released || task.status != sentinel::v1::TASK_STATUS_PENDING) {
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
        if (close && working && owner.active) {
            auto& mutable_owner = vehicle(task.spec.assigned_agent_id());
            const auto service_energy = task.spec.service_energy_mj_per_tick();
            if (service_energy > mutable_owner.energy_mj) {
                task.progress_ticks = 0;
                continue;
            }
            mutable_owner.energy_mj -= service_energy;
            energy_consumed_mj_ += static_cast<std::uint64_t>(service_energy);
            ++task.progress_ticks;
            if (task.progress_ticks >= task.spec.service_ticks()) {
                task.status = sentinel::v1::TASK_STATUS_COMPLETED;
            }
        } else {
            task.progress_ticks = 0;
        }
    }
}

void Simulator::apply_events() {
    while (next_event_ < static_cast<std::size_t>(scenario_.events_size())
           && scenario_.events(static_cast<int>(next_event_)).tick() == tick_) {
        const auto& event = scenario_.events(static_cast<int>(next_event_++));
        dispatch_event(event, resolve_event_value(event));
    }
}

std::int64_t Simulator::resolve_event_value(
    const sentinel::v1::TapeEvent& event) {
    if (event.value_min() == event.value_max()) {
        return event.value_min();
    }
    if (event.rng_stream().empty()) {
        throw std::invalid_argument(
            "variable event value requires an rng stream");
    }
    return rng_streams_.stream(event.rng_stream()).uniform(
        event.value_min(), event.value_max());
}

void Simulator::dispatch_event(
    const sentinel::v1::TapeEvent& event,
    std::int64_t resolved_value) {
    switch (event.kind()) {
    case sentinel::v1::TAPE_EVENT_KIND_RELEASE_TASK:
        release_task(event);
        return;
    case sentinel::v1::TAPE_EVENT_KIND_DISABLE_VEHICLE:
        disable_vehicle(event);
        return;
    case sentinel::v1::TAPE_EVENT_KIND_ENERGY_DELTA:
        adjust_vehicle_energy(event, resolved_value);
        return;
    case sentinel::v1::TAPE_EVENT_KIND_SET_NETWORK_PROFILE:
        select_network_profile(event);
        return;
    case sentinel::v1::TAPE_EVENT_KIND_SET_REGION_CLOSED:
        update_region_access(event);
        return;
    default:
        throw std::invalid_argument("unsupported tape event");
    }
}

void Simulator::release_task(
    const sentinel::v1::TapeEvent& event) {
    auto& current = task(event.target_id());
    if (current.status != sentinel::v1::TASK_STATUS_PENDING) {
        throw std::invalid_argument(
            "cannot release a terminal task");
    }
    current.released = true;
}

void Simulator::disable_vehicle(
    const sentinel::v1::TapeEvent& event) {
    auto& current = vehicle(event.target_id());
    current.active = false;
    current.velocity_x_mm_s = 0;
    current.velocity_y_mm_s = 0;
}

void Simulator::adjust_vehicle_energy(
    const sentinel::v1::TapeEvent& event,
    std::int64_t resolved_value) {
    auto& current = vehicle(event.target_id());
    current.energy_mj = std::clamp(
        current.energy_mj + resolved_value,
        std::int64_t{0},
        current.energy_capacity_mj);
}

void Simulator::select_network_profile(
    const sentinel::v1::TapeEvent& event) {
    const auto& profile = network_profile(event.text_value());
    scenario_.set_network_profile(profile.id());
    network_.set_profile(profile);
}

void Simulator::update_region_access(
    const sentinel::v1::TapeEvent& event) {
    auto& current = region(event.target_id());
    if (current.closed() == event.bool_value()) {
        return;
    }
    current.set_closed(event.bool_value());
    scenario_.mutable_world()->set_map_version(
        scenario_.world().map_version() + 1);
}

Simulator::Task& Simulator::task(std::string_view id) {
    const auto current = std::find_if(
        tasks_.begin(), tasks_.end(),
        [&](const Task& value) {
            return value.spec.id() == id;
        });
    if (current == tasks_.end()) {
        throw std::invalid_argument(
            "event references an unknown task");
    }
    return *current;
}

sentinel::v1::Region& Simulator::region(
    std::string_view id) {
    auto* regions =
        scenario_.mutable_world()->mutable_regions();
    const auto current = std::find_if(
        regions->begin(), regions->end(),
        [&](const sentinel::v1::Region& value) {
            return value.id() == id;
        });
    if (current == regions->end()) {
        throw std::invalid_argument(
            "event references an unknown region");
    }
    return *current;
}

const sentinel::v1::NetworkProfile&
Simulator::network_profile(std::string_view id) const {
    const auto current = std::find_if(
        scenario_.network_profiles().begin(),
        scenario_.network_profiles().end(),
        [&](const sentinel::v1::NetworkProfile& value) {
            return value.id() == id;
        });
    if (current == scenario_.network_profiles().end()) {
        throw std::invalid_argument(
            "event references an unknown network profile");
    }
    return *current;
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
        hash.signed_integer(current.energy_mj);
        hash.boolean(current.active);
    }
    for (const auto& task : tasks_) {
        hash.text(task.spec.id());
        hash.boolean(task.released);
        hash.unsigned_integer(task.status);
        hash.unsigned_integer(task.progress_ticks);
    }
    for (const auto& state : rng_streams_.states()) {
        hash.text(state.first);
        hash.unsigned_integer(state.second);
    }
    network_.append_hash(hash);
    hash.unsigned_integer(wait_ticks_);
    hash.unsigned_integer(replan_count_);
    hash.unsigned_integer(recharge_ticks_);
    hash.unsigned_integer(return_ticks_);
    hash.unsigned_integer(travel_distance_mm_);
    hash.unsigned_integer(energy_consumed_mj_);
    state_hash_ = hash.finish();
    tick_hashes_.push_back(state_hash_);
}

}
