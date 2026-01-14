#include <sentinel/core/simulator.hpp>

#include <sentinel/core/hash.hpp>
#include <sentinel/core/scenario.hpp>

#include <algorithm>
#include <cstdlib>
#include <stdexcept>
#include <tuple>
#include <utility>

namespace sentinel::core {

namespace {

std::int64_t clamp_velocity(
    std::int64_t value, std::int64_t maximum) {
    return std::clamp(value, -maximum, maximum);
}

std::int64_t squared_distance(
    std::int64_t ax, std::int64_t ay,
    std::int64_t bx, std::int64_t by) {
    const auto dx = ax - bx;
    const auto dy = ay - by;
    return dx * dx + dy * dy;
}

bool inside(
    std::int64_t x_mm, std::int64_t y_mm,
    const sentinel::v1::ServiceLocation& location) {
    return squared_distance(
               x_mm, y_mm,
               location.position().x_mm(),
               location.position().y_mm())
           <= location.radius_mm() * location.radius_mm();
}

}

Simulator::Simulator(sentinel::v1::Scenario scenario)
    : scenario_(std::move(scenario)),
      rng_streams_(scenario_.seed()),
      network_(scenario_.seed(), scenario_.step_ms()) {
    normalize_scenario(scenario_);
    validate_scenario(scenario_);
    initialize_network();
    initialize_vehicles();
    initialize_tasks();
    record_hash();
}

void Simulator::initialize_network() {
    network_.set_profile(
        network_profile(scenario_.network_profile()));
}

void Simulator::initialize_vehicles() {
    vehicles_.reserve(
        static_cast<std::size_t>(scenario_.vehicles_size()));
    for (const auto& spec : scenario_.vehicles()) {
        Vehicle current;
        current.spec = spec;
        current.x_mm = spec.initial_position().x_mm();
        current.y_mm = spec.initial_position().y_mm();
        current.energy_mj = spec.initial_energy_mj();
        current.energy_capacity_mj =
            spec.initial_energy_mj();
        vehicles_.push_back(std::move(current));
    }
}

void Simulator::initialize_tasks() {
    tasks_.reserve(
        static_cast<std::size_t>(scenario_.tasks_size()));
    for (const auto& spec : scenario_.tasks()) {
        Task current;
        current.spec = spec;
        current.released = spec.released();
        tasks_.push_back(std::move(current));
    }
}

sentinel::v1::ObservationBatch Simulator::observe() const {
    sentinel::v1::ObservationBatch result;
    result.set_tick(tick_);
    result.set_finished(finished());
    for (const auto& current : vehicles_) {
        if (current.active) {
            result.add_observations()->CopyFrom(
                observation_for(current));
        }
    }
    if (result.finished()) {
        result.mutable_summary()->CopyFrom(summary());
    }
    return result;
}

sentinel::v1::Envelope Simulator::observation_for(
    const Vehicle& current) const {
    sentinel::v1::Envelope envelope;
    envelope.set_schema_version(1);
    envelope.set_sequence(tick_ + 1);
    envelope.set_simulation_time_ms(
        tick_ * scenario_.step_ms());
    envelope.set_sender_id("sim");
    envelope.set_recipient_id(current.spec.id());
    auto* observation = envelope.mutable_observation();
    observation->set_tick(tick_);
    observation->set_step_ms(scenario_.step_ms());
    observation->set_network_profile(
        scenario_.network_profile());
    observation->mutable_self()->CopyFrom(
        vehicle_state(current));
    observation->mutable_world()->CopyFrom(
        scenario_.world());

    for (const auto& peer : vehicles_) {
        if (peer.spec.id() != current.spec.id()) {
            observation->add_peer_ids(peer.spec.id());
        }
    }
    for (const auto& message : delivered_messages_) {
        if (message.recipient_id() == current.spec.id()) {
            observation->add_delivered_messages()->CopyFrom(
                message);
        }
    }
    for (const auto& current_task : tasks_) {
        if (!current_task.released
            || current_task.status
                   != sentinel::v1::TASK_STATUS_PENDING
            || current_task.spec.assigned_agent_id()
                   != current.spec.id()) {
            continue;
        }
        observation->add_assigned_tasks()->CopyFrom(
            task_state(current_task));
    }
    return envelope;
}

sentinel::v1::ObservationBatch Simulator::step(
    const sentinel::v1::ActionBatch& actions) {
    if (finished()) {
        throw std::logic_error("simulation already finished");
    }
    if (actions.tick() != tick_) {
        throw std::invalid_argument(
            "action batch tick mismatch");
    }

    apply_events();
    const auto accepted = accept_actions(actions);
    const auto outgoing = collect_outgoing(accepted);
    apply_action_commands(accepted);
    account_behaviors(accepted);
    apply_charging(accepted);
    const auto network_step =
        network_.step(tick_, outgoing);
    delivered_messages_ = network_step.delivered;
    advance_motion();
    advance_tasks(accepted);
    ++tick_;
    record_hash();
    return observe();
}

std::vector<Simulator::AcceptedAction>
Simulator::accept_actions(
    const sentinel::v1::ActionBatch& actions) {
    std::vector<AcceptedAction> accepted;
    accepted.reserve(
        static_cast<std::size_t>(actions.actions_size()));
    for (const auto& envelope : actions.actions()) {
        if (!envelope.has_action()) {
            throw std::invalid_argument(
                "action envelope has no action");
        }
        if (envelope.sender_id().empty()) {
            throw std::invalid_argument(
                "action sender is required");
        }
        if (envelope.recipient_id() != "sim") {
            throw std::invalid_argument(
                "action is not addressed to the simulator");
        }
        if (envelope.action().tick() != tick_) {
            throw std::invalid_argument(
                "action tick mismatch");
        }
        auto& current = vehicle(envelope.sender_id());
        if (!current.active) {
            throw std::invalid_argument(
                "inactive vehicle produced an action");
        }
        const auto duplicate = std::find_if(
            accepted.begin(), accepted.end(),
            [&](const AcceptedAction& value) {
                return value.vehicle->spec.id()
                       == current.spec.id();
            });
        if (duplicate != accepted.end()) {
            throw std::invalid_argument(
                "duplicate action sender");
        }
        for (const auto& message :
             envelope.action().outgoing_messages()) {
            if (message.sender_id()
                != envelope.sender_id()) {
                throw std::invalid_argument(
                    "network sender does not match action");
            }
        }
        if (!envelope.action()
                 .charge_location_id()
                 .empty()) {
            const auto location = std::find_if(
                scenario_.world().locations().begin(),
                scenario_.world().locations().end(),
                [&](const auto& value) {
                    return value.id()
                               == envelope.action()
                                      .charge_location_id()
                           && value.kind()
                                  == sentinel::v1::
                                      LOCATION_KIND_CHARGING;
                });
            if (location
                == scenario_.world().locations().end()) {
                throw std::invalid_argument(
                    "unknown charging location");
            }
            if (!inside(current.x_mm, current.y_mm, *location)) {
                throw std::invalid_argument(
                    "vehicle is outside charging location");
            }
        }
        accepted.push_back(
            AcceptedAction{&current, &envelope});
    }
    return accepted;
}

void Simulator::apply_action_commands(
    const std::vector<AcceptedAction>& accepted) {
    for (const auto& current : accepted) {
        const auto& action =
            current.envelope->action();
        current.vehicle->velocity_x_mm_s =
            clamp_velocity(
                action.velocity_x_mm_s(),
                current.vehicle->spec.max_speed_mm_s());
        current.vehicle->velocity_y_mm_s =
            clamp_velocity(
                action.velocity_y_mm_s(),
                current.vehicle->spec.max_speed_mm_s());
    }
}

std::vector<sentinel::v1::NetworkMessage>
Simulator::collect_outgoing(
    const std::vector<AcceptedAction>& accepted) const {
    std::vector<sentinel::v1::NetworkMessage> outgoing;
    for (const auto& current : accepted) {
        outgoing.insert(
            outgoing.end(),
            current.envelope->action()
                .outgoing_messages()
                .begin(),
            current.envelope->action()
                .outgoing_messages()
                .end());
    }
    std::sort(
        outgoing.begin(), outgoing.end(),
        [](const auto& left, const auto& right) {
            return std::tuple(
                       left.sender_id(),
                       left.recipient_id(),
                       left.version())
                   < std::tuple(
                       right.sender_id(),
                       right.recipient_id(),
                       right.version());
        });
    return outgoing;
}

void Simulator::account_behaviors(
    const std::vector<AcceptedAction>& accepted) {
    for (const auto& current : accepted) {
        const auto& action =
            current.envelope->action();
        if (action.replanned()) {
            ++replan_count_;
        }
        if (action.behavior_mode()
            == sentinel::v1::BEHAVIOR_MODE_WAITING) {
            ++wait_ticks_;
        }
        if (action.behavior_mode()
            == sentinel::v1::BEHAVIOR_MODE_RETURNING) {
            ++return_ticks_;
        }
    }
}

void Simulator::apply_charging(
    const std::vector<AcceptedAction>& accepted) {
    for (const auto& current : accepted) {
        const auto& location_id =
            current.envelope->action()
                .charge_location_id();
        if (location_id.empty()) {
            continue;
        }
        const auto location = std::find_if(
            scenario_.world().locations().begin(),
            scenario_.world().locations().end(),
            [&](const auto& value) {
                return value.id() == location_id;
            });
        current.vehicle->energy_mj = std::min(
            current.vehicle->energy_capacity_mj,
            current.vehicle->energy_mj
                + location->charge_mj_per_tick());
        ++recharge_ticks_;
    }
}

void Simulator::apply_events() {
    while (
        next_event_
            < static_cast<std::size_t>(
                scenario_.events_size())
        && scenario_.events(
               static_cast<int>(next_event_))
               .tick()
               == tick_) {
        const auto& event =
            scenario_.events(
                static_cast<int>(next_event_++));
        dispatch_event(
            event, resolve_event_value(event));
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
    return rng_streams_
        .stream(event.rng_stream())
        .uniform(event.value_min(), event.value_max());
}

void Simulator::dispatch_event(
    const sentinel::v1::TapeEvent& event,
    std::int64_t resolved_value) {
    if (event.kind()
        == sentinel::v1::
            TAPE_EVENT_KIND_RELEASE_TASK) {
        auto& current = task(event.target_id());
        if (current.status
            != sentinel::v1::TASK_STATUS_PENDING) {
            throw std::invalid_argument(
                "cannot release a terminal task");
        }
        current.released = true;
        return;
    }
    if (event.kind()
        == sentinel::v1::
            TAPE_EVENT_KIND_DISABLE_VEHICLE) {
        auto& current = vehicle(event.target_id());
        current.active = false;
        current.velocity_x_mm_s = 0;
        current.velocity_y_mm_s = 0;
        return;
    }
    if (event.kind()
        == sentinel::v1::
            TAPE_EVENT_KIND_ENERGY_DELTA) {
        auto& current = vehicle(event.target_id());
        current.energy_mj = std::clamp(
            current.energy_mj + resolved_value,
            std::int64_t{0},
            current.energy_capacity_mj);
        return;
    }
    if (event.kind()
        == sentinel::v1::
            TAPE_EVENT_KIND_SET_NETWORK_PROFILE) {
        const auto& profile =
            network_profile(event.text_value());
        scenario_.set_network_profile(profile.id());
        network_.set_profile(profile);
        return;
    }
    if (event.kind()
        == sentinel::v1::
            TAPE_EVENT_KIND_SET_REGION_CLOSED) {
        auto& current = region(event.target_id());
        if (current.closed() != event.bool_value()) {
            current.set_closed(event.bool_value());
            scenario_.mutable_world()->set_map_version(
                scenario_.world().map_version() + 1);
        }
        return;
    }
    throw std::invalid_argument(
        "unsupported tape event");
}

void Simulator::advance_motion() {
    for (auto& current : vehicles_) {
        if (current.active) {
            advance_vehicle(current);
        }
    }
}

void Simulator::advance_vehicle(Vehicle& current) {
    const auto dx =
        current.velocity_x_mm_s
        * static_cast<std::int64_t>(
            scenario_.step_ms())
        / 1000;
    const auto dy =
        current.velocity_y_mm_s
        * static_cast<std::int64_t>(
            scenario_.step_ms())
        / 1000;
    const auto next_x = std::clamp(
        current.x_mm + dx,
        std::int64_t{0},
        scenario_.world().width_mm());
    const auto next_y = std::clamp(
        current.y_mm + dy,
        std::int64_t{0},
        scenario_.world().height_mm());
    const auto distance =
        std::llabs(next_x - current.x_mm)
        + std::llabs(next_y - current.y_mm);
    const auto energy =
        (distance
             * current.spec.energy_cost_mj_per_meter()
         + 999)
        / 1000;
    if (energy > current.energy_mj) {
        current.velocity_x_mm_s = 0;
        current.velocity_y_mm_s = 0;
        return;
    }
    current.x_mm = next_x;
    current.y_mm = next_y;
    current.energy_mj -= energy;
    travel_distance_mm_ +=
        static_cast<std::uint64_t>(distance);
    energy_consumed_mj_ +=
        static_cast<std::uint64_t>(energy);
}

void Simulator::advance_tasks(
    const std::vector<AcceptedAction>& accepted) {
    for (auto& current : tasks_) {
        advance_task(current, accepted);
    }
}

void Simulator::advance_task(
    Task& current,
    const std::vector<AcceptedAction>& accepted) {
    if (!current.released
        || current.status
               != sentinel::v1::TASK_STATUS_PENDING) {
        return;
    }
    if (tick_ + 1 >= current.spec.deadline_tick()) {
        current.status =
            sentinel::v1::TASK_STATUS_MISSED;
        current.progress_ticks = 0;
        return;
    }
    auto& owner =
        vehicle(current.spec.assigned_agent_id());
    const auto* action =
        action_for(accepted, owner.spec.id());
    if (!owner.active
        || !at_task(owner, current)
        || !reports_work(action, current.spec.id())) {
        current.progress_ticks = 0;
        return;
    }
    const auto service_energy =
        current.spec.service_energy_mj_per_tick();
    if (service_energy > owner.energy_mj) {
        current.progress_ticks = 0;
        return;
    }
    owner.energy_mj -= service_energy;
    energy_consumed_mj_ +=
        static_cast<std::uint64_t>(service_energy);
    ++current.progress_ticks;
    if (current.progress_ticks
        >= current.spec.service_ticks()) {
        current.status =
            sentinel::v1::TASK_STATUS_COMPLETED;
    }
}

const sentinel::v1::AgentAction*
Simulator::action_for(
    const std::vector<AcceptedAction>& accepted,
    std::string_view agent_id) const {
    const auto current = std::find_if(
        accepted.begin(), accepted.end(),
        [&](const AcceptedAction& value) {
            return value.vehicle->spec.id() == agent_id;
        });
    return current == accepted.end()
               ? nullptr
               : &current->envelope->action();
}

bool Simulator::reports_work(
    const sentinel::v1::AgentAction* action,
    std::string_view task_id) const {
    if (action == nullptr) {
        return false;
    }
    return std::any_of(
        action->task_reports().begin(),
        action->task_reports().end(),
        [&](const auto& report) {
            return report.task_id() == task_id
                   && report.kind()
                          == sentinel::v1::
                              TASK_REPORT_KIND_WORKING;
        });
}

bool Simulator::at_task(
    const Vehicle& current,
    const Task& current_task) const {
    const auto radius =
        current_task.spec.completion_radius_mm();
    return squared_distance(
               current.x_mm, current.y_mm,
               current_task.spec.target().x_mm(),
               current_task.spec.target().y_mm())
           <= radius * radius;
}

sentinel::v1::SimulationSummary
Simulator::summary() const {
    sentinel::v1::SimulationSummary result;
    const auto completed =
        static_cast<std::uint32_t>(std::count_if(
            tasks_.begin(), tasks_.end(),
            [](const Task& current) {
                return current.status
                       == sentinel::v1::
                           TASK_STATUS_COMPLETED;
            }));
    result.set_success(
        completed
        == static_cast<std::uint32_t>(tasks_.size()));
    result.set_ticks(tick_);
    result.set_terminal_hash(state_hash_);
    for (const auto& value : tick_hashes_) {
        result.add_tick_hashes(value);
    }
    result.set_completed_tasks(completed);
    result.set_total_tasks(
        static_cast<std::uint32_t>(tasks_.size()));
    result.set_active_agents(
        static_cast<std::uint32_t>(std::count_if(
            vehicles_.begin(), vehicles_.end(),
            [](const Vehicle& current) {
                return current.active;
            })));
    result.set_wait_ticks(wait_ticks_);
    result.set_replan_count(replan_count_);
    result.set_recharge_ticks(recharge_ticks_);
    result.set_return_ticks(return_ticks_);
    result.set_travel_distance_mm(
        travel_distance_mm_);
    result.set_energy_consumed_mj(
        energy_consumed_mj_);
    result.set_communication_bytes(
        network_.communication_bytes());
    result.set_communication_messages(
        network_.communication_messages());
    result.set_delivered_messages(
        network_.delivered_messages());
    return result;
}

std::string Simulator::state_hash() const {
    return state_hash_;
}

bool Simulator::finished() const {
    const auto all_terminal = std::all_of(
        tasks_.begin(), tasks_.end(),
        [](const Task& current) {
            return current.status
                   != sentinel::v1::TASK_STATUS_PENDING;
        });
    return all_terminal
           || tick_ >= scenario_.max_ticks();
}

std::uint64_t Simulator::tick() const {
    return tick_;
}

const sentinel::v1::Scenario&
Simulator::scenario() const {
    return scenario_;
}

Simulator::Vehicle& Simulator::vehicle(
    std::string_view id) {
    const auto current = std::find_if(
        vehicles_.begin(), vehicles_.end(),
        [&](const Vehicle& value) {
            return value.spec.id() == id;
        });
    if (current == vehicles_.end()) {
        throw std::invalid_argument(
            "unknown vehicle");
    }
    return *current;
}

const Simulator::Vehicle& Simulator::vehicle(
    std::string_view id) const {
    const auto current = std::find_if(
        vehicles_.begin(), vehicles_.end(),
        [&](const Vehicle& value) {
            return value.spec.id() == id;
        });
    if (current == vehicles_.end()) {
        throw std::invalid_argument(
            "unknown vehicle");
    }
    return *current;
}

Simulator::Task& Simulator::task(
    std::string_view id) {
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
Simulator::network_profile(
    std::string_view id) const {
    const auto current = std::find_if(
        scenario_.network_profiles().begin(),
        scenario_.network_profiles().end(),
        [&](const auto& value) {
            return value.id() == id;
        });
    if (current
        == scenario_.network_profiles().end()) {
        throw std::invalid_argument(
            "event references an unknown network profile");
    }
    return *current;
}

sentinel::v1::VehicleState
Simulator::vehicle_state(
    const Vehicle& current) const {
    sentinel::v1::VehicleState result;
    result.set_id(current.spec.id());
    result.set_kind(current.spec.kind());
    result.mutable_position()->set_x_mm(current.x_mm);
    result.mutable_position()->set_y_mm(current.y_mm);
    result.set_velocity_x_mm_s(
        current.velocity_x_mm_s);
    result.set_velocity_y_mm_s(
        current.velocity_y_mm_s);
    result.set_max_speed_mm_s(
        current.spec.max_speed_mm_s());
    result.set_energy_mj(current.energy_mj);
    result.set_payload_grams(
        current.spec.payload_grams());
    for (const auto capability :
         current.spec.capabilities()) {
        result.add_capabilities(
            static_cast<sentinel::v1::Capability>(
                capability));
    }
    for (const auto& terrain :
         current.spec.terrain_access()) {
        result.add_terrain_access(terrain);
    }
    result.set_active(current.active);
    result.set_initial_energy_mj(
        current.spec.initial_energy_mj());
    result.set_energy_cost_mj_per_meter(
        current.spec.energy_cost_mj_per_meter());
    result.set_return_location_id(
        current.spec.return_location_id());
    result.set_energy_capacity_mj(
        current.energy_capacity_mj);
    return result;
}

sentinel::v1::TaskState Simulator::task_state(
    const Task& current) const {
    sentinel::v1::TaskState result;
    result.set_id(current.spec.id());
    result.set_kind(current.spec.kind());
    result.mutable_target()->CopyFrom(
        current.spec.target());
    result.set_required_capability(
        current.spec.required_capability());
    result.set_payload_required_grams(
        current.spec.payload_required_grams());
    result.set_deadline_tick(
        current.spec.deadline_tick());
    result.set_completion_radius_mm(
        current.spec.completion_radius_mm());
    result.set_assigned_agent_id(
        current.spec.assigned_agent_id());
    result.set_status(current.status);
    result.set_service_ticks(
        current.spec.service_ticks());
    result.set_service_energy_mj_per_tick(
        current.spec.service_energy_mj_per_tick());
    result.set_progress_ticks(
        current.progress_ticks);
    result.set_priority(current.spec.priority());
    return result;
}

void Simulator::append_vehicle_hash(
    HashBuilder& hash,
    const Vehicle& current) const {
    hash.text(current.spec.id());
    hash.signed_integer(current.x_mm);
    hash.signed_integer(current.y_mm);
    hash.signed_integer(
        current.velocity_x_mm_s);
    hash.signed_integer(
        current.velocity_y_mm_s);
    hash.signed_integer(current.energy_mj);
    hash.boolean(current.active);
}

void Simulator::append_task_hash(
    HashBuilder& hash,
    const Task& current) const {
    hash.text(current.spec.id());
    hash.boolean(current.released);
    hash.unsigned_integer(current.status);
    hash.unsigned_integer(
        current.progress_ticks);
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
    for (const auto& state :
         rng_streams_.states()) {
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
