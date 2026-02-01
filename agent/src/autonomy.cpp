#include <sentinel/agent/autonomy.hpp>

#include <sentinel/agent/allocation.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace sentinel::agent {

namespace {

std::int64_t velocity_component(std::int64_t delta, std::int64_t maximum) {
    if (delta == 0) {
        return 0;
    }
    const auto magnitude = std::min<std::int64_t>(std::llabs(delta), maximum);
    return delta < 0 ? -magnitude : magnitude;
}

bool close_to(const v1::Point& point, const v1::TaskState& task) {
    const auto dx = point.x_mm() - task.target().x_mm();
    const auto dy = point.y_mm() - task.target().y_mm();
    const auto radius = task.completion_radius_mm();
    return dx * dx + dy * dy <= radius * radius;
}

bool inside(const v1::Point& point, const v1::ServiceLocation& location) {
    const auto dx = point.x_mm() - location.position().x_mm();
    const auto dy = point.y_mm() - location.position().y_mm();
    return dx * dx + dy * dy <= location.radius_mm() * location.radius_mm();
}

}

class Controller::Impl {
public:
    explicit Impl(std::string agent_id)
        : agent_id_(std::move(agent_id)), allocator_(agent_id_) {
        if (agent_id_.empty()) {
            throw std::invalid_argument("agent id is required");
        }
    }

    v1::Envelope act(const v1::Envelope& input) {
        if (!input.has_observation() || input.recipient_id() != agent_id_) {
            throw std::invalid_argument("observation addressed to another agent");
        }
        const auto& observation = input.observation();
        if (observation.self().id() != agent_id_) {
            throw std::invalid_argument("observation identity mismatch");
        }

        v1::Envelope output;
        output.set_schema_version(1);
        output.set_sequence(input.sequence());
        output.set_simulation_time_ms(input.simulation_time_ms());
        output.set_sender_id(agent_id_);
        output.set_recipient_id("sim");
        auto* action = output.mutable_action();
        action->set_tick(observation.tick());
        const auto allocation = allocator_.update(observation);
        action->mutable_allocation_state()->CopyFrom(allocation.state);
        for (const auto& message : allocation.outgoing_messages) {
            action->add_outgoing_messages()->CopyFrom(message);
        }
        for (const auto& commit : allocation.commits) {
            action->add_allocation_commits()->CopyFrom(commit);
        }

        const auto charger = std::find_if(
            observation.world().locations().begin(), observation.world().locations().end(),
            [](const auto& location) {
                return location.kind() == v1::LOCATION_KIND_CHARGING;
            });
        const auto reserve = observation.self().energy_capacity_mj() / 4;
        if (charger != observation.world().locations().end()
            && observation.self().energy_mj() < reserve) {
            if (inside(observation.self().position(), *charger)) {
                action->set_behavior_mode(v1::BEHAVIOR_MODE_CHARGING);
                action->set_charge_location_id(charger->id());
            } else {
                navigate(*action, observation.self(), charger->position());
                action->set_replanned(true);
            }
            return output;
        }

        if (observation.assigned_tasks().empty()) {
            if (allocation.pending) {
                action->set_behavior_mode(v1::BEHAVIOR_MODE_WAITING);
                return output;
            }
            const auto home = std::find_if(
                observation.world().locations().begin(), observation.world().locations().end(),
                [&](const auto& location) {
                    return location.id() == observation.self().return_location_id();
                });
            if (home != observation.world().locations().end()
                && !inside(observation.self().position(), *home)) {
                navigate(*action, observation.self(), home->position());
                action->set_behavior_mode(v1::BEHAVIOR_MODE_RETURNING);
                return output;
            }
            action->set_behavior_mode(v1::BEHAVIOR_MODE_IDLE);
            return output;
        }

        const auto task = std::min_element(
            observation.assigned_tasks().begin(), observation.assigned_tasks().end(),
            [](const auto& left, const auto& right) {
                if (left.deadline_tick() != right.deadline_tick()) {
                    return left.deadline_tick() < right.deadline_tick();
                }
                return left.id() < right.id();
            });

        if (close_to(observation.self().position(), *task)) {
            action->set_behavior_mode(v1::BEHAVIOR_MODE_EXECUTING);
            auto* report = action->add_task_reports();
            report->set_task_id(task->id());
            report->set_kind(v1::TASK_REPORT_KIND_WORKING);
            return output;
        }

        navigate(*action, observation.self(), task->target());
        return output;
    }

private:
    static void navigate(
        v1::AgentAction& action, const v1::VehicleState& vehicle, const v1::Point& target) {
        action.set_behavior_mode(v1::BEHAVIOR_MODE_NAVIGATING);
        action.set_velocity_x_mm_s(
            velocity_component(
                target.x_mm() - vehicle.position().x_mm(), vehicle.max_speed_mm_s()));
        if (action.velocity_x_mm_s() == 0) {
            action.set_velocity_y_mm_s(
                velocity_component(
                    target.y_mm() - vehicle.position().y_mm(), vehicle.max_speed_mm_s()));
        }
    }

    std::string agent_id_;
    Allocator allocator_;
};

Controller::Controller(std::string agent_id) : impl_(std::make_unique<Impl>(std::move(agent_id))) {}
Controller::~Controller() = default;

v1::Envelope Controller::act(const v1::Envelope& input) {
    return impl_->act(input);
}

}
