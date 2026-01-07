#include <sentinel/agent/autonomy.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>
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

std::int64_t manhattan_distance(
    const v1::Point& left, const v1::Point& right) {
    return std::llabs(left.x_mm() - right.x_mm())
           + std::llabs(left.y_mm() - right.y_mm());
}

bool earlier_task(
    const v1::TaskState& left, const v1::TaskState& right,
    const v1::Point& position) {
    if (left.deadline_tick() != right.deadline_tick()) {
        return left.deadline_tick() < right.deadline_tick();
    }
    const auto left_distance =
        manhattan_distance(position, left.target());
    const auto right_distance =
        manhattan_distance(position, right.target());
    if (left_distance != right_distance) {
        return left_distance < right_distance;
    }
    return left.id() < right.id();
}

}

class Controller::Impl {
public:
    explicit Impl(std::string agent_id) : agent_id_(std::move(agent_id)) {
        if (agent_id_.empty()) {
            throw std::invalid_argument("agent id is required");
        }
    }

    v1::Envelope act(const v1::Envelope& input) {
        validate_input(input);
        const auto& observation = input.observation();
        auto output = make_output(input);
        auto& action = *output.mutable_action();

        if (!observation.self().active()) {
            selected_task_id_.clear();
            action.set_behavior_mode(v1::BEHAVIOR_MODE_WAITING);
            remember_tick(observation.tick());
            return output;
        }

        const auto* selected = select_task(observation);
        if (selected == nullptr) {
            selected_task_id_.clear();
            action.set_behavior_mode(v1::BEHAVIOR_MODE_IDLE);
            remember_tick(observation.tick());
            return output;
        }

        selected_task_id_ = selected->id();
        if (close_to(observation.self().position(), *selected)) {
            write_service(action, *selected);
        } else {
            write_navigation(
                action, observation.self(), selected->target());
        }
        remember_tick(observation.tick());
        return output;
    }

private:
    void validate_input(const v1::Envelope& input) const {
        if (!input.has_observation()) {
            throw std::invalid_argument("agent input is not an observation");
        }
        if (input.recipient_id() != agent_id_) {
            throw std::invalid_argument("observation addressed to another agent");
        }
        if (input.sender_id() != "sim") {
            throw std::invalid_argument("observation sender is not the simulator");
        }
        const auto& observation = input.observation();
        if (observation.self().id() != agent_id_) {
            throw std::invalid_argument("observation identity mismatch");
        }
        if (last_tick_.has_value()
            && observation.tick() <= *last_tick_) {
            throw std::invalid_argument("observation tick did not advance");
        }
    }

    v1::Envelope make_output(const v1::Envelope& input) const {
        v1::Envelope output;
        output.set_schema_version(1);
        output.set_sequence(input.sequence());
        output.set_simulation_time_ms(input.simulation_time_ms());
        output.set_sender_id(agent_id_);
        output.set_recipient_id("sim");
        output.mutable_action()->set_tick(
            input.observation().tick());
        return output;
    }

    const v1::TaskState* select_task(
        const v1::AgentObservation& observation) const {
        const auto current = std::find_if(
            observation.assigned_tasks().begin(),
            observation.assigned_tasks().end(),
            [&](const v1::TaskState& task) {
                return task.id() == selected_task_id_;
            });
        if (current != observation.assigned_tasks().end()) {
            return &*current;
        }
        if (observation.assigned_tasks().empty()) {
            return nullptr;
        }
        return &*std::min_element(
            observation.assigned_tasks().begin(),
            observation.assigned_tasks().end(),
            [&](const v1::TaskState& left, const v1::TaskState& right) {
                return earlier_task(
                    left, right, observation.self().position());
            });
    }

    static void write_service(
        v1::AgentAction& action,
        const v1::TaskState& task) {
        action.set_behavior_mode(v1::BEHAVIOR_MODE_EXECUTING);
        action.set_velocity_x_mm_s(0);
        action.set_velocity_y_mm_s(0);
        auto* report = action.add_task_reports();
        report->set_task_id(task.id());
        report->set_kind(v1::TASK_REPORT_KIND_WORKING);
    }

    static void write_navigation(
        v1::AgentAction& action,
        const v1::VehicleState& vehicle,
        const v1::Point& target) {
        action.set_behavior_mode(v1::BEHAVIOR_MODE_NAVIGATING);
        const auto dx =
            target.x_mm() - vehicle.position().x_mm();
        const auto dy =
            target.y_mm() - vehicle.position().y_mm();
        if (std::llabs(dx) >= std::llabs(dy) && dx != 0) {
            action.set_velocity_x_mm_s(
                velocity_component(dx, vehicle.max_speed_mm_s()));
            action.set_velocity_y_mm_s(0);
        } else {
            action.set_velocity_x_mm_s(0);
            action.set_velocity_y_mm_s(
                velocity_component(dy, vehicle.max_speed_mm_s()));
        }
    }

    void remember_tick(std::uint64_t tick) {
        last_tick_ = tick;
    }

    std::string agent_id_;
    std::string selected_task_id_;
    std::optional<std::uint64_t> last_tick_;
};

Controller::Controller(std::string agent_id) : impl_(std::make_unique<Impl>(std::move(agent_id))) {}
Controller::~Controller() = default;

v1::Envelope Controller::act(const v1::Envelope& input) {
    return impl_->act(input);
}

}
