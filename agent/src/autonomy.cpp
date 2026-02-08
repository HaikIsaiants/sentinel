#include <sentinel/agent/autonomy.hpp>

#include <sentinel/agent/allocation.hpp>
#include <sentinel/planning/planner.hpp>

#include <behaviortree_cpp/bt_factory.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <tuple>
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
        factory_.registerSimpleAction(
            "Prepare", [this](BT::TreeNode&) { return prepare(); });
        factory_.registerSimpleAction(
            "Allocate", [this](BT::TreeNode&) { return allocate(); });
        factory_.registerSimpleAction(
            "NeedsCharge", [this](BT::TreeNode&) { return needs_charge(); });
        factory_.registerSimpleAction(
            "ChargeOrDivert", [this](BT::TreeNode&) { return charge_or_divert(); });
        factory_.registerSimpleAction(
            "SelectTask", [this](BT::TreeNode&) { return select_task(); });
        factory_.registerSimpleAction(
            "WorkAtTask", [this](BT::TreeNode&) { return work_at_task(); });
        factory_.registerSimpleAction(
            "NavigateToTask", [this](BT::TreeNode&) { return navigate_to_task(); });
        factory_.registerSimpleAction(
            "AllocationPending", [this](BT::TreeNode&) { return allocation_pending(); });
        factory_.registerSimpleAction(
            "HoldPosition", [this](BT::TreeNode&) { return hold_position(); });
        factory_.registerSimpleAction(
            "ReturnOrIdle", [this](BT::TreeNode&) { return return_or_idle(); });
        tree_ = std::make_unique<BT::Tree>(factory_.createTreeFromText(
            R"(<root BTCPP_format="4">
  <BehaviorTree ID="Autonomy">
    <Sequence>
      <Prepare/>
      <Allocate/>
      <Fallback>
        <Sequence>
          <NeedsCharge/>
          <ChargeOrDivert/>
        </Sequence>
        <Sequence>
          <SelectTask/>
          <Fallback>
            <WorkAtTask/>
            <NavigateToTask/>
          </Fallback>
        </Sequence>
        <Sequence>
          <AllocationPending/>
          <HoldPosition/>
        </Sequence>
        <ReturnOrIdle/>
      </Fallback>
    </Sequence>
  </BehaviorTree>
</root>)"));
    }

    v1::Envelope act(const v1::Envelope& input) {
        if (!input.has_observation() || input.recipient_id() != agent_id_) {
            throw std::invalid_argument("observation addressed to another agent");
        }
        if (input.observation().self().id() != agent_id_) {
            throw std::invalid_argument("observation identity mismatch");
        }

        v1::Envelope output;
        output.set_schema_version(1);
        output.set_sequence(input.sequence());
        output.set_simulation_time_ms(input.simulation_time_ms());
        output.set_sender_id(agent_id_);
        output.set_recipient_id("sim");
        output.mutable_action()->set_tick(input.observation().tick());

        observation_ = &input.observation();
        action_ = output.mutable_action();
        const auto status = tree_->tickOnce();
        observation_ = nullptr;
        action_ = nullptr;
        task_ = nullptr;
        charger_ = nullptr;
        if (status != BT::NodeStatus::SUCCESS) {
            throw std::logic_error("autonomy behavior tree did not select an action");
        }
        return output;
    }

private:
    BT::NodeStatus prepare() {
        task_ = nullptr;
        charger_ = nullptr;
        allocation_pending_ = false;
        action_->set_behavior_mode(v1::BEHAVIOR_MODE_IDLE);
        return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus allocate() {
        const auto allocation = allocator_.update(*observation_);
        action_->mutable_allocation_state()->CopyFrom(allocation.state);
        for (const auto& message : allocation.outgoing_messages) {
            action_->add_outgoing_messages()->CopyFrom(message);
        }
        for (const auto& commit : allocation.commits) {
            action_->add_allocation_commits()->CopyFrom(commit);
        }
        allocation_pending_ = allocation.pending || !allocation.commits.empty();
        return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus needs_charge() {
        const auto charger = std::find_if(
            observation_->world().locations().begin(),
            observation_->world().locations().end(),
            [](const auto& location) {
                return location.kind() == v1::LOCATION_KIND_CHARGING;
            });
        const auto reserve = observation_->self().energy_capacity_mj() / 4;
        if (charger == observation_->world().locations().end()
            || observation_->self().energy_mj() >= reserve) {
            return BT::NodeStatus::FAILURE;
        }
        charger_ = &*charger;
        return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus charge_or_divert() {
        if (inside(observation_->self().position(), *charger_)) {
            action_->set_behavior_mode(v1::BEHAVIOR_MODE_CHARGING);
            action_->set_charge_location_id(charger_->id());
        } else {
            navigate(observation_->self(), charger_->position());
            action_->set_replanned(true);
        }
        return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus select_task() {
        if (observation_->assigned_tasks().empty()) {
            return BT::NodeStatus::FAILURE;
        }
        task_ = &*std::min_element(
            observation_->assigned_tasks().begin(),
            observation_->assigned_tasks().end(),
            [](const auto& left, const auto& right) {
                if (left.deadline_tick() != right.deadline_tick()) {
                    return left.deadline_tick() < right.deadline_tick();
                }
                return left.id() < right.id();
            });
        return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus work_at_task() {
        if (!close_to(observation_->self().position(), *task_)) {
            return BT::NodeStatus::FAILURE;
        }
        action_->set_behavior_mode(v1::BEHAVIOR_MODE_EXECUTING);
        auto* report = action_->add_task_reports();
        report->set_task_id(task_->id());
        report->set_kind(v1::TASK_REPORT_KIND_WORKING);
        return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus navigate_to_task() {
        navigate(observation_->self(), task_->target());
        return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus allocation_pending() const {
        return allocation_pending_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus hold_position() {
        action_->set_behavior_mode(v1::BEHAVIOR_MODE_WAITING);
        return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus return_or_idle() {
        const auto home = std::find_if(
            observation_->world().locations().begin(),
            observation_->world().locations().end(),
            [this](const auto& location) {
                return location.id() == observation_->self().return_location_id();
            });
        if (home != observation_->world().locations().end()
            && !inside(observation_->self().position(), *home)) {
            navigate(observation_->self(), home->position());
            action_->set_behavior_mode(v1::BEHAVIOR_MODE_RETURNING);
        } else {
            action_->set_behavior_mode(v1::BEHAVIOR_MODE_IDLE);
        }
        return BT::NodeStatus::SUCCESS;
    }

    void navigate(const v1::VehicleState& vehicle, const v1::Point& target) {
        const auto map_version = observation_->world().map_version();
        const auto goal_changed =
            route_version_ == 0 || route_map_version_ != map_version
            || route_goal_.x_mm() != target.x_mm()
            || route_goal_.y_mm() != target.y_mm();
        if (goal_changed) {
            ++route_version_;
            route_map_version_ = map_version;
            route_goal_.CopyFrom(target);
        }
        const auto cell = observation_->world().grid_cell_mm();
        const auto snap = [cell](std::int64_t value) {
            if (cell <= 0) {
                return value;
            }
            const auto lower = value / cell * cell;
            const auto upper = lower + cell;
            return value - lower < upper - value ? lower : upper;
        };
        const planning::Coordinate start{
            vehicle.position().x_mm(), vehicle.position().y_mm()};
        const planning::Coordinate goal{
            std::clamp<std::int64_t>(
                snap(target.x_mm()), 0, observation_->world().width_mm()),
            std::clamp<std::int64_t>(
                snap(target.y_mm()), 0, observation_->world().height_mm())};
        const auto route = planning::route_from(
            observation_->world(), vehicle, start, goal,
            observation_->step_ms());
        action_->set_route_version(route_version_);
        action_->set_replanned(goal_changed);
        auto* plan = action_->mutable_route_plan();
        plan->set_version(route_version_);
        plan->set_map_version(map_version);
        plan->set_goal(
            std::to_string(target.x_mm()) + ","
            + std::to_string(target.y_mm()));
        if (!route) {
            action_->set_behavior_mode(v1::BEHAVIOR_MODE_WAITING);
            return;
        }
        for (const auto& point : route->points) {
            auto* waypoint = plan->add_waypoints();
            waypoint->set_x_mm(point.x());
            waypoint->set_y_mm(point.y());
        }
        action_->set_behavior_mode(v1::BEHAVIOR_MODE_NAVIGATING);
        const auto next = std::find_if(
            route->points.begin(), route->points.end(),
            [&start](const auto& point) { return point != start; });
        if (next == route->points.end()) {
            return;
        }
        action_->set_velocity_x_mm_s(
            velocity_component(
                next->x() - vehicle.position().x_mm(),
                vehicle.max_speed_mm_s()));
        if (action_->velocity_x_mm_s() == 0) {
            action_->set_velocity_y_mm_s(
                velocity_component(
                    next->y() - vehicle.position().y_mm(),
                    vehicle.max_speed_mm_s()));
        }
    }

    std::string agent_id_;
    Allocator allocator_;
    BT::BehaviorTreeFactory factory_;
    std::unique_ptr<BT::Tree> tree_;
    const v1::AgentObservation* observation_{};
    v1::AgentAction* action_{};
    const v1::TaskState* task_{};
    const v1::ServiceLocation* charger_{};
    bool allocation_pending_{};
    std::uint64_t route_version_{};
    std::uint64_t route_map_version_{};
    v1::Point route_goal_;
};

Controller::Controller(std::string agent_id)
    : impl_(std::make_unique<Impl>(std::move(agent_id))) {}

Controller::~Controller() = default;

v1::Envelope Controller::act(const v1::Envelope& input) {
    return impl_->act(input);
}

}
