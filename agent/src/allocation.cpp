#include <sentinel/agent/allocation.hpp>

#include <sentinel/planning/planner.hpp>

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace sentinel::agent {

class Allocator::Impl {
public:
    explicit Impl(std::string agent_id) : agent_id_(std::move(agent_id)) {
        if (agent_id_.empty()) {
            throw std::invalid_argument("allocator agent id is required");
        }
    }

    AllocationResult update(const v1::AgentObservation& observation) {
        if (observation.self().id() != agent_id_) {
            throw std::invalid_argument("allocator observation identity mismatch");
        }
        AllocationResult result;
        if (observation.allocation_policy() != v1::ALLOCATION_POLICY_NEAREST_CAPABLE) {
            return result;
        }
        for (const auto& task : observation.available_tasks()) {
            const auto claim = planning::make_claim(
                observation.world(), observation.self(), task,
                observation.tick(), observation.step_ms(),
                observation.allocation_epoch(), ++version_);
            if (!claim.feasible()) {
                continue;
            }
            auto* commit = &result.commits.emplace_back();
            commit->set_epoch(claim.epoch());
            commit->set_version(claim.version());
            commit->set_task_id(claim.task_id());
            commit->set_agent_id(claim.agent_id());
            commit->set_distance_mm(claim.distance_mm());
            commit->set_score(-claim.distance_mm());
        }
        std::sort(
            result.commits.begin(), result.commits.end(),
            [](const auto& left, const auto& right) {
                if (left.task_id() != right.task_id()) {
                    return left.task_id() < right.task_id();
                }
                return left.version() < right.version();
            });
        result.pending = !result.commits.empty();
        return result;
    }

private:
    std::string agent_id_;
    std::uint64_t version_{};
};

Allocator::Allocator(std::string agent_id)
    : impl_(std::make_unique<Impl>(std::move(agent_id))) {}
Allocator::~Allocator() = default;

AllocationResult Allocator::update(const v1::AgentObservation& observation) {
    return impl_->update(observation);
}

}
