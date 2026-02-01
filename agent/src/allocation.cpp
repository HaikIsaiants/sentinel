#include <sentinel/agent/allocation.hpp>

#include <sentinel/planning/planner.hpp>
#include <sentinel/protocol/framing.hpp>

#include <algorithm>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace sentinel::agent {
namespace {

using BidMap = std::map<std::string, v1::AllocationBid>;

bool same_bid(const v1::AllocationBid& left, const v1::AllocationBid& right) {
    return std::tuple{left.epoch(), left.version(), left.task_id(), left.bidder_id(),
                      left.distance_mm(), left.energy_mj(), left.completion_tick()}
           == std::tuple{right.epoch(), right.version(), right.task_id(), right.bidder_id(),
                         right.distance_mm(), right.energy_mj(), right.completion_tick()};
}

bool better(const v1::AllocationBid& left, const v1::AllocationBid& right) {
    return std::tuple{left.distance_mm(), left.bidder_id()}
           < std::tuple{right.distance_mm(), right.bidder_id()};
}

std::vector<std::string> peers(const v1::AgentObservation& observation) {
    std::vector<std::string> result(
        observation.peer_ids().begin(), observation.peer_ids().end());
    std::sort(result.begin(), result.end());
    result.erase(std::unique(result.begin(), result.end()), result.end());
    return result;
}

const v1::AllocationBid* winner_for(
    const v1::AllocationState& state, const std::string& task_id) {
    const auto current = std::lower_bound(
        state.winners().begin(), state.winners().end(), task_id,
        [](const auto& bid, const auto& id) { return bid.task_id() < id; });
    return current != state.winners().end() && current->task_id() == task_id
               ? &*current
               : nullptr;
}

bool sorted_unique_bids(
    const google::protobuf::RepeatedPtrField<v1::AllocationBid>& bids) {
    for (int index = 0; index < bids.size(); ++index) {
        if (bids[index].task_id().empty() || bids[index].bidder_id().empty()) {
            return false;
        }
        if (index != 0 && bids[index - 1].task_id() >= bids[index].task_id()) {
            return false;
        }
    }
    return true;
}

}

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
        reset_basis(observation);
        const auto member_ids = peers(observation);
        accept_messages(observation, member_ids);

        const auto tasks = available_tasks(observation);
        revise_own_bids(observation, tasks);
        const auto winners = select_winners(tasks);
        revise_state(observation, winners);

        result.state.CopyFrom(state_);
        result.coordinated = coordinated(winners, member_ids);
        result.pending = !winners.empty();
        if (result.coordinated) {
            result.commits = commits(winners);
        }
        result.outgoing_messages = outgoing(observation.tick(), member_ids);
        return result;
    }

private:
    void reset_basis(const v1::AgentObservation& observation) {
        if (epoch_ == observation.allocation_epoch()
            && map_version_ == observation.world().map_version()) {
            return;
        }
        epoch_ = observation.allocation_epoch();
        map_version_ = observation.world().map_version();
        bid_version_ = 0;
        own_bids_.clear();
        peer_states_.clear();
        state_.Clear();
        serialized_state_.clear();
        broadcast_version_ = 0;
    }

    void accept_messages(
        const v1::AgentObservation& observation,
        const std::vector<std::string>& member_ids) {
        struct Candidate {
            v1::AllocationState state;
            std::string bytes;
            bool conflicted{};
        };
        std::map<std::string, Candidate> candidates;
        for (const auto& message : observation.delivered_messages()) {
            if (message.recipient_id() != agent_id_
                || !std::binary_search(
                    member_ids.begin(), member_ids.end(), message.sender_id())) {
                continue;
            }
            v1::AllocationState state;
            if (!state.ParseFromString(message.payload())
                || state.sender_id() != message.sender_id()
                || state.version() != message.version()
                || state.epoch() != epoch_ || state.map_version() != map_version_
                || !sorted_unique_bids(state.bids())
                || !sorted_unique_bids(state.winners())) {
                continue;
            }
            const auto bytes = protocol::deterministic_bytes(state);
            auto position = candidates.find(message.sender_id());
            if (position == candidates.end()
                || state.version() > position->second.state.version()) {
                candidates[message.sender_id()] = Candidate{
                    std::move(state), bytes, false};
            } else if (state.version() == position->second.state.version()
                       && bytes != position->second.bytes) {
                position->second.conflicted = true;
            }
        }
        for (auto& [sender, candidate] : candidates) {
            if (candidate.conflicted) {
                continue;
            }
            const auto current = peer_states_.find(sender);
            if (current == peer_states_.end()
                || candidate.state.version() > current->second.version()) {
                peer_states_[sender] = std::move(candidate.state);
            }
        }
        std::erase_if(peer_states_, [&member_ids](const auto& item) {
            return !std::binary_search(
                member_ids.begin(), member_ids.end(), item.first);
        });
    }

    static std::map<std::string, v1::TaskState> available_tasks(
        const v1::AgentObservation& observation) {
        std::map<std::string, v1::TaskState> result;
        for (const auto& task : observation.available_tasks()) {
            if (task.status() != v1::TASK_STATUS_COMPLETED
                && task.status() != v1::TASK_STATUS_MISSED
                && task.assigned_agent_id().empty()) {
                result.emplace(task.id(), task);
            }
        }
        return result;
    }

    void revise_own_bids(
        const v1::AgentObservation& observation,
        const std::map<std::string, v1::TaskState>& tasks) {
        BidMap next;
        for (const auto& [id, task] : tasks) {
            const auto route = planning::feasible_route(
                observation.world(), observation.self(), task,
                observation.tick(), observation.step_ms());
            if (!route) {
                continue;
            }
            auto bid = v1::AllocationBid{};
            bid.set_epoch(epoch_);
            bid.set_task_id(id);
            bid.set_bidder_id(agent_id_);
            bid.set_score(-route->distance_mm);
            bid.set_distance_mm(route->distance_mm);
            bid.set_energy_mj(
                route->energy_mj
                + static_cast<std::int64_t>(task.service_ticks())
                      * task.service_energy_mj_per_tick());
            bid.set_completion_tick(
                observation.tick() + route->travel_ticks + task.service_ticks());
            next.emplace(id, std::move(bid));
        }
        const auto content_changed =
            next.size() != own_bids_.size()
            || !std::equal(
                next.begin(), next.end(), own_bids_.begin(),
                [](const auto& left, const auto& right) {
                    return left.first == right.first
                           && std::tuple{
                                  left.second.task_id(),
                                  left.second.bidder_id(),
                                  left.second.distance_mm(),
                                  left.second.energy_mj()}
                                  == std::tuple{
                                      right.second.task_id(),
                                      right.second.bidder_id(),
                                      right.second.distance_mm(),
                                      right.second.energy_mj()};
                });
        if (content_changed) {
            ++bid_version_;
        } else {
            for (auto& [id, bid] : next) {
                bid.set_completion_tick(own_bids_.at(id).completion_tick());
            }
        }
        for (auto& [id, bid] : next) {
            static_cast<void>(id);
            bid.set_version(bid_version_);
        }
        own_bids_ = std::move(next);
    }

    BidMap select_winners(
        const std::map<std::string, v1::TaskState>& tasks) const {
        BidMap result;
        for (const auto& [id, task] : tasks) {
            static_cast<void>(task);
            std::optional<v1::AllocationBid> selected;
            const auto own = own_bids_.find(id);
            if (own != own_bids_.end()) {
                selected = own->second;
            }
            for (const auto& [peer, state] : peer_states_) {
                static_cast<void>(peer);
                const auto candidate = std::lower_bound(
                    state.bids().begin(), state.bids().end(), id,
                    [](const auto& bid, const auto& task_id) {
                        return bid.task_id() < task_id;
                    });
                if (candidate == state.bids().end()
                    || candidate->task_id() != id) {
                    continue;
                }
                if (!selected || better(*candidate, *selected)) {
                    selected = *candidate;
                }
            }
            if (selected) {
                result.emplace(id, std::move(*selected));
            }
        }
        return result;
    }

    void revise_state(
        const v1::AgentObservation& observation, const BidMap& winners) {
        v1::AllocationState next;
        next.set_epoch(epoch_);
        next.set_version(state_.version());
        next.set_sender_id(agent_id_);
        next.set_map_version(map_version_);
        for (const auto& [id, bid] : own_bids_) {
            static_cast<void>(id);
            next.add_bids()->CopyFrom(bid);
        }
        for (const auto& [id, bid] : winners) {
            static_cast<void>(id);
            next.add_winners()->CopyFrom(bid);
            auto* relay = next.add_winner_relays();
            relay->mutable_bid()->CopyFrom(bid);
            relay->add_path_agent_ids(bid.bidder_id());
            if (bid.bidder_id() != agent_id_) {
                relay->add_path_agent_ids(agent_id_);
            }
        }
        for (const auto& task : observation.known_tasks()) {
            if (task.assigned_agent_id().empty()) {
                continue;
            }
            auto* owner = next.add_owners();
            owner->set_epoch(task.allocation_epoch());
            owner->set_version(task.allocation_version());
            owner->set_task_id(task.id());
            owner->set_agent_id(task.assigned_agent_id());
            owner->set_score(task.allocation_score());
        }
        const auto previous = protocol::deterministic_bytes(state_);
        const auto candidate = protocol::deterministic_bytes(next);
        if (candidate != previous) {
            next.set_version(state_.version() + 1);
        }
        state_ = std::move(next);
        serialized_state_ = protocol::deterministic_bytes(state_);
    }

    bool coordinated(
        const BidMap& winners,
        const std::vector<std::string>& member_ids) const {
        for (const auto& peer : member_ids) {
            const auto state = peer_states_.find(peer);
            if (state == peer_states_.end()) {
                return false;
            }
            for (const auto& [id, winner] : winners) {
                static_cast<void>(id);
                const auto* peer_winner = winner_for(state->second, winner.task_id());
                if (peer_winner == nullptr || !same_bid(*peer_winner, winner)) {
                    return false;
                }
            }
        }
        return true;
    }

    std::vector<v1::AllocationCommit> commits(
        const BidMap& winners) const {
        std::vector<v1::AllocationCommit> result;
        for (const auto& [id, winner] : winners) {
            static_cast<void>(id);
            if (winner.bidder_id() != agent_id_) {
                continue;
            }
            auto& commit = result.emplace_back();
            commit.set_epoch(epoch_);
            commit.set_version(state_.version());
            commit.set_task_id(winner.task_id());
            commit.set_agent_id(agent_id_);
            commit.set_distance_mm(winner.distance_mm());
            commit.set_score(winner.score());
        }
        return result;
    }

    std::vector<v1::NetworkMessage> outgoing(
        std::uint64_t tick, const std::vector<std::string>& member_ids) {
        const auto changed = state_.version() != broadcast_version_;
        const auto refresh = tick >= last_broadcast_tick_ + 5;
        if (!changed && !refresh) {
            return {};
        }
        std::vector<v1::NetworkMessage> result;
        for (const auto& peer : member_ids) {
            auto& message = result.emplace_back();
            message.set_sender_id(agent_id_);
            message.set_recipient_id(peer);
            message.set_version(state_.version());
            message.set_payload(serialized_state_);
        }
        broadcast_version_ = state_.version();
        last_broadcast_tick_ = tick;
        return result;
    }

    std::string agent_id_;
    std::uint64_t epoch_{};
    std::uint64_t map_version_{};
    std::uint64_t bid_version_{};
    std::uint64_t broadcast_version_{};
    std::uint64_t last_broadcast_tick_{};
    BidMap own_bids_;
    std::map<std::string, v1::AllocationState> peer_states_;
    v1::AllocationState state_;
    std::string serialized_state_;
};

Allocator::Allocator(std::string agent_id)
    : impl_(std::make_unique<Impl>(std::move(agent_id))) {}

Allocator::~Allocator() = default;

AllocationResult Allocator::update(
    const v1::AgentObservation& observation) {
    return impl_->update(observation);
}

}
