#include <sentinel/core/network.hpp>

#include <sentinel/protocol/framing.hpp>

#include <algorithm>
#include <stdexcept>
#include <tuple>
#include <utility>

namespace sentinel::core {

NetworkEmulator::NetworkEmulator(
    std::uint64_t seed, std::uint64_t step_ms)
    : seed_(seed), step_ms_(step_ms) {
    if (step_ms_ == 0) {
        throw std::invalid_argument("network step cannot be zero");
    }
}

void NetworkEmulator::set_profile(
    const sentinel::v1::NetworkProfile& profile) {
    if (profile.id().empty()) {
        throw std::invalid_argument("network profile id is required");
    }
    profile_ = profile;
}

NetworkStep NetworkEmulator::step(
    std::uint64_t tick,
    const std::vector<sentinel::v1::NetworkMessage>& outgoing) {
    if (profile_.id().empty()) {
        throw std::logic_error("network profile has not been selected");
    }
    auto prepared = prepare_batch(outgoing);
    NetworkStep result;
    release_due(tick, result);
    enqueue_batch(tick, std::move(prepared), result);
    sort_delivered(result);
    return result;
}

std::vector<NetworkEmulator::PreparedPacket>
NetworkEmulator::prepare_batch(
    const std::vector<sentinel::v1::NetworkMessage>& outgoing) const {
    std::vector<PreparedPacket> prepared;
    prepared.reserve(outgoing.size());
    for (const auto& message : outgoing) {
        if (message.sender_id().empty()) {
            throw std::invalid_argument("network message sender is required");
        }
        if (message.recipient_id().empty()) {
            throw std::invalid_argument("network message recipient is required");
        }
        if (message.sender_id() == message.recipient_id()) {
            throw std::invalid_argument("network message cannot target its sender");
        }
        const auto duplicate = std::find_if(
            prepared.begin(), prepared.end(),
            [&](const PreparedPacket& current) {
                return std::tuple(
                           current.message.sender_id(),
                           current.message.recipient_id(),
                           current.message.version())
                       == std::tuple(
                           message.sender_id(),
                           message.recipient_id(),
                           message.version());
            });
        if (duplicate != prepared.end()) {
            throw std::invalid_argument("duplicate network message");
        }
        PreparedPacket packet;
        packet.message = message;
        packet.serialized_bytes = static_cast<std::uint64_t>(
            sentinel::protocol::deterministic_bytes(message).size());
        prepared.push_back(std::move(packet));
    }
    return prepared;
}

void NetworkEmulator::release_due(
    std::uint64_t tick, NetworkStep& result) {
    while (!queue_.empty()
           && queue_.front().delivery_tick <= tick) {
        auto packet = std::move(queue_.front());
        queue_.pop_front();
        result.delivered.push_back(packet.message);
        result.outcomes.push_back(outcome(
            packet,
            sentinel::v1::NETWORK_DISPOSITION_DELIVERED,
            tick));
        ++delivered_messages_;
    }
}

void NetworkEmulator::enqueue_batch(
    std::uint64_t tick,
    std::vector<PreparedPacket> prepared,
    NetworkStep& result) {
    for (auto& current : prepared) {
        Packet packet;
        packet.message = std::move(current.message);
        packet.sequence = next_sequence_++;
        packet.serialized_bytes = current.serialized_bytes;
        packet.enqueue_tick = tick;
        packet.delivery_tick =
            tick + profile_.latency_ticks() + 1;
        communication_bytes_ += packet.serialized_bytes;
        ++communication_messages_;
        result.outcomes.push_back(outcome(
            packet,
            sentinel::v1::NETWORK_DISPOSITION_QUEUED,
            tick));
        queue_.push_back(std::move(packet));
    }
}

void NetworkEmulator::sort_delivered(
    NetworkStep& result) {
    std::sort(
        result.delivered.begin(),
        result.delivered.end(),
        [](const auto& left, const auto& right) {
            return std::tuple(
                       left.recipient_id(),
                       left.sender_id(),
                       left.version())
                   < std::tuple(
                       right.recipient_id(),
                       right.sender_id(),
                       right.version());
        });
}

std::uint64_t NetworkEmulator::communication_bytes() const {
    return communication_bytes_;
}

std::uint64_t NetworkEmulator::communication_messages() const {
    return communication_messages_;
}

std::uint64_t NetworkEmulator::delivered_messages() const {
    return delivered_messages_;
}

void NetworkEmulator::append_hash(HashBuilder& hash) const {
    hash.unsigned_integer(seed_);
    hash.unsigned_integer(step_ms_);
    hash.text(profile_.id());
    hash.unsigned_integer(profile_.latency_ticks());
    hash.unsigned_integer(queue_.size());
    for (const auto& packet : queue_) {
        hash.unsigned_integer(packet.sequence);
        hash.text(packet.message.sender_id());
        hash.text(packet.message.recipient_id());
        hash.unsigned_integer(packet.message.version());
        hash.text(packet.message.payload());
        hash.unsigned_integer(packet.enqueue_tick);
        hash.unsigned_integer(packet.delivery_tick);
    }
}

sentinel::v1::NetworkOutcome NetworkEmulator::outcome(
    const Packet& packet,
    sentinel::v1::NetworkDisposition disposition,
    std::uint64_t tick) const {
    sentinel::v1::NetworkOutcome result;
    result.set_sequence(packet.sequence);
    result.set_disposition(disposition);
    result.set_tick(tick);
    result.mutable_message()->CopyFrom(packet.message);
    result.set_serialized_bytes(packet.serialized_bytes);
    result.set_enqueue_tick(packet.enqueue_tick);
    result.set_transmit_start_tick(packet.enqueue_tick);
    result.set_transmit_end_tick(packet.enqueue_tick);
    result.set_delivery_tick(packet.delivery_tick);
    result.set_profile_id(profile_.id());
    return result;
}

}
