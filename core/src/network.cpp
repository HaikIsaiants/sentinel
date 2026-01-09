#include <sentinel/core/network.hpp>

#include <sentinel/protocol/framing.hpp>

#include <algorithm>
#include <stdexcept>

namespace sentinel::core {

NetworkEmulator::NetworkEmulator(std::uint64_t seed, std::uint64_t step_ms)
    : seed_(seed), step_ms_(step_ms) {
    if (step_ms_ == 0) {
        throw std::invalid_argument("network step cannot be zero");
    }
}

void NetworkEmulator::set_profile(const sentinel::v1::NetworkProfile& profile) {
    validate_profile(profile);
    profile_ = profile;
}

NetworkStep NetworkEmulator::step(
    std::uint64_t tick, const std::vector<sentinel::v1::NetworkMessage>& outgoing) {
    NetworkStep result;
    accept_outgoing(tick, outgoing, result);
    deliver_ready(tick, result);
    sort_deliveries(result);
    return result;
}

void NetworkEmulator::validate_profile(
    const sentinel::v1::NetworkProfile& profile) const {
    if (profile.id().empty()) {
        throw std::invalid_argument("network profile id is required");
    }
}

void NetworkEmulator::validate_message(
    const sentinel::v1::NetworkMessage& message) const {
    if (message.sender_id().empty()) {
        throw std::invalid_argument("network message sender is required");
    }
    if (message.recipient_id().empty()) {
        throw std::invalid_argument("network message recipient is required");
    }
    if (message.sender_id() == message.recipient_id()) {
        throw std::invalid_argument("network message cannot target its sender");
    }
}

std::uint64_t NetworkEmulator::serialized_size(
    const sentinel::v1::NetworkMessage& message) const {
    return static_cast<std::uint64_t>(
        sentinel::protocol::deterministic_bytes(message).size());
}

NetworkEmulator::Packet NetworkEmulator::make_packet(
    std::uint64_t tick,
    const sentinel::v1::NetworkMessage& message) {
    validate_message(message);
    Packet packet;
    packet.message = message;
    packet.sequence = next_sequence_++;
    packet.serialized_bytes = serialized_size(message);
    packet.enqueue_tick = tick;
    packet.delivery_tick =
        tick + profile_.latency_ticks() + 1;
    return packet;
}

void NetworkEmulator::accept_outgoing(
    std::uint64_t tick,
    const std::vector<sentinel::v1::NetworkMessage>& outgoing,
    NetworkStep& result) {
    for (const auto& message : outgoing) {
        auto packet = make_packet(tick, message);
        communication_bytes_ += packet.serialized_bytes;
        ++communication_messages_;
        append_queued_outcome(packet, tick, result);
        queue_.push_back(std::move(packet));
    }
}

void NetworkEmulator::deliver_ready(
    std::uint64_t tick, NetworkStep& result) {
    while (!queue_.empty() && ready_to_deliver(queue_.front(), tick)) {
        auto packet = std::move(queue_.front());
        queue_.pop_front();
        result.delivered.push_back(packet.message);
        append_delivered_outcome(packet, tick, result);
        ++delivered_messages_;
    }
}

bool NetworkEmulator::ready_to_deliver(
    const Packet& packet, std::uint64_t tick) const {
    return packet.delivery_tick <= tick;
}

void NetworkEmulator::append_queued_outcome(
    const Packet& packet, std::uint64_t tick,
    NetworkStep& result) const {
    result.outcomes.push_back(outcome(
        packet, sentinel::v1::NETWORK_DISPOSITION_QUEUED, tick));
}

void NetworkEmulator::append_delivered_outcome(
    const Packet& packet, std::uint64_t tick,
    NetworkStep& result) const {
    result.outcomes.push_back(outcome(
        packet, sentinel::v1::NETWORK_DISPOSITION_DELIVERED, tick));
}

void NetworkEmulator::sort_deliveries(NetworkStep& result) const {
    std::sort(result.delivered.begin(), result.delivered.end(), [](const auto& left, const auto& right) {
        if (left.recipient_id() != right.recipient_id()) {
            return left.recipient_id() < right.recipient_id();
        }
        if (left.sender_id() != right.sender_id()) {
            return left.sender_id() < right.sender_id();
        }
        return left.version() < right.version();
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
    const Packet& packet, sentinel::v1::NetworkDisposition disposition,
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
