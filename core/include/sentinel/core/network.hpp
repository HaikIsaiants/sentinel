#pragma once

#include <sentinel/core/hash.hpp>
#include <sentinel/v1/sentinel.pb.h>

#include <cstdint>
#include <deque>
#include <string_view>
#include <vector>

namespace sentinel::core {

struct NetworkStep {
    std::vector<sentinel::v1::NetworkMessage> delivered;
    std::vector<sentinel::v1::NetworkOutcome> outcomes;
};

class NetworkEmulator {
public:
    NetworkEmulator(std::uint64_t seed, std::uint64_t step_ms);
    void set_profile(const sentinel::v1::NetworkProfile& profile);
    NetworkStep step(std::uint64_t tick, const std::vector<sentinel::v1::NetworkMessage>& outgoing);
    std::uint64_t communication_bytes() const;
    std::uint64_t communication_messages() const;
    std::uint64_t delivered_messages() const;
    void append_hash(HashBuilder& hash) const;

private:
    struct Packet {
        sentinel::v1::NetworkMessage message;
        std::uint64_t sequence{};
        std::uint64_t serialized_bytes{};
        std::uint64_t enqueue_tick{};
        std::uint64_t delivery_tick{};
    };

    void validate_profile(const sentinel::v1::NetworkProfile& profile) const;
    void validate_message(const sentinel::v1::NetworkMessage& message) const;
    Packet make_packet(
        std::uint64_t tick, const sentinel::v1::NetworkMessage& message);
    void accept_outgoing(
        std::uint64_t tick,
        const std::vector<sentinel::v1::NetworkMessage>& outgoing,
        NetworkStep& result);
    void deliver_ready(std::uint64_t tick, NetworkStep& result);
    bool ready_to_deliver(const Packet& packet, std::uint64_t tick) const;
    void append_queued_outcome(
        const Packet& packet, std::uint64_t tick, NetworkStep& result) const;
    void append_delivered_outcome(
        const Packet& packet, std::uint64_t tick, NetworkStep& result) const;
    void sort_deliveries(NetworkStep& result) const;
    std::uint64_t serialized_size(
        const sentinel::v1::NetworkMessage& message) const;
    sentinel::v1::NetworkOutcome outcome(
        const Packet& packet, sentinel::v1::NetworkDisposition disposition,
        std::uint64_t tick) const;

    std::uint64_t seed_{};
    std::uint64_t step_ms_{};
    sentinel::v1::NetworkProfile profile_;
    std::deque<Packet> queue_;
    std::uint64_t next_sequence_{1};
    std::uint64_t communication_bytes_{};
    std::uint64_t communication_messages_{};
    std::uint64_t delivered_messages_{};
};

}
