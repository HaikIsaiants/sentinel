#include <sentinel/core/rng.hpp>

#include <sentinel/core/hash.hpp>

#include <algorithm>
#include <limits>
#include <stdexcept>

namespace sentinel::core {

namespace {

std::uint64_t mix(std::uint64_t value) {
    value += 0x9e3779b97f4a7c15ULL;
    value = (value ^ (value >> 30U)) * 0xbf58476d1ce4e5b9ULL;
    value = (value ^ (value >> 27U)) * 0x94d049bb133111ebULL;
    return value ^ (value >> 31U);
}

}

DeterministicRng::DeterministicRng(std::uint64_t seed) : state_(seed) {}

std::uint64_t DeterministicRng::next() {
    state_ = mix(state_);
    return state_;
}

std::int64_t DeterministicRng::uniform(std::int64_t minimum, std::int64_t maximum) {
    if (minimum > maximum) {
        throw std::invalid_argument("invalid random range");
    }
    const auto width = static_cast<std::uint64_t>(maximum - minimum) + 1U;
    const auto limit = std::numeric_limits<std::uint64_t>::max()
                       - (std::numeric_limits<std::uint64_t>::max() % width);
    std::uint64_t value{};
    do {
        value = next();
    } while (value >= limit);
    return minimum + static_cast<std::int64_t>(value % width);
}

std::uint64_t DeterministicRng::state() const {
    return state_;
}

RngStreams::RngStreams(std::uint64_t root_seed) : root_seed_(root_seed) {}

DeterministicRng& RngStreams::stream(std::string_view name) {
    const auto current = std::find_if(entries_.begin(), entries_.end(),
                                      [&](const Entry& entry) { return entry.name == name; });
    if (current != entries_.end()) {
        return current->rng;
    }
    entries_.push_back(Entry{std::string(name), DeterministicRng(stream_seed(root_seed_, name))});
    std::sort(entries_.begin(), entries_.end(),
              [](const Entry& left, const Entry& right) { return left.name < right.name; });
    return std::find_if(entries_.begin(), entries_.end(),
                        [&](const Entry& entry) { return entry.name == name; })->rng;
}

std::vector<std::pair<std::string, std::uint64_t>> RngStreams::states() const {
    std::vector<std::pair<std::string, std::uint64_t>> result;
    for (const auto& entry : entries_) {
        result.emplace_back(entry.name, entry.rng.state());
    }
    return result;
}

std::uint64_t stream_seed(std::uint64_t root_seed, std::string_view name) {
    HashBuilder hash;
    hash.unsigned_integer(root_seed);
    hash.text(name);
    const auto text = hash.finish();
    return std::stoull(text, nullptr, 16);
}

}
