#include <sentinel/core/hash.hpp>

#include <array>
#include <iomanip>
#include <sstream>

namespace sentinel::core {

void HashBuilder::bytes(const void* data, std::size_t size) {
    const auto* current = static_cast<const unsigned char*>(data);
    for (std::size_t index = 0; index < size; ++index) {
        state_ ^= current[index];
        state_ *= 1099511628211ULL;
    }
}

void HashBuilder::unsigned_integer(std::uint64_t value) {
    std::array<unsigned char, 8> encoded{};
    for (std::size_t index = 0; index < encoded.size(); ++index) {
        encoded[index] = static_cast<unsigned char>((value >> ((encoded.size() - index - 1U) * 8U)) & 0xffU);
    }
    bytes(encoded.data(), encoded.size());
}

void HashBuilder::signed_integer(std::int64_t value) {
    unsigned_integer(static_cast<std::uint64_t>(value));
}

void HashBuilder::boolean(bool value) {
    const unsigned char encoded = value ? 1U : 0U;
    bytes(&encoded, 1);
}

void HashBuilder::text(std::string_view value) {
    unsigned_integer(value.size());
    bytes(value.data(), value.size());
}

std::string HashBuilder::finish() const {
    std::ostringstream output;
    output << std::hex << std::setfill('0') << std::setw(16) << state_;
    return output.str();
}

std::string hash_bytes(std::string_view value) {
    HashBuilder hash;
    hash.text(value);
    return hash.finish();
}

}
