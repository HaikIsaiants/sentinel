#pragma once

#include <stdexcept>
#include <string>
#include <string_view>

namespace sentinel::protocol {

inline std::string required_option(int argc, char* const argv[], std::string_view name, int first = 1) {
    for (int index = first; index + 1 < argc; ++index) {
        if (argv[index] == name) {
            return argv[index + 1];
        }
    }
    throw std::invalid_argument("missing required option");
}

}
