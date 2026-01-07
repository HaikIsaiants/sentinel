#include <sentinel/core/scenario.hpp>
#include <sentinel/core/simulator.hpp>
#include <sentinel/protocol/arguments.hpp>
#include <sentinel/protocol/framing.hpp>
#include <sentinel/v1/sentinel.pb.h>

#include <iostream>

int main(int argc, char* argv[]) {
    try {
        sentinel::protocol::configure_binary_stdio();
        const auto scenario_path = sentinel::protocol::required_option(argc, argv, "--scenario");
        sentinel::core::Simulator simulator(sentinel::core::load_scenario(scenario_path));
        sentinel::v1::SimulationFrame output;
        output.mutable_observations()->CopyFrom(simulator.observe());
        sentinel::protocol::write_frame(std::cout, output);
        sentinel::v1::SimulationFrame input;
        while (!simulator.finished() && sentinel::protocol::read_frame(std::cin, input)) {
            if (!input.has_actions()) {
                throw std::invalid_argument("expected action batch");
            }
            output.Clear();
            output.mutable_observations()->CopyFrom(simulator.step(input.actions()));
            sentinel::protocol::write_frame(std::cout, output);
            input.Clear();
        }
        return simulator.finished() ? 0 : 2;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
