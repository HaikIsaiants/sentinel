#include <sentinel/core/event_log.hpp>

#include <sentinel/core/scenario.hpp>
#include <sentinel/core/simulator.hpp>
#include <sentinel/protocol/framing.hpp>

#include <fstream>
#include <stdexcept>

namespace sentinel::core {

namespace {

void write_entry(std::ostream& output, const sentinel::v1::EventLogEntry& entry) {
    sentinel::protocol::write_frame(output, entry, false);
}

sentinel::v1::EventLogEntry read_required(std::istream& input) {
    sentinel::v1::EventLogEntry entry;
    if (!sentinel::protocol::read_frame(input, entry)) {
        throw std::runtime_error("event log ended unexpectedly");
    }
    return entry;
}

void write_json_string(std::ostream& output, std::string_view value) {
    output << '"';
    for (const auto character : value) {
        if (character == '"' || character == '\\') {
            output << '\\';
        }
        output << character;
    }
    output << '"';
}

}

EventLogWriter::EventLogWriter(
    const std::filesystem::path& path, const sentinel::v1::Scenario& scenario)
    : output_(path, std::ios::binary | std::ios::trunc) {
    if (!output_) {
        throw std::runtime_error("failed to open event log");
    }
    sentinel::v1::EventLogEntry entry;
    auto* header = entry.mutable_header();
    header->set_schema_version(1);
    header->set_scenario_name(scenario.name());
    header->set_scenario_seed(scenario.seed());
    header->set_scenario_hash(scenario_hash(scenario));
    write_entry(output_, entry);
}

void EventLogWriter::append(
    std::uint64_t tick, const sentinel::v1::ActionBatch& actions,
    const std::vector<sentinel::v1::NetworkOutcome>& network_outcomes,
    const std::string& state_hash) {
    if (finished_ || actions.tick() != tick) {
        throw std::logic_error("invalid event log append");
    }
    sentinel::v1::EventLogEntry entry;
    auto* record = entry.mutable_record();
    record->set_sequence(++sequence_);
    record->set_tick(tick);
    record->mutable_actions()->CopyFrom(actions);
    record->set_state_hash(state_hash);
    for (const auto& outcome : network_outcomes) {
        record->add_network_outcomes()->CopyFrom(outcome);
    }
    write_entry(output_, entry);
}

void EventLogWriter::finish(const sentinel::v1::SimulationSummary& summary) {
    if (finished_) {
        throw std::logic_error("event log already finished");
    }
    sentinel::v1::EventLogEntry entry;
    entry.mutable_footer()->mutable_summary()->CopyFrom(summary);
    write_entry(output_, entry);
    output_.flush();
    if (!output_) {
        throw std::runtime_error("failed to finish event log");
    }
    finished_ = true;
}

sentinel::v1::SimulationSummary replay_event_log(
    const sentinel::v1::Scenario& scenario, const std::filesystem::path& path,
    std::vector<sentinel::v1::ObservationBatch>* observations) {
    std::ifstream input(path, std::ios::binary);
    if (!input) {
        throw std::runtime_error("failed to open event log");
    }
    const auto first = read_required(input);
    if (!first.has_header() || first.header().schema_version() != 1
        || first.header().scenario_hash() != scenario_hash(scenario)) {
        throw std::runtime_error("event log header does not match scenario");
    }
    Simulator simulator(scenario);
    if (observations) {
        observations->push_back(simulator.observe());
    }
    std::uint64_t expected_sequence = 1;
    while (true) {
        const auto entry = read_required(input);
        if (entry.has_footer()) {
            const auto actual = simulator.summary();
            if (entry.footer().summary().terminal_hash() != actual.terminal_hash()) {
                throw std::runtime_error("event log footer hash mismatch");
            }
            return actual;
        }
        if (!entry.has_record() || entry.record().sequence() != expected_sequence++
            || entry.record().tick() != simulator.tick()) {
            throw std::runtime_error("invalid event record order");
        }
        const auto observation = simulator.step(entry.record().actions());
        if (simulator.state_hash() != entry.record().state_hash()) {
            throw std::runtime_error("event log state hash mismatch");
        }
        if (observations) {
            observations->push_back(observation);
        }
    }
}

sentinel::v1::SimulationSummary export_replay_trace(
    const sentinel::v1::Scenario& scenario, const std::filesystem::path& log_path,
    const std::filesystem::path& trace_path) {
    std::vector<sentinel::v1::ObservationBatch> observations;
    const auto summary = replay_event_log(scenario, log_path, &observations);
    std::ofstream output(trace_path, std::ios::trunc);
    if (!output) {
        throw std::runtime_error("failed to open trace output");
    }
    output << "{\"scenario\":";
    write_json_string(output, scenario.name());
    output << ",\"terminal_hash\":";
    write_json_string(output, summary.terminal_hash());
    output << ",\"frames\":[";
    bool first_frame = true;
    for (const auto& batch : observations) {
        for (const auto& envelope : batch.observations()) {
            if (!first_frame) {
                output << ',';
            }
            first_frame = false;
            const auto& self = envelope.observation().self();
            output << "{\"tick\":" << batch.tick() << ",\"agent\":";
            write_json_string(output, self.id());
            output << ",\"x_mm\":" << self.position().x_mm()
                   << ",\"y_mm\":" << self.position().y_mm()
                   << ",\"energy_mj\":" << self.energy_mj() << '}';
        }
    }
    output << "]}\n";
    if (!output) {
        throw std::runtime_error("failed to write trace");
    }
    return summary;
}

}
