#pragma once

#include <sentinel/core/network.hpp>
#include <sentinel/v1/sentinel.pb.h>

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace sentinel::core {

class Simulator {
public:
    explicit Simulator(sentinel::v1::Scenario scenario);
    sentinel::v1::ObservationBatch observe() const;
    sentinel::v1::ObservationBatch step(const sentinel::v1::ActionBatch& actions);
    sentinel::v1::SimulationSummary summary() const;
    std::string state_hash() const;
    bool finished() const;
    std::uint64_t tick() const;
    const sentinel::v1::Scenario& scenario() const;

private:
    struct Vehicle {
        sentinel::v1::VehicleSpec spec;
        std::int64_t x_mm{};
        std::int64_t y_mm{};
        std::int64_t velocity_x_mm_s{};
        std::int64_t velocity_y_mm_s{};
        std::int64_t energy_mj{};
    };
    struct Task {
        sentinel::v1::TaskSpec spec;
        sentinel::v1::TaskStatus status{sentinel::v1::TASK_STATUS_PENDING};
        std::uint64_t progress_ticks{};
    };

    Vehicle& vehicle(std::string_view id);
    const Vehicle& vehicle(std::string_view id) const;
    sentinel::v1::VehicleState vehicle_state(const Vehicle& vehicle) const;
    sentinel::v1::TaskState task_state(const Task& task) const;
    std::vector<sentinel::v1::NetworkMessage> apply_actions(
        const sentinel::v1::ActionBatch& actions);
    void advance_motion();
    void advance_tasks(const sentinel::v1::ActionBatch& actions);
    void record_hash();

    sentinel::v1::Scenario scenario_;
    std::vector<Vehicle> vehicles_;
    std::vector<Task> tasks_;
    NetworkEmulator network_;
    std::vector<sentinel::v1::NetworkMessage> delivered_messages_;
    std::uint64_t tick_{};
    std::string state_hash_;
    std::vector<std::string> tick_hashes_;
};

}
