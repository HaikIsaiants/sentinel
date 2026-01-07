#pragma once

#include <sentinel/v1/sentinel.pb.h>

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace sentinel::core {

class HashBuilder;

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
    struct Command {
        std::string agent_id;
        std::int64_t velocity_x_mm_s{};
        std::int64_t velocity_y_mm_s{};
        std::vector<sentinel::v1::TaskReport> reports;
    };

    void initialize_vehicles();
    void initialize_tasks();
    Vehicle& vehicle(std::string_view id);
    const Vehicle& vehicle(std::string_view id) const;
    sentinel::v1::VehicleState vehicle_state(const Vehicle& vehicle) const;
    sentinel::v1::TaskState task_state(const Task& task) const;
    sentinel::v1::Envelope observation_for(const Vehicle& vehicle) const;
    std::vector<Command> read_commands(const sentinel::v1::ActionBatch& actions) const;
    const Command* command_for(
        const std::vector<Command>& commands, std::string_view agent_id) const;
    void apply_commands(const std::vector<Command>& commands);
    void apply_command(Vehicle& vehicle, const Command& command);
    void advance_motion(const std::vector<Command>& commands);
    void advance_vehicle(Vehicle& vehicle);
    void consume_motion_energy(Vehicle& vehicle, std::int64_t distance_mm);
    void advance_tasks(const std::vector<Command>& commands);
    void advance_task(Task& task, const std::vector<Command>& commands);
    bool command_reports_work(const Command* command, std::string_view task_id) const;
    bool vehicle_can_service(const Vehicle& vehicle, const Task& task) const;
    bool vehicle_is_at_task(const Vehicle& vehicle, const Task& task) const;
    bool deadline_elapsed(const Task& task) const;
    bool all_tasks_terminal() const;
    std::uint32_t completed_task_count() const;
    std::uint32_t active_vehicle_count() const;
    void append_vehicle_hash(HashBuilder& hash, const Vehicle& vehicle) const;
    void append_task_hash(HashBuilder& hash, const Task& task) const;
    void record_hash();

    sentinel::v1::Scenario scenario_;
    std::vector<Vehicle> vehicles_;
    std::vector<Task> tasks_;
    std::uint64_t tick_{};
    std::string state_hash_;
    std::vector<std::string> tick_hashes_;
};

}
