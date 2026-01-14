#pragma once

#include <sentinel/core/network.hpp>
#include <sentinel/core/rng.hpp>
#include <sentinel/v1/sentinel.pb.h>

#include <cstddef>
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
    sentinel::v1::ObservationBatch step(
        const sentinel::v1::ActionBatch& actions);
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
        std::int64_t energy_capacity_mj{};
        bool active{true};
    };
    struct Task {
        sentinel::v1::TaskSpec spec;
        bool released{};
        sentinel::v1::TaskStatus status{
            sentinel::v1::TASK_STATUS_PENDING};
        std::uint64_t progress_ticks{};
    };
    struct AcceptedAction {
        Vehicle* vehicle{};
        const sentinel::v1::Envelope* envelope{};
    };

    void initialize_network();
    void initialize_vehicles();
    void initialize_tasks();
    Vehicle& vehicle(std::string_view id);
    const Vehicle& vehicle(std::string_view id) const;
    Task& task(std::string_view id);
    sentinel::v1::Region& region(std::string_view id);
    const sentinel::v1::NetworkProfile& network_profile(
        std::string_view id) const;
    sentinel::v1::VehicleState vehicle_state(
        const Vehicle& vehicle) const;
    sentinel::v1::TaskState task_state(const Task& task) const;
    sentinel::v1::Envelope observation_for(
        const Vehicle& vehicle) const;
    std::vector<AcceptedAction> accept_actions(
        const sentinel::v1::ActionBatch& actions);
    void apply_action_commands(
        const std::vector<AcceptedAction>& accepted);
    std::vector<sentinel::v1::NetworkMessage>
    collect_outgoing(
        const std::vector<AcceptedAction>& accepted) const;
    void account_behaviors(
        const std::vector<AcceptedAction>& accepted);
    void apply_charging(
        const std::vector<AcceptedAction>& accepted);
    void apply_events();
    std::int64_t resolve_event_value(
        const sentinel::v1::TapeEvent& event);
    void dispatch_event(
        const sentinel::v1::TapeEvent& event,
        std::int64_t resolved_value);
    void advance_motion();
    void advance_vehicle(Vehicle& vehicle);
    void advance_tasks(
        const std::vector<AcceptedAction>& accepted);
    void advance_task(
        Task& task,
        const std::vector<AcceptedAction>& accepted);
    const sentinel::v1::AgentAction* action_for(
        const std::vector<AcceptedAction>& accepted,
        std::string_view agent_id) const;
    bool reports_work(
        const sentinel::v1::AgentAction* action,
        std::string_view task_id) const;
    bool at_task(const Vehicle& vehicle, const Task& task) const;
    void append_vehicle_hash(
        HashBuilder& hash, const Vehicle& vehicle) const;
    void append_task_hash(
        HashBuilder& hash, const Task& task) const;
    void record_hash();

    sentinel::v1::Scenario scenario_;
    std::vector<Vehicle> vehicles_;
    std::vector<Task> tasks_;
    RngStreams rng_streams_;
    NetworkEmulator network_;
    std::vector<sentinel::v1::NetworkMessage>
        delivered_messages_;
    std::uint64_t tick_{};
    std::size_t next_event_{};
    std::string state_hash_;
    std::vector<std::string> tick_hashes_;
    std::uint64_t wait_ticks_{};
    std::uint64_t replan_count_{};
    std::uint64_t recharge_ticks_{};
    std::uint64_t return_ticks_{};
    std::uint64_t travel_distance_mm_{};
    std::uint64_t energy_consumed_mj_{};
};

}
