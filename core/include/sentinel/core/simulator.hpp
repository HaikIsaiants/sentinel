#pragma once

#include <sentinel/core/network.hpp>
#include <sentinel/core/rng.hpp>
#include <sentinel/planning/planner.hpp>
#include <sentinel/v1/sentinel.pb.h>

#include <cstddef>
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
        std::int64_t energy_capacity_mj{};
        std::uint64_t route_version{};
        bool active{true};
    };
    struct Task {
        sentinel::v1::TaskSpec spec;
        bool released{};
        sentinel::v1::TaskStatus status{sentinel::v1::TASK_STATUS_PENDING};
        std::uint64_t progress_ticks{};
        std::uint64_t allocation_epoch{};
        std::uint64_t allocation_version{};
        std::int64_t allocation_score{};
        std::uint32_t bundle_position{};
    };
    struct FailureMonitor {
        std::string failed_id;
        std::string detector_id;
        std::uint64_t failure_tick{};
        std::uint64_t detection_tick{};
    };

    Vehicle& vehicle(std::string_view id);
    const Vehicle& vehicle(std::string_view id) const;
    sentinel::v1::VehicleState vehicle_state(const Vehicle& vehicle) const;
    sentinel::v1::TaskState task_state(const Task& task) const;
    std::vector<sentinel::v1::NetworkMessage> apply_actions(
        const sentinel::v1::ActionBatch& actions);
    void apply_commits(const sentinel::v1::ActionBatch& actions);
    void apply_failure_detections(
        const sentinel::v1::ActionBatch& actions);
    void apply_reservations(const sentinel::v1::ActionBatch& actions);
    bool valid_reservation(
        const sentinel::v1::SpaceTimeReservation& proposal,
        const sentinel::v1::Envelope& envelope) const;
    bool has_reservation(
        std::string_view agent_id, std::string_view resource_id,
        std::uint64_t route_version) const;
    void apply_events();
    void advance_motion();
    void advance_tasks(const sentinel::v1::ActionBatch& actions);
    void record_hash();

    sentinel::v1::Scenario scenario_;
    std::vector<Vehicle> vehicles_;
    std::vector<Task> tasks_;
    RngStreams rng_streams_;
    NetworkEmulator network_;
    planning::ReservationTable reservations_;
    std::vector<sentinel::v1::NetworkMessage> delivered_messages_;
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
    std::uint64_t allocation_epoch_{1};
    std::uint64_t rejected_commits_{};
    std::vector<FailureMonitor> failure_monitors_;
    std::vector<std::string> handled_failures_;
    std::vector<sentinel::v1::FailureDetection>
        failure_detections_;
    std::vector<sentinel::v1::TaskReassignment>
        task_reassignments_;
    std::vector<sentinel::v1::ReplanningSample>
        replanning_samples_;
};

}
