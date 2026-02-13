#include <sentinel/core/simulator.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <stdexcept>

namespace {

sentinel::v1::Scenario scenario() {
    sentinel::v1::Scenario result;
    result.set_schema_version(1);
    result.set_name("failure-recovery");
    result.set_seed(17);
    result.set_step_ms(100);
    result.set_max_ticks(20);
    result.set_network_profile("local");
    result.set_allocation_policy(
        sentinel::v1::ALLOCATION_POLICY_NEAREST_CAPABLE);
    result.set_failure_detection_ticks(2);
    result.mutable_world()->set_width_mm(5000);
    result.mutable_world()->set_height_mm(5000);
    result.mutable_world()->set_grid_cell_mm(1000);
    result.mutable_world()->set_map_version(1);
    result.add_network_profiles()->set_id("local");
    for (const auto* id : {"alpha", "bravo"}) {
        auto* vehicle = result.add_vehicles();
        vehicle->set_id(id);
        vehicle->set_kind("ugv");
        vehicle->set_max_speed_mm_s(1000);
        vehicle->set_initial_energy_mj(100000);
        vehicle->set_energy_cost_mj_per_meter(100);
        vehicle->set_payload_grams(1000);
        vehicle->add_capabilities(
            sentinel::v1::CAPABILITY_SEARCH);
    }
    auto* task = result.add_tasks();
    task->set_id("search");
    task->set_kind("search");
    task->mutable_target()->set_x_mm(3000);
    task->set_required_capability(
        sentinel::v1::CAPABILITY_SEARCH);
    task->set_deadline_tick(19);
    task->set_completion_radius_mm(100);
    task->set_assigned_agent_id("bravo");
    task->set_released(true);
    task->set_service_ticks(1);
    auto* failure = result.add_events();
    failure->set_id("disable-bravo");
    failure->set_tick(1);
    failure->set_kind(
        sentinel::v1::TAPE_EVENT_KIND_DISABLE_VEHICLE);
    failure->set_target_id("bravo");
    return result;
}

sentinel::v1::ActionBatch idle(
    const sentinel::v1::ObservationBatch& observations) {
    sentinel::v1::ActionBatch result;
    result.set_tick(observations.tick());
    for (const auto& current : observations.observations()) {
        auto* envelope = result.add_actions();
        envelope->set_schema_version(1);
        envelope->set_sequence(current.sequence());
        envelope->set_sender_id(current.recipient_id());
        envelope->set_recipient_id("sim");
        envelope->mutable_action()->set_tick(
            observations.tick());
        envelope->mutable_action()->set_behavior_mode(
            sentinel::v1::BEHAVIOR_MODE_IDLE);
    }
    return result;
}

const sentinel::v1::AgentObservation& observation(
    const sentinel::v1::ObservationBatch& batch,
    const char* agent_id) {
    const auto current = std::find_if(
        batch.observations().begin(),
        batch.observations().end(),
        [&](const auto& envelope) {
            return envelope.recipient_id() == agent_id;
        });
    if (current == batch.observations().end()) {
        throw std::invalid_argument("missing observation");
    }
    return current->observation();
}

}

TEST(FailureRecovery, DetectsThenReopensTheFailedOwnersTask) {
    sentinel::core::Simulator simulator(scenario());
    auto observations = simulator.observe();
    observations = simulator.step(idle(observations));
    observations = simulator.step(idle(observations));
    EXPECT_TRUE(
        observation(observations, "alpha")
            .failure_detections()
            .empty());

    observations = simulator.step(idle(observations));
    const auto& detected = observation(observations, "alpha");
    ASSERT_EQ(detected.failure_detections_size(), 1);
    EXPECT_EQ(
        detected.failure_detections(0).failed_agent_id(),
        "bravo");
    EXPECT_EQ(detected.failure_detections(0).failure_tick(), 1);
    EXPECT_EQ(detected.failure_detections(0).detection_tick(), 3);

    auto report = idle(observations);
    report.mutable_actions(0)
        ->mutable_action()
        ->add_failure_detections()
        ->CopyFrom(detected.failure_detections(0));
    observations = simulator.step(report);
    const auto& reopened = observation(observations, "alpha");
    ASSERT_EQ(reopened.available_tasks_size(), 1);
    EXPECT_EQ(reopened.available_tasks(0).id(), "search");
    EXPECT_EQ(reopened.allocation_epoch(), 2);
    EXPECT_EQ(simulator.summary().failure_detections_size(), 1);
    ASSERT_EQ(simulator.summary().task_reassignments_size(), 1);
    EXPECT_FALSE(
        simulator.summary().task_reassignments(0).complete());
}

TEST(FailureRecovery, RecordsTheReplacementCommit) {
    sentinel::core::Simulator simulator(scenario());
    auto observations = simulator.observe();
    observations = simulator.step(idle(observations));
    observations = simulator.step(idle(observations));
    observations = simulator.step(idle(observations));
    auto report = idle(observations);
    report.mutable_actions(0)
        ->mutable_action()
        ->add_failure_detections()
        ->CopyFrom(
            observation(observations, "alpha")
                .failure_detections(0));
    observations = simulator.step(report);

    auto commit = idle(observations);
    auto* proposal = commit.mutable_actions(0)
                         ->mutable_action()
                         ->add_allocation_commits();
    proposal->set_epoch(2);
    proposal->set_version(1);
    proposal->set_task_id("search");
    proposal->set_agent_id("alpha");
    proposal->set_distance_mm(3000);
    observations = simulator.step(commit);

    ASSERT_EQ(
        observation(observations, "alpha")
            .assigned_tasks_size(),
        1);
    const auto summary = simulator.summary();
    ASSERT_EQ(summary.task_reassignments_size(), 1);
    EXPECT_TRUE(summary.task_reassignments(0).complete());
    EXPECT_EQ(
        summary.task_reassignments(0).new_agent_id(),
        "alpha");
    EXPECT_EQ(summary.missing_reassignments(), 0);
    EXPECT_EQ(summary.maximum_reassignment_delay_ms(), 100);
}
