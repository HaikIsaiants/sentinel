#pragma once

#include <sentinel/v1/sentinel.pb.h>

namespace sentinel::test {

inline v1::Scenario baseline_scenario() {
    v1::Scenario result;
    result.set_schema_version(1);
    result.set_name("baseline");
    result.set_seed(41);
    result.set_step_ms(100);
    result.set_max_ticks(40);
    result.set_network_profile("local");
    result.set_allocation_policy(v1::ALLOCATION_POLICY_SCRIPTED);
    result.mutable_world()->set_width_mm(10000);
    result.mutable_world()->set_height_mm(10000);
    result.mutable_world()->set_grid_cell_mm(1000);
    auto* profile = result.add_network_profiles();
    profile->set_id("local");
    auto* vehicle = result.add_vehicles();
    vehicle->set_id("agent-a");
    vehicle->set_kind("ugv");
    vehicle->mutable_initial_position()->set_x_mm(0);
    vehicle->mutable_initial_position()->set_y_mm(0);
    vehicle->set_max_speed_mm_s(1000);
    vehicle->set_initial_energy_mj(100000);
    vehicle->set_energy_cost_mj_per_meter(100);
    vehicle->set_payload_grams(1000);
    vehicle->add_capabilities(v1::CAPABILITY_SEARCH);
    auto* task = result.add_tasks();
    task->set_id("task-a");
    task->set_kind("search");
    task->mutable_target()->set_x_mm(1000);
    task->mutable_target()->set_y_mm(0);
    task->set_required_capability(v1::CAPABILITY_SEARCH);
    task->set_deadline_tick(30);
    task->set_completion_radius_mm(100);
    task->set_assigned_agent_id("agent-a");
    task->set_released(true);
    task->set_service_ticks(2);
    return result;
}

inline v1::ActionBatch action(std::uint64_t tick, std::int64_t velocity_x, bool working = false) {
    v1::ActionBatch result;
    result.set_tick(tick);
    auto* envelope = result.add_actions();
    envelope->set_schema_version(1);
    envelope->set_sequence(tick + 1);
    envelope->set_sender_id("agent-a");
    envelope->set_recipient_id("sim");
    envelope->mutable_action()->set_tick(tick);
    envelope->mutable_action()->set_velocity_x_mm_s(velocity_x);
    if (working) {
        auto* report = envelope->mutable_action()->add_task_reports();
        report->set_task_id("task-a");
        report->set_kind(v1::TASK_REPORT_KIND_WORKING);
    }
    return result;
}

}
