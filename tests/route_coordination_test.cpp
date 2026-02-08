#include <sentinel/agent/autonomy.hpp>

#include <gtest/gtest.h>

namespace {

sentinel::v1::Envelope observation(std::uint64_t tick) {
    sentinel::v1::Envelope result;
    result.set_schema_version(1);
    result.set_sequence(tick + 1);
    result.set_sender_id("sim");
    result.set_recipient_id("alpha");
    auto* value = result.mutable_observation();
    value->set_tick(tick);
    value->set_step_ms(1000);
    value->set_allocation_epoch(1);
    value->set_allocation_policy(
        sentinel::v1::ALLOCATION_POLICY_NEAREST_CAPABLE);
    auto* world = value->mutable_world();
    world->set_width_mm(4000);
    world->set_height_mm(4000);
    world->set_grid_cell_mm(1000);
    world->set_map_version(7);
    auto* region = world->add_regions();
    region->set_id("bridge");
    region->set_kind(
        sentinel::v1::REGION_KIND_CHOKEPOINT);
    region->mutable_minimum()->set_x_mm(1000);
    region->mutable_minimum()->set_y_mm(0);
    region->mutable_maximum()->set_x_mm(2000);
    region->mutable_maximum()->set_y_mm(1000);
    region->set_energy_multiplier_permille(1000);

    auto* self = value->mutable_self();
    self->set_id("alpha");
    self->set_kind("ugv");
    self->set_max_speed_mm_s(1000);
    self->set_initial_energy_mj(100000);
    self->set_energy_capacity_mj(100000);
    self->set_energy_mj(100000);
    self->set_energy_cost_mj_per_meter(100);
    self->set_payload_grams(1000);
    self->set_active(true);
    self->add_capabilities(sentinel::v1::CAPABILITY_SEARCH);

    auto* task = value->add_assigned_tasks();
    task->set_id("search");
    task->set_kind("search");
    task->mutable_target()->set_x_mm(3000);
    task->mutable_target()->set_y_mm(0);
    task->set_required_capability(
        sentinel::v1::CAPABILITY_SEARCH);
    task->set_deadline_tick(100);
    task->set_completion_radius_mm(100);
    task->set_status(sentinel::v1::TASK_STATUS_PENDING);
    task->set_service_ticks(1);
    task->set_allocation_epoch(1);
    task->set_allocation_version(1);
    task->set_assigned_agent_id("alpha");
    return result;
}

}

TEST(RouteCoordination, ReservesChokepointBeforeMoving) {
    sentinel::agent::Controller controller("alpha");
    const auto waiting = controller.act(observation(5));
    EXPECT_EQ(
        waiting.action().behavior_mode(),
        sentinel::v1::BEHAVIOR_MODE_WAITING);
    EXPECT_EQ(waiting.action().velocity_x_mm_s(), 0);
    ASSERT_EQ(waiting.action().reservation_proposals_size(), 1);
    const auto& proposal =
        waiting.action().reservation_proposals(0);
    EXPECT_EQ(proposal.resource_id(), "bridge");
    EXPECT_EQ(proposal.agent_id(), "alpha");
    EXPECT_EQ(proposal.start_tick(), 6);
    EXPECT_GE(proposal.end_tick(), proposal.start_tick());
    EXPECT_EQ(
        proposal.route_version(),
        waiting.action().route_version());
    EXPECT_EQ(proposal.map_version(), 7);
}

TEST(RouteCoordination, MovesAfterReceivingItsGrant) {
    sentinel::agent::Controller controller("alpha");
    const auto waiting = controller.act(observation(5));
    ASSERT_EQ(waiting.action().reservation_proposals_size(), 1);

    auto granted = observation(6);
    granted.mutable_observation()
        ->add_committed_reservations()
        ->CopyFrom(waiting.action().reservation_proposals(0));
    const auto moving = controller.act(granted);
    EXPECT_EQ(
        moving.action().behavior_mode(),
        sentinel::v1::BEHAVIOR_MODE_NAVIGATING);
    EXPECT_EQ(moving.action().velocity_x_mm_s(), 1000);
    EXPECT_TRUE(
        moving.action().reservation_proposals().empty());
}
