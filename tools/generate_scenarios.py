from __future__ import annotations

import argparse
import json
import pathlib
import random
import re
from dataclasses import dataclass


POLICIES = {
    "nearest": "ALLOCATION_POLICY_NEAREST_CAPABLE",
    "cbba": "ALLOCATION_POLICY_SENTINEL_CBBA",
}
STRATA = {
    "nominal",
    "latency",
    "loss",
    "partition",
    "blocked_route",
    "agent_loss",
}
IDENTIFIER = re.compile(r"^[a-z0-9][a-z0-9-]{0,63}$")


@dataclass(frozen=True)
class SeedRecord:
    identifier: str
    seed: int
    stratum: str


class Draws:
    """Sequential draws are deterministic for a fixed scenario schema."""

    def __init__(self, seed: int):
        self._rng = random.Random(seed)

    def integer(self, minimum: int, maximum: int) -> int:
        return self._rng.randint(minimum, maximum)

    def choose(self, values):
        return values[self._rng.randrange(len(values))]

    def chance(self, numerator: int, denominator: int) -> bool:
        return self._rng.randrange(denominator) < numerator


def load_records(path: pathlib.Path) -> list[SeedRecord]:
    records = []
    seen_ids = set()
    seen_seeds = set()
    for number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if not line.strip():
            continue
        value = json.loads(line)
        record = SeedRecord(
            identifier=str(value["id"]),
            seed=int(value["seed"]),
            stratum=str(value["stratum"]),
        )
        if not IDENTIFIER.fullmatch(record.identifier):
            raise ValueError(f"invalid seed id on line {number}")
        if record.stratum not in STRATA:
            raise ValueError(f"invalid stratum on line {number}")
        if record.seed < 0 or record.seed > 0x7FFFFFFFFFFFFFFF:
            raise ValueError(f"invalid seed on line {number}")
        if record.identifier in seen_ids or record.seed in seen_seeds:
            raise ValueError(f"duplicate seed record on line {number}")
        seen_ids.add(record.identifier)
        seen_seeds.add(record.seed)
        records.append(record)
    if not records:
        raise ValueError("seed corpus is empty")
    return records


def point(x: int, y: int) -> str:
    return f"x_mm: {x} y_mm: {y}"


def region(identifier: str, kind: str, minimum: tuple[int, int], maximum: tuple[int, int]) -> str:
    return "\n".join(
        [
            "  regions {",
            f'    id: "{identifier}"',
            f"    kind: {kind}",
            f"    minimum {{ {point(*minimum)} }}",
            f"    maximum {{ {point(*maximum)} }}",
            "    energy_multiplier_permille: 1000",
            "  }",
        ]
    )


def vehicle(identifier: str, x: int, y: int, capability: str) -> str:
    return "\n".join(
        [
            "vehicles {",
            f'  id: "{identifier}"',
            '  kind: "ugv"',
            f"  initial_position {{ {point(x, y)} }}",
            "  max_speed_mm_s: 2000",
            "  initial_energy_mj: 600000",
            "  energy_cost_mj_per_meter: 1000",
            "  payload_grams: 5000",
            f"  capabilities: {capability}",
            '  terrain_access: "floor"',
            '  return_location_id: "base"',
            "}",
        ]
    )


def task(identifier: str, x: int, y: int, capability: str, priority: int) -> str:
    return "\n".join(
        [
            "tasks {",
            f'  id: "{identifier}"',
            f'  kind: "{capability.lower().removeprefix("capability_")}"',
            f"  target {{ {point(x, y)} }}",
            f"  required_capability: {capability}",
            "  deadline_tick: 4000",
            "  completion_radius_mm: 500",
            "  released: true",
            "  service_ticks: 5",
            "  service_energy_mj_per_tick: 100",
            f"  priority: {priority}",
            "}",
        ]
    )


def network_profile(record: SeedRecord, draws: Draws) -> tuple[str, list[str]]:
    latency = 0
    jitter = 0
    loss = 0
    events = []
    if record.stratum == "latency":
        latency = draws.integer(2, 6)
        jitter = draws.integer(0, 2)
    elif record.stratum == "loss":
        loss = draws.integer(500, 1800)
    elif record.stratum == "partition":
        start = draws.integer(60, 120)
        end = start + draws.integer(20, 50)
        events.extend(
            [
                (
                    'events { id: "partition-open" tick: %d '
                    "kind: TAPE_EVENT_KIND_SET_LINK_BLOCKED "
                    'target_id: "alpha|bravo" bool_value: true }'
                )
                % start,
                (
                    'events { id: "partition-close" tick: %d '
                    "kind: TAPE_EVENT_KIND_SET_LINK_BLOCKED "
                    'target_id: "alpha|bravo" bool_value: false }'
                )
                % end,
            ]
        )
    profile = "\n".join(
        [
            "network_profiles {",
            '  id: "mission"',
            f"  latency_ticks: {latency}",
            f"  jitter_ticks: {jitter}",
            f"  packet_loss_permyriad: {loss}",
            "  bandwidth_bytes_per_tick: 65536",
            "}",
        ]
    )
    return profile, events


def disruption(record: SeedRecord, draws: Draws) -> tuple[list[str], list[str]]:
    regions = []
    events = []
    if record.stratum == "blocked_route":
        x = draws.choose((5000, 6000, 7000))
        regions.append(region("choke", "REGION_KIND_CHOKEPOINT", (x, 0), (x + 1000, 7000)))
        close = draws.integer(80, 140)
        events.append(
            (
                'events { id: "close-route" tick: %d '
                "kind: TAPE_EVENT_KIND_SET_REGION_CLOSED "
                'target_id: "choke" bool_value: true }'
            )
            % close
        )
    elif record.stratum == "agent_loss":
        tick = draws.integer(100, 180)
        events.append(
            (
                'events { id: "disable-bravo" tick: %d '
                "kind: TAPE_EVENT_KIND_DISABLE_VEHICLE "
                'target_id: "bravo" }'
            )
            % tick
        )
    return regions, events


def generate_scenario(record: SeedRecord, policy: str) -> str:
    if policy not in POLICIES:
        raise ValueError("unknown allocation policy")
    draws = Draws(record.seed)
    profile, network_events = network_profile(record, draws)
    extra_regions, disruption_events = disruption(record, draws)
    starts = [(1000, 1000), (1000, 6000), (11000, 3500)]
    targets = [(11000, 1000), (11000, 6000), (6000, 3500)]
    capabilities = [
        "CAPABILITY_SEARCH",
        "CAPABILITY_INSPECTION",
        "CAPABILITY_DELIVERY",
    ]
    pieces = [
        "schema_version: 1",
        f'name: "{record.identifier}"',
        f"seed: {record.seed}",
        "step_ms: 100",
        "max_ticks: 4000",
        "world {",
        "  width_mm: 12000",
        "  height_mm: 7000",
        "  grid_cell_mm: 1000",
        "  map_version: 1",
        region("center", "REGION_KIND_OBSTACLE", (5000, 2000), (7000, 4000)),
        *extra_regions,
        '  locations { id: "base" kind: LOCATION_KIND_RETURN position { x_mm: 1000 y_mm: 3500 } radius_mm: 500 }',
        "}",
    ]
    for index, identifier in enumerate(("alpha", "bravo", "charlie")):
        pieces.append(vehicle(identifier, *starts[index], capabilities[index]))
    for index, capability in enumerate(capabilities):
        pieces.append(task(f"task-{index + 1:02d}", *targets[index], capability, 3 - index))
    pieces.extend(
        [
            profile,
            *network_events,
            *disruption_events,
            'network_profile: "mission"',
            f"allocation_policy: {POLICIES[policy]}",
            "failure_detection_ticks: 5",
            "",
        ]
    )
    return "\n".join(pieces)


def write_scenarios(records: list[SeedRecord], output: pathlib.Path, policies: tuple[str, ...]) -> list[pathlib.Path]:
    output.mkdir(parents=True, exist_ok=True)
    paths = []
    for record in records:
        for policy in policies:
            path = output / f"{record.identifier}.{policy}.textproto"
            path.write_text(generate_scenario(record, policy), encoding="utf-8", newline="\n")
            paths.append(path)
    return paths


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--seeds", type=pathlib.Path, required=True)
    parser.add_argument("--output", type=pathlib.Path, required=True)
    parser.add_argument("--start", type=int, default=0)
    parser.add_argument("--count", type=int)
    parser.add_argument("--policy", choices=tuple(POLICIES), action="append")
    arguments = parser.parse_args()
    records = load_records(arguments.seeds)
    if arguments.start < 0 or (arguments.count is not None and arguments.count <= 0):
        raise ValueError("invalid seed selection")
    selected = records[arguments.start :]
    if arguments.count is not None:
        selected = selected[: arguments.count]
    if not selected:
        raise ValueError("seed selection is empty")
    policies = tuple(arguments.policy or POLICIES)
    for path in write_scenarios(selected, arguments.output, policies):
        print(path)


if __name__ == "__main__":
    main()
