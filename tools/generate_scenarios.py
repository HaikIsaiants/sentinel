from __future__ import annotations

import argparse
import json
import pathlib
import random
import re
from dataclasses import dataclass


STRATA = {"nominal", "latency", "blocked_route", "agent_loss"}
IDENTIFIER = re.compile(r"^[a-z0-9][a-z0-9-]{0,63}$")


@dataclass(frozen=True)
class SeedRecord:
    identifier: str
    seed: int
    stratum: str


def load_records(path: pathlib.Path) -> list[SeedRecord]:
    records = []
    identifiers = set()
    seeds = set()
    for number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if not line.strip():
            continue
        value = json.loads(line)
        record = SeedRecord(str(value["id"]), int(value["seed"]), str(value["stratum"]))
        if (
            not IDENTIFIER.fullmatch(record.identifier)
            or record.stratum not in STRATA
            or not 0 <= record.seed <= 0x7FFFFFFFFFFFFFFF
        ):
            raise ValueError(f"invalid seed record on line {number}")
        if record.identifier in identifiers or record.seed in seeds:
            raise ValueError(f"duplicate seed record on line {number}")
        identifiers.add(record.identifier)
        seeds.add(record.seed)
        records.append(record)
    if not records:
        raise ValueError("seed corpus is empty")
    return records


def block(name: str, fields: list[str], indent: int = 0) -> list[str]:
    prefix = " " * indent
    return [f"{prefix}{name} {{", *(f"{prefix}  {field}" for field in fields), f"{prefix}}}"]


def point(x: int, y: int) -> str:
    return f"x_mm: {x} y_mm: {y}"


def network(record: SeedRecord, draws: random.Random) -> list[str]:
    latency = 0
    if record.stratum == "latency":
        latency = draws.randint(2, 6)
    return block(
        "network_profiles",
        [
            'id: "mission"',
            f"latency_ticks: {latency}",
        ],
    )


def disruptions(record: SeedRecord, draws: random.Random) -> list[str]:
    events = []
    if record.stratum == "blocked_route":
        events.extend(
            block(
                "events",
                [
                    'id: "close-route"',
                    f"tick: {draws.randint(100, 200)}",
                    "kind: TAPE_EVENT_KIND_SET_REGION_CLOSED",
                    'target_id: "center"',
                    "bool_value: true",
                ],
            )
        )
    elif record.stratum == "agent_loss":
        events.extend(
            block(
                "events",
                [
                    'id: "disable-bravo"',
                    f"tick: {draws.randint(120, 220)}",
                    "kind: TAPE_EVENT_KIND_DISABLE_VEHICLE",
                    'target_id: "bravo"',
                ],
            )
        )
    return events


def vehicle(identifier: str, x: int, y: int, capability: str) -> list[str]:
    return block(
        "vehicles",
        [
            f'id: "{identifier}"',
            'kind: "ugv"',
            f"initial_position {{ {point(x, y)} }}",
            "max_speed_mm_s: 2000",
            "initial_energy_mj: 700000",
            "energy_cost_mj_per_meter: 1000",
            "payload_grams: 6000",
            f"capabilities: {capability}",
            'terrain_access: "floor"',
            'return_location_id: "base"',
        ],
    )


def task(
    identifier: str,
    agent: str,
    x: int,
    y: int,
    capability: str,
    priority: int,
) -> list[str]:
    return block(
        "tasks",
        [
            f'id: "{identifier}"',
            f'kind: "{capability.lower().removeprefix("capability_")}"',
            f"target {{ {point(x, y)} }}",
            f"required_capability: {capability}",
            "deadline_tick: 4000",
            "completion_radius_mm: 500",
            f'assigned_agent_id: "{agent}"',
            "released: true",
            "service_ticks: 5",
            "service_energy_mj_per_tick: 100",
            f"priority: {priority}",
        ],
    )


def generate_scenario(record: SeedRecord) -> str:
    draws = random.Random(record.seed)
    lines = [
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
        *block(
            "regions",
            [
                'id: "center"',
                "kind: REGION_KIND_OBSTACLE",
                f"minimum {{ {point(5000, 2000)} }}",
                f"maximum {{ {point(7000, 4000)} }}",
                "energy_multiplier_permille: 1000",
            ],
            2,
        ),
        *block(
            "locations",
            [
                'id: "base"',
                "kind: LOCATION_KIND_RETURN",
                f"position {{ {point(1000, 3500)} }}",
                "radius_mm: 500",
            ],
            2,
        ),
        "}",
        *vehicle("alpha", 1000, 1000, "CAPABILITY_SEARCH"),
        *vehicle("bravo", 1000, 6000, "CAPABILITY_INSPECTION"),
        *vehicle("charlie", 11000, 3500, "CAPABILITY_DELIVERY"),
        *task("task-01", "alpha", 11000, 1000, "CAPABILITY_SEARCH", 3),
        *task("task-02", "bravo", 11000, 6000, "CAPABILITY_INSPECTION", 2),
        *task("task-03", "charlie", 6000, 3500, "CAPABILITY_DELIVERY", 1),
        *network(record, draws),
        *disruptions(record, draws),
        'network_profile: "mission"',
        "allocation_policy: ALLOCATION_POLICY_SCRIPTED",
        "",
    ]
    return "\n".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--seeds", type=pathlib.Path, required=True)
    parser.add_argument("--output", type=pathlib.Path, required=True)
    parser.add_argument("--start", type=int, default=0)
    parser.add_argument("--count", type=int, default=1)
    arguments = parser.parse_args()
    records = load_records(arguments.seeds)
    selected = records[arguments.start : arguments.start + arguments.count]
    if arguments.start < 0 or arguments.count <= 0 or not selected:
        raise ValueError("invalid scenario selection")
    arguments.output.mkdir(parents=True, exist_ok=True)
    for record in selected:
        path = arguments.output / f"{record.identifier}.textproto"
        path.write_text(generate_scenario(record), encoding="utf-8", newline="\n")
        print(path)


if __name__ == "__main__":
    main()
