from __future__ import annotations

import argparse
import hashlib
import json
import pathlib
import re
from dataclasses import dataclass


POLICIES = {
    "nearest": "ALLOCATION_POLICY_NEAREST_CAPABLE",
    "cbba": "ALLOCATION_POLICY_SENTINEL_CBBA",
}
IDENTIFIER = re.compile(r"^[a-z0-9][a-z0-9-]{0,79}$")
DEFAULT_CONFIG = {
    "world": {"width_cells": 16, "height_cells": 12, "cell_mm": 1000},
    "mission": {"vehicles": [3, 5], "tasks": [8, 14], "max_ticks": 6000},
    "strata": {
        "nominal": {"network": "nominal", "event": "none"},
        "latency": {"network": "latency", "event": "none"},
        "loss": {"network": "loss", "event": "none"},
        "partition": {"network": "nominal", "event": "partition"},
        "blocked_route": {"network": "nominal", "event": "blockage"},
        "agent_loss": {"network": "nominal", "event": "agent_loss"},
    },
}


@dataclass(frozen=True)
class SeedRecord:
    identifier: str
    seed: int
    stratum: str


class Draws:
    """Independent named draws keep unrelated scenario choices stable."""

    def __init__(self, seed: int, namespace: str = "sentinel-scenario-v1"):
        self.seed = seed
        self.namespace = namespace

    def value(self, path: str, draw: int = 0, attempt: int = 0) -> int:
        material = f"{self.namespace}|{self.seed}|{path}|{draw}|{attempt}".encode()
        return int.from_bytes(hashlib.sha256(material).digest()[:8], "big")

    def integer(self, path: str, minimum: int, maximum: int, draw: int = 0) -> int:
        if minimum > maximum:
            raise ValueError(f"invalid range for {path}")
        return minimum + self.value(path, draw) % (maximum - minimum + 1)

    def choose(self, path: str, values, draw: int = 0):
        if not values:
            raise ValueError(f"empty choices for {path}")
        return values[self.value(path, draw) % len(values)]

    def order(self, path: str, values):
        return sorted(values, key=lambda value: (self.value(f"{path}/{value}"), str(value)))


def read_config(path: pathlib.Path | None) -> dict:
    config = DEFAULT_CONFIG if path is None else json.loads(path.read_text(encoding="utf-8"))
    required = {"world", "mission", "strata"}
    if set(config) != required:
        raise ValueError("scenario config has unexpected sections")
    world = config["world"]
    mission = config["mission"]
    if world["width_cells"] <= 4 or world["height_cells"] <= 4 or world["cell_mm"] <= 0:
        raise ValueError("invalid world dimensions")
    for name in ("vehicles", "tasks"):
        values = mission[name]
        if len(values) != 2 or values[0] <= 0 or values[0] > values[1]:
            raise ValueError(f"invalid mission {name} range")
    if mission["max_ticks"] <= 0 or not config["strata"]:
        raise ValueError("invalid mission configuration")
    return json.loads(json.dumps(config))


def load_records(path: pathlib.Path, config: dict) -> list[SeedRecord]:
    result = []
    identities = set()
    seeds = set()
    for number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if not line.strip():
            continue
        value = json.loads(line)
        record = SeedRecord(str(value["id"]), int(value["seed"]), str(value["stratum"]))
        if (
            not IDENTIFIER.fullmatch(record.identifier)
            or record.stratum not in config["strata"]
            or not 0 <= record.seed <= 0x7FFFFFFFFFFFFFFF
        ):
            raise ValueError(f"invalid seed record on line {number}")
        if record.identifier in identities or record.seed in seeds:
            raise ValueError(f"duplicate seed record on line {number}")
        identities.add(record.identifier)
        seeds.add(record.seed)
        result.append(record)
    if not result:
        raise ValueError("seed corpus is empty")
    return result


def emit_block(name: str, fields: list[str], indent: int = 0) -> list[str]:
    prefix = " " * indent
    lines = [f"{prefix}{name} {{"]
    lines.extend(f"{prefix}  {field}" for field in fields)
    lines.append(f"{prefix}}}")
    return lines


def point(x: int, y: int) -> str:
    return f"x_mm: {x} y_mm: {y}"


def generated_world(draws: Draws, config: dict) -> list[str]:
    world = config["world"]
    cell = world["cell_mm"]
    width = world["width_cells"] * cell
    height = world["height_cells"] * cell
    lines = [
        f"width_mm: {width}",
        f"height_mm: {height}",
        f"grid_cell_mm: {cell}",
        "map_version: 1",
    ]
    obstacle_count = draws.integer("world/obstacle-count", 2, 4)
    available = [
        (x, y)
        for x in range(3, world["width_cells"] - 3)
        for y in range(2, world["height_cells"] - 2)
    ]
    for index, (x, y) in enumerate(draws.order("world/obstacles", available)[:obstacle_count]):
        lines.extend(
            emit_block(
                "regions",
                [
                    f'id: "obstacle-{index:02d}"',
                    "kind: REGION_KIND_OBSTACLE",
                    f"minimum {{ {point(x * cell, y * cell)} }}",
                    f"maximum {{ {point(x * cell, y * cell)} }}",
                    "energy_multiplier_permille: 1000",
                ],
            )
        )
    lines.extend(
        emit_block(
            "locations",
            [
                'id: "base"',
                "kind: LOCATION_KIND_RETURN",
                f"position {{ {point(cell, height // 2)} }}",
                f"radius_mm: {cell // 2}",
            ],
        )
    )
    lines.extend(
        emit_block(
            "locations",
            [
                'id: "charger"',
                "kind: LOCATION_KIND_CHARGING",
                f"position {{ {point(width - cell, height // 2)} }}",
                f"radius_mm: {cell // 2}",
                "charge_mj_per_tick: 10000",
            ],
        )
    )
    return emit_block("world", lines)


def generated_vehicles(draws: Draws, config: dict) -> list[str]:
    count = draws.integer("mission/vehicle-count", *config["mission"]["vehicles"])
    kinds = ("ugv", "ugv", "uav")
    capabilities = (
        "CAPABILITY_SEARCH",
        "CAPABILITY_INSPECTION",
        "CAPABILITY_DELIVERY",
        "CAPABILITY_RELAY",
    )
    lines = []
    for index in range(count):
        capability = capabilities[index % len(capabilities)]
        lines.extend(
            emit_block(
                "vehicles",
                [
                    f'id: "agent-{index:02d}"',
                    f'kind: "{kinds[index % len(kinds)]}"',
                    f"initial_position {{ {point(1000, 1000 + index * 1000)} }}",
                    f"max_speed_mm_s: {draws.integer(f'vehicles/{index}/speed', 1200, 2400)}",
                    "initial_energy_mj: 800000",
                    "energy_cost_mj_per_meter: 1000",
                    "payload_grams: 8000",
                    f"capabilities: {capability}",
                    'terrain_access: "floor"',
                    'return_location_id: "base"',
                ],
            )
        )
    return lines


def generated_tasks(draws: Draws, config: dict) -> list[str]:
    count = draws.integer("mission/task-count", *config["mission"]["tasks"])
    world = config["world"]
    cell = world["cell_mm"]
    capabilities = (
        "CAPABILITY_SEARCH",
        "CAPABILITY_INSPECTION",
        "CAPABILITY_DELIVERY",
    )
    lines = []
    for index in range(count):
        x = draws.integer(f"tasks/{index}/x", 2, world["width_cells"] - 2) * cell
        y = draws.integer(f"tasks/{index}/y", 1, world["height_cells"] - 2) * cell
        capability = capabilities[index % len(capabilities)]
        release = draws.integer(f"tasks/{index}/release", 0, 30)
        lines.extend(
            emit_block(
                "tasks",
                [
                    f'id: "task-{index:03d}"',
                    f'kind: "{capability.lower().removeprefix("capability_")}"',
                    f"target {{ {point(x, y)} }}",
                    f"required_capability: {capability}",
                    "deadline_tick: 5000",
                    f"completion_radius_mm: {cell // 2}",
                    f"released: {'true' if release == 0 else 'false'}",
                    "service_ticks: 5",
                    "service_energy_mj_per_tick: 100",
                    f"priority: {draws.integer(f'tasks/{index}/priority', 1, 5)}",
                ],
            )
        )
        if release:
            lines.extend(
                emit_block(
                    "events",
                    [
                        f'id: "release-{index:03d}"',
                        f"tick: {release}",
                        "kind: TAPE_EVENT_KIND_RELEASE_TASK",
                        f'target_id: "task-{index:03d}"',
                    ],
                )
            )
    return lines


def generated_disruption(record: SeedRecord, draws: Draws, config: dict) -> list[str]:
    source = config["strata"][record.stratum]
    lines = []
    if source["event"] == "partition":
        start = draws.integer("events/partition/start", 80, 160)
        duration = draws.integer("events/partition/duration", 20, 60)
        for suffix, tick, blocked in (("open", start, True), ("close", start + duration, False)):
            lines.extend(
                emit_block(
                    "events",
                    [
                        f'id: "partition-{suffix}"',
                        f"tick: {tick}",
                        "kind: TAPE_EVENT_KIND_SET_LINK_BLOCKED",
                        'target_id: "agent-00|agent-01"',
                        f"bool_value: {str(blocked).lower()}",
                    ],
                )
            )
    elif source["event"] == "blockage":
        lines.extend(
            emit_block(
                "events",
                [
                    'id: "block-route"',
                    f"tick: {draws.integer('events/blockage/tick', 100, 200)}",
                    "kind: TAPE_EVENT_KIND_SET_REGION_CLOSED",
                    'target_id: "obstacle-00"',
                    "bool_value: true",
                ],
            )
        )
    elif source["event"] == "agent_loss":
        lines.extend(
            emit_block(
                "events",
                [
                    'id: "disable-agent"',
                    f"tick: {draws.integer('events/loss/tick', 120, 240)}",
                    "kind: TAPE_EVENT_KIND_DISABLE_VEHICLE",
                    'target_id: "agent-01"',
                ],
            )
        )
    return lines


def generated_network(record: SeedRecord, draws: Draws, config: dict) -> list[str]:
    profile = config["strata"][record.stratum]["network"]
    latency = draws.integer("network/latency", 2, 5) if profile == "latency" else 0
    jitter = draws.integer("network/jitter", 0, 2) if profile == "latency" else 0
    loss = draws.integer("network/loss", 500, 1800) if profile == "loss" else 0
    return emit_block(
        "network_profiles",
        [
            'id: "mission"',
            f"latency_ticks: {latency}",
            f"jitter_ticks: {jitter}",
            f"packet_loss_permyriad: {loss}",
            "bandwidth_bytes_per_tick: 65536",
            "reorder_permyriad: 0",
            "reorder_window_ticks: 0",
        ],
    )


def generate_scenario(record: SeedRecord, policy: str, config: dict) -> str:
    if policy not in POLICIES or record.stratum not in config["strata"]:
        raise ValueError("invalid scenario identity")
    draws = Draws(record.seed)
    lines = [
        "schema_version: 1",
        f'name: "{record.identifier}"',
        f"seed: {record.seed}",
        "step_ms: 100",
        f"max_ticks: {config['mission']['max_ticks']}",
        *generated_world(draws, config),
        *generated_vehicles(draws, config),
        *generated_tasks(draws, config),
        *generated_network(record, draws, config),
        *generated_disruption(record, draws, config),
        'network_profile: "mission"',
        f"allocation_policy: {POLICIES[policy]}",
        "failure_detection_ticks: 5",
        "",
    ]
    return "\n".join(lines)


def paired_digest(record: SeedRecord, config: dict) -> str:
    values = []
    for policy in POLICIES:
        text = generate_scenario(record, policy, config)
        values.append(text.replace(POLICIES[policy], "ALLOCATION_POLICY_PAIRED"))
    if len(set(values)) != 1:
        raise ValueError("paired scenarios differ outside allocator policy")
    return hashlib.sha256(values[0].encode()).hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--seeds", type=pathlib.Path, required=True)
    parser.add_argument("--config", type=pathlib.Path)
    parser.add_argument("--output", type=pathlib.Path, required=True)
    parser.add_argument("--start", type=int, default=0)
    parser.add_argument("--count", type=int, default=1)
    arguments = parser.parse_args()
    config = read_config(arguments.config)
    records = load_records(arguments.seeds, config)
    if arguments.start < 0 or arguments.count <= 0:
        raise ValueError("invalid selection")
    selected = records[arguments.start : arguments.start + arguments.count]
    if not selected:
        raise ValueError("selection is empty")
    arguments.output.mkdir(parents=True, exist_ok=True)
    manifest = []
    for record in selected:
        digest = paired_digest(record, config)
        for policy in POLICIES:
            path = arguments.output / f"{record.identifier}.{policy}.textproto"
            path.write_text(generate_scenario(record, policy, config), encoding="utf-8", newline="\n")
        manifest.append({"id": record.identifier, "seed": record.seed, "stratum": record.stratum, "paired_sha256": digest})
    (arguments.output / "manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
        newline="\n",
    )


if __name__ == "__main__":
    main()
