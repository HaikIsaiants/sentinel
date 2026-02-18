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
DEFAULT_CONFIG = {
    "namespace": "sentinel-scenario-v1",
    "world_cells": [16, 12],
    "vehicles": [3, 5],
    "tasks": [10, 16],
    "max_ticks": 6000,
    "strata": {
        "nominal": {"latency": [0, 0], "loss": [0, 0], "event": "none"},
        "latency": {"latency": [2, 6], "loss": [0, 0], "event": "none"},
        "loss": {"latency": [0, 2], "loss": [500, 2000], "event": "none"},
        "partition": {"latency": [0, 2], "loss": [0, 0], "event": "partition"},
        "blocked_route": {"latency": [0, 0], "loss": [0, 0], "event": "blockage"},
        "agent_loss": {"latency": [0, 2], "loss": [0, 500], "event": "agent_loss"},
    },
}
IDENTIFIER = re.compile(r"^[a-z0-9][a-z0-9-]{0,95}$")


@dataclass(frozen=True)
class SeedRecord:
    identifier: str
    seed: int
    stratum: str


class Draws:
    def __init__(self, namespace: str, seed: int):
        self.namespace = namespace
        self.seed = seed

    def raw(self, path: str, draw: int = 0) -> int:
        material = f"{self.namespace}|{self.seed}|{path}|{draw}".encode()
        return int.from_bytes(hashlib.sha256(material).digest()[:8], "big")

    def integer(self, path: str, limits: list[int], draw: int = 0) -> int:
        minimum, maximum = limits
        if minimum > maximum:
            raise ValueError(f"invalid draw range {path}")
        return minimum + self.raw(path, draw) % (maximum - minimum + 1)

    def order(self, path: str, values):
        return sorted(values, key=lambda value: (self.raw(f"{path}/{value}"), str(value)))


def read_config(path: pathlib.Path | None) -> dict:
    value = DEFAULT_CONFIG if path is None else json.loads(path.read_text(encoding="utf-8"))
    required = {"namespace", "world_cells", "vehicles", "tasks", "max_ticks", "strata"}
    if set(value) != required or not value["namespace"] or value["max_ticks"] <= 0:
        raise ValueError("invalid scenario configuration")
    if len(value["world_cells"]) != 2 or any(item <= 4 for item in value["world_cells"]):
        raise ValueError("invalid world dimensions")
    for name in ("vehicles", "tasks"):
        limits = value[name]
        if len(limits) != 2 or limits[0] <= 0 or limits[0] > limits[1]:
            raise ValueError(f"invalid {name} limits")
    if not value["strata"]:
        raise ValueError("scenario strata are empty")
    return json.loads(json.dumps(value))


def load_records(path: pathlib.Path, config: dict) -> list[SeedRecord]:
    records = []
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
        records.append(record)
    if not records:
        raise ValueError("seed corpus is empty")
    return records


def block(name: str, fields: list[str], indent: int = 0) -> list[str]:
    prefix = " " * indent
    return [f"{prefix}{name} {{", *(f"{prefix}  {field}" for field in fields), f"{prefix}}}"]


def point(x: int, y: int, cell: int = 1000) -> str:
    return f"x_mm: {x * cell} y_mm: {y * cell}"


def generate_scenario(record: SeedRecord, policy: str, config: dict) -> str:
    if policy not in POLICIES or record.stratum not in config["strata"]:
        raise ValueError("invalid scenario identity")
    draws = Draws(config["namespace"], record.seed)
    width, height = config["world_cells"]
    vehicle_count = draws.integer("mission/vehicles", config["vehicles"])
    task_count = draws.integer("mission/tasks", config["tasks"])
    cells = [(x, y) for x in range(2, width - 2) for y in range(1, height - 1)]
    ordered = draws.order("world/cells", cells)
    obstacles = ordered[: draws.integer("world/obstacles", [2, 5])]
    targets = [cell for cell in ordered if cell not in obstacles][:task_count]
    capabilities = ("SEARCH", "INSPECTION", "DELIVERY")
    lines = [
        "schema_version: 1",
        f'name: "{record.identifier}"',
        f"seed: {record.seed}",
        "step_ms: 100",
        f"max_ticks: {config['max_ticks']}",
        "world {",
        f"  width_mm: {width * 1000}",
        f"  height_mm: {height * 1000}",
        "  grid_cell_mm: 1000",
        "  map_version: 1",
    ]
    for index, (x, y) in enumerate(obstacles):
        lines.extend(
            block(
                "regions",
                [
                    f'id: "obstacle-{index:02d}"',
                    "kind: REGION_KIND_OBSTACLE",
                    f"minimum {{ {point(x, y)} }}",
                    f"maximum {{ {point(x, y)} }}",
                    "energy_multiplier_permille: 1000",
                ],
                2,
            )
        )
    lines.extend(block("locations", ['id: "base"', "kind: LOCATION_KIND_RETURN", f"position {{ {point(1, 1)} }}", "radius_mm: 500"], 2))
    lines.extend(block("locations", ['id: "charger"', "kind: LOCATION_KIND_CHARGING", f"position {{ {point(width - 1, 1)} }}", "radius_mm: 500", "charge_mj_per_tick: 10000"], 2))
    lines.append("}")
    for index in range(vehicle_count):
        capability = capabilities[index % len(capabilities)]
        lines.extend(
            block(
                "vehicles",
                [
                    f'id: "agent-{index:02d}"',
                    f'kind: "{"uav" if index == vehicle_count - 1 else "ugv"}"',
                    f"initial_position {{ {point(1, 1 + index)} }}",
                    f"max_speed_mm_s: {draws.integer(f'vehicles/{index}/speed', [1400, 2600])}",
                    "initial_energy_mj: 800000",
                    "energy_cost_mj_per_meter: 1000",
                    "payload_grams: 8000",
                    f"capabilities: CAPABILITY_{capability}",
                    'terrain_access: "floor"',
                    'return_location_id: "base"',
                ],
            )
        )
    events = []
    for index, (x, y) in enumerate(targets):
        capability = capabilities[index % len(capabilities)]
        release = draws.integer(f"tasks/{index}/release", [0, 50])
        lines.extend(
            block(
                "tasks",
                [
                    f'id: "task-{index:03d}"',
                    f'kind: "{capability.lower()}"',
                    f"target {{ {point(x, y)} }}",
                    f"required_capability: CAPABILITY_{capability}",
                    "deadline_tick: 5000",
                    "completion_radius_mm: 500",
                    f"released: {str(release == 0).lower()}",
                    "service_ticks: 5",
                    "service_energy_mj_per_tick: 100",
                    f"priority: {draws.integer(f'tasks/{index}/priority', [1, 5])}",
                ],
            )
        )
        if release:
            events.append((release, f"release-{index:03d}", "TAPE_EVENT_KIND_RELEASE_TASK", f"task-{index:03d}", None))
    profile = config["strata"][record.stratum]
    latency = draws.integer("network/latency", profile["latency"])
    loss = draws.integer("network/loss", profile["loss"])
    lines.extend(
        block(
            "network_profiles",
            [
                'id: "mission"',
                f"latency_ticks: {latency}",
                f"jitter_ticks: {max(0, latency // 2)}",
                f"packet_loss_permyriad: {loss}",
                f"bandwidth_bytes_per_tick: {draws.integer('network/bandwidth', [20000, 80000])}",
                f"reorder_permyriad: {draws.integer('network/reorder', [0, 1000]) if latency or loss else 0}",
                "reorder_window_ticks: 3",
            ],
        )
    )
    event_profile = profile["event"]
    if event_profile == "partition":
        start = draws.integer("events/partition/start", [80, 180])
        duration = draws.integer("events/partition/duration", [20, 70])
        events.extend(
            [
                (start, "partition-open", "TAPE_EVENT_KIND_SET_LINK_BLOCKED", "agent-00|agent-01", True),
                (start + duration, "partition-close", "TAPE_EVENT_KIND_SET_LINK_BLOCKED", "agent-00|agent-01", False),
            ]
        )
    elif event_profile == "blockage":
        events.append((draws.integer("events/blockage/tick", [100, 240]), "close-route", "TAPE_EVENT_KIND_SET_REGION_CLOSED", "obstacle-00", True))
    elif event_profile == "agent_loss":
        events.append((draws.integer("events/loss/tick", [120, 260]), "disable-agent", "TAPE_EVENT_KIND_DISABLE_VEHICLE", "agent-01", None))
    for tick, identifier, kind, target, boolean in sorted(events):
        fields = [f'id: "{identifier}"', f"tick: {tick}", f"kind: {kind}", f'target_id: "{target}"']
        if boolean is not None:
            fields.append(f"bool_value: {str(boolean).lower()}")
        lines.extend(block("events", fields))
    lines.extend(
        [
            'network_profile: "mission"',
            f"allocation_policy: {POLICIES[policy]}",
            "failure_detection_ticks: 5",
            "",
        ]
    )
    return "\n".join(lines)


def paired_digest(record: SeedRecord, config: dict) -> str:
    normalized = [
        generate_scenario(record, policy, config).replace(POLICIES[policy], "ALLOCATION_POLICY_PAIRED")
        for policy in POLICIES
    ]
    if len(set(normalized)) != 1:
        raise ValueError("allocator changed generated mission")
    return hashlib.sha256(normalized[0].encode()).hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--seeds", type=pathlib.Path, required=True)
    parser.add_argument("--config", type=pathlib.Path)
    parser.add_argument("--output", type=pathlib.Path, required=True)
    parser.add_argument("--start", type=int, default=0)
    parser.add_argument("--count", type=int, default=1)
    arguments = parser.parse_args()
    config = read_config(arguments.config)
    selected = load_records(arguments.seeds, config)[arguments.start : arguments.start + arguments.count]
    if arguments.start < 0 or arguments.count <= 0 or not selected:
        raise ValueError("invalid scenario selection")
    arguments.output.mkdir(parents=True, exist_ok=True)
    for record in selected:
        for policy in POLICIES:
            path = arguments.output / f"{record.identifier}.{policy}.textproto"
            path.write_text(generate_scenario(record, policy, config), encoding="utf-8", newline="\n")
        print(record.identifier, paired_digest(record, config))


if __name__ == "__main__":
    main()
