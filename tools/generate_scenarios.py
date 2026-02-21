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
IDENTIFIER = re.compile(r"^[a-z0-9][a-z0-9-]{0,95}$")
DEFAULT_CONFIG = {
    "namespace": "sentinel-scenario-v1",
    "step_ms": 100,
    "max_ticks": 6000,
    "world_cells": [16, 12],
    "vehicle_count": [3, 5],
    "task_count": [10, 16],
    "strata": {
        "nominal": ["nominal", "none"],
        "latency": ["latency", "none"],
        "loss": ["loss", "none"],
        "partition": ["nominal", "partition"],
        "blocked_route": ["nominal", "blockage"],
        "agent_loss": ["nominal", "agent_loss"],
        "compound": ["loss", "agent_loss"],
    },
}


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
        value = f"{self.namespace}|{self.seed}|{path}|{draw}".encode()
        return int.from_bytes(hashlib.sha256(value).digest()[:8], "big")

    def integer(self, path: str, limits: list[int] | tuple[int, int], draw: int = 0) -> int:
        minimum, maximum = map(int, limits)
        if minimum > maximum:
            raise ValueError(f"invalid range at {path}")
        return minimum + self.raw(path, draw) % (maximum - minimum + 1)

    def ordered(self, path: str, values):
        return sorted(values, key=lambda item: (self.raw(f"{path}/{item}"), str(item)))


def read_config(path: pathlib.Path | None) -> dict:
    config = DEFAULT_CONFIG if path is None else json.loads(path.read_text(encoding="utf-8"))
    required = {"namespace", "step_ms", "max_ticks", "world_cells", "vehicle_count", "task_count", "strata"}
    if set(config) != required or not config["namespace"]:
        raise ValueError("invalid generator configuration")
    if len(config["world_cells"]) != 2 or any(value <= 4 for value in config["world_cells"]):
        raise ValueError("invalid world_cells")
    for name in ("vehicle_count", "task_count"):
        values = config[name]
        if len(values) != 2 or values[0] <= 0 or values[0] > values[1]:
            raise ValueError(f"invalid {name}")
    if config["step_ms"] <= 0 or config["max_ticks"] <= 0 or not config["strata"]:
        raise ValueError("invalid timing or strata")
    return json.loads(json.dumps(config))


def load_records(path: pathlib.Path, config: dict) -> list[SeedRecord]:
    rows = []
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
            raise ValueError(f"invalid seed record at line {number}")
        if record.identifier in identities or record.seed in seeds:
            raise ValueError(f"duplicate seed record at line {number}")
        identities.add(record.identifier)
        seeds.add(record.seed)
        rows.append(record)
    if not rows:
        raise ValueError("seed corpus is empty")
    return rows


def build_model(record: SeedRecord, config: dict) -> dict:
    if record.stratum not in config["strata"]:
        raise ValueError("unknown stratum")
    draws = Draws(config["namespace"], record.seed)
    width_cells, height_cells = config["world_cells"]
    vehicle_count = draws.integer("mission/vehicle-count", config["vehicle_count"])
    task_count = draws.integer("mission/task-count", config["task_count"])
    candidates = [
        (x, y)
        for x in range(2, width_cells - 2)
        for y in range(1, height_cells - 1)
    ]
    occupied = draws.ordered("world/occupied", candidates)
    obstacle_count = draws.integer("world/obstacle-count", [2, 5])
    obstacle_cells = occupied[:obstacle_count]
    target_cells = occupied[obstacle_count : obstacle_count + task_count]
    capabilities = ["SEARCH", "INSPECTION", "DELIVERY"]
    vehicles = []
    for index in range(vehicle_count):
        vehicles.append(
            {
                "id": f"agent-{index:02d}",
                "kind": "uav" if index == vehicle_count - 1 else "ugv",
                "x": 1,
                "y": 1 + index,
                "speed": draws.integer(f"vehicles/{index}/speed", [1400, 2600]),
                "energy": draws.integer(f"vehicles/{index}/energy", [650000, 950000]),
                "capability": capabilities[index % len(capabilities)],
            }
        )
    tasks = []
    events = []
    for index, (x, y) in enumerate(target_cells):
        release = draws.integer(f"tasks/{index}/release", [0, 60])
        tasks.append(
            {
                "id": f"task-{index:03d}",
                "x": x,
                "y": y,
                "capability": capabilities[index % len(capabilities)],
                "priority": draws.integer(f"tasks/{index}/priority", [1, 5]),
                "released": release == 0,
            }
        )
        if release:
            events.append(
                {
                    "id": f"release-{index:03d}",
                    "tick": release,
                    "kind": "TAPE_EVENT_KIND_RELEASE_TASK",
                    "target": f"task-{index:03d}",
                }
            )
    network, event_profile = config["strata"][record.stratum]
    if event_profile == "partition":
        start = draws.integer("events/partition/start", [80, 180])
        duration = draws.integer("events/partition/duration", [20, 70])
        events.extend(
            [
                {
                    "id": "partition-open",
                    "tick": start,
                    "kind": "TAPE_EVENT_KIND_SET_LINK_BLOCKED",
                    "target": "agent-00|agent-01",
                    "bool": True,
                },
                {
                    "id": "partition-close",
                    "tick": start + duration,
                    "kind": "TAPE_EVENT_KIND_SET_LINK_BLOCKED",
                    "target": "agent-00|agent-01",
                    "bool": False,
                },
            ]
        )
    elif event_profile == "blockage":
        events.append(
            {
                "id": "close-obstacle",
                "tick": draws.integer("events/blockage/tick", [100, 240]),
                "kind": "TAPE_EVENT_KIND_SET_REGION_CLOSED",
                "target": "obstacle-00",
                "bool": True,
            }
        )
    elif event_profile == "agent_loss":
        events.append(
            {
                "id": "disable-agent",
                "tick": draws.integer("events/agent-loss/tick", [120, 260]),
                "kind": "TAPE_EVENT_KIND_DISABLE_VEHICLE",
                "target": "agent-01",
            }
        )
    profile = {
        "latency": draws.integer("network/latency", [2, 6]) if network == "latency" else 0,
        "jitter": draws.integer("network/jitter", [0, 3]) if network == "latency" else 0,
        "loss": draws.integer("network/loss", [500, 2000]) if network == "loss" else 0,
        "bandwidth": draws.integer("network/bandwidth", [20000, 80000]),
        "reorder": draws.integer("network/reorder", [0, 1000]) if network != "nominal" else 0,
    }
    model = {
        "id": record.identifier,
        "seed": record.seed,
        "stratum": record.stratum,
        "width_cells": width_cells,
        "height_cells": height_cells,
        "cell_mm": 1000,
        "max_ticks": config["max_ticks"],
        "step_ms": config["step_ms"],
        "obstacles": obstacle_cells,
        "vehicles": vehicles,
        "tasks": tasks,
        "events": sorted(events, key=lambda event: (event["tick"], event["id"])),
        "network": profile,
    }
    validate_model(model)
    return model


def validate_model(model: dict) -> None:
    if len(model["vehicles"]) < 3 or len(model["tasks"]) < 1:
        raise ValueError("generated mission is too small")
    identifiers = [item["id"] for item in model["vehicles"] + model["tasks"] + model["events"]]
    if len(identifiers) != len(set(identifiers)):
        raise ValueError("generated identifiers are not unique")
    capabilities = {item["capability"] for item in model["vehicles"]}
    if any(task["capability"] not in capabilities for task in model["tasks"]):
        raise ValueError("generated task has no capable vehicle")
    if any(event["tick"] >= model["max_ticks"] for event in model["events"]):
        raise ValueError("generated event lies outside horizon")
    blocked = set(map(tuple, model["obstacles"]))
    if any((task["x"], task["y"]) in blocked for task in model["tasks"]):
        raise ValueError("generated task lies inside obstacle")


def block(name: str, fields: list[str], indent: int = 0) -> list[str]:
    prefix = " " * indent
    return [f"{prefix}{name} {{", *(f"{prefix}  {field}" for field in fields), f"{prefix}}}"]


def point(x: int, y: int, cell: int) -> str:
    return f"x_mm: {x * cell} y_mm: {y * cell}"


def render(model: dict, policy: str) -> str:
    if policy not in POLICIES:
        raise ValueError("unknown policy")
    cell = model["cell_mm"]
    lines = [
        "schema_version: 1",
        f'name: "{model["id"]}"',
        f"seed: {model['seed']}",
        f"step_ms: {model['step_ms']}",
        f"max_ticks: {model['max_ticks']}",
        "world {",
        f"  width_mm: {model['width_cells'] * cell}",
        f"  height_mm: {model['height_cells'] * cell}",
        f"  grid_cell_mm: {cell}",
        "  map_version: 1",
    ]
    for index, (x, y) in enumerate(model["obstacles"]):
        lines.extend(
            block(
                "regions",
                [
                    f'id: "obstacle-{index:02d}"',
                    "kind: REGION_KIND_OBSTACLE",
                    f"minimum {{ {point(x, y, cell)} }}",
                    f"maximum {{ {point(x, y, cell)} }}",
                    "energy_multiplier_permille: 1000",
                ],
                2,
            )
        )
    lines.extend(block("locations", ['id: "base"', "kind: LOCATION_KIND_RETURN", f"position {{ {point(1, 1, cell)} }}", "radius_mm: 500"], 2))
    lines.extend(block("locations", ['id: "charger"', "kind: LOCATION_KIND_CHARGING", f"position {{ {point(model['width_cells'] - 1, 1, cell)} }}", "radius_mm: 500", "charge_mj_per_tick: 10000"], 2))
    lines.append("}")
    for vehicle in model["vehicles"]:
        lines.extend(
            block(
                "vehicles",
                [
                    f'id: "{vehicle["id"]}"',
                    f'kind: "{vehicle["kind"]}"',
                    f"initial_position {{ {point(vehicle['x'], vehicle['y'], cell)} }}",
                    f"max_speed_mm_s: {vehicle['speed']}",
                    f"initial_energy_mj: {vehicle['energy']}",
                    "energy_cost_mj_per_meter: 1000",
                    "payload_grams: 8000",
                    f"capabilities: CAPABILITY_{vehicle['capability']}",
                    'terrain_access: "floor"',
                    'return_location_id: "base"',
                ],
            )
        )
    for task in model["tasks"]:
        lines.extend(
            block(
                "tasks",
                [
                    f'id: "{task["id"]}"',
                    f'kind: "{task["capability"].lower()}"',
                    f"target {{ {point(task['x'], task['y'], cell)} }}",
                    f"required_capability: CAPABILITY_{task['capability']}",
                    "deadline_tick: 5000",
                    "completion_radius_mm: 500",
                    f"released: {str(task['released']).lower()}",
                    "service_ticks: 5",
                    "service_energy_mj_per_tick: 100",
                    f"priority: {task['priority']}",
                ],
            )
        )
    profile = model["network"]
    lines.extend(
        block(
            "network_profiles",
            [
                'id: "mission"',
                f"latency_ticks: {profile['latency']}",
                f"jitter_ticks: {profile['jitter']}",
                f"packet_loss_permyriad: {profile['loss']}",
                f"bandwidth_bytes_per_tick: {profile['bandwidth']}",
                f"reorder_permyriad: {profile['reorder']}",
                "reorder_window_ticks: 3",
            ],
        )
    )
    for event in model["events"]:
        fields = [
            f'id: "{event["id"]}"',
            f"tick: {event['tick']}",
            f"kind: {event['kind']}",
            f'target_id: "{event["target"]}"',
        ]
        if "bool" in event:
            fields.append(f"bool_value: {str(event['bool']).lower()}")
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


def generate_scenario(record: SeedRecord, policy: str, config: dict) -> str:
    return render(build_model(record, config), policy)


def paired_digest(record: SeedRecord, config: dict) -> str:
    model = build_model(record, config)
    normalized = [
        render(model, policy).replace(POLICIES[policy], "ALLOCATION_POLICY_PAIRED")
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
