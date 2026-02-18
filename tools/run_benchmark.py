from __future__ import annotations

import argparse
import json
import pathlib
import subprocess
import tempfile
import time
from dataclasses import asdict, dataclass
from typing import Callable

from generate_scenarios import POLICIES, SeedRecord, generate_scenario, load_records


@dataclass(frozen=True)
class Mission:
    record: SeedRecord
    policy: str

    @property
    def key(self) -> tuple[str, str]:
        return self.record.identifier, self.policy


@dataclass
class Result:
    seed_id: str
    seed: int
    stratum: str
    allocator: str
    success: bool
    elapsed_seconds: float
    ticks: int = 0
    completed_tasks: int = 0
    total_tasks: int = 0
    terminal_hash: str = ""
    error: str = ""


def mission_plan(records: list[SeedRecord], policies: tuple[str, ...] = tuple(POLICIES)) -> list[Mission]:
    missions = [Mission(record, policy) for record in records for policy in policies]
    missions.sort(key=lambda mission: mission.key)
    return missions


def read_rows(path: pathlib.Path) -> list[dict]:
    if not path.exists():
        return []
    rows = []
    for number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if not line.strip():
            continue
        value = json.loads(line)
        if not isinstance(value, dict):
            raise ValueError(f"result line {number} is not an object")
        rows.append(value)
    return rows


def write_rows(path: pathlib.Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = "".join(json.dumps(row, sort_keys=True, separators=(",", ":")) + "\n" for row in rows)
    path.write_text(payload, encoding="utf-8", newline="\n")


def validate_summary(value: dict) -> dict:
    required = {"success", "ticks", "completed_tasks", "total_tasks", "terminal_hash"}
    missing = required.difference(value)
    if missing:
        raise ValueError(f"summary is missing {sorted(missing)}")
    if not isinstance(value["success"], bool):
        raise ValueError("summary success must be boolean")
    for name in ("ticks", "completed_tasks", "total_tasks"):
        if not isinstance(value[name], int) or value[name] < 0:
            raise ValueError(f"summary {name} is invalid")
    if not isinstance(value["terminal_hash"], str) or not value["terminal_hash"]:
        raise ValueError("summary terminal hash is invalid")
    return value


def command(
    runner: pathlib.Path,
    simulator: pathlib.Path,
    agent: pathlib.Path,
    scenario: pathlib.Path,
    summary: pathlib.Path,
) -> list[str]:
    return [
        str(runner),
        "--simulator",
        str(simulator),
        "--agent",
        str(agent),
        "--scenario",
        str(scenario),
        "--summary",
        str(summary),
    ]


def subprocess_executor(arguments: list[str], timeout: float) -> subprocess.CompletedProcess:
    return subprocess.run(
        arguments,
        check=False,
        capture_output=True,
        text=True,
        timeout=timeout,
    )


def run_one(
    mission: Mission,
    runner: pathlib.Path,
    simulator: pathlib.Path,
    agent: pathlib.Path,
    scratch: pathlib.Path,
    timeout: float,
    executor: Callable[[list[str], float], subprocess.CompletedProcess] = subprocess_executor,
) -> Result:
    scenario = scratch / f"{mission.record.identifier}.{mission.policy}.textproto"
    summary = scratch / f"{mission.record.identifier}.{mission.policy}.summary.json"
    scenario.write_text(
        generate_scenario(mission.record, mission.policy),
        encoding="utf-8",
        newline="\n",
    )
    started = time.monotonic()
    try:
        completed = executor(command(runner, simulator, agent, scenario, summary), timeout)
        elapsed = time.monotonic() - started
        if completed.returncode:
            detail = (completed.stderr or completed.stdout or "mission process failed").strip()
            raise RuntimeError(detail[:1000])
        value = validate_summary(json.loads(summary.read_text(encoding="utf-8")))
        return Result(
            seed_id=mission.record.identifier,
            seed=mission.record.seed,
            stratum=mission.record.stratum,
            allocator=mission.policy,
            success=value["success"],
            elapsed_seconds=elapsed,
            ticks=value["ticks"],
            completed_tasks=value["completed_tasks"],
            total_tasks=value["total_tasks"],
            terminal_hash=value["terminal_hash"],
        )
    except (OSError, ValueError, RuntimeError, subprocess.TimeoutExpired) as error:
        return Result(
            seed_id=mission.record.identifier,
            seed=mission.record.seed,
            stratum=mission.record.stratum,
            allocator=mission.policy,
            success=False,
            elapsed_seconds=time.monotonic() - started,
            error=str(error)[:1000],
        )


def run_all(
    records: list[SeedRecord],
    output: pathlib.Path,
    runner: pathlib.Path,
    simulator: pathlib.Path,
    agent: pathlib.Path,
    timeout: float,
    limit: int | None = None,
    executor: Callable[[list[str], float], subprocess.CompletedProcess] = subprocess_executor,
) -> list[dict]:
    plan = mission_plan(records)
    if limit is not None:
        if limit <= 0:
            raise ValueError("limit must be positive")
        plan = plan[:limit]
    rows = []
    with tempfile.TemporaryDirectory(prefix="sentinel-benchmark-") as directory:
        scratch = pathlib.Path(directory)
        for mission in plan:
            result = run_one(mission, runner, simulator, agent, scratch, timeout, executor)
            rows.append(asdict(result))
            write_rows(output, rows)
    return rows


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--seeds", type=pathlib.Path, required=True)
    parser.add_argument("--output", type=pathlib.Path, required=True)
    parser.add_argument("--runner", type=pathlib.Path, required=True)
    parser.add_argument("--simulator", type=pathlib.Path, required=True)
    parser.add_argument("--agent", type=pathlib.Path, required=True)
    parser.add_argument("--timeout", type=float, default=120.0)
    parser.add_argument("--limit", type=int)
    arguments = parser.parse_args()
    if arguments.timeout <= 0:
        raise ValueError("timeout must be positive")
    rows = run_all(
        load_records(arguments.seeds),
        arguments.output,
        arguments.runner,
        arguments.simulator,
        arguments.agent,
        arguments.timeout,
        arguments.limit,
    )
    failures = sum(not row["success"] for row in rows)
    print(f"wrote {len(rows)} rows to {arguments.output} ({failures} unsuccessful)")


if __name__ == "__main__":
    main()
