from __future__ import annotations

import argparse
import hashlib
import json
import os
import pathlib
import shutil
import subprocess
import tempfile
import time
from dataclasses import asdict, dataclass
from typing import Callable

from generate_scenarios import POLICIES, SeedRecord, generate_scenario, load_records, paired_digest, read_config


@dataclass(frozen=True)
class Mission:
    record: SeedRecord
    policy: str

    @property
    def identity(self) -> str:
        return f"{self.record.identifier}.{self.policy}"


def canonical(value) -> bytes:
    return (json.dumps(value, sort_keys=True, separators=(",", ":")) + "\n").encode()


def file_digest(path: pathlib.Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def atomic_bytes(path: pathlib.Path, value: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + f".{os.getpid()}.tmp")
    with temporary.open("wb") as stream:
        stream.write(value)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)


class Journal:
    def __init__(self, root: pathlib.Path):
        self.root = root
        self.root.mkdir(parents=True, exist_ok=True)

    def path(self, identity: str) -> pathlib.Path:
        return self.root / f"{identity}.json"

    def write(self, row: dict) -> None:
        atomic_bytes(self.path(row["identity"]), canonical(row))

    def read(self) -> list[dict]:
        rows = []
        for path in sorted(self.root.glob("*.json")):
            value = json.loads(path.read_text(encoding="utf-8"))
            if value.get("identity") != path.stem:
                raise ValueError(f"journal identity mismatch: {path}")
            rows.append(value)
        return rows

    def clear(self) -> None:
        if self.root.exists():
            for path in self.root.iterdir():
                if path.is_file():
                    path.unlink()


def mission_plan(
    records: list[SeedRecord],
    shard_index: int = 0,
    shard_count: int = 1,
    limit: int | None = None,
) -> list[Mission]:
    if shard_count <= 0 or not 0 <= shard_index < shard_count:
        raise ValueError("invalid shard")
    missions = sorted(
        (Mission(record, policy) for record in records for policy in POLICIES),
        key=lambda mission: mission.identity,
    )
    missions = [
        mission
        for ordinal, mission in enumerate(missions)
        if ordinal % shard_count == shard_index
    ]
    if limit is not None:
        if limit <= 0:
            raise ValueError("limit must be positive")
        missions = missions[:limit]
    return missions


def result_path(output: pathlib.Path, shard_index: int, shard_count: int) -> pathlib.Path:
    if shard_count == 1:
        return output
    return output.with_name(f"{output.stem}.shard-{shard_index:03d}-of-{shard_count:03d}{output.suffix}")


def read_rows(path: pathlib.Path) -> list[dict]:
    if not path.exists():
        return []
    rows = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]
    identities = [row.get("identity") for row in rows]
    if any(not identity for identity in identities) or len(set(identities)) != len(identities):
        raise ValueError("results contain invalid identities")
    return rows


def write_rows(path: pathlib.Path, rows: list[dict]) -> None:
    ordered = sorted(rows, key=lambda row: row["identity"])
    atomic_bytes(path, b"".join(canonical(row) for row in ordered))


def validate_summary(value: dict) -> dict:
    required = ("success", "ticks", "completed_tasks", "total_tasks", "terminal_hash")
    if any(name not in value for name in required):
        raise ValueError("summary is incomplete")
    if (
        not isinstance(value["success"], bool)
        or any(not isinstance(value[name], int) or value[name] < 0 for name in required[1:4])
        or not isinstance(value["terminal_hash"], str)
        or not value["terminal_hash"]
    ):
        raise ValueError("summary is invalid")
    return value


def execute_subprocess(arguments: list[str], timeout: float) -> subprocess.CompletedProcess:
    return subprocess.run(arguments, check=False, capture_output=True, text=True, timeout=timeout)


def artifact_record(path: pathlib.Path, root: pathlib.Path) -> dict:
    return {
        "path": path.relative_to(root).as_posix(),
        "sha256": file_digest(path),
        "bytes": path.stat().st_size,
    }


def failed_row(mission: Mission, run_id: str, elapsed: float, error: Exception | str) -> dict:
    return {
        "identity": mission.identity,
        "run_id": run_id,
        "seed_id": mission.record.identifier,
        "seed": mission.record.seed,
        "stratum": mission.record.stratum,
        "allocator": mission.policy,
        "success": False,
        "elapsed_seconds": elapsed,
        "ticks": 0,
        "completed_tasks": 0,
        "total_tasks": 0,
        "terminal_hash": "",
        "scenario_sha256": "",
        "artifacts": [],
        "error": str(error)[:2000],
    }


def run_transaction(
    mission: Mission,
    runner: pathlib.Path,
    simulator: pathlib.Path,
    agent: pathlib.Path,
    artifacts: pathlib.Path,
    scratch: pathlib.Path,
    config: dict,
    run_id: str,
    timeout: float,
    executor: Callable[[list[str], float], subprocess.CompletedProcess] = execute_subprocess,
) -> dict:
    started = time.monotonic()
    transaction = pathlib.Path(tempfile.mkdtemp(prefix=f"{mission.identity}-", dir=scratch))
    scenario = transaction / "scenario.textproto"
    summary = transaction / "summary.json"
    event_log = transaction / "events.pb"
    scenario.write_text(generate_scenario(mission.record, mission.policy, config), encoding="utf-8", newline="\n")
    command = [
        str(runner),
        "--simulator",
        str(simulator),
        "--agent",
        str(agent),
        "--scenario",
        str(scenario),
        "--summary",
        str(summary),
        "--event-log",
        str(event_log),
    ]
    try:
        completed = executor(command, timeout)
        if completed.returncode:
            raise RuntimeError((completed.stderr or completed.stdout or "mission failed").strip())
        value = validate_summary(json.loads(summary.read_text(encoding="utf-8")))
        if not event_log.is_file() or event_log.stat().st_size == 0:
            raise ValueError("mission did not produce an event log")
        target = artifacts / mission.identity
        if target.exists():
            shutil.rmtree(target)
        target.parent.mkdir(parents=True, exist_ok=True)
        os.replace(transaction, target)
        published_scenario = target / scenario.name
        published_summary = target / summary.name
        published_log = target / event_log.name
        return {
            "identity": mission.identity,
            "run_id": run_id,
            "seed_id": mission.record.identifier,
            "seed": mission.record.seed,
            "stratum": mission.record.stratum,
            "allocator": mission.policy,
            "success": value["success"],
            "elapsed_seconds": time.monotonic() - started,
            "ticks": value["ticks"],
            "completed_tasks": value["completed_tasks"],
            "total_tasks": value["total_tasks"],
            "terminal_hash": value["terminal_hash"],
            "scenario_sha256": file_digest(published_scenario),
            "paired_scenario_sha256": paired_digest(mission.record, config),
            "artifacts": [
                artifact_record(published_scenario, artifacts),
                artifact_record(published_summary, artifacts),
                artifact_record(published_log, artifacts),
            ],
            "error": "",
        }
    except (OSError, ValueError, RuntimeError, subprocess.TimeoutExpired) as error:
        shutil.rmtree(transaction, ignore_errors=True)
        return failed_row(mission, run_id, time.monotonic() - started, error)


def verify_row(row: dict, artifacts: pathlib.Path, run_id: str) -> bool:
    if row.get("run_id") != run_id or not row.get("identity"):
        return False
    for artifact in row.get("artifacts", []):
        path = artifacts / artifact["path"]
        if (
            not path.is_file()
            or path.stat().st_size != artifact["bytes"]
            or file_digest(path) != artifact["sha256"]
        ):
            return False
    return True


def run_all(
    records: list[SeedRecord],
    output: pathlib.Path,
    runner: pathlib.Path,
    simulator: pathlib.Path,
    agent: pathlib.Path,
    config: dict,
    run_id: str,
    timeout: float,
    shard_index: int = 0,
    shard_count: int = 1,
    limit: int | None = None,
    resume: bool = False,
    executor: Callable[[list[str], float], subprocess.CompletedProcess] = execute_subprocess,
) -> list[dict]:
    output = result_path(output, shard_index, shard_count)
    journal = Journal(output.with_suffix(output.suffix + ".journal"))
    artifacts = output.with_suffix(output.suffix + ".artifacts")
    scratch = output.with_suffix(output.suffix + ".scratch")
    scratch.mkdir(parents=True, exist_ok=True)
    if not resume:
        journal.clear()
        shutil.rmtree(artifacts, ignore_errors=True)
    existing = {
        row["identity"]: row
        for row in journal.read()
        if verify_row(row, artifacts, run_id)
    }
    for mission in mission_plan(records, shard_index, shard_count, limit):
        if mission.identity in existing:
            continue
        row = run_transaction(
            mission,
            runner,
            simulator,
            agent,
            artifacts,
            scratch,
            config,
            run_id,
            timeout,
            executor,
        )
        journal.write(row)
        existing[mission.identity] = row
    rows = sorted(existing.values(), key=lambda row: row["identity"])
    write_rows(output, rows)
    return rows


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--seeds", type=pathlib.Path, required=True)
    parser.add_argument("--config", type=pathlib.Path)
    parser.add_argument("--output", type=pathlib.Path, required=True)
    parser.add_argument("--runner", type=pathlib.Path, required=True)
    parser.add_argument("--simulator", type=pathlib.Path, required=True)
    parser.add_argument("--agent", type=pathlib.Path, required=True)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--timeout", type=float, default=120.0)
    parser.add_argument("--shard-index", type=int, default=0)
    parser.add_argument("--shard-count", type=int, default=1)
    parser.add_argument("--limit", type=int)
    parser.add_argument("--resume", action="store_true")
    arguments = parser.parse_args()
    config = read_config(arguments.config)
    rows = run_all(
        load_records(arguments.seeds, config),
        arguments.output,
        arguments.runner,
        arguments.simulator,
        arguments.agent,
        config,
        arguments.run_id,
        arguments.timeout,
        arguments.shard_index,
        arguments.shard_count,
        arguments.limit,
        arguments.resume,
    )
    print(f"published {len(rows)} transactional benchmark rows")


if __name__ == "__main__":
    main()
