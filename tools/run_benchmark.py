from __future__ import annotations

import argparse
import hashlib
import json
import os
import pathlib
import shutil
import tempfile
import time
from dataclasses import dataclass
from typing import Callable

from benchmark_io import canonical_bytes, contained, file_digest, write_output_record
from benchmark_runtime import StreamWorker, WorkerPool, request, validate_response, worker_command
from generate_scenarios import POLICIES, SeedRecord, generate_scenario, load_records, paired_digest, read_config


@dataclass(frozen=True)
class Mission:
    record: SeedRecord
    policy: str

    @property
    def identity(self) -> str:
        return f"{self.record.identifier}.{self.policy}"


def atomic_write(path: pathlib.Path, value: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + f".{os.getpid()}.tmp")
    with temporary.open("wb") as stream:
        stream.write(value)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)


def journal_path(root: pathlib.Path, identity: str) -> pathlib.Path:
    return root / f"{identity}.json"


def read_journal(root: pathlib.Path) -> list[dict]:
    if not root.exists():
        return []
    rows = []
    for path in sorted(root.glob("*.json")):
        row = json.loads(path.read_text(encoding="utf-8"))
        if row.get("identity") != path.stem:
            raise ValueError("journal identity mismatch")
        rows.append(row)
    return rows


def write_journal(root: pathlib.Path, row: dict) -> None:
    atomic_write(journal_path(root, row["identity"]), canonical_bytes(row))


def plan(records: list[SeedRecord], shard_index: int, shard_count: int, limit: int | None) -> list[Mission]:
    if shard_count <= 0 or not 0 <= shard_index < shard_count:
        raise ValueError("invalid shard")
    values = sorted(
        (Mission(record, policy) for record in records for policy in POLICIES),
        key=lambda mission: mission.identity,
    )
    values = [mission for ordinal, mission in enumerate(values) if ordinal % shard_count == shard_index]
    if limit is not None:
        if limit <= 0:
            raise ValueError("limit must be positive")
        values = values[:limit]
    return values


def output_path(path: pathlib.Path, shard_index: int, shard_count: int) -> pathlib.Path:
    if shard_count == 1:
        return path
    return path.with_name(f"{path.stem}.shard-{shard_index:03d}-of-{shard_count:03d}{path.suffix}")


def artifact_inventory(root: pathlib.Path) -> tuple[list[dict], str]:
    records = []
    aggregate = hashlib.sha256()
    for path in sorted((item for item in root.rglob("*") if item.is_file()), key=lambda item: item.relative_to(root).as_posix()):
        record = {
            "path": path.relative_to(root).as_posix(),
            "sha256": file_digest(path),
            "bytes": path.stat().st_size,
        }
        records.append(record)
        aggregate.update(canonical_bytes(record))
    return records, aggregate.hexdigest()


def verified(row: dict, root: pathlib.Path, run_id: str) -> bool:
    if row.get("run_id") != run_id or not row.get("identity"):
        return False
    for artifact in row.get("artifacts", []):
        try:
            path = contained(root, artifact["path"])
        except (KeyError, ValueError):
            return False
        if (
            not path.is_file()
            or path.stat().st_size != artifact.get("bytes")
            or file_digest(path) != artifact.get("sha256")
        ):
            return False
    return True


def failure(mission: Mission, run_id: str, elapsed: float, error: Exception | str) -> dict:
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
        "paired_scenario_sha256": "",
        "artifacts": [],
        "error": str(error)[:2000],
    }


def execute_mission(
    mission: Mission,
    pool: WorkerPool,
    artifacts: pathlib.Path,
    scratch: pathlib.Path,
    config: dict,
    run_id: str,
    timeout: float,
) -> dict:
    started = time.monotonic()
    transaction = pathlib.Path(tempfile.mkdtemp(prefix=f"{mission.identity}-", dir=scratch))
    scenario = transaction / "scenario.textproto"
    worker_output = transaction / "worker"
    worker_output.mkdir()
    scenario.write_text(generate_scenario(mission.record, mission.policy, config), encoding="utf-8", newline="\n")
    try:
        response = pool.execute(request(mission.identity, scenario, worker_output), timeout)
        response = validate_response(response, mission.identity)
        summary = response["summary"]
        for item in response["artifacts"]:
            path = contained(worker_output, item)
            if not path.is_file():
                raise ValueError(f"worker omitted artifact {item}")
        target = artifacts / mission.identity
        if target.exists():
            shutil.rmtree(target)
        target.parent.mkdir(parents=True, exist_ok=True)
        os.replace(transaction, target)
        inventory, _ = artifact_inventory(target)
        for artifact in inventory:
            artifact["path"] = f"{mission.identity}/{artifact['path']}"
        return {
            "identity": mission.identity,
            "run_id": run_id,
            "seed_id": mission.record.identifier,
            "seed": mission.record.seed,
            "stratum": mission.record.stratum,
            "allocator": mission.policy,
            "success": summary["success"],
            "elapsed_seconds": time.monotonic() - started,
            "ticks": summary["ticks"],
            "completed_tasks": summary["completed_tasks"],
            "total_tasks": summary["total_tasks"],
            "terminal_hash": summary["terminal_hash"],
            "scenario_sha256": file_digest(target / "scenario.textproto"),
            "paired_scenario_sha256": paired_digest(mission.record, config),
            "artifacts": inventory,
            "error": "",
        }
    except Exception as error:
        shutil.rmtree(transaction, ignore_errors=True)
        return failure(mission, run_id, time.monotonic() - started, error)


def run_all(
    records: list[SeedRecord],
    output: pathlib.Path,
    config: dict,
    run_id: str,
    pool: WorkerPool,
    timeout: float,
    shard_index: int = 0,
    shard_count: int = 1,
    limit: int | None = None,
    resume: bool = False,
) -> list[dict]:
    output = output_path(output, shard_index, shard_count)
    journal = output.with_suffix(output.suffix + ".journal")
    artifacts = output.with_suffix(output.suffix + ".artifacts")
    scratch = output.with_suffix(output.suffix + ".scratch")
    scratch.mkdir(parents=True, exist_ok=True)
    if not resume:
        shutil.rmtree(journal, ignore_errors=True)
        shutil.rmtree(artifacts, ignore_errors=True)
    journal.mkdir(parents=True, exist_ok=True)
    existing = {row["identity"]: row for row in read_journal(journal) if verified(row, artifacts, run_id)}
    for mission in plan(records, shard_index, shard_count, limit):
        if mission.identity in existing:
            continue
        row = execute_mission(mission, pool, artifacts, scratch, config, run_id, timeout)
        write_journal(journal, row)
        existing[mission.identity] = row
    rows = sorted(existing.values(), key=lambda row: row["identity"])
    atomic_write(output, b"".join(canonical_bytes(row) for row in rows))
    _, artifact_digest = artifact_inventory(artifacts)
    write_output_record(output, run_id, file_digest(output), artifact_digest)
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
    parser.add_argument("--workers", type=int, default=2)
    parser.add_argument("--timeout", type=float, default=120.0)
    parser.add_argument("--shard-index", type=int, default=0)
    parser.add_argument("--shard-count", type=int, default=1)
    parser.add_argument("--limit", type=int)
    parser.add_argument("--resume", action="store_true")
    arguments = parser.parse_args()
    config = read_config(arguments.config)
    scratch = arguments.output.with_suffix(arguments.output.suffix + ".workers")
    scratch.mkdir(parents=True, exist_ok=True)
    factory = lambda: StreamWorker(worker_command(arguments.runner, arguments.simulator, arguments.agent, scratch))
    pool = WorkerPool(arguments.workers, factory)
    try:
        rows = run_all(
            load_records(arguments.seeds, config),
            arguments.output,
            config,
            arguments.run_id,
            pool,
            arguments.timeout,
            arguments.shard_index,
            arguments.shard_count,
            arguments.limit,
            arguments.resume,
        )
    finally:
        pool.close()
    print(f"published {len(rows)} persistent benchmark rows")


if __name__ == "__main__":
    main()
