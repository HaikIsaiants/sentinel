from __future__ import annotations

import argparse
import hashlib
import json
import pathlib
import shutil

from benchmark_io import atomic_write, canonical_bytes, file_digest
from benchmark_runtime import Attempt, ParallelDispatcher, verify_artifacts
from generate_scenarios import POLICIES, SeedRecord, generate_scenario, load_records, paired_digest, read_config


def identity(record: SeedRecord, policy: str) -> str:
    return f"{record.identifier}.{policy}"


def plan(records: list[SeedRecord], shard_index: int, shard_count: int, limit: int | None) -> list[tuple[SeedRecord, str]]:
    if shard_count <= 0 or not 0 <= shard_index < shard_count:
        raise ValueError("invalid shard")
    values = sorted(((record, policy) for record in records for policy in POLICIES), key=lambda item: identity(*item))
    values = [value for ordinal, value in enumerate(values) if ordinal % shard_count == shard_index]
    if limit is not None:
        if limit <= 0:
            raise ValueError("limit must be positive")
        values = values[:limit]
    return values


def output_path(path: pathlib.Path, shard_index: int, shard_count: int) -> pathlib.Path:
    if shard_count == 1:
        return path
    return path.with_name(f"{path.stem}.shard-{shard_index:03d}-of-{shard_count:03d}{path.suffix}")


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
    atomic_write(root / f"{row['identity']}.json", canonical_bytes(row))


def failed(record: SeedRecord, policy: str, run_id: str, result: dict) -> dict:
    return {
        "identity": identity(record, policy),
        "run_id": run_id,
        "seed_id": record.identifier,
        "seed": record.seed,
        "stratum": record.stratum,
        "allocator": policy,
        "success": False,
        "elapsed_seconds": result["elapsed_seconds"],
        "ticks": 0,
        "completed_tasks": 0,
        "total_tasks": 0,
        "terminal_hash": "",
        "scenario_sha256": "",
        "paired_scenario_sha256": "",
        "artifacts": [],
        "attempt_processes": 1,
        "dispatch_thread": result["thread_id"],
        "error": result["error"],
    }


def completed(record: SeedRecord, policy: str, run_id: str, result: dict, config: dict, artifacts: pathlib.Path) -> dict:
    value = result["summary"]
    scenario = artifacts / identity(record, policy) / "scenario.textproto"
    return {
        "identity": identity(record, policy),
        "run_id": run_id,
        "seed_id": record.identifier,
        "seed": record.seed,
        "stratum": record.stratum,
        "allocator": policy,
        "success": value["success"],
        "elapsed_seconds": result["elapsed_seconds"],
        "ticks": value["ticks"],
        "completed_tasks": value["completed_tasks"],
        "total_tasks": value["total_tasks"],
        "terminal_hash": value["terminal_hash"],
        "scenario_sha256": file_digest(scenario),
        "paired_scenario_sha256": paired_digest(record, config),
        "artifacts": result["artifacts"],
        "attempt_processes": 1,
        "dispatch_thread": result["thread_id"],
        "error": "",
    }


def publish_output(path: pathlib.Path, rows: list[dict], artifacts: pathlib.Path, run_id: str) -> pathlib.Path:
    ordered = sorted(rows, key=lambda row: row["identity"])
    atomic_write(path, b"".join(canonical_bytes(row) for row in ordered))
    aggregate = hashlib.sha256()
    for row in ordered:
        for artifact in row["artifacts"]:
            aggregate.update(canonical_bytes(artifact))
    record = {
        "schema_version": 1,
        "run_id": run_id,
        "results_sha256": file_digest(path),
        "artifacts_sha256": aggregate.hexdigest(),
        "rows": len(ordered),
    }
    record_path = path.with_suffix(path.suffix + ".record.json")
    atomic_write(record_path, canonical_bytes(record))
    return record_path


def run_all(
    records: list[SeedRecord],
    output: pathlib.Path,
    config: dict,
    run_id: str,
    dispatcher: ParallelDispatcher,
    shard_index: int = 0,
    shard_count: int = 1,
    limit: int | None = None,
    resume: bool = False,
) -> list[dict]:
    output = output_path(output, shard_index, shard_count)
    journal = output.with_suffix(output.suffix + ".journal")
    artifacts = output.with_suffix(output.suffix + ".artifacts")
    scenarios = output.with_suffix(output.suffix + ".scenarios")
    if not resume:
        shutil.rmtree(journal, ignore_errors=True)
        shutil.rmtree(artifacts, ignore_errors=True)
        shutil.rmtree(scenarios, ignore_errors=True)
    journal.mkdir(parents=True, exist_ok=True)
    artifacts.mkdir(parents=True, exist_ok=True)
    scenarios.mkdir(parents=True, exist_ok=True)
    existing = {
        row["identity"]: row
        for row in read_journal(journal)
        if row.get("run_id") == run_id and verify_artifacts(row, artifacts)
    }
    pending = []
    lookup = {}
    for record, policy in plan(records, shard_index, shard_count, limit):
        key = identity(record, policy)
        if key in existing:
            continue
        scenario = scenarios / f"{key}.textproto"
        scenario.write_text(generate_scenario(record, policy, config), encoding="utf-8", newline="\n")
        pending.append(Attempt(key, scenario, artifacts / key))
        lookup[key] = record, policy
    results = dispatcher.dispatch(pending)
    for result in results:
        record, policy = lookup[result["identity"]]
        row = (
            completed(record, policy, run_id, result, config, artifacts)
            if result["ok"]
            else failed(record, policy, run_id, result)
        )
        write_journal(journal, row)
        existing[row["identity"]] = row
    rows = sorted(existing.values(), key=lambda row: row["identity"])
    publish_output(output, rows, artifacts, run_id)
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
    dispatcher = ParallelDispatcher(
        arguments.workers,
        arguments.runner,
        arguments.simulator,
        arguments.agent,
        arguments.output.with_suffix(arguments.output.suffix + ".scratch"),
        arguments.timeout,
    )
    rows = run_all(
        load_records(arguments.seeds, config),
        arguments.output,
        config,
        arguments.run_id,
        dispatcher,
        arguments.shard_index,
        arguments.shard_count,
        arguments.limit,
        arguments.resume,
    )
    print(f"published {len(rows)} bounded-parallel benchmark rows")


if __name__ == "__main__":
    main()
