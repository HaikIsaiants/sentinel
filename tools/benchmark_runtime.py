from __future__ import annotations

import concurrent.futures
import json
import os
import pathlib
import shutil
import subprocess
import tempfile
import threading
import time
from dataclasses import dataclass
from typing import Callable

from benchmark_io import canonical_bytes, contained, file_digest


@dataclass(frozen=True)
class Attempt:
    identity: str
    scenario: pathlib.Path
    target: pathlib.Path


def command(
    runner: pathlib.Path,
    simulator: pathlib.Path,
    agent: pathlib.Path,
    scenario: pathlib.Path,
    summary: pathlib.Path,
    event_log: pathlib.Path,
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
        "--event-log",
        str(event_log),
    ]


def execute_subprocess(arguments: list[str], timeout: float) -> subprocess.CompletedProcess:
    return subprocess.run(arguments, check=False, capture_output=True, text=True, timeout=timeout)


def validate_summary(path: pathlib.Path) -> dict:
    value = json.loads(path.read_text(encoding="utf-8"))
    required = ("success", "ticks", "completed_tasks", "total_tasks", "terminal_hash")
    if any(name not in value for name in required):
        raise ValueError("mission summary is incomplete")
    if (
        not isinstance(value["success"], bool)
        or any(not isinstance(value[name], int) or value[name] < 0 for name in required[1:4])
        or not isinstance(value["terminal_hash"], str)
        or not value["terminal_hash"]
    ):
        raise ValueError("mission summary is invalid")
    return value


def inventory(root: pathlib.Path) -> list[dict]:
    values = []
    for path in sorted((item for item in root.rglob("*") if item.is_file()), key=lambda item: item.relative_to(root).as_posix()):
        values.append(
            {
                "path": path.relative_to(root).as_posix(),
                "sha256": file_digest(path),
                "bytes": path.stat().st_size,
            }
        )
    return values


def run_attempt(
    attempt: Attempt,
    runner: pathlib.Path,
    simulator: pathlib.Path,
    agent: pathlib.Path,
    scratch: pathlib.Path,
    timeout: float,
    executor: Callable[[list[str], float], subprocess.CompletedProcess] = execute_subprocess,
) -> dict:
    started = time.monotonic()
    transaction = pathlib.Path(tempfile.mkdtemp(prefix=f"{attempt.identity}-", dir=scratch))
    scenario = transaction / "scenario.textproto"
    summary = transaction / "summary.json"
    event_log = transaction / "events.pb"
    shutil.copyfile(attempt.scenario, scenario)
    try:
        completed = executor(command(runner, simulator, agent, scenario, summary, event_log), timeout)
        if completed.returncode:
            raise RuntimeError((completed.stderr or completed.stdout or "mission process failed").strip())
        value = validate_summary(summary)
        if not event_log.is_file() or event_log.stat().st_size == 0:
            raise ValueError("mission event log is missing")
        if attempt.target.exists():
            shutil.rmtree(attempt.target)
        attempt.target.parent.mkdir(parents=True, exist_ok=True)
        os.replace(transaction, attempt.target)
        artifacts = inventory(attempt.target)
        for artifact in artifacts:
            artifact["path"] = f"{attempt.identity}/{artifact['path']}"
        return {
            "identity": attempt.identity,
            "ok": True,
            "summary": value,
            "artifacts": artifacts,
            "elapsed_seconds": time.monotonic() - started,
            "thread_id": threading.get_ident(),
            "error": "",
        }
    except (OSError, ValueError, RuntimeError, subprocess.TimeoutExpired) as error:
        shutil.rmtree(transaction, ignore_errors=True)
        return {
            "identity": attempt.identity,
            "ok": False,
            "summary": {},
            "artifacts": [],
            "elapsed_seconds": time.monotonic() - started,
            "thread_id": threading.get_ident(),
            "error": str(error)[:2000],
        }


class ParallelDispatcher:
    def __init__(
        self,
        workers: int,
        runner: pathlib.Path,
        simulator: pathlib.Path,
        agent: pathlib.Path,
        scratch: pathlib.Path,
        timeout: float,
        executor: Callable[[list[str], float], subprocess.CompletedProcess] = execute_subprocess,
    ):
        if workers <= 0 or timeout <= 0:
            raise ValueError("invalid dispatcher configuration")
        self.workers = workers
        self.runner = runner
        self.simulator = simulator
        self.agent = agent
        self.scratch = scratch
        self.timeout = timeout
        self.executor = executor
        self.scratch.mkdir(parents=True, exist_ok=True)

    def dispatch(self, attempts: list[Attempt]) -> list[dict]:
        identities = [attempt.identity for attempt in attempts]
        if len(set(identities)) != len(identities):
            raise ValueError("duplicate parallel attempt")
        completed = []
        with concurrent.futures.ThreadPoolExecutor(max_workers=self.workers, thread_name_prefix="sentinel") as pool:
            futures = {
                pool.submit(
                    run_attempt,
                    attempt,
                    self.runner,
                    self.simulator,
                    self.agent,
                    self.scratch,
                    self.timeout,
                    self.executor,
                ): attempt.identity
                for attempt in attempts
            }
            for future in concurrent.futures.as_completed(futures):
                try:
                    completed.append(future.result())
                except Exception as error:
                    completed.append(
                        {
                            "identity": futures[future],
                            "ok": False,
                            "summary": {},
                            "artifacts": [],
                            "elapsed_seconds": 0.0,
                            "thread_id": 0,
                            "error": str(error)[:2000],
                        }
                    )
        return completed


def verify_artifacts(row: dict, root: pathlib.Path) -> bool:
    if not row.get("identity"):
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
