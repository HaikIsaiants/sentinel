from __future__ import annotations

import argparse
import json
import math
import pathlib
import statistics
from collections import defaultdict

from benchmark_io import contained, file_digest, validate_digest


ALLOCATORS = ("nearest", "cbba")
INVARIANTS = (
    "agent_energy_never_drops_below_zero",
    "committed_reservations_never_overlap",
    "completed_tasks_are_never_reassigned",
    "incapable_agents_never_commit_tasks",
)


def read_rows(paths: list[pathlib.Path]) -> list[dict]:
    rows = []
    identities = set()
    run_ids = set()
    for path in paths:
        for number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
            if not line.strip():
                continue
            row = json.loads(line)
            required = {"identity", "run_id", "seed_id", "seed", "stratum", "allocator", "success", "ticks", "artifacts"}
            if not isinstance(row, dict) or required.difference(row):
                raise ValueError(f"invalid benchmark row at {path}:{number}")
            identity = row["run_id"], row["identity"]
            if identity in identities:
                raise ValueError(f"duplicate benchmark row {identity}")
            if row["identity"] != f"{row['seed_id']}.{row['allocator']}":
                raise ValueError("benchmark row identity mismatch")
            validate_digest(row["run_id"])
            identities.add(identity)
            run_ids.add(row["run_id"])
            rows.append(row)
    if not rows or len(run_ids) != 1:
        raise ValueError("analysis requires one non-empty run")
    return rows


def wilson_lower(successes: int, total: int, z: float = 1.96) -> float:
    if total == 0:
        return 0.0
    probability = successes / total
    denominator = 1 + z * z / total
    center = probability + z * z / (2 * total)
    margin = z * math.sqrt(probability * (1 - probability) / total + z * z / (4 * total * total))
    return max(0.0, (center - margin) / denominator)


def nearest_rank(values: list[float], fraction: float) -> float:
    if not values:
        return 0.0
    values = sorted(values)
    return values[max(0, math.ceil(len(values) * fraction) - 1)]


def invariant_failures(row: dict) -> list[str]:
    summary = row.get("summary") or {}
    return [name for name in INVARIANTS if summary.get(name) is not True]


def effective_success(row: dict) -> bool:
    return bool(row.get("success")) and not row.get("error") and not invariant_failures(row)


def describe(rows: list[dict]) -> dict:
    successful = [row for row in rows if effective_success(row)]
    ticks = [row["ticks"] for row in successful]
    elapsed = [float(row.get("elapsed_seconds", 0)) for row in rows]
    failures = defaultdict(int)
    for row in rows:
        if row.get("error"):
            failures["runtime"] += 1
        for name in invariant_failures(row):
            failures[name] += 1
    return {
        "missions": len(rows),
        "successes": len(successful),
        "success_rate": len(successful) / len(rows),
        "success_wilson_lower_95": wilson_lower(len(successful), len(rows)),
        "mean_ticks": statistics.fmean(ticks) if ticks else 0.0,
        "p95_ticks": nearest_rank(ticks, 0.95),
        "mean_elapsed_seconds": statistics.fmean(elapsed),
        "p95_elapsed_seconds": nearest_rank(elapsed, 0.95),
        "failures": dict(sorted(failures.items())),
    }


def paired(rows: list[dict]) -> dict:
    grouped = defaultdict(dict)
    for row in rows:
        grouped[row["seed_id"]][row["allocator"]] = row
    pairs = [value for value in grouped.values() if set(value) == set(ALLOCATORS)]
    wins = losses = ties = 0
    deltas = []
    for pair in pairs:
        nearest = pair["nearest"]
        cbba = pair["cbba"]
        left = (effective_success(nearest), -nearest["ticks"])
        right = (effective_success(cbba), -cbba["ticks"])
        if right > left:
            wins += 1
        elif right < left:
            losses += 1
        else:
            ties += 1
        if effective_success(nearest) and effective_success(cbba):
            deltas.append(nearest["ticks"] - cbba["ticks"])
    return {
        "pairs": len(pairs),
        "candidate_wins": wins,
        "baseline_wins": losses,
        "ties": ties,
        "candidate_win_wilson_lower_95": wilson_lower(wins, len(pairs)),
        "mean_tick_improvement": statistics.fmean(deltas) if deltas else 0.0,
    }


def audit_artifacts(rows: list[dict], roots: list[pathlib.Path]) -> list[str]:
    failures = []
    by_name = {root.name: root for root in roots}
    for row in rows:
        root_name = row.get("artifact_root") or roots[0].name
        root = by_name.get(root_name)
        if root is None:
            failures.append(row["identity"])
            continue
        for artifact in row.get("artifacts", []):
            try:
                path = contained(root, artifact["path"])
            except (KeyError, ValueError):
                failures.append(row["identity"])
                break
            if (
                not path.is_file()
                or path.stat().st_size != artifact.get("bytes")
                or file_digest(path) != artifact.get("sha256")
            ):
                failures.append(row["identity"])
                break
    return sorted(set(failures))


def analyze(rows: list[dict], artifact_failures: list[str] | None = None) -> dict:
    artifact_failures = set(artifact_failures or [])
    normalized = []
    for row in rows:
        value = dict(row)
        if row["identity"] in artifact_failures:
            value["success"] = False
            value["error"] = "artifact verification failed"
        normalized.append(value)
    groups = defaultdict(list)
    strata = defaultdict(lambda: defaultdict(list))
    for row in normalized:
        if row["allocator"] not in ALLOCATORS:
            raise ValueError("unexpected allocator")
        groups[row["allocator"]].append(row)
        strata[row["stratum"]][row["allocator"]].append(row)
    if set(groups) != set(ALLOCATORS):
        raise ValueError("both allocators are required")
    return {
        "schema_version": 3,
        "run_id": normalized[0]["run_id"],
        "rows": len(normalized),
        "artifact_failure_ids": sorted(artifact_failures),
        "allocators": {name: describe(groups[name]) for name in ALLOCATORS},
        "strata": {
            key: {name: describe(value[name]) for name in ALLOCATORS if value[name]}
            for key, value in sorted(strata.items())
        },
        "paired": paired(normalized),
    }


def markdown(report: dict) -> str:
    lines = [
        "# Persistent benchmark evidence",
        "",
        f"Run: `{report['run_id']}`",
        "",
        "| Allocator | Missions | Success | Wilson lower | Mean ticks | P95 ticks |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for name, value in report["allocators"].items():
        lines.append(
            f"| {name} | {value['missions']} | {value['success_rate']:.4f} | "
            f"{value['success_wilson_lower_95']:.4f} | {value['mean_ticks']:.2f} | {value['p95_ticks']:.0f} |"
        )
    pair = report["paired"]
    lines.extend(
        [
            "",
            f"Paired missions: {pair['pairs']}",
            "",
            f"CBBA wins: {pair['candidate_wins']}; nearest wins: {pair['baseline_wins']}; ties: {pair['ties']}.",
            "",
            f"Artifact failures: {len(report['artifact_failure_ids'])}.",
        ]
    )
    return "\n".join(lines) + "\n"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("results", nargs="+", type=pathlib.Path)
    parser.add_argument("--artifact-root", action="append", type=pathlib.Path, default=[])
    parser.add_argument("--json", type=pathlib.Path, required=True)
    parser.add_argument("--markdown", type=pathlib.Path)
    arguments = parser.parse_args()
    rows = read_rows(arguments.results)
    failures = audit_artifacts(rows, arguments.artifact_root) if arguments.artifact_root else []
    report = analyze(rows, failures)
    arguments.json.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8", newline="\n")
    if arguments.markdown:
        arguments.markdown.write_text(markdown(report), encoding="utf-8", newline="\n")


if __name__ == "__main__":
    main()
