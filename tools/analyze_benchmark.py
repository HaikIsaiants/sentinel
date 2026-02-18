from __future__ import annotations

import argparse
import json
import math
import pathlib
import statistics
from collections import defaultdict


ALLOCATORS = ("nearest", "cbba")
REQUIRED = {
    "identity",
    "run_id",
    "seed_id",
    "seed",
    "stratum",
    "allocator",
    "success",
    "elapsed_seconds",
    "ticks",
    "completed_tasks",
    "total_tasks",
    "terminal_hash",
    "scenario_sha256",
    "artifacts",
    "error",
}


def read_rows(paths: list[pathlib.Path]) -> list[dict]:
    rows = []
    identities = set()
    run_ids = set()
    for path in paths:
        for number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
            if not line.strip():
                continue
            value = json.loads(line)
            missing = REQUIRED.difference(value)
            if missing:
                raise ValueError(f"{path}:{number} misses {sorted(missing)}")
            if value["identity"] != f"{value['seed_id']}.{value['allocator']}":
                raise ValueError(f"{path}:{number} has invalid identity")
            identity = value["run_id"], value["identity"]
            if identity in identities:
                raise ValueError(f"duplicate result {identity}")
            identities.add(identity)
            run_ids.add(value["run_id"])
            rows.append(value)
    if not rows or len(run_ids) != 1:
        raise ValueError("results must contain one non-empty run")
    return rows


def nearest_rank(values: list[float], fraction: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    return ordered[max(0, math.ceil(len(ordered) * fraction) - 1)]


def wilson_lower(successes: int, total: int, z: float = 1.96) -> float:
    if total == 0:
        return 0.0
    probability = successes / total
    denominator = 1 + z * z / total
    center = probability + z * z / (2 * total)
    margin = z * math.sqrt(probability * (1 - probability) / total + z * z / (4 * total * total))
    return max(0.0, (center - margin) / denominator)


def describe(rows: list[dict]) -> dict:
    successes = [row for row in rows if row["success"]]
    ticks = [row["ticks"] for row in successes]
    elapsed = [float(row["elapsed_seconds"]) for row in rows]
    completed = sum(row["completed_tasks"] for row in rows)
    total_tasks = sum(row["total_tasks"] for row in rows)
    return {
        "missions": len(rows),
        "successes": len(successes),
        "success_rate": len(successes) / len(rows),
        "success_wilson_lower_95": wilson_lower(len(successes), len(rows)),
        "mean_ticks": statistics.fmean(ticks) if ticks else 0.0,
        "p95_ticks": nearest_rank(ticks, 0.95),
        "mean_elapsed_seconds": statistics.fmean(elapsed),
        "p95_elapsed_seconds": nearest_rank(elapsed, 0.95),
        "task_completion_rate": completed / total_tasks if total_tasks else 0.0,
    }


def paired(rows: list[dict]) -> dict:
    grouped = defaultdict(dict)
    for row in rows:
        grouped[row["seed_id"]][row["allocator"]] = row
    pairs = [value for value in grouped.values() if set(value) == set(ALLOCATORS)]
    candidate_wins = 0
    baseline_wins = 0
    ties = 0
    tick_deltas = []
    for pair in pairs:
        baseline = pair["nearest"]
        candidate = pair["cbba"]
        baseline_score = (bool(baseline["success"]), -baseline["ticks"])
        candidate_score = (bool(candidate["success"]), -candidate["ticks"])
        if candidate_score > baseline_score:
            candidate_wins += 1
        elif candidate_score < baseline_score:
            baseline_wins += 1
        else:
            ties += 1
        if baseline["success"] and candidate["success"]:
            tick_deltas.append(baseline["ticks"] - candidate["ticks"])
    return {
        "pairs": len(pairs),
        "candidate_wins": candidate_wins,
        "baseline_wins": baseline_wins,
        "ties": ties,
        "candidate_win_wilson_lower_95": wilson_lower(candidate_wins, len(pairs)),
        "mean_tick_improvement": statistics.fmean(tick_deltas) if tick_deltas else 0.0,
    }


def analyze(rows: list[dict]) -> dict:
    allocator_rows = defaultdict(list)
    strata = defaultdict(lambda: defaultdict(list))
    for row in rows:
        if row["allocator"] not in ALLOCATORS:
            raise ValueError("unexpected allocator")
        allocator_rows[row["allocator"]].append(row)
        strata[row["stratum"]][row["allocator"]].append(row)
    if set(allocator_rows) != set(ALLOCATORS):
        raise ValueError("both allocators are required")
    return {
        "schema_version": 2,
        "run_id": rows[0]["run_id"],
        "rows": len(rows),
        "allocators": {name: describe(allocator_rows[name]) for name in ALLOCATORS},
        "strata": {
            stratum: {name: describe(values[name]) for name in ALLOCATORS if values[name]}
            for stratum, values in sorted(strata.items())
        },
        "paired": paired(rows),
    }


def markdown(report: dict) -> str:
    lines = [
        "# Transactional benchmark report",
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
    value = report["paired"]
    lines.extend(
        [
            "",
            f"Paired missions: {value['pairs']}",
            "",
            f"CBBA wins: {value['candidate_wins']}; nearest wins: {value['baseline_wins']}; ties: {value['ties']}.",
        ]
    )
    return "\n".join(lines) + "\n"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("results", nargs="+", type=pathlib.Path)
    parser.add_argument("--json", type=pathlib.Path, required=True)
    parser.add_argument("--markdown", type=pathlib.Path)
    arguments = parser.parse_args()
    report = analyze(read_rows(arguments.results))
    arguments.json.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8", newline="\n")
    if arguments.markdown:
        arguments.markdown.write_text(markdown(report), encoding="utf-8", newline="\n")


if __name__ == "__main__":
    main()
