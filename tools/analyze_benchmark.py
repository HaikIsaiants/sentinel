from __future__ import annotations

import argparse
import json
import math
import pathlib
import statistics
from collections import defaultdict


REQUIRED = {
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
    "error",
}


def read_rows(paths: list[pathlib.Path]) -> list[dict]:
    rows = []
    identities = set()
    for path in paths:
        for number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
            if not line.strip():
                continue
            row = json.loads(line)
            missing = REQUIRED.difference(row)
            if missing:
                raise ValueError(f"{path}:{number} is missing {sorted(missing)}")
            identity = row["seed_id"], row["allocator"]
            if identity in identities:
                raise ValueError(f"duplicate result {identity}")
            identities.add(identity)
            rows.append(row)
    if not rows:
        raise ValueError("no benchmark rows")
    return rows


def percentile(values: list[float], fraction: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    rank = max(0, math.ceil(len(ordered) * fraction) - 1)
    return ordered[rank]


def describe(rows: list[dict]) -> dict:
    elapsed = [float(row["elapsed_seconds"]) for row in rows]
    ticks = [int(row["ticks"]) for row in rows if row["success"]]
    completion = [
        row["completed_tasks"] / row["total_tasks"]
        for row in rows
        if row["total_tasks"]
    ]
    successes = sum(bool(row["success"]) for row in rows)
    return {
        "missions": len(rows),
        "successes": successes,
        "success_rate": successes / len(rows),
        "mean_elapsed_seconds": statistics.fmean(elapsed),
        "p95_elapsed_seconds": percentile(elapsed, 0.95),
        "mean_ticks": statistics.fmean(ticks) if ticks else 0.0,
        "mean_completion_rate": statistics.fmean(completion) if completion else 0.0,
    }


def analyze(rows: list[dict]) -> dict:
    groups = defaultdict(list)
    paired = defaultdict(dict)
    for row in rows:
        groups[row["allocator"]].append(row)
        paired[row["seed_id"]][row["allocator"]] = row
    allocators = sorted(groups)
    complete_pairs = [
        values
        for values in paired.values()
        if set(values) == set(allocators)
    ]
    comparison = {}
    if len(allocators) == 2 and complete_pairs:
        first, second = allocators
        wins = 0
        ties = 0
        for pair in complete_pairs:
            left = pair[first]
            right = pair[second]
            left_score = (bool(left["success"]), -int(left["ticks"]))
            right_score = (bool(right["success"]), -int(right["ticks"]))
            if right_score > left_score:
                wins += 1
            elif right_score == left_score:
                ties += 1
        comparison = {
            "baseline": first,
            "candidate": second,
            "pairs": len(complete_pairs),
            "candidate_wins": wins,
            "ties": ties,
        }
    return {
        "rows": len(rows),
        "allocators": {name: describe(groups[name]) for name in allocators},
        "comparison": comparison,
    }


def markdown(report: dict) -> str:
    lines = ["# Benchmark report", "", f"Rows: {report['rows']}", ""]
    lines.append("| Allocator | Missions | Success | Mean ticks |")
    lines.append("|---|---:|---:|---:|")
    for name, values in report["allocators"].items():
        lines.append(
            f"| {name} | {values['missions']} | {values['success_rate']:.3f} | "
            f"{values['mean_ticks']:.2f} |"
        )
    comparison = report.get("comparison") or {}
    if comparison:
        lines.extend(
            [
                "",
                (
                    f"{comparison['candidate']} beat {comparison['baseline']} on "
                    f"{comparison['candidate_wins']} of {comparison['pairs']} paired missions "
                    f"with {comparison['ties']} ties."
                ),
            ]
        )
    return "\n".join(lines) + "\n"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("results", nargs="+", type=pathlib.Path)
    parser.add_argument("--json", type=pathlib.Path)
    parser.add_argument("--markdown", type=pathlib.Path)
    arguments = parser.parse_args()
    report = analyze(read_rows(arguments.results))
    payload = json.dumps(report, indent=2, sort_keys=True) + "\n"
    if arguments.json:
        arguments.json.write_text(payload, encoding="utf-8", newline="\n")
    else:
        print(payload, end="")
    if arguments.markdown:
        arguments.markdown.write_text(markdown(report), encoding="utf-8", newline="\n")


if __name__ == "__main__":
    main()
