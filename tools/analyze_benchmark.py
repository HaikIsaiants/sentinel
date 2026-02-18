from __future__ import annotations

import argparse
import json
import math
import pathlib
import statistics
from collections import defaultdict

from benchmark_io import contained, file_digest


ALLOCATORS = ("nearest", "cbba")


def read_rows(paths: list[pathlib.Path]) -> list[dict]:
    rows = []
    identities = set()
    run_ids = set()
    for path in paths:
        for number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
            if not line.strip():
                continue
            row = json.loads(line)
            required = {
                "identity",
                "run_id",
                "seed_id",
                "stratum",
                "allocator",
                "success",
                "ticks",
                "elapsed_seconds",
                "artifacts",
                "attempt_processes",
            }
            if not isinstance(row, dict) or required.difference(row):
                raise ValueError(f"invalid benchmark row at {path}:{number}")
            identity = row["run_id"], row["identity"]
            if identity in identities:
                raise ValueError(f"duplicate benchmark row {identity}")
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
    ordered = sorted(values)
    return ordered[max(0, math.ceil(len(ordered) * fraction) - 1)]


def describe(rows: list[dict]) -> dict:
    successes = [row for row in rows if row["success"] and not row.get("error")]
    ticks = [row["ticks"] for row in successes]
    elapsed = [float(row["elapsed_seconds"]) for row in rows]
    threads = {row.get("dispatch_thread") for row in rows if row.get("dispatch_thread")}
    return {
        "missions": len(rows),
        "successes": len(successes),
        "success_rate": len(successes) / len(rows),
        "success_wilson_lower_95": wilson_lower(len(successes), len(rows)),
        "mean_ticks": statistics.fmean(ticks) if ticks else 0.0,
        "p95_ticks": nearest_rank(ticks, 0.95),
        "mean_elapsed_seconds": statistics.fmean(elapsed),
        "p95_elapsed_seconds": nearest_rank(elapsed, 0.95),
        "attempt_processes": sum(row["attempt_processes"] for row in rows),
        "dispatch_threads": len(threads),
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
        left = (bool(nearest["success"]), -nearest["ticks"])
        right = (bool(cbba["success"]), -cbba["ticks"])
        if right > left:
            wins += 1
        elif right < left:
            losses += 1
        else:
            ties += 1
        if nearest["success"] and cbba["success"]:
            deltas.append(nearest["ticks"] - cbba["ticks"])
    return {
        "pairs": len(pairs),
        "candidate_wins": wins,
        "baseline_wins": losses,
        "ties": ties,
        "mean_tick_improvement": statistics.fmean(deltas) if deltas else 0.0,
    }


def audit_artifacts(rows: list[dict], root: pathlib.Path) -> list[str]:
    failures = []
    for row in rows:
        for artifact in row["artifacts"]:
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


def analyze(rows: list[dict], artifact_failures: list[str] = ()) -> dict:
    failures = set(artifact_failures)
    effective = []
    for row in rows:
        value = dict(row)
        if row["identity"] in failures:
            value["success"] = False
            value["error"] = "artifact verification failed"
        effective.append(value)
    allocators = defaultdict(list)
    strata = defaultdict(lambda: defaultdict(list))
    for row in effective:
        if row["allocator"] not in ALLOCATORS:
            raise ValueError("unexpected allocator")
        allocators[row["allocator"]].append(row)
        strata[row["stratum"]][row["allocator"]].append(row)
    if set(allocators) != set(ALLOCATORS):
        raise ValueError("both allocators are required")
    return {
        "schema_version": 3,
        "run_id": effective[0]["run_id"],
        "rows": len(effective),
        "artifact_failure_ids": sorted(failures),
        "allocators": {name: describe(allocators[name]) for name in ALLOCATORS},
        "strata": {
            stratum: {name: describe(values[name]) for name in ALLOCATORS if values[name]}
            for stratum, values in sorted(strata.items())
        },
        "paired": paired(effective),
    }


def markdown(report: dict) -> str:
    lines = [
        "# Bounded-parallel benchmark report",
        "",
        f"Run: `{report['run_id']}`",
        "",
        "| Allocator | Missions | Success | Mean ticks | P95 elapsed | Processes |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for name, value in report["allocators"].items():
        lines.append(
            f"| {name} | {value['missions']} | {value['success_rate']:.4f} | "
            f"{value['mean_ticks']:.2f} | {value['p95_elapsed_seconds']:.3f} | {value['attempt_processes']} |"
        )
    pair = report["paired"]
    lines.extend(
        [
            "",
            f"Paired missions: {pair['pairs']}",
            "",
            f"CBBA wins: {pair['candidate_wins']}; nearest wins: {pair['baseline_wins']}; ties: {pair['ties']}.",
        ]
    )
    return "\n".join(lines) + "\n"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("results", nargs="+", type=pathlib.Path)
    parser.add_argument("--artifact-root", type=pathlib.Path)
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
