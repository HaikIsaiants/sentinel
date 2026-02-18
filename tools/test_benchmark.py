from __future__ import annotations

import json
import pathlib
import subprocess
import sys
import tempfile
import threading
import time
import unittest


TOOLS = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(TOOLS))

import analyze_benchmark
import benchmark_io
import benchmark_runtime
import generate_scenarios
import run_benchmark
import run_manifest


RUN_ID = "a" * 64


def records():
    return [
        generate_scenarios.SeedRecord("development-nominal-0000", 11, "nominal"),
        generate_scenarios.SeedRecord("development-loss-0000", 19, "loss"),
    ]


class GeneratorTests(unittest.TestCase):
    def test_named_draws_are_stable(self):
        draws = generate_scenarios.Draws("sentinel-scenario-v1", 11)
        expected = draws.integer("tasks/4/x", [0, 100])
        for index in range(100):
            draws.integer(f"unrelated/{index}", [0, 100])
        self.assertEqual(expected, draws.integer("tasks/4/x", [0, 100]))

    def test_pair_differs_only_by_policy(self):
        config = generate_scenarios.read_config(None)
        record = records()[0]
        self.assertEqual(len(generate_scenarios.paired_digest(record, config)), 64)
        nearest = generate_scenarios.generate_scenario(record, "nearest", config)
        cbba = generate_scenarios.generate_scenario(record, "cbba", config)
        self.assertEqual(
            nearest.replace("ALLOCATION_POLICY_NEAREST_CAPABLE", "POLICY"),
            cbba.replace("ALLOCATION_POLICY_SENTINEL_CBBA", "POLICY"),
        )


class ParallelRuntimeTests(unittest.TestCase):
    def executor(self, arguments, timeout):
        scenario = pathlib.Path(arguments[arguments.index("--scenario") + 1])
        summary = pathlib.Path(arguments[arguments.index("--summary") + 1])
        event_log = pathlib.Path(arguments[arguments.index("--event-log") + 1])
        if "slow" in scenario.read_text(encoding="utf-8"):
            time.sleep(0.05)
        summary.write_text(
            json.dumps(
                {
                    "success": True,
                    "ticks": 50,
                    "completed_tasks": 10,
                    "total_tasks": 10,
                    "terminal_hash": "stable",
                }
            ),
            encoding="utf-8",
        )
        event_log.write_bytes(b"events")
        return subprocess.CompletedProcess(arguments, 0, "", "")

    def test_dispatch_is_bounded_and_completion_order_can_vary(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            scenarios = []
            for name, value in (("slow", "slow"), ("fast", "fast")):
                scenario = root / f"{name}.textproto"
                scenario.write_text(value, encoding="utf-8")
                scenarios.append(benchmark_runtime.Attempt(name, scenario, root / "artifacts" / name))
            dispatcher = benchmark_runtime.ParallelDispatcher(
                2,
                pathlib.Path("runner"),
                pathlib.Path("sim"),
                pathlib.Path("agent"),
                root / "scratch",
                2,
                self.executor,
            )
            completed = dispatcher.dispatch(scenarios)
            self.assertEqual({row["identity"] for row in completed}, {"fast", "slow"})
            self.assertEqual(len({row["thread_id"] for row in completed}), 2)

    def test_process_failure_is_a_result_not_pool_failure(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            scenario = root / "scenario.textproto"
            scenario.write_text("mission", encoding="utf-8")

            def fail(arguments, timeout):
                return subprocess.CompletedProcess(arguments, 7, "", "boom")

            dispatcher = benchmark_runtime.ParallelDispatcher(
                1,
                pathlib.Path("runner"),
                pathlib.Path("sim"),
                pathlib.Path("agent"),
                root / "scratch",
                2,
                fail,
            )
            result = dispatcher.dispatch([benchmark_runtime.Attempt("one", scenario, root / "artifacts" / "one")])[0]
            self.assertFalse(result["ok"])
            self.assertIn("boom", result["error"])

    def test_duplicate_attempts_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            scenario = root / "scenario"
            scenario.write_text("x", encoding="utf-8")
            dispatcher = benchmark_runtime.ParallelDispatcher(
                1,
                pathlib.Path("runner"),
                pathlib.Path("sim"),
                pathlib.Path("agent"),
                root / "scratch",
                1,
                self.executor,
            )
            attempt = benchmark_runtime.Attempt("same", scenario, root / "target")
            with self.assertRaisesRegex(ValueError, "duplicate"):
                dispatcher.dispatch([attempt, attempt])


class RunnerTests(unittest.TestCase):
    def executor(self, arguments, timeout):
        summary = pathlib.Path(arguments[arguments.index("--summary") + 1])
        event_log = pathlib.Path(arguments[arguments.index("--event-log") + 1])
        summary.write_text(
            json.dumps(
                {
                    "success": True,
                    "ticks": 40,
                    "completed_tasks": 10,
                    "total_tasks": 10,
                    "terminal_hash": "hash",
                }
            ),
            encoding="utf-8",
        )
        event_log.write_bytes(b"log")
        return subprocess.CompletedProcess(arguments, 0, "", "")

    def dispatcher(self, root):
        return benchmark_runtime.ParallelDispatcher(
            2,
            pathlib.Path("runner"),
            pathlib.Path("sim"),
            pathlib.Path("agent"),
            root / "scratch",
            2,
            self.executor,
        )

    def test_publication_order_is_deterministic(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            output = root / "rows.jsonl"
            rows = run_benchmark.run_all(
                list(reversed(records())),
                output,
                generate_scenarios.read_config(None),
                RUN_ID,
                self.dispatcher(root),
            )
            identities = [row["identity"] for row in rows]
            self.assertEqual(identities, sorted(identities))
            disk = [json.loads(line)["identity"] for line in output.read_text(encoding="utf-8").splitlines()]
            self.assertEqual(disk, sorted(disk))

    def test_resume_skips_verified_attempts(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            output = root / "rows.jsonl"
            first = run_benchmark.run_all(
                records()[:1],
                output,
                generate_scenarios.read_config(None),
                RUN_ID,
                self.dispatcher(root),
            )

            class EmptyDispatcher:
                def dispatch(self, attempts):
                    self.attempts = attempts
                    return []

            empty = EmptyDispatcher()
            second = run_benchmark.run_all(
                records()[:1],
                output,
                generate_scenarios.read_config(None),
                RUN_ID,
                empty,
                resume=True,
            )
            self.assertEqual(first, second)
            self.assertEqual(empty.attempts, [])

    def test_shards_do_not_overlap(self):
        first = run_benchmark.plan(records(), 0, 2, None)
        second = run_benchmark.plan(records(), 1, 2, None)
        self.assertFalse({run_benchmark.identity(*value) for value in first} & {run_benchmark.identity(*value) for value in second})


class AnalyzerTests(unittest.TestCase):
    def rows(self):
        result = []
        for index, stratum in enumerate(("nominal", "loss")):
            for allocator, ticks in (("nearest", 60), ("cbba", 40)):
                result.append(
                    {
                        "identity": f"seed-{index}.{allocator}",
                        "run_id": RUN_ID,
                        "seed_id": f"seed-{index}",
                        "stratum": stratum,
                        "allocator": allocator,
                        "success": True,
                        "ticks": ticks,
                        "elapsed_seconds": 0.2,
                        "artifacts": [],
                        "attempt_processes": 1,
                        "dispatch_thread": threading.get_ident(),
                        "error": "",
                    }
                )
        return result

    def test_parallel_process_counts_are_reported(self):
        report = analyze_benchmark.analyze(self.rows())
        self.assertEqual(report["allocators"]["nearest"]["attempt_processes"], 2)
        self.assertEqual(report["paired"]["candidate_wins"], 2)

    def test_artifact_path_escape_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            rows = self.rows()
            rows[0]["artifacts"] = [{"path": "../escape", "sha256": "0" * 64, "bytes": 1}]
            self.assertEqual(analyze_benchmark.audit_artifacts(rows, pathlib.Path(directory)), [rows[0]["identity"]])


class ManifestTests(unittest.TestCase):
    def test_manifest_records_bounded_dispatch(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            source = root / "source"
            runner = root / "runner"
            simulator = root / "simulator"
            agent = root / "agent"
            output = root / "rows.jsonl"
            for path in (source, runner, simulator, agent):
                path.write_text(path.name, encoding="utf-8")
            manifest = run_manifest.create_manifest(root, runner, simulator, agent, output, 3, 45)
            self.assertEqual(manifest["dispatch"]["strategy"], "bounded-subprocess")
            self.assertEqual(manifest["dispatch"]["workers"], 3)
            target = root / "manifest.json"
            run_manifest.write_manifest(target, manifest)
            run_manifest.verify_manifest(target, root, output)


if __name__ == "__main__":
    unittest.main()
