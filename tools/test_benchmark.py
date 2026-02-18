from __future__ import annotations

import json
import pathlib
import subprocess
import sys
import tempfile
import unittest
from unittest import mock


TOOLS = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(TOOLS))

import analyze_benchmark
import generate_scenarios
import run_benchmark
import run_manifest


def records():
    return [
        generate_scenarios.SeedRecord("development-nominal-0000", 11, "nominal"),
        generate_scenarios.SeedRecord("development-loss-0000", 19, "loss"),
    ]


def seed_path(root: pathlib.Path) -> pathlib.Path:
    path = root / "seeds.jsonl"
    path.write_text(
        "".join(
            json.dumps({"id": value.identifier, "seed": value.seed, "stratum": value.stratum}) + "\n"
            for value in records()
        ),
        encoding="utf-8",
    )
    return path


class GeneratorTests(unittest.TestCase):
    def test_named_draws_do_not_shift(self):
        first = generate_scenarios.Draws(123)
        original = first.integer("tasks/7/x", 0, 100)
        for index in range(100):
            first.integer(f"unrelated/{index}", 0, 100)
        self.assertEqual(original, first.integer("tasks/7/x", 0, 100))

    def test_paired_digest_ignores_only_policy(self):
        config = generate_scenarios.read_config(None)
        record = records()[0]
        self.assertEqual(len(generate_scenarios.paired_digest(record, config)), 64)
        nearest = generate_scenarios.generate_scenario(record, "nearest", config)
        cbba = generate_scenarios.generate_scenario(record, "cbba", config)
        self.assertNotEqual(nearest, cbba)

    def test_configuration_is_validated(self):
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / "config.json"
            path.write_text('{"world":{},"mission":{},"strata":{}}', encoding="utf-8")
            with self.assertRaises((KeyError, ValueError)):
                generate_scenarios.read_config(path)

    def test_seed_records_require_known_stratum(self):
        with tempfile.TemporaryDirectory() as directory:
            path = seed_path(pathlib.Path(directory))
            config = generate_scenarios.read_config(None)
            self.assertEqual(len(generate_scenarios.load_records(path, config)), 2)
            path.write_text('{"id":"bad","seed":7,"stratum":"future"}\n', encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "invalid"):
                generate_scenarios.load_records(path, config)


class JournalTests(unittest.TestCase):
    def test_atomic_journal_round_trip(self):
        with tempfile.TemporaryDirectory() as directory:
            journal = run_benchmark.Journal(pathlib.Path(directory) / "journal")
            journal.write({"identity": "one.nearest", "success": True})
            journal.write({"identity": "two.cbba", "success": False})
            self.assertEqual([row["identity"] for row in journal.read()], ["one.nearest", "two.cbba"])
            journal.clear()
            self.assertEqual(journal.read(), [])

    def test_result_shards_have_stable_names(self):
        path = pathlib.Path("results.jsonl")
        self.assertEqual(run_benchmark.result_path(path, 0, 1), path)
        self.assertEqual(
            run_benchmark.result_path(path, 2, 5).name,
            "results.shard-002-of-005.jsonl",
        )

    def test_plan_partitions_without_overlap(self):
        first = run_benchmark.mission_plan(records(), 0, 2)
        second = run_benchmark.mission_plan(records(), 1, 2)
        self.assertFalse({item.identity for item in first} & {item.identity for item in second})
        self.assertEqual(len(first) + len(second), 4)


class TransactionTests(unittest.TestCase):
    def execute(self, arguments, timeout):
        summary = pathlib.Path(arguments[arguments.index("--summary") + 1])
        event_log = pathlib.Path(arguments[arguments.index("--event-log") + 1])
        summary.write_text(
            json.dumps(
                {
                    "success": True,
                    "ticks": 37,
                    "completed_tasks": 9,
                    "total_tasks": 9,
                    "terminal_hash": "stable-hash",
                }
            ),
            encoding="utf-8",
        )
        event_log.write_bytes(b"event-log")
        return subprocess.CompletedProcess(arguments, 0, "", "")

    def test_transaction_publishes_verified_artifacts(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            artifacts = root / "artifacts"
            scratch = root / "scratch"
            scratch.mkdir()
            mission = run_benchmark.Mission(records()[0], "nearest")
            row = run_benchmark.run_transaction(
                mission,
                pathlib.Path("runner"),
                pathlib.Path("sim"),
                pathlib.Path("agent"),
                artifacts,
                scratch,
                generate_scenarios.read_config(None),
                "run-1",
                5,
                self.execute,
            )
            self.assertTrue(row["success"])
            self.assertEqual(len(row["artifacts"]), 3)
            self.assertTrue(run_benchmark.verify_row(row, artifacts, "run-1"))

    def test_failed_transaction_leaves_no_partial_artifacts(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            scratch = root / "scratch"
            scratch.mkdir()

            def fail(arguments, timeout):
                return subprocess.CompletedProcess(arguments, 7, "", "boom")

            row = run_benchmark.run_transaction(
                run_benchmark.Mission(records()[0], "cbba"),
                pathlib.Path("runner"),
                pathlib.Path("sim"),
                pathlib.Path("agent"),
                root / "artifacts",
                scratch,
                generate_scenarios.read_config(None),
                "run-1",
                5,
                fail,
            )
            self.assertFalse(row["success"])
            self.assertIn("boom", row["error"])
            self.assertEqual(list((root / "artifacts").glob("**/*")), [])

    def test_resume_skips_verified_row(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            output = root / "results.jsonl"
            first = run_benchmark.run_all(
                records()[:1],
                output,
                pathlib.Path("runner"),
                pathlib.Path("sim"),
                pathlib.Path("agent"),
                generate_scenarios.read_config(None),
                "run-1",
                5,
                executor=self.execute,
            )
            self.assertEqual(len(first), 2)
            with mock.patch.object(run_benchmark, "run_transaction", side_effect=AssertionError("reran")):
                second = run_benchmark.run_all(
                    records()[:1],
                    output,
                    pathlib.Path("runner"),
                    pathlib.Path("sim"),
                    pathlib.Path("agent"),
                    generate_scenarios.read_config(None),
                    "run-1",
                    5,
                    resume=True,
                    executor=self.execute,
                )
            self.assertEqual(first, second)


class AnalyzerTests(unittest.TestCase):
    def sample(self):
        rows = []
        for index, stratum in enumerate(("nominal", "loss")):
            for allocator, ticks in (("nearest", 30), ("cbba", 20)):
                rows.append(
                    {
                        "identity": f"seed-{index}.{allocator}",
                        "run_id": "run-1",
                        "seed_id": f"seed-{index}",
                        "seed": index,
                        "stratum": stratum,
                        "allocator": allocator,
                        "success": True,
                        "elapsed_seconds": 0.1,
                        "ticks": ticks,
                        "completed_tasks": 4,
                        "total_tasks": 4,
                        "terminal_hash": f"hash-{allocator}",
                        "scenario_sha256": "a" * 64,
                        "artifacts": [],
                        "error": "",
                    }
                )
        return rows

    def test_analysis_is_stratified_and_paired(self):
        report = analyze_benchmark.analyze(self.sample())
        self.assertEqual(set(report["strata"]), {"loss", "nominal"})
        self.assertEqual(report["paired"]["candidate_wins"], 2)
        self.assertLess(report["allocators"]["cbba"]["success_wilson_lower_95"], 1)

    def test_markdown_records_run(self):
        output = analyze_benchmark.markdown(analyze_benchmark.analyze(self.sample()))
        self.assertIn("`run-1`", output)
        self.assertIn("Wilson lower", output)


class ManifestTests(unittest.TestCase):
    def test_source_manifest_detects_change(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            source = root / "source.txt"
            runner = root / "runner"
            simulator = root / "simulator"
            agent = root / "agent"
            source.write_text("first", encoding="utf-8")
            for path in (runner, simulator, agent):
                path.write_text(path.name, encoding="utf-8")
            manifest = run_manifest.create_manifest(root, runner, simulator, agent)
            target = root / "manifest.json"
            run_manifest.write_manifest(target, manifest)
            run_manifest.verify_manifest(target, root)
            source.write_text("second", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "source tree"):
                run_manifest.verify_manifest(target, root)


if __name__ == "__main__":
    unittest.main()
