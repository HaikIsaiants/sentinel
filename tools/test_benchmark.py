from __future__ import annotations

import json
import pathlib
import subprocess
import sys
import tempfile
import unittest


TOOLS = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(TOOLS))

import analyze_benchmark
import generate_scenarios
import run_benchmark


def seed_file(root: pathlib.Path) -> pathlib.Path:
    path = root / "seeds.jsonl"
    rows = [
        {"id": "development-nominal-0000", "seed": 11, "stratum": "nominal"},
        {"id": "development-loss-0000", "seed": 19, "stratum": "loss"},
    ]
    path.write_text(
        "".join(json.dumps(row, sort_keys=True) + "\n" for row in rows),
        encoding="utf-8",
        newline="\n",
    )
    return path


class GeneratorTests(unittest.TestCase):
    def test_records_are_validated_and_stable(self):
        with tempfile.TemporaryDirectory() as directory:
            records = generate_scenarios.load_records(seed_file(pathlib.Path(directory)))
            self.assertEqual([record.seed for record in records], [11, 19])
            first = generate_scenarios.generate_scenario(records[0], "nearest")
            second = generate_scenarios.generate_scenario(records[0], "nearest")
            self.assertEqual(first, second)
            self.assertIn('name: "development-nominal-0000"', first)

    def test_paired_scenarios_only_change_allocator(self):
        record = generate_scenarios.SeedRecord("development-nominal-0000", 11, "nominal")
        nearest = generate_scenarios.generate_scenario(record, "nearest")
        cbba = generate_scenarios.generate_scenario(record, "cbba")
        self.assertEqual(
            nearest.replace("ALLOCATION_POLICY_NEAREST_CAPABLE", "POLICY"),
            cbba.replace("ALLOCATION_POLICY_SENTINEL_CBBA", "POLICY"),
        )

    def test_duplicate_seeds_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / "bad.jsonl"
            path.write_text(
                '{"id":"development-nominal-0000","seed":1,"stratum":"nominal"}\n'
                '{"id":"development-loss-0000","seed":1,"stratum":"loss"}\n',
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "duplicate"):
                generate_scenarios.load_records(path)


class RunnerTests(unittest.TestCase):
    def test_plan_is_paired_and_sorted(self):
        records = [
            generate_scenarios.SeedRecord("zulu", 2, "loss"),
            generate_scenarios.SeedRecord("alpha", 1, "nominal"),
        ]
        plan = run_benchmark.mission_plan(records)
        self.assertEqual(
            [mission.key for mission in plan],
            [("alpha", "cbba"), ("alpha", "nearest"), ("zulu", "cbba"), ("zulu", "nearest")],
        )

    def test_run_one_collects_summary(self):
        with tempfile.TemporaryDirectory() as directory:
            scratch = pathlib.Path(directory)
            mission = run_benchmark.Mission(
                generate_scenarios.SeedRecord("development-nominal-0000", 11, "nominal"),
                "nearest",
            )

            def execute(arguments, timeout):
                self.assertEqual(timeout, 5)
                summary = pathlib.Path(arguments[arguments.index("--summary") + 1])
                summary.write_text(
                    json.dumps(
                        {
                            "success": True,
                            "ticks": 42,
                            "completed_tasks": 3,
                            "total_tasks": 3,
                            "terminal_hash": "abc123",
                        }
                    ),
                    encoding="utf-8",
                )
                return subprocess.CompletedProcess(arguments, 0, "", "")

            result = run_benchmark.run_one(
                mission,
                pathlib.Path("runner"),
                pathlib.Path("sim"),
                pathlib.Path("agent"),
                scratch,
                5,
                execute,
            )
            self.assertTrue(result.success)
            self.assertEqual(result.ticks, 42)
            self.assertEqual(result.terminal_hash, "abc123")

    def test_run_one_records_process_failure(self):
        with tempfile.TemporaryDirectory() as directory:
            mission = run_benchmark.Mission(
                generate_scenarios.SeedRecord("development-loss-0000", 19, "loss"),
                "cbba",
            )

            def execute(arguments, timeout):
                return subprocess.CompletedProcess(arguments, 9, "", "agent exited")

            result = run_benchmark.run_one(
                mission,
                pathlib.Path("runner"),
                pathlib.Path("sim"),
                pathlib.Path("agent"),
                pathlib.Path(directory),
                5,
                execute,
            )
            self.assertFalse(result.success)
            self.assertIn("agent exited", result.error)

    def test_rows_round_trip(self):
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / "rows.jsonl"
            rows = [{"seed_id": "one", "allocator": "nearest", "success": True}]
            run_benchmark.write_rows(path, rows)
            self.assertEqual(run_benchmark.read_rows(path), rows)


class AnalyzerTests(unittest.TestCase):
    def rows(self):
        result = []
        for seed, stratum in (("one", "nominal"), ("two", "loss")):
            for allocator, ticks in (("nearest", 20), ("cbba", 15)):
                result.append(
                    {
                        "seed_id": seed,
                        "seed": 1,
                        "stratum": stratum,
                        "allocator": allocator,
                        "success": True,
                        "elapsed_seconds": 0.5,
                        "ticks": ticks,
                        "completed_tasks": 3,
                        "total_tasks": 3,
                        "terminal_hash": f"{seed}-{allocator}",
                        "error": "",
                    }
                )
        return result

    def test_analysis_is_paired(self):
        report = analyze_benchmark.analyze(self.rows())
        self.assertEqual(report["rows"], 4)
        self.assertEqual(report["comparison"]["candidate"], "nearest")
        self.assertEqual(report["comparison"]["pairs"], 2)

    def test_markdown_has_allocator_table(self):
        output = analyze_benchmark.markdown(analyze_benchmark.analyze(self.rows()))
        self.assertIn("| Allocator |", output)
        self.assertIn("| cbba |", output)

    def test_duplicate_rows_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / "rows.jsonl"
            row = self.rows()[0]
            path.write_text(
                json.dumps(row) + "\n" + json.dumps(row) + "\n",
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "duplicate"):
                analyze_benchmark.read_rows([path])


if __name__ == "__main__":
    unittest.main()
