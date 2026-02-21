from __future__ import annotations

import json
import pathlib
import tempfile
import unittest


TOOLS = pathlib.Path(__file__).resolve().parent
import sys

sys.path.insert(0, str(TOOLS))

import analyze_benchmark
import benchmark_io
import benchmark_runtime
import generate_scenarios
import run_benchmark
import run_manifest


RUN_ID = "a" * 64


class FakeWorker:
    def __init__(self):
        self.requests = []
        self.closed = False

    def execute(self, request, timeout):
        self.requests.append(request)
        output = pathlib.Path(request["output"])
        (output / "summary.json").write_text("{}", encoding="utf-8")
        (output / "events.pb").write_bytes(b"events")
        return {
            "request_id": request["request_id"],
            "ok": True,
            "summary": {
                "success": True,
                "ticks": 50,
                "completed_tasks": 10,
                "total_tasks": 10,
                "terminal_hash": "stable",
            },
            "artifacts": ["summary.json", "events.pb"],
        }

    def close(self, force=False):
        self.closed = True


def seed_records():
    return [
        generate_scenarios.SeedRecord("development-nominal-0000", 11, "nominal"),
        generate_scenarios.SeedRecord("development-loss-0000", 19, "loss"),
    ]


class FrameTests(unittest.TestCase):
    def test_decoder_accepts_fragmented_and_coalesced_frames(self):
        first = benchmark_runtime.encode_frame({"request_id": "one"})
        second = benchmark_runtime.encode_frame({"request_id": "two"})
        decoder = benchmark_runtime.FrameDecoder()
        self.assertEqual(decoder.feed(first[:2]), [])
        self.assertEqual(decoder.feed(first[2:] + second), [{"request_id": "one"}, {"request_id": "two"}])
        decoder.finish()

    def test_decoder_rejects_truncated_frame(self):
        decoder = benchmark_runtime.FrameDecoder()
        decoder.feed(benchmark_runtime.encode_frame({"value": 1})[:-1])
        with self.assertRaisesRegex(ValueError, "truncated"):
            decoder.finish()

    def test_decoder_rejects_oversized_frame(self):
        decoder = benchmark_runtime.FrameDecoder(maximum=2)
        with self.assertRaisesRegex(ValueError, "size"):
            decoder.feed(benchmark_runtime.encode_frame({"value": 1}))


class WorkerPoolTests(unittest.TestCase):
    def test_pool_reuses_then_recycles_workers(self):
        created = []

        def factory():
            worker = FakeWorker()
            created.append(worker)
            return worker

        pool = benchmark_runtime.WorkerPool(1, factory, recycle_after=2)
        with tempfile.TemporaryDirectory() as directory:
            output = pathlib.Path(directory)
            for index in range(3):
                response = pool.execute(
                    {
                        "request_id": str(index),
                        "output": str(output),
                    },
                    1,
                )
                self.assertTrue(response["ok"])
        self.assertEqual(len(created), 2)
        self.assertTrue(created[0].closed)
        pool.close()
        self.assertTrue(created[1].closed)

    def test_pool_replaces_failed_worker(self):
        class Broken(FakeWorker):
            def execute(self, request, timeout):
                raise RuntimeError("broken")

        created = [Broken(), FakeWorker()]
        pool = benchmark_runtime.WorkerPool(1, lambda: created.pop(0))
        with self.assertRaisesRegex(RuntimeError, "broken"):
            pool.execute({"request_id": "x"}, 1)
        pool.close()


class GeneratorTests(unittest.TestCase):
    def test_model_is_deterministic_and_valid(self):
        config = generate_scenarios.read_config(None)
        first = generate_scenarios.build_model(seed_records()[0], config)
        second = generate_scenarios.build_model(seed_records()[0], config)
        self.assertEqual(first, second)
        generate_scenarios.validate_model(first)

    def test_named_draws_are_independent(self):
        draws = generate_scenarios.Draws("test", 7)
        expected = draws.integer("tasks/4/x", [0, 100])
        for index in range(100):
            draws.integer(f"unrelated/{index}", [0, 100])
        self.assertEqual(draws.integer("tasks/4/x", [0, 100]), expected)

    def test_paired_digest_normalizes_policy(self):
        config = generate_scenarios.read_config(None)
        self.assertEqual(len(generate_scenarios.paired_digest(seed_records()[0], config)), 64)


class RunnerTests(unittest.TestCase):
    def pool(self):
        return benchmark_runtime.WorkerPool(1, FakeWorker)

    def test_persistent_run_publishes_and_resumes(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            output = root / "rows.jsonl"
            config = generate_scenarios.read_config(None)
            pool = self.pool()
            first = run_benchmark.run_all(seed_records()[:1], output, config, RUN_ID, pool, 2)
            self.assertEqual(len(first), 2)
            record = benchmark_io.read_output_record(output, RUN_ID)
            self.assertEqual(record["rows_sha256"], benchmark_io.file_digest(output))
            pool.close()
            second_pool = self.pool()
            second = run_benchmark.run_all(
                seed_records()[:1],
                output,
                config,
                RUN_ID,
                second_pool,
                2,
                resume=True,
            )
            self.assertEqual(first, second)
            self.assertEqual(second_pool.slots[0].worker.requests, [])
            second_pool.close()

    def test_tampered_artifact_is_not_resumed(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            output = root / "rows.jsonl"
            config = generate_scenarios.read_config(None)
            pool = self.pool()
            run_benchmark.run_all(seed_records()[:1], output, config, RUN_ID, pool, 2, limit=1)
            pool.close()
            event = next(output.with_suffix(output.suffix + ".artifacts").glob("**/events.pb"))
            event.write_bytes(b"tampered")
            replacement = self.pool()
            run_benchmark.run_all(
                seed_records()[:1],
                output,
                config,
                RUN_ID,
                replacement,
                2,
                limit=1,
                resume=True,
            )
            self.assertEqual(len(replacement.slots[0].worker.requests), 1)
            replacement.close()

    def test_plan_shards_without_overlap(self):
        first = run_benchmark.plan(seed_records(), 0, 2, None)
        second = run_benchmark.plan(seed_records(), 1, 2, None)
        self.assertFalse({value.identity for value in first} & {value.identity for value in second})


class AnalyzerTests(unittest.TestCase):
    def rows(self):
        rows = []
        for index, stratum in enumerate(("nominal", "loss")):
            for allocator, ticks in (("nearest", 60), ("cbba", 40)):
                rows.append(
                    {
                        "identity": f"seed-{index}.{allocator}",
                        "run_id": RUN_ID,
                        "seed_id": f"seed-{index}",
                        "seed": index,
                        "stratum": stratum,
                        "allocator": allocator,
                        "success": True,
                        "elapsed_seconds": 0.2,
                        "ticks": ticks,
                        "artifacts": [],
                        "summary": {name: True for name in analyze_benchmark.INVARIANTS},
                        "error": "",
                    }
                )
        return rows

    def test_invariants_affect_effective_success(self):
        rows = self.rows()
        rows[0]["summary"][analyze_benchmark.INVARIANTS[0]] = False
        report = analyze_benchmark.analyze(rows)
        self.assertEqual(report["allocators"]["nearest"]["successes"], 1)

    def test_artifact_failure_is_recorded(self):
        rows = self.rows()
        report = analyze_benchmark.analyze(rows, [rows[0]["identity"]])
        self.assertEqual(report["artifact_failure_ids"], [rows[0]["identity"]])
        self.assertIn("Artifact failures: 1", analyze_benchmark.markdown(report))

    def test_artifact_paths_cannot_escape_root(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            rows = self.rows()
            rows[0]["artifact_root"] = root.name
            rows[0]["artifacts"] = [{"path": "../escape", "sha256": "0" * 64, "bytes": 1}]
            self.assertEqual(analyze_benchmark.audit_artifacts(rows, [root]), [rows[0]["identity"]])


class ManifestTests(unittest.TestCase):
    def test_manifest_and_output_are_bound_to_run(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            source = root / "source.txt"
            runner = root / "runner"
            simulator = root / "simulator"
            agent = root / "agent"
            output = root / "rows.jsonl"
            source.write_text("source", encoding="utf-8")
            output.write_text("{}\n", encoding="utf-8")
            for path in (runner, simulator, agent):
                path.write_text(path.name, encoding="utf-8")
            manifest = run_manifest.create_manifest(root, runner, simulator, agent, output, 2)
            target = root / "manifest.json"
            run_manifest.write_manifest(target, manifest)
            run_manifest.verify_manifest(target, root, output)
            benchmark_io.write_output_record(
                output,
                manifest["run_id"],
                benchmark_io.file_digest(output),
                "b" * 64,
            )
            run_manifest.verify_output(output, manifest["run_id"])


if __name__ == "__main__":
    unittest.main()
