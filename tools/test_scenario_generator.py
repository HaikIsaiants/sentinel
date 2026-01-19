from __future__ import annotations

import json
import os
import pathlib
import subprocess
import sys
import tempfile
import unittest


TOOLS = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(TOOLS))

import generate_scenarios


class ScenarioGeneratorTests(unittest.TestCase):
    def record(self, stratum="nominal"):
        return generate_scenarios.SeedRecord(f"development-{stratum}-0000", 11, stratum)

    def test_scripted_generation_is_deterministic(self):
        first = generate_scenarios.generate_scenario(self.record())
        second = generate_scenarios.generate_scenario(self.record())
        self.assertEqual(first, second)
        self.assertIn("ALLOCATION_POLICY_SCRIPTED", first)
        self.assertIn('assigned_agent_id: "alpha"', first)

    def test_generator_has_no_allocator_comparison_vocabulary(self):
        source = pathlib.Path(generate_scenarios.__file__).read_text(encoding="utf-8").upper()
        self.assertNotIn("NEAR" + "EST", source)
        self.assertNotIn("CB" + "BA", source)

    def test_disruptions_are_added_only_for_requested_stratum(self):
        nominal = generate_scenarios.generate_scenario(self.record())
        blocked = generate_scenarios.generate_scenario(self.record("blocked_route"))
        self.assertNotIn("close-route", nominal)
        self.assertIn("close-route", blocked)

    def test_every_supported_stratum_matches_the_current_proto(self):
        repository = pathlib.Path(
            os.environ.get("SENTINEL_PROTO_ROOT", TOOLS.parent)
        )
        proto_root = repository / "proto"
        proto = proto_root / "sentinel" / "v1" / "sentinel.proto"
        for stratum in sorted(generate_scenarios.STRATA):
            scenario = generate_scenarios.generate_scenario(self.record(stratum))
            completed = subprocess.run(
                [
                    "protoc",
                    "-I",
                    str(proto_root),
                    "--encode=sentinel.v1.Scenario",
                    str(proto),
                ],
                input=scenario,
                text=True,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
                check=False,
            )
            self.assertEqual(
                completed.returncode,
                0,
                f"{stratum}: {completed.stderr}",
            )

    def test_seed_corpus_validation(self):
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / "seeds.jsonl"
            path.write_text(
                json.dumps({"id": "development-nominal-0000", "seed": 11, "stratum": "nominal"}) + "\n",
                encoding="utf-8",
            )
            self.assertEqual(generate_scenarios.load_records(path), [self.record()])
            path.write_text(
                '{"id":"one","seed":11,"stratum":"nominal"}\n'
                '{"id":"two","seed":11,"stratum":"latency"}\n',
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "duplicate"):
                generate_scenarios.load_records(path)


if __name__ == "__main__":
    unittest.main()
