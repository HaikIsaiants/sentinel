import argparse
import io
import struct
import subprocess
import sys
import threading
import unittest

import run_scenario


class Message:
    def __init__(self, payload=b""):
        self.payload = payload

    def SerializeToString(self, deterministic=False):
        if not deterministic:
            raise AssertionError("messages must use deterministic serialization")
        return self.payload

    def ParseFromString(self, payload):
        self.payload = payload


class BlockingStream:
    def read(self, size):
        threading.Event().wait()


class RunnerTests(unittest.TestCase):
    def test_round_trip_uses_big_endian_framing(self):
        stream = io.BytesIO()
        run_scenario.write_message(stream, Message(b"mission"))
        self.assertEqual(stream.getvalue()[:4], struct.pack(">I", 7))
        stream.seek(0)
        self.assertEqual(
            run_scenario.read_message(stream, Message).payload,
            b"mission",
        )

    def test_oversized_frame_is_rejected(self):
        stream = io.BytesIO(
            struct.pack(">I", run_scenario.MAX_FRAME_BYTES + 1)
        )
        with self.assertRaisesRegex(RuntimeError, "oversized"):
            run_scenario.read_message(stream, Message)

    def test_truncated_frame_is_rejected(self):
        stream = io.BytesIO(struct.pack(">I", 8) + b"short")
        with self.assertRaisesRegex(RuntimeError, "truncated"):
            run_scenario.read_message(stream, Message)

    def test_unresponsive_process_stream_times_out(self):
        timeout = run_scenario.RESPONSE_TIMEOUT_SECONDS
        run_scenario.RESPONSE_TIMEOUT_SECONDS = 0.01
        try:
            with self.assertRaisesRegex(RuntimeError, "timed out"):
                run_scenario.read_message(BlockingStream(), Message)
        finally:
            run_scenario.RESPONSE_TIMEOUT_SECONDS = timeout

    def test_child_error_is_preserved(self):
        process = subprocess.Popen(
            [
                sys.executable,
                "-c",
                "import sys; sys.stderr.write('child failed\\n'); raise SystemExit(3)",
            ],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        self.assertEqual(run_scenario.stop_process(process), "child failed")

    def test_unresponsive_child_is_terminated(self):
        process = subprocess.Popen(
            [sys.executable, "-c", "import time; time.sleep(60)"],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        error = run_scenario.stop_process(process, wait_seconds=0.01)
        self.assertIsNotNone(process.returncode)
        self.assertTrue(error)


def runner_smoke():
    parser = argparse.ArgumentParser()
    parser.add_argument("--runner", required=True)
    options = parser.parse_args()
    result = subprocess.run(
        [options.runner],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        check=False,
    )
    if result.returncode == 0:
        raise SystemExit("runner accepted missing options")


if __name__ == "__main__":
    if "--runner" in sys.argv:
        runner_smoke()
    else:
        unittest.main()
