from __future__ import annotations

import json
import os
import pathlib
import queue
import struct
import subprocess
import threading
import time
from dataclasses import dataclass
from typing import Callable

from benchmark_io import canonical_bytes


MAX_FRAME_BYTES = 32 * 1024 * 1024


def encode_frame(value: dict) -> bytes:
    payload = canonical_bytes(value).rstrip(b"\n")
    if not payload or len(payload) > MAX_FRAME_BYTES:
        raise ValueError("invalid frame payload size")
    return struct.pack(">I", len(payload)) + payload


class FrameDecoder:
    def __init__(self, maximum: int = MAX_FRAME_BYTES):
        self.maximum = maximum
        self.buffer = bytearray()
        self.expected = None

    def feed(self, value: bytes) -> list[dict]:
        self.buffer.extend(value)
        messages = []
        while True:
            if self.expected is None:
                if len(self.buffer) < 4:
                    break
                self.expected = struct.unpack(">I", self.buffer[:4])[0]
                del self.buffer[:4]
                if self.expected == 0 or self.expected > self.maximum:
                    raise ValueError("invalid frame size")
            if len(self.buffer) < self.expected:
                break
            payload = bytes(self.buffer[: self.expected])
            del self.buffer[: self.expected]
            self.expected = None
            value = json.loads(payload)
            if not isinstance(value, dict):
                raise ValueError("worker frame is not an object")
            messages.append(value)
        return messages

    def finish(self) -> None:
        if self.buffer or self.expected is not None:
            raise ValueError("truncated worker frame")


class StreamWorker:
    """Client for a persistent JSON-framed child process."""

    def __init__(self, command: list[str]):
        self.command = command
        self.process = subprocess.Popen(
            command,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            creationflags=getattr(subprocess, "CREATE_NEW_PROCESS_GROUP", 0),
        )
        self.responses = queue.Queue()
        self.errors = queue.Queue()
        self.decoder = FrameDecoder()
        self.reader = threading.Thread(target=self._read, daemon=True)
        self.reader.start()

    def _read(self) -> None:
        try:
            while True:
                block = self.process.stdout.read(65536)
                if not block:
                    break
                for message in self.decoder.feed(block):
                    self.responses.put(message)
            self.decoder.finish()
            if self.process.poll() not in (None, 0):
                error = self.process.stderr.read().decode(errors="replace")
                raise RuntimeError(error or f"worker exited {self.process.returncode}")
        except Exception as error:
            self.errors.put(error)

    def execute(self, request: dict, timeout: float) -> dict:
        if self.process.poll() is not None:
            raise RuntimeError("worker is not running")
        payload = encode_frame(request)
        self.process.stdin.write(payload)
        self.process.stdin.flush()
        deadline = time.monotonic() + timeout
        while True:
            try:
                raise self.errors.get_nowait()
            except queue.Empty:
                pass
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("persistent worker timed out")
            try:
                response = self.responses.get(timeout=min(remaining, 0.1))
            except queue.Empty:
                continue
            if response.get("request_id") != request.get("request_id"):
                raise RuntimeError("worker response identity mismatch")
            return response

    def close(self, force: bool = False) -> None:
        if self.process.poll() is not None:
            return
        if not force:
            try:
                self.process.stdin.write(encode_frame({"type": "shutdown"}))
                self.process.stdin.flush()
                self.process.wait(timeout=2)
                return
            except (OSError, subprocess.TimeoutExpired):
                pass
        self.process.kill()
        self.process.wait(timeout=5)


@dataclass
class Slot:
    worker: object
    uses: int = 0
    failures: int = 0


class WorkerPool:
    def __init__(self, size: int, factory: Callable[[], object], recycle_after: int = 100):
        if size <= 0 or recycle_after <= 0:
            raise ValueError("invalid worker pool configuration")
        self.factory = factory
        self.recycle_after = recycle_after
        self.slots = [Slot(factory()) for _ in range(size)]
        self.cursor = 0

    def _replace(self, index: int, force: bool) -> None:
        slot = self.slots[index]
        try:
            slot.worker.close(force=force)
        finally:
            self.slots[index] = Slot(self.factory(), failures=slot.failures)

    def execute(self, request: dict, timeout: float) -> dict:
        index = self.cursor
        self.cursor = (self.cursor + 1) % len(self.slots)
        slot = self.slots[index]
        if slot.uses >= self.recycle_after:
            self._replace(index, force=False)
            slot = self.slots[index]
        try:
            response = slot.worker.execute(request, timeout)
            slot.uses += 1
            if response.get("ok") is not True:
                raise RuntimeError(response.get("error") or "worker rejected mission")
            return response
        except Exception:
            slot.failures += 1
            self._replace(index, force=True)
            raise

    def close(self) -> None:
        errors = []
        for slot in self.slots:
            try:
                slot.worker.close(force=False)
            except Exception as error:
                errors.append(error)
        if errors:
            raise RuntimeError("; ".join(map(str, errors)))


def worker_command(
    runner: pathlib.Path,
    simulator: pathlib.Path,
    agent: pathlib.Path,
    scratch: pathlib.Path,
) -> list[str]:
    return [
        str(runner),
        "--worker",
        "--simulator",
        str(simulator),
        "--agent",
        str(agent),
        "--scratch",
        str(scratch),
    ]


def request(identifier: str, scenario: pathlib.Path, output: pathlib.Path) -> dict:
    return {
        "type": "mission",
        "request_id": identifier,
        "scenario": str(scenario.resolve()),
        "output": str(output.resolve()),
    }


def validate_response(value: dict, identifier: str) -> dict:
    if value.get("request_id") != identifier or value.get("ok") is not True:
        raise ValueError("invalid worker response")
    summary = value.get("summary")
    if not isinstance(summary, dict):
        raise ValueError("worker response has no summary")
    required = ("success", "ticks", "completed_tasks", "total_tasks", "terminal_hash")
    if any(name not in summary for name in required):
        raise ValueError("worker summary is incomplete")
    if (
        not isinstance(summary["success"], bool)
        or any(not isinstance(summary[name], int) or summary[name] < 0 for name in required[1:4])
        or not isinstance(summary["terminal_hash"], str)
        or not summary["terminal_hash"]
    ):
        raise ValueError("worker summary is invalid")
    artifacts = value.get("artifacts")
    if not isinstance(artifacts, list) or not artifacts:
        raise ValueError("worker artifacts are missing")
    return value
