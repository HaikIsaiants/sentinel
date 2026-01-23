import argparse
import json
import pathlib
import struct
import subprocess
import threading


MAX_FRAME_BYTES = 16 * 1024 * 1024
RESPONSE_TIMEOUT_SECONDS = 10


def write_message(stream, message):
    payload = message.SerializeToString(deterministic=True)
    if len(payload) > MAX_FRAME_BYTES:
        raise RuntimeError("protocol frame is too large")
    stream.write(struct.pack(">I", len(payload)))
    stream.write(payload)
    stream.flush()


def read_message(stream, message_type):
    result = []
    failure = []
    complete = threading.Event()

    def receive():
        try:
            header = stream.read(4)
            if len(header) != 4:
                raise RuntimeError("process closed its protocol stream")
            size = struct.unpack(">I", header)[0]
            if size > MAX_FRAME_BYTES:
                raise RuntimeError("process returned an oversized protocol frame")
            payload = stream.read(size)
            if len(payload) != size:
                raise RuntimeError("process returned a truncated protocol frame")
            message = message_type()
            message.ParseFromString(payload)
            result.append(message)
        except Exception as error:
            failure.append(error)
        finally:
            complete.set()

    threading.Thread(target=receive, daemon=True).start()
    if not complete.wait(RESPONSE_TIMEOUT_SECONDS):
        raise RuntimeError("process response timed out")
    if failure:
        raise failure[0]
    return result[0]


def stop_process(process, wait_seconds=5):
    if process.stdin and not process.stdin.closed:
        process.stdin.close()
    try:
        return_code = process.wait(timeout=wait_seconds)
    except subprocess.TimeoutExpired:
        process.terminate()
        try:
            return_code = process.wait(timeout=wait_seconds)
        except subprocess.TimeoutExpired:
            process.kill()
            return_code = process.wait(timeout=wait_seconds)
    error = process.stderr.read() if process.stderr else b""
    if isinstance(error, bytes):
        error = error.decode("utf-8", errors="replace")
    for stream in (process.stdout, process.stderr):
        if stream and not stream.closed:
            stream.close()
    if return_code != 0:
        return error.strip() or f"process exited with {return_code}"
    return ""


def run_once(simulator, agent, scenario, output, run_index):
    from sentinel.v1 import sentinel_pb2

    log_path = output / f"run-{run_index}.events.pb"
    summary_path = output / f"run-{run_index}.json"
    replay_path = output / f"replay-{run_index}.json"
    simulation = subprocess.Popen(
        [
            str(simulator),
            "run",
            "--scenario",
            str(scenario),
            "--log",
            str(log_path),
            "--summary",
            str(summary_path),
        ],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    agents = {}
    summary = None
    run_error = None
    cleanup_errors = []
    current_tick = 0
    try:
        while True:
            frame = read_message(simulation.stdout, sentinel_pb2.SimulationFrame)
            if not frame.HasField("observations"):
                raise RuntimeError("simulator returned an invalid frame")
            observations = frame.observations
            current_tick = observations.tick
            if observations.finished:
                summary = sentinel_pb2.SimulationSummary()
                summary.CopyFrom(observations.summary)
                break
            batch = sentinel_pb2.ActionBatch(tick=current_tick)
            for envelope in sorted(
                observations.observations,
                key=lambda value: value.recipient_id,
            ):
                agent_id = envelope.recipient_id
                if envelope.observation.self.id != agent_id:
                    raise RuntimeError("observation crossed an agent boundary")
                if agent_id not in agents:
                    agents[agent_id] = subprocess.Popen(
                        [str(agent), "--id", agent_id],
                        stdin=subprocess.PIPE,
                        stdout=subprocess.PIPE,
                        stderr=subprocess.PIPE,
                    )
                write_message(agents[agent_id].stdin, envelope)
                action = read_message(agents[agent_id].stdout, sentinel_pb2.Envelope)
                batch.actions.add().CopyFrom(action)
            request = sentinel_pb2.SimulationFrame()
            request.actions.CopyFrom(batch)
            write_message(simulation.stdin, request)
    except Exception as error:
        run_error = RuntimeError(f"tick {current_tick}: {error}")
    finally:
        for process in agents.values():
            error = stop_process(process)
            if error:
                cleanup_errors.append(error)
        error = stop_process(simulation)
        if error:
            cleanup_errors.append(error)
    if run_error:
        raise RuntimeError("; ".join([str(run_error), *cleanup_errors])) from run_error
    if cleanup_errors:
        raise RuntimeError("; ".join(cleanup_errors))
    if summary is None or not summary.success:
        raise RuntimeError("mission did not complete")
    process_ids = {simulation.pid, *(process.pid for process in agents.values())}
    if len(process_ids) != len(agents) + 1:
        raise RuntimeError("agent processes were not isolated")
    replay = subprocess.run(
        [
            str(simulator),
            "replay",
            "--scenario",
            str(scenario),
            "--log",
            str(log_path),
            "--summary",
            str(replay_path),
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    replay_summary = json.loads(replay_path.read_text(encoding="utf-8"))
    if (
        replay.stdout.strip() != summary.terminal_hash
        or replay_summary["terminal_hash"] != summary.terminal_hash
    ):
        raise RuntimeError("event log replay changed the terminal hash")
    return summary, log_path, len(agents)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sim", type=pathlib.Path, required=True)
    parser.add_argument("--agent", type=pathlib.Path, required=True)
    parser.add_argument("--scenario", type=pathlib.Path, required=True)
    parser.add_argument("--output", type=pathlib.Path, required=True)
    parser.add_argument("--repeat", type=int, default=3)
    args = parser.parse_args()
    if args.repeat < 2:
        raise RuntimeError("repeat must be at least two")
    args.output.mkdir(parents=True, exist_ok=True)
    results = [
        run_once(args.sim, args.agent, args.scenario, args.output, run_index)
        for run_index in range(args.repeat)
    ]
    expected_hashes = list(results[0][0].tick_hashes)
    expected_log = results[0][1].read_bytes()
    for summary, log_path, agent_count in results[1:]:
        if (
            summary.terminal_hash != results[0][0].terminal_hash
            or list(summary.tick_hashes) != expected_hashes
        ):
            raise RuntimeError("repeated simulation changed state hashes")
        if log_path.read_bytes() != expected_log:
            raise RuntimeError("repeated simulation changed event log bytes")
        if agent_count != results[0][2]:
            raise RuntimeError("repeated simulation changed agent count")
    verification = {
        "agent_count": results[0][2],
        "event_log_bytes": len(expected_log),
        "repeat_count": args.repeat,
        "terminal_hash": results[0][0].terminal_hash,
        "tick_count": results[0][0].ticks,
    }
    (args.output / "verification.json").write_text(
        json.dumps(verification, indent=2) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(verification, sort_keys=True))


if __name__ == "__main__":
    main()
