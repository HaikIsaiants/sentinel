from __future__ import annotations

import hashlib
import json
import pathlib
import re


DIGEST = re.compile(r"^[0-9a-f]{64}$")


def canonical_bytes(value) -> bytes:
    return (json.dumps(value, sort_keys=True, separators=(",", ":")) + "\n").encode()


def file_digest(path: pathlib.Path) -> str:
    value = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(block)
    return value.hexdigest()


def validate_digest(value: str) -> str:
    if not isinstance(value, str) or not DIGEST.fullmatch(value):
        raise ValueError("invalid SHA-256 digest")
    return value


def contained(root: pathlib.Path, relative: str) -> pathlib.Path:
    if not relative or pathlib.PurePosixPath(relative).is_absolute():
        raise ValueError("artifact path must be relative")
    target = (root / pathlib.PurePosixPath(relative)).resolve()
    try:
        target.relative_to(root.resolve())
    except ValueError as error:
        raise ValueError("artifact path escapes output root") from error
    return target


def output_record_path(path: pathlib.Path, run_id: str) -> pathlib.Path:
    validate_digest(run_id)
    return path.with_name(f"{path.name}.{run_id}.record.json")


def write_output_record(path: pathlib.Path, run_id: str, rows_sha256: str, artifacts_sha256: str) -> pathlib.Path:
    record = {
        "schema_version": 1,
        "run_id": validate_digest(run_id),
        "results": path.name,
        "rows_sha256": validate_digest(rows_sha256),
        "artifacts_sha256": validate_digest(artifacts_sha256),
    }
    target = output_record_path(path, run_id)
    temporary = target.with_suffix(target.suffix + ".tmp")
    temporary.write_bytes(canonical_bytes(record))
    temporary.replace(target)
    return target


def read_output_record(path: pathlib.Path, run_id: str) -> dict:
    target = output_record_path(path, run_id)
    value = json.loads(target.read_text(encoding="utf-8"))
    if (
        value.get("schema_version") != 1
        or value.get("run_id") != run_id
        or value.get("results") != path.name
    ):
        raise ValueError("output record identity mismatch")
    validate_digest(value.get("rows_sha256"))
    validate_digest(value.get("artifacts_sha256"))
    return value
