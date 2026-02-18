from __future__ import annotations

import hashlib
import json
import os
import pathlib
import stat

from benchmark_io import canonical_bytes, file_digest, validate_digest


EXCLUDED = {".git", "build", "out", "vcpkg_installed", "__pycache__"}


def source_files(root: pathlib.Path, excluded: tuple[pathlib.Path, ...] = ()) -> list[pathlib.Path]:
    excluded = tuple(path.resolve() for path in excluded)
    values = []
    for path in root.rglob("*"):
        if not path.is_file() or any(part in EXCLUDED for part in path.relative_to(root).parts):
            continue
        resolved = path.resolve()
        if any(resolved == item or item in resolved.parents for item in excluded):
            continue
        values.append(path)
    return sorted(values, key=lambda path: path.relative_to(root).as_posix())


def tree_digest(root: pathlib.Path, files: list[pathlib.Path]) -> str:
    value = hashlib.sha256()
    for path in files:
        value.update(path.relative_to(root).as_posix().encode())
        value.update(b"\0")
        value.update(file_digest(path).encode())
        value.update(b"\n")
    return value.hexdigest()


def executable(path: pathlib.Path) -> dict:
    if not path.is_file():
        raise ValueError(f"missing executable: {path}")
    return {
        "path": str(path.resolve()),
        "sha256": file_digest(path),
        "mode": stat.S_IMODE(path.stat().st_mode),
    }


def create_manifest(
    root: pathlib.Path,
    runner: pathlib.Path,
    simulator: pathlib.Path,
    agent: pathlib.Path,
    output: pathlib.Path,
    workers: int,
    timeout: float,
) -> dict:
    if workers <= 0 or timeout <= 0:
        raise ValueError("invalid parallel execution settings")
    files = source_files(root, (output,))
    value = {
        "schema_version": 2,
        "source_tree_sha256": tree_digest(root, files),
        "source_files": [{"path": path.relative_to(root).as_posix(), "sha256": file_digest(path)} for path in files],
        "executables": {
            "runner": executable(runner),
            "simulator": executable(simulator),
            "agent": executable(agent),
        },
        "dispatch": {
            "strategy": "bounded-subprocess",
            "workers": workers,
            "timeout_seconds": timeout,
            "publication_order": "mission-identity",
        },
    }
    value["run_id"] = hashlib.sha256(canonical_bytes(value)).hexdigest()
    return value


def write_manifest(path: pathlib.Path, value: dict) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.parent.mkdir(parents=True, exist_ok=True)
    temporary.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8", newline="\n")
    os.replace(temporary, path)


def verify_manifest(path: pathlib.Path, root: pathlib.Path, output: pathlib.Path) -> dict:
    value = json.loads(path.read_text(encoding="utf-8"))
    run_id = validate_digest(value.get("run_id"))
    identity = dict(value)
    identity.pop("run_id")
    if value.get("schema_version") != 2 or hashlib.sha256(canonical_bytes(identity)).hexdigest() != run_id:
        raise ValueError("manifest identity mismatch")
    files = source_files(root, (path, output))
    if tree_digest(root, files) != value["source_tree_sha256"]:
        raise ValueError("source tree differs from manifest")
    current = {item.relative_to(root).as_posix(): file_digest(item) for item in files}
    recorded = {item["path"]: item["sha256"] for item in value["source_files"]}
    if current != recorded:
        raise ValueError("source inventory differs from manifest")
    return value
