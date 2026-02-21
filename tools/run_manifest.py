from __future__ import annotations

import hashlib
import json
import os
import pathlib
import stat

from benchmark_io import canonical_bytes, file_digest, read_output_record, validate_digest


EXCLUDED = {".git", "build", "out", "vcpkg_installed", "__pycache__"}


def skipped(path: pathlib.Path, root: pathlib.Path, excluded: tuple[pathlib.Path, ...] = ()) -> bool:
    relative = path.relative_to(root)
    if any(part in EXCLUDED for part in relative.parts):
        return True
    resolved = path.resolve()
    return any(resolved == item.resolve() or item.resolve() in resolved.parents for item in excluded)


def source_files(root: pathlib.Path, excluded: tuple[pathlib.Path, ...] = ()) -> list[pathlib.Path]:
    return sorted(
        (path for path in root.rglob("*") if path.is_file() and not skipped(path, root, excluded)),
        key=lambda path: path.relative_to(root).as_posix(),
    )


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
    output: pathlib.Path | None = None,
    worker_count: int = 1,
) -> dict:
    if worker_count <= 0:
        raise ValueError("worker count must be positive")
    excluded = (output,) if output else ()
    files = source_files(root, excluded)
    manifest = {
        "schema_version": 2,
        "source_tree_sha256": tree_digest(root, files),
        "source_files": [
            {"path": path.relative_to(root).as_posix(), "sha256": file_digest(path)}
            for path in files
        ],
        "executables": {
            "runner": executable(runner),
            "simulator": executable(simulator),
            "agent": executable(agent),
        },
        "worker_count": worker_count,
    }
    manifest["run_id"] = hashlib.sha256(canonical_bytes(manifest)).hexdigest()
    return manifest


def write_manifest(path: pathlib.Path, manifest: dict) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.parent.mkdir(parents=True, exist_ok=True)
    temporary.write_bytes(json.dumps(manifest, indent=2, sort_keys=True).encode() + b"\n")
    os.replace(temporary, path)


def verify_manifest(path: pathlib.Path, root: pathlib.Path, output: pathlib.Path | None = None) -> dict:
    value = json.loads(path.read_text(encoding="utf-8"))
    if value.get("schema_version") != 2:
        raise ValueError("unsupported manifest schema")
    run_id = validate_digest(value.get("run_id"))
    identity = dict(value)
    identity.pop("run_id")
    if hashlib.sha256(canonical_bytes(identity)).hexdigest() != run_id:
        raise ValueError("run manifest identity mismatch")
    excluded = tuple(item for item in (path, output) if item is not None)
    files = source_files(root, excluded)
    if tree_digest(root, files) != value["source_tree_sha256"]:
        raise ValueError("source tree differs from run manifest")
    recorded = {item["path"]: item["sha256"] for item in value["source_files"]}
    current = {item.relative_to(root).as_posix(): file_digest(item) for item in files}
    if current != recorded:
        raise ValueError("source file inventory differs from run manifest")
    for item in value["executables"].values():
        if file_digest(pathlib.Path(item["path"])) != item["sha256"]:
            raise ValueError("run executable changed")
    return value


def verify_output(path: pathlib.Path, run_id: str) -> dict:
    value = read_output_record(path, run_id)
    if file_digest(path) != value["rows_sha256"]:
        raise ValueError("benchmark results changed")
    return value
