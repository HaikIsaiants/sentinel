from __future__ import annotations

import hashlib
import json
import os
import pathlib
import stat
import time


def digest(path: pathlib.Path) -> str:
    value = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(block)
    return value.hexdigest()


def source_files(root: pathlib.Path) -> list[pathlib.Path]:
    excluded = {".git", "build", "out", "vcpkg_installed", "__pycache__"}
    files = []
    for path in root.rglob("*"):
        if not path.is_file() or any(part in excluded for part in path.relative_to(root).parts):
            continue
        files.append(path)
    return sorted(files, key=lambda path: path.relative_to(root).as_posix())


def executable(path: pathlib.Path) -> dict:
    if not path.is_file():
        raise ValueError(f"missing executable: {path}")
    mode = path.stat().st_mode
    return {
        "path": str(path.resolve()),
        "sha256": digest(path),
        "executable": bool(mode & (stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)) or os.name == "nt",
    }


def create_manifest(
    root: pathlib.Path,
    runner: pathlib.Path,
    simulator: pathlib.Path,
    agent: pathlib.Path,
) -> dict:
    files = source_files(root)
    source = [
        {"path": path.relative_to(root).as_posix(), "sha256": digest(path)}
        for path in files
    ]
    tree = hashlib.sha256()
    for entry in source:
        tree.update(entry["path"].encode())
        tree.update(b"\0")
        tree.update(entry["sha256"].encode())
        tree.update(b"\n")
    manifest = {
        "schema_version": 1,
        "created_unix_seconds": int(time.time()),
        "source_tree_sha256": tree.hexdigest(),
        "source_files": source,
        "executables": {
            "runner": executable(runner),
            "simulator": executable(simulator),
            "agent": executable(agent),
        },
    }
    identity = json.dumps(manifest, sort_keys=True, separators=(",", ":")).encode()
    manifest["run_id"] = hashlib.sha256(identity).hexdigest()
    return manifest


def write_manifest(path: pathlib.Path, manifest: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8", newline="\n")
    os.replace(temporary, path)


def verify_manifest(path: pathlib.Path, root: pathlib.Path) -> dict:
    manifest = json.loads(path.read_text(encoding="utf-8"))
    if manifest.get("schema_version") != 1 or not manifest.get("run_id"):
        raise ValueError("invalid run manifest")
    current = {
        item.relative_to(root).as_posix(): digest(item)
        for item in source_files(root)
        if item.resolve() != path.resolve()
    }
    recorded = {item["path"]: item["sha256"] for item in manifest["source_files"]}
    if current != recorded:
        raise ValueError("source tree differs from run manifest")
    for value in manifest["executables"].values():
        target = pathlib.Path(value["path"])
        if digest(target) != value["sha256"]:
            raise ValueError(f"executable changed: {target}")
    return manifest
