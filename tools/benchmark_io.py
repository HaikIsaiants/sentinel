from __future__ import annotations

import hashlib
import json
import os
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


def atomic_write(path: pathlib.Path, value: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + f".{os.getpid()}.tmp")
    with temporary.open("wb") as stream:
        stream.write(value)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)


def contained(root: pathlib.Path, relative: str) -> pathlib.Path:
    if not relative or pathlib.PurePosixPath(relative).is_absolute():
        raise ValueError("artifact path must be relative")
    target = (root / pathlib.PurePosixPath(relative)).resolve()
    try:
        target.relative_to(root.resolve())
    except ValueError as error:
        raise ValueError("artifact path escapes output root") from error
    return target
