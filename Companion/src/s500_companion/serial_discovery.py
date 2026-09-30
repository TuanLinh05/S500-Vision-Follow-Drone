from __future__ import annotations

import glob
import os
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True, slots=True)
class SerialCandidate:
    path: str
    resolved_path: str
    score: int


_PIXHAWK_HINTS = ("pixhawk", "px4", "holybro", "fmu", "autopilot")


def _score(path: str) -> int:
    lowered = path.lower()
    return sum(10 for hint in _PIXHAWK_HINTS if hint in lowered)


def list_serial_candidates(by_id_root: str = "/dev/serial/by-id") -> list[SerialCandidate]:
    candidates: list[SerialCandidate] = []
    for path in glob.glob(str(Path(by_id_root) / "*")):
        resolved = os.path.realpath(path)
        candidates.append(SerialCandidate(path, resolved, _score(path)))
    return sorted(candidates, key=lambda item: (-item.score, item.path))


def discover_pixhawk(by_id_root: str = "/dev/serial/by-id") -> str:
    candidates = list_serial_candidates(by_id_root)
    if not candidates:
        raise FileNotFoundError(
            f"No serial devices found in {by_id_root}. Check the USB data cable, "
            "dmesg, and Linux permissions."
        )
    best = candidates[0]
    if best.score <= 0 and len(candidates) > 1:
        paths = ", ".join(item.path for item in candidates)
        raise RuntimeError(
            "Several serial devices were found but none looks like Pixhawk: " + paths
        )
    return best.path


def resolve_connection(connection: str) -> str:
    if connection != "auto":
        return connection
    return discover_pixhawk()


def is_serial_connection(connection: str) -> bool:
    return connection.startswith("/") or connection.upper().startswith("COM")

