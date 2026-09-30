from __future__ import annotations

import csv
import json
from dataclasses import asdict, dataclass
from pathlib import Path


@dataclass(frozen=True, slots=True)
class P1ValidationResult:
    passed: bool
    row_count: int
    duration_s: float
    disconnect_rows: int
    unexpected_system_ids: tuple[int, ...]
    max_rx_age_s: float
    max_heartbeat_age_s: float
    local_position_ratio: float
    global_position_ratio: float
    rc_available_ratio: float
    minimum_battery_pct: int | None
    reasons: tuple[str, ...]

    def as_dict(self) -> dict[str, object]:
        return asdict(self)


def validate_p1_csv(
    path: str | Path,
    expected_system_id: int = 1,
    minimum_duration_s: float = 1740.0,
) -> P1ValidationResult:
    with Path(path).open(encoding="utf-8", newline="") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        return P1ValidationResult(
            False, 0, 0.0, 0, (), float("inf"), float("inf"), 0.0, 0.0, 0.0, None,
            ("empty_log",),
        )

    times = [_float(row["monotonic_ts"]) for row in rows]
    duration = max(times) - min(times)
    disconnects = sum(not _bool(row["connected"]) for row in rows)
    system_ids = {
        int(row["system_id"])
        for row in rows
        if row.get("system_id") not in {None, "", "None"}
    }
    unexpected = tuple(sorted(system_ids - {expected_system_id}))
    rx_ages = [
        max(0.0, _float(row["monotonic_ts"]) - _float(row["last_rx_ts"]))
        for row in rows
        if _float(row["last_rx_ts"]) > 0
    ]
    heartbeat_ages = [
        max(0.0, _float(row["monotonic_ts"]) - _float(row["last_heartbeat_ts"]))
        for row in rows
        if _float(row["last_heartbeat_ts"]) > 0
    ]
    count = len(rows)
    local_ratio = sum(_bool(row["local_position_valid"]) for row in rows) / count
    global_ratio = sum(_bool(row["global_position_valid"]) for row in rows) / count
    rc_ratio = sum(_bool(row["rc_available"]) for row in rows) / count
    batteries = [
        int(float(row["battery_remaining_pct"]))
        for row in rows
        if row.get("battery_remaining_pct") not in {None, "", "None"}
    ]
    reasons: list[str] = []
    if duration < minimum_duration_s:
        reasons.append("duration_below_29_minutes")
    if disconnects:
        reasons.append("link_disconnected")
    if unexpected:
        reasons.append("unexpected_system_id")
    max_rx_age = max(rx_ages, default=float("inf"))
    max_heartbeat_age = max(heartbeat_ages, default=float("inf"))
    if max_rx_age > 0.30:
        reasons.append("mavlink_rx_gap_over_300ms")
    if max_heartbeat_age > 2.0:
        reasons.append("heartbeat_gap_over_2s")
    if local_ratio < 0.95:
        reasons.append("local_position_availability_below_95pct")
    return P1ValidationResult(
        passed=not reasons,
        row_count=count,
        duration_s=duration,
        disconnect_rows=disconnects,
        unexpected_system_ids=unexpected,
        max_rx_age_s=max_rx_age,
        max_heartbeat_age_s=max_heartbeat_age,
        local_position_ratio=local_ratio,
        global_position_ratio=global_ratio,
        rc_available_ratio=rc_ratio,
        minimum_battery_pct=min(batteries) if batteries else None,
        reasons=tuple(reasons),
    )


def write_validation_json(result: P1ValidationResult, path: str | Path) -> None:
    Path(path).write_text(
        json.dumps(result.as_dict(), indent=2, ensure_ascii=False),
        encoding="utf-8",
    )


def _float(value: str | None) -> float:
    if value in {None, "", "None"}:
        return 0.0
    return float(value)


def _bool(value: str | None) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes"}
