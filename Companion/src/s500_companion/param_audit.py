from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from pathlib import Path


@dataclass(frozen=True, slots=True)
class ParamCheck:
    status: str
    parameter: str
    current: float | int | None
    expected: str
    message: str


def parse_qgc_params(path: str | Path) -> dict[str, float]:
    values: dict[str, float] = {}
    for raw_line in Path(path).read_text(encoding="utf-8-sig").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        parts = line.split("\t")
        if len(parts) < 4:
            parts = line.split()
        if len(parts) < 4:
            continue
        try:
            values[parts[2]] = float(parts[3])
        except ValueError:
            continue
    return values


def audit_yaw_test_params(values: dict[str, float]) -> list[ParamCheck]:
    checks: list[ParamCheck] = []

    def require(name: str) -> float | None:
        value = values.get(name)
        if value is None:
            checks.append(ParamCheck("FAIL", name, None, "present", "Parameter missing"))
        return value

    value = require("COM_RC_OVERRIDE")
    if value is not None:
        enabled = int(value) & 0b10
        checks.append(
            ParamCheck(
                "PASS" if enabled else "FAIL",
                "COM_RC_OVERRIDE",
                int(value),
                "Offboard override bit enabled (candidate bitmask 3)",
                "RC sticks can leave Offboard" if enabled else "Offboard stick override is disabled",
            )
        )

    value = require("COM_OBL_RC_ACT")
    if value is not None:
        checks.append(
            ParamCheck(
                "PASS" if int(value) == 0 else "FAIL",
                "COM_OBL_RC_ACT",
                int(value),
                "0 / Position",
                "Offboard-loss with RC should enter Position for P7",
            )
        )

    value = require("COM_OF_LOSS_T")
    if value is not None:
        checks.append(
            ParamCheck(
                "PASS" if 0.2 <= value <= 1.5 else "WARN",
                "COM_OF_LOSS_T",
                value,
                "0.2..1.5 s, validated in SITL/bench",
                "Offboard stream-loss timeout",
            )
        )

    value = require("COM_RCL_EXCEPT")
    if value is not None:
        ignored = bool(int(value) & (1 << 2))
        checks.append(
            ParamCheck(
                "FAIL" if ignored else "PASS",
                "COM_RCL_EXCEPT",
                int(value),
                "Offboard RC-loss exception bit clear",
                "RC loss must not be ignored in Offboard",
            )
        )

    for name, expected in (("GF_MAX_HOR_DIST", "> 0 m"), ("GF_MAX_VER_DIST", "> 0 m")):
        value = require(name)
        if value is not None:
            checks.append(
                ParamCheck(
                    "PASS" if value > 0 else "FAIL",
                    name,
                    value,
                    expected,
                    "PX4 hard geofence must be enabled before P7",
                )
            )

    value = require("GF_ACTION")
    if value is not None:
        checks.append(
            ParamCheck(
                "PASS" if int(value) == 2 else "WARN",
                "GF_ACTION",
                int(value),
                "2 / Hold for initial test",
                "Confirm enum label in QGroundControl",
            )
        )

    value = require("MPC_XY_VEL_MAX")
    if value is not None:
        checks.append(
            ParamCheck(
                "PASS" if value <= 2.0 else "FAIL",
                "MPC_XY_VEL_MAX",
                value,
                "<= 2 m/s in AI yaw-test profile",
                "Independent PX4 ceiling for the first Offboard flight",
            )
        )

    value = require("RC_MAP_RETURN_SW")
    if value is not None:
        checks.append(
            ParamCheck(
                "PASS" if value > 0 else "FAIL",
                "RC_MAP_RETURN_SW",
                int(value),
                "> 0",
                "Dedicated RTL switch",
            )
        )

    value = require("RC_MAP_KILL_SW")
    if value is not None:
        checks.append(
            ParamCheck(
                "PASS" if value > 0 else "WARN",
                "RC_MAP_KILL_SW",
                int(value),
                "guarded dedicated channel, if approved",
                "Kill is last resort and causes a fall",
            )
        )

    value = require("MAV_2_CONFIG")
    if value is not None:
        checks.append(
            ParamCheck(
                "PASS" if int(value) == 0 else "WARN",
                "MAV_2_CONFIG",
                int(value),
                "0 when USB is the companion link",
                "TELEM2 is not needed for direct USB",
            )
        )
    return checks


def audit_summary(checks: list[ParamCheck]) -> dict[str, object]:
    counts = {status: sum(item.status == status for item in checks) for status in ("PASS", "WARN", "FAIL")}
    return {
        "ready_for_p7": counts["FAIL"] == 0,
        "counts": counts,
        "checks": [asdict(item) for item in checks],
    }


def audit_file(path: str | Path) -> dict[str, object]:
    return audit_summary(audit_yaw_test_params(parse_qgc_params(path)))


def write_audit_json(result: dict[str, object], path: str | Path) -> None:
    Path(path).write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")

