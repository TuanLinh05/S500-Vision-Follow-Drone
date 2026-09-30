from __future__ import annotations

import json
from typing import Any

from .models import GuidanceProposal, VisionStatus


class VisionProtocolError(ValueError):
    pass


def parse_vision_packet(payload: str | bytes) -> tuple[VisionStatus, GuidanceProposal]:
    try:
        data: dict[str, Any] = json.loads(payload)
    except (json.JSONDecodeError, UnicodeDecodeError) as exc:
        raise VisionProtocolError(f"Invalid JSON: {exc}") from exc

    if data.get("schema") != "s500.vision.v1":
        raise VisionProtocolError("schema must be s500.vision.v1")
    required = {
        "sequence",
        "frame_ts",
        "track_ts",
        "guidance_ts",
        "track_id",
        "track_state",
        "confidence",
        "camera_ok",
        "detector_ok",
        "tracker_ok",
        "yaw_rate_deg_s",
    }
    missing = sorted(required - data.keys())
    if missing:
        raise VisionProtocolError("Missing keys: " + ", ".join(missing))

    track_id = data["track_id"]
    if track_id is not None:
        track_id = int(track_id)
    vision = VisionStatus(
        camera_ok=bool(data["camera_ok"]),
        detector_ok=bool(data["detector_ok"]),
        tracker_ok=bool(data["tracker_ok"]),
        frame_ts=float(data["frame_ts"]),
        track_ts=float(data["track_ts"]),
        selected_track_id=track_id,
        track_state=str(data["track_state"]).upper(),
        confidence=float(data["confidence"]),
    )
    guidance = GuidanceProposal(
        created_ts=float(data["guidance_ts"]),
        sequence=int(data["sequence"]),
        track_id=track_id,
        vx_body_m_s=float(data.get("vx_body_m_s", 0.0)),
        vy_body_m_s=float(data.get("vy_body_m_s", 0.0)),
        vz_body_m_s=float(data.get("vz_body_m_s", 0.0)),
        yaw_rate_deg_s=float(data["yaw_rate_deg_s"]),
    )
    if not guidance.finite():
        raise VisionProtocolError("Guidance contains NaN or Infinity")
    return vision, guidance

