"""Drive the local follow web API for a time-bounded real-PX4 bench test.

The script does not arm PX4. It waits until telemetry reports that the human
operator has armed, selects the synthetic target, maintains the operator
heartbeat locally (without SSH latency), requests ENGAGE once, and always
requests DISENGAGE on exit.
"""

from __future__ import annotations

import argparse
import json
import time
import urllib.parse
import urllib.request


def _url(base: str, path: str, token: str, **query) -> str:
    query["t"] = token
    return f"{base}{path}?{urllib.parse.urlencode(query)}"


def _get_json(base: str, path: str, token: str, timeout: float = 1.0):
    with urllib.request.urlopen(
            _url(base, path, token), timeout=timeout) as response:
        return json.loads(response.read().decode("utf-8"))


def _post(base: str, path: str, token: str, timeout: float = 1.0, **query):
    request = urllib.request.Request(
        _url(base, path, token, **query), data=b"", method="POST")
    with urllib.request.urlopen(request, timeout=timeout) as response:
        response.read()


def _summary(stats) -> tuple:
    return (
        stats.get("armed"), stats.get("state"), stats.get("engaged"),
        stats.get("mode"), stats.get("block"), stats.get("note"),
        stats.get("sp_hz"), stats.get("yaw"),
    )


def _print_stats(prefix: str, stats) -> None:
    print(
        f"{prefix} armed={stats.get('armed')} state={stats.get('state')} "
        f"engaged={stats.get('engaged')} mode={stats.get('mode')} "
        f"sp_hz={stats.get('sp_hz')} yaw={stats.get('yaw')} "
        f"block={stats.get('block')!r} note={stats.get('note')!r}",
        flush=True,
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--token", required=True)
    parser.add_argument("--port", type=int, default=8092)
    parser.add_argument("--wait-arm-s", type=float, default=120.0)
    parser.add_argument("--run-s", type=float, default=7.0)
    parser.add_argument("--alive-hz", type=float, default=5.0)
    parser.add_argument("--x", type=float, default=0.68)
    parser.add_argument("--y", type=float, default=0.48)
    args = parser.parse_args()

    base = f"http://127.0.0.1:{args.port}"
    wait_deadline = time.monotonic() + args.wait_arm_s
    previous = None
    print("WAITING_FOR_MANUAL_ARM", flush=True)
    while time.monotonic() < wait_deadline:
        stats = _get_json(base, "/stats", args.token)
        current = _summary(stats)
        if current != previous:
            _print_stats("WAIT", stats)
            previous = current
        if stats.get("armed"):
            break
        time.sleep(0.1)
    else:
        print("TIMEOUT_WAITING_FOR_ARM", flush=True)
        return 2

    activity_seen = False
    last_alive = 0.0
    previous = None
    try:
        _post(base, "/pick", args.token, x=args.x, y=args.y)
        _post(base, "/alive", args.token)
        last_alive = time.monotonic()
        _post(base, "/engage", args.token)
        print("ENGAGE_REQUESTED", flush=True)

        deadline = time.monotonic() + args.run_s
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now - last_alive >= 1.0 / args.alive_hz:
                _post(base, "/alive", args.token)
                last_alive = now
            stats = _get_json(base, "/stats", args.token)
            current = _summary(stats)
            if current != previous:
                _print_stats("RUN", stats)
                previous = current
            if stats.get("state") != "idle" or stats.get("engaged"):
                activity_seen = True
            if (activity_seen and stats.get("state") == "idle" and
                    not stats.get("engaged")):
                print("APP_RETURNED_TO_IDLE", flush=True)
                break
            time.sleep(0.05)
        return 0
    finally:
        try:
            _post(base, "/disengage", args.token)
            print("DISENGAGE_REQUESTED", flush=True)
        except Exception as exc:
            print(f"DISENGAGE_FAILED {exc!r}", flush=True)


if __name__ == "__main__":
    raise SystemExit(main())
