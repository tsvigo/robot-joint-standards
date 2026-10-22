#!/usr/bin/env python3
"""Stage 2B.1 — static IK sweep over the base-to-bed clearance.

For P6 and P7, both arms and c = 0.05 / 0.10 / 0.12 / 0.15 / 0.20 m this script
solves the six named states of the operation and records, per state:

* reachability and the static IK residual (position and full 6D orientation),
* joint-limit margin and the limiting joint,
* conditioning of the task Jacobian,
* clearances against the bed, the torso, the base and the other arm.

The other arm is parked at P0 for the single-arm clearance, and is set to its
own same-named state for the simultaneous two-arm clearance.

Read-only relative to the .blend and to the stage 2A/2B sources.
Writes one JSON and one CSV next to this file.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import multiprocessing
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
BASE = HERE.parents[1]
for _sub in ("stage2B", "stage2A", "stage2A_ik"):
    sys.path.insert(0, str(BASE / _sub))

import motion_study as ms  # noqa: E402
import ik_study as ik  # noqa: E402
import bed_study as bed  # noqa: E402

import stage2b1_unilateral as uni  # noqa: E402

CLEARANCES = (0.05, 0.10, 0.12, 0.15, 0.20)
OPERATIONS = ("P6", "P7")
P7_FINGER_YAW_DEG = 80.0


def p7_finger() -> np.ndarray:
    yaw = math.radians(P7_FINGER_YAW_DEG)
    return np.array([math.sin(yaw), math.cos(yaw), 0.0], dtype=float)


def cross_clearance(left_q, right_q, geom: dict) -> dict:
    left = bed.collision_clearances("L", left_q, ms.LIFT_M, geom, other_q=right_q)
    right = bed.collision_clearances("R", right_q, ms.LIFT_M, geom, other_q=left_q)
    return {
        "arm_arm_m": min(value for value in (left["arm_arm"], right["arm_arm"]) if value is not None),
        "collision_free": bool(left["collision_free"] and right["collision_free"]),
    }


def solve_case(task: tuple) -> dict:
    """One (operation, clearance, side) block — runs in a worker process.

    P7 uses the confirmed yaw = 80 deg finger; `operation_sequence` hard-codes
    PLUS_Y, so the finger has to be passed in explicitly, exactly as the
    trajectory scripts do.
    """
    operation, clearance, side, height, width = task
    ms.MODE = "6d"
    case_started = time.time()
    geom = bed.bed_geometry("lateral", height, width, clearance)
    finger = p7_finger() if operation == "P7" else None
    references = uni.build_references(operation, side, geom, finger)
    states = []
    for reference in references:
        if not reference["reachable"]:
            states.append(
                {
                    "label": reference["label"],
                    "reachable": False,
                    "position_floor": reference["position_floor"],
                }
            )
            continue
        metrics = ik.task_metrics(side, reference["q_deg"], ms.LIFT_M, reference["target"], "6d")
        clearances = bed.collision_clearances(side, reference["q_deg"], ms.LIFT_M, geom, other_q=ms.P0_Q)
        margin = metrics["min_margin"]
        states.append(
            {
                "label": reference["label"],
                "reachable": True,
                "position_floor": reference["position_floor"],
                "q_deg": reference["q_deg"],
                "position_error_mm": float(np.linalg.norm(metrics["position_error"]) * 1000.0),
                "orientation_error_deg": float(metrics["orientation_error_deg"]),
                "min_margin_deg": float(margin["margin_deg"]),
                "min_margin_joint": margin["joint"],
                "condition": float(metrics["condition"]) if metrics["condition"] is not None else None,
                "within_limits": bool(metrics["within_limits"]),
                "arm_bed_m": clearances["arm_bed"],
                "arm_torso_m": clearances["arm_torso"],
                "arm_base_m": clearances["arm_base"],
                "arm_arm_parked_m": clearances["arm_arm"],
                "collision_free_parked": bool(clearances["collision_free"]),
            }
        )
    return {
        "operation": operation,
        "side": side,
        "clearance_m": clearance,
        "finger": [float(value) for value in finger] if finger is not None else None,
        "bed": {key: geom[key] for key in ("x_near", "x_far", "height_m", "width_m", "clearance_m")},
        "seconds": round(time.time() - case_started, 1),
        "states": states,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--height", type=float, default=0.7)
    parser.add_argument("--width", type=float, default=1.0)
    parser.add_argument("--workers", type=int, default=4)
    args = parser.parse_args()

    ms.MODE = "6d"
    started = time.time()
    rows = []
    payload = {
        "metadata": {
            "stage": "2B.1",
            "kind": "static_ik_sweep",
            "created_utc": datetime.now(timezone.utc).isoformat(),
            "mode": "6d",
            "approach": "lateral",
            "height_m": args.height,
            "width_m": args.width,
            "clearances_m": list(CLEARANCES),
            "operations": list(OPERATIONS),
            "sides": ["L", "R"],
            "p7_finger_yaw_deg": P7_FINGER_YAW_DEG,
            "script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            "git_head": subprocess.run(
                ["git", "-C", str(BASE), "rev-parse", "HEAD"], capture_output=True, text=True, check=False
            ).stdout.strip() or None,
            "definitions": {
                "position_error_mm": "static IK residual to the commanded point",
                "orientation_error_deg": "static IK residual of the full 6D rotation",
                "arm_arm_parked_m": "clearance to the other arm parked at P0",
                "arm_arm_same_state_m": "clearance when both arms hold the same named state",
            },
        },
        "cases": [],
    }

    tasks = [
        (operation, clearance, side, args.height, args.width)
        for operation in OPERATIONS
        for clearance in CLEARANCES
        for side in ("L", "R")
    ]
    with multiprocessing.Pool(processes=max(1, args.workers)) as pool:
        solved = pool.map(solve_case, tasks)

    order = {(operation, clearance, side): index for index, (operation, clearance, side, _, _) in enumerate(tasks)}
    payload["cases"] = sorted(
        solved,
        key=lambda case: order[(case["operation"], case["clearance_m"], case["side"])],
    )

    for case in payload["cases"]:
        reachable = sum(1 for state in case["states"] if state.get("reachable"))
        print(
            f"{case['operation']} {case['side']} c={case['clearance_m']:.2f}: "
            f"reachable {reachable}/{len(case['states'])} ({case['seconds']} s)"
        )

    # simultaneous two-arm static check for the same named states
    payload["simultaneous"] = []
    for operations in OPERATIONS:
        for clearance in CLEARANCES:
            geom = bed.bed_geometry("lateral", args.height, args.width, clearance)
            left = {state["label"]: state for state in next(
                case["states"] for case in payload["cases"]
                if case["operation"] == operations and case["side"] == "L" and case["clearance_m"] == clearance
            )}
            right = {state["label"]: state for state in next(
                case["states"] for case in payload["cases"]
                if case["operation"] == operations and case["side"] == "R" and case["clearance_m"] == clearance
            )}
            for label in left:
                if not left[label].get("reachable") or not right[label].get("reachable"):
                    payload["simultaneous"].append(
                        {"operation": operations, "clearance_m": clearance, "label": label, "reachable_pair": False}
                    )
                    continue
                clearance_pair = cross_clearance(left[label]["q_deg"], right[label]["q_deg"], geom)
                payload["simultaneous"].append(
                    {
                        "operation": operations,
                        "clearance_m": clearance,
                        "label": label,
                        "reachable_pair": True,
                        "arm_arm_m": clearance_pair["arm_arm_m"],
                        "collision_free": clearance_pair["collision_free"],
                        "min_margin_deg": min(left[label]["min_margin_deg"], right[label]["min_margin_deg"]),
                        "max_condition": max(left[label]["condition"] or 0.0, right[label]["condition"] or 0.0),
                    }
                )
            print(f"simultaneous {operations} c={clearance:.2f}: done")

    # flat rows for the CSV table
    for case in payload["cases"]:
        for state in case["states"]:
            rows.append(
                {
                    "operation": case["operation"],
                    "side": case["side"],
                    "clearance_m": case["clearance_m"],
                    "label": state["label"],
                    "reachable": state.get("reachable", False),
                    "position_error_mm": state.get("position_error_mm"),
                    "orientation_error_deg": state.get("orientation_error_deg"),
                    "min_margin_deg": state.get("min_margin_deg"),
                    "min_margin_joint": state.get("min_margin_joint"),
                    "condition": state.get("condition"),
                    "arm_bed_m": state.get("arm_bed_m"),
                    "arm_torso_m": state.get("arm_torso_m"),
                    "arm_base_m": state.get("arm_base_m"),
                    "arm_arm_parked_m": state.get("arm_arm_parked_m"),
                }
            )

    payload["metadata"]["seconds"] = round(time.time() - started, 1)
    out_json = HERE / "stage2b1_static_sweep.json"
    out_json.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")

    out_csv = HERE / "stage2b1_static_sweep.csv"
    with out_csv.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    # per-c aggregates using both arms and both operations
    print("\nclearance | op  | reachable L/R | min margin | max cond | min bed | min arm-arm same-state")
    for clearance in CLEARANCES:
        for operations in OPERATIONS:
            cases = [case for case in payload["cases"] if case["operation"] == operations and case["clearance_m"] == clearance]
            reach = []
            margins = []
            conditions = []
            beds = []
            for case in cases:
                reach.append(sum(1 for state in case["states"] if state.get("reachable")))
                margins += [state["min_margin_deg"] for state in case["states"] if state.get("reachable")]
                conditions += [state["condition"] for state in case["states"] if state.get("reachable") and state["condition"]]
                beds += [state["arm_bed_m"] for state in case["states"] if state.get("reachable") and state["arm_bed_m"] is not None]
            pairs = [
                item for item in payload["simultaneous"]
                if item["operation"] == operations and item["clearance_m"] == clearance and item.get("reachable_pair")
            ]
            pair_min = min(item["arm_arm_m"] for item in pairs) if pairs else None
            print(
                f"{clearance:9.2f} | {operations} | {reach[0]:2d}/6 {reach[1]:2d}/6      | "
                f"{min(margins) if margins else float('nan'):6.2f}     | {max(conditions) if conditions else float('nan'):5.2f}   | "
                f"{min(beds)*1000 if beds else float('nan'):7.1f}mm | "
                f"{pair_min*1000 if pair_min is not None else float('nan'):7.1f}mm"
            )

    print(f"\nwritten: {out_json}")
    print(f"written: {out_csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
