#!/usr/bin/env python3
"""Stage 2B.1 — control run for P6 in 6D.

Read-only relative to the .blend, to stage 2A/2B sources and to existing
artifacts.  Writes one JSON next to this file.

Purpose
-------
Reproduce the one-arm 6D trajectory of P6 and store every per-sample metric:
joint coordinates, position error, orientation error, joint-limit margin,
condition number, clearances and joint step.

Important definitions
---------------------
* ``position_error_mm`` of a sample is the distance between the palm face and
  the *linear* reference point between the transition start and end targets.
  This is a deviation from the straight-line reference, not an IK residual.
  For ``task_space`` it should be near zero by construction; for
  ``joint_lerp`` it measures corner cutting.
* ``orientation_error_deg`` is the full 6D rotation-vector angle between the
  commanded palm rotation and the actual palm rotation.  The commanded
  rotation is constant within an operation (normal = -Z, finger fixed), so it
  can be evaluated against the start target of the transition.
* ``endpoint_*`` fields describe achievement of the transition end pose.
"""

from __future__ import annotations

import hashlib
import json
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
BASE = HERE.parents[1]  # robot_pose_study_v1
for _sub in ("stage2B", "stage2A", "stage2A_ik"):
    sys.path.insert(0, str(BASE / _sub))

import motion_study as ms  # noqa: E402
import ik_study as ik  # noqa: E402
import bed_study as bed  # noqa: E402

OP = "P6"
SIDE = "L"
CLEARANCE = 0.10
HEIGHT = 0.7
WIDTH = 1.0
OUT = HERE / "stage2b1_control_p6_L.json"


def qdict(q: dict[str, float]) -> dict[str, float]:
    return {key: float(q.get(key, 0.0)) for key in ms.JOINT_KEYS}


def with_position(target: dict, position) -> dict:
    out = dict(target)
    out["position_floor"] = [float(value) for value in position]
    return out


def sample_record(side: str, q: dict[str, float], target: dict, geom: dict, t: float, previous) -> dict:
    metrics = ik.task_metrics(side, q, ms.LIFT_M, target, "6d")
    clearances = bed.collision_clearances(side, q, ms.LIFT_M, geom, other_q=ms.P0_Q)
    margin = metrics["min_margin"]
    return {
        "t": float(t),
        "q_deg": qdict(q),
        "position_error_mm": float(np.linalg.norm(metrics["position_error"]) * 1000.0),
        "orientation_error_deg": float(metrics["orientation_error_deg"]),
        "min_margin_deg": float(margin["margin_deg"]),
        "min_margin_joint": margin["joint"],
        "condition": float(metrics["condition"]) if metrics["condition"] is not None else None,
        "within_limits": bool(metrics["within_limits"]),
        "clearance_min_m": clearances["minimum"],
        "arm_arm_m": clearances["arm_arm"],
        "arm_bed_m": clearances["arm_bed"],
        "arm_torso_m": clearances["arm_torso"],
        "arm_base_m": clearances["arm_base"],
        "collision_free": bool(clearances["collision_free"]),
        "joint_step_deg": float(ms.q_distance(side, previous, q)) if previous is not None else 0.0,
    }


def aggregate(records: list[dict]) -> dict:
    def values(key):
        return [record[key] for record in records if record[key] is not None]

    return {
        "sample_count": len(records),
        "max_position_error_mm": max(values("position_error_mm")) if records else None,
        "max_orientation_error_deg": max(values("orientation_error_deg")) if records else None,
        "min_margin_deg": min(values("min_margin_deg")) if records else None,
        "max_condition": max(values("condition")) if values("condition") else None,
        "max_joint_step_deg": max(values("joint_step_deg")) if records else None,
        "min_clearance_m": min(values("clearance_min_m")) if values("clearance_min_m") else None,
        "min_arm_bed_m": min(values("arm_bed_m")) if values("arm_bed_m") else None,
        "min_arm_torso_m": min(values("arm_torso_m")) if values("arm_torso_m") else None,
        "min_arm_base_m": min(values("arm_base_m")) if values("arm_base_m") else None,
        "all_collision_free": all(record["collision_free"] for record in records),
        "all_within_limits": all(record["within_limits"] for record in records),
        "endpoint_position_error_mm": records[-1]["position_error_mm"] if records else None,
        "endpoint_orientation_error_deg": records[-1]["orientation_error_deg"] if records else None,
    }


def main() -> int:
    ms.MODE = "6d"
    geom = bed.bed_geometry("lateral", HEIGHT, WIDTH, CLEARANCE)

    started = time.time()
    operation = ms.plan_operation(OP, SIDE, geom)
    elapsed = time.time() - started

    references = {reference["label"]: reference for reference in operation["references"]}

    reference_checks = []
    for label, reference in references.items():
        if not reference.get("reachable"):
            reference_checks.append({"label": label, "reachable": False})
            continue
        metrics = ik.task_metrics(SIDE, reference["q_deg"], ms.LIFT_M, reference["target"], "6d")
        reference_checks.append(
            {
                "label": label,
                "reachable": True,
                "position_floor": reference["position_floor"],
                "position_error_mm": float(np.linalg.norm(metrics["position_error"]) * 1000.0),
                "orientation_error_deg": float(metrics["orientation_error_deg"]),
                "min_margin_deg": float(metrics["min_margin"]["margin_deg"]),
                "condition": float(metrics["condition"]) if metrics["condition"] is not None else None,
            }
        )

    transitions_out = []
    for transition in operation["transitions"]:
        selected = transition.get("selected")
        if not selected or transition["status"] != "valid":
            transitions_out.append(
                {
                    "from": transition["from"],
                    "to": transition["to"],
                    "status": transition["status"],
                    "reason": transition.get("reason"),
                    "candidates": transition.get("candidates", []),
                }
            )
            continue

        start = references[transition["from"]]
        end = references[transition["to"]]
        start_position = np.asarray(start["position_floor"], dtype=float)
        end_position = np.asarray(end["position_floor"], dtype=float)

        records = []
        previous = None
        for sample in selected["samples"]:
            t = float(sample["t"])
            position = start_position + (end_position - start_position) * t
            target = with_position(start["target"], position)
            record = sample_record(SIDE, sample["q_deg"], target, geom, t, previous)
            record["reference_linear_position"] = [float(value) for value in position]
            records.append(record)
            previous = sample["q_deg"]

        summary = aggregate(records)
        summary["method"] = selected.get("method")
        summary["max_joint_step_reported_deg"] = selected.get("max_joint_step_deg")
        transitions_out.append(
            {
                "from": transition["from"],
                "to": transition["to"],
                "status": transition["status"],
                "selected_method": selected.get("method"),
                "summary": summary,
                "candidates": transition.get("candidates", []),
                "samples": records,
            }
        )

    valid = [item for item in transitions_out if item["status"] == "valid"]
    all_samples = [sample for item in valid for sample in item["samples"]]

    overall = aggregate(all_samples)
    result = {
        "metadata": {
            "stage": "2B.1",
            "kind": "control_run",
            "created_utc": datetime.now(timezone.utc).isoformat(),
            "operation": OP,
            "side": SIDE,
            "mode": "6d",
            "approach": "lateral",
            "height_m": HEIGHT,
            "width_m": WIDTH,
            "clearance_m": CLEARANCE,
            "lift_m": ms.LIFT_M,
            "other_arm": "P0 (parked)",
            "script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            "git_head": subprocess.run(
                ["git", "-C", str(BASE), "rev-parse", "HEAD"],
                capture_output=True, text=True, check=False,
            ).stdout.strip() or None,
            "plan_seconds": round(elapsed, 1),
            "units": {
                "position_error_mm": "distance palm face -> linear reference point, millimetres",
                "orientation_error_deg": "rotation-vector angle vs commanded rotation, degrees",
                "clearances": "metres",
            },
            "definitions": {
                "position_error": "deviation from the straight-line start->end reference at the same t",
                "endpoint_position_error": "same metric at t=1, i.e. achievement of the transition end pose",
                "reference_checks": "static IK residuals at the six named states",
            },
        },
        "operation_status": operation["status"],
        "reference_checks": reference_checks,
        "transitions": transitions_out,
        "overall": overall,
    }

    OUT.write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")

    print(f"operation_status={operation['status']} transitions={len(valid)}/{len(transitions_out)} "
          f"plan_seconds={elapsed:.1f}")
    header = f"{'transition':28s} {'method':16s} {'n':>4s} {'max|dP|mm':>10s} {'max ori':>8s} " \
             f"{'min mar':>8s} {'max cond':>8s} {'min clr mm':>10s} {'end dP':>8s} {'end ori':>8s}"
    print(header)
    for item in transitions_out:
        if item["status"] != "valid":
            print(f"{item['from']+'->'+item['to']:28s} {item['status']:16s} reason={item.get('reason')}")
            continue
        s = item["summary"]
        print(
            f"{item['from']+'->'+item['to']:28s} {str(s['method']):16s} {s['sample_count']:4d} "
            f"{s['max_position_error_mm']:10.2f} {s['max_orientation_error_deg']:8.3f} "
            f"{s['min_margin_deg']:8.2f} {s['max_condition']:8.2f} "
            f"{(s['min_clearance_m'] or 0.0) * 1000:10.1f} "
            f"{s['endpoint_position_error_mm']:8.3f} {s['endpoint_orientation_error_deg']:8.4f}"
        )
    print(
        f"OVERALL samples={overall['sample_count']} max|dP|={overall['max_position_error_mm']:.2f} mm "
        f"max_ori={overall['max_orientation_error_deg']:.3f} deg min_margin={overall['min_margin_deg']:.2f} deg "
        f"max_cond={overall['max_condition']:.2f} min_clear={(overall['min_clearance_m'] or 0.0)*1000:.1f} mm"
    )
    print(f"written: {OUT}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
