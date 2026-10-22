#!/usr/bin/env python3
"""Stage 2B.1 — one-arm 6D trajectories with two selection policies.

Read-only relative to the .blend, to stage 2A/2B sources and to existing
artifacts.  Writes result JSONs and light path JSONs next to this file.

Policies
--------
``as_is``
    Reproduces ``motion_study.plan_operation``: the candidate pool is
    task_space, via_lift and joint_lerp variants; a candidate is valid when it
    is collision free, inside the joint limits and its maximal joint step does
    not exceed ``MAX_PREFERRED_JOINT_STEP_DEG``; the winner maximises
    ``candidate_key`` (margin, clearance, -condition, -length).
``strict_6d``
    Same candidate generators, but a candidate is accepted only if, in
    addition, its worst orientation error along the path is <= ``tol_ori`` and
    its worst deviation from the straight-line position reference is
    <= ``tol_pos``.  If no candidate qualifies the transition is ``not_found``.

The two policies are planned independently (a different winning candidate
changes the joint configuration at the start of the next transition).

Per-sample definitions
----------------------
``position_error_mm``
    Distance between the palm face and the linear reference point between the
    transition start and end targets at the same ``t``.
``orientation_error_deg``
    Rotation-vector angle between the commanded palm rotation and the actual
    palm rotation.  The commanded rotation is constant inside an operation.
``clearances``
    Simplified capsule/box distances in metres, other arm parked at P0.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import subprocess
import sys
import time
import zlib
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


def qdict(q: dict[str, float]) -> dict[str, float]:
    return {key: float(q.get(key, 0.0)) for key in ms.JOINT_KEYS}


def with_position(target: dict, position) -> dict:
    out = dict(target)
    out["position_floor"] = [float(value) for value in position]
    return out


def serialise_target(target: dict) -> dict:
    return {
        "position_floor": [float(value) for value in target["position_floor"]],
        "normal": [float(value) for value in target["normal"]],
        "rotation": [[float(value) for value in row] for row in target["rotation"]],
    }


def sample_metrics(side: str, sample: dict, target: dict, geom: dict) -> dict:
    metrics = ik.task_metrics(side, sample["q_deg"], ms.LIFT_M, target, "6d")
    clearances = bed.collision_clearances(side, sample["q_deg"], ms.LIFT_M, geom, other_q=ms.P0_Q)
    margin = metrics["min_margin"]
    return {
        "t": float(sample["t"]),
        "q_deg": qdict(sample["q_deg"]),
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
    }


def candidate_profile(side: str, candidate: dict, start_pos, end_pos, rotation_target: dict, geom: dict) -> dict:
    """Worst orientation error and worst linear deviation of a candidate."""
    if candidate["status"] != "valid" or not candidate.get("samples"):
        return {"max_orientation_error_deg": None, "max_position_deviation_mm": None}
    worst_ori = 0.0
    worst_pos = 0.0
    for sample in candidate["samples"]:
        t = float(sample["t"])
        position = start_pos + (end_pos - start_pos) * t
        metrics = ik.task_metrics(side, sample["q_deg"], ms.LIFT_M, with_position(rotation_target, position), "6d")
        worst_ori = max(worst_ori, float(metrics["orientation_error_deg"]))
        worst_pos = max(worst_pos, float(np.linalg.norm(metrics["position_error"]) * 1000.0))
    return {"max_orientation_error_deg": worst_ori, "max_position_deviation_mm": worst_pos}


def build_references(operation: str, side: str, geom: dict) -> list[dict]:
    sequence = ms.operation_sequence(operation, side, geom)
    references = []
    for item in sequence:
        seed = zlib.crc32(f"{operation}|{side}|{item['label']}|{geom['clearance_m']}".encode("utf-8"))
        result = ms.static_solve(side, item["target"], int(seed))
        chosen = ms.pick_reference(result, side, geom)
        references.append(
            {
                "label": item["label"],
                "position_floor": item["position_floor"],
                "target": item["target"],
                "reachable": chosen is not None,
                "q_deg": chosen.get("q_deg") if chosen else None,
                "solutions": [solution["q_deg"] for solution in result.get("solutions", []) if solution.get("converged")],
                "static": {
                    "position_error_mm": ms.finite(chosen.get("position_error_mm")) if chosen else None,
                    "normal_error_deg": ms.finite(chosen.get("normal_error_deg")) if chosen else None,
                    "min_margin_deg": ms.finite(chosen.get("min_margin_deg")) if chosen else None,
                    "condition": ms.finite(chosen.get("condition")) if chosen else None,
                },
            }
        )
    return references


def candidates_for(side, start, end, q_start, end_candidates, geom, seed):
    candidates = [
        ms.plan_task_segment(side, start["target"], end["target"], q_start, geom, ms.P0_Q, seed + 10),
        ms.plan_via_segment(side, start["target"], end["target"], q_start, geom, ms.P0_Q, seed + 20),
    ]
    primary_end = end_candidates[0] if end_candidates else None
    if primary_end is not None:
        candidates.append(
            ms.plan_joint_lerp(side, start["target"], end["target"], q_start, primary_end, geom, ms.P0_Q)
        )
        alternatives = sorted(end_candidates[1:], key=lambda candidate: ms.q_distance(side, q_start, candidate))[:2]
        for index, alternative in enumerate(alternatives):
            candidates.append(
                ms.plan_joint_lerp(
                    side,
                    start["target"],
                    end["target"],
                    q_start,
                    alternative,
                    geom,
                    ms.P0_Q,
                    method=f"joint_lerp_alt{index + 1}",
                )
            )
    return candidates


def plan_operation(operation: str, side: str, geom: dict, policy: str, tol_ori: float, tol_pos_mm: float):
    references = build_references(operation, side, geom)
    if not all(reference["reachable"] for reference in references):
        return {"status": "not_found", "reason": "static_reference_unreachable", "references": references, "transitions": []}

    current_q = references[0]["q_deg"]
    transitions = []

    for index in range(len(references) - 1):
        start = references[index]
        end = references[index + 1]
        end_candidates = list(end["solutions"])
        if end["q_deg"] and all(ms.q_distance(side, end["q_deg"], candidate) > 1e-9 for candidate in end_candidates):
            end_candidates.insert(0, end["q_deg"])
        seed = zlib.crc32(
            f"{operation}|{side}|{start['label']}|{end['label']}|{geom['clearance_m']}".encode("utf-8")
        )
        candidates = candidates_for(side, start, end, current_q, end_candidates, geom, int(seed))

        start_pos = np.asarray(start["position_floor"], dtype=float)
        end_pos = np.asarray(end["position_floor"], dtype=float)
        rotation_target = start["target"]

        profiles = [candidate_profile(side, candidate, start_pos, end_pos, rotation_target, geom) for candidate in candidates]

        valid = [candidate for candidate in candidates if candidate["status"] == "valid"]
        preferred = [
            candidate
            for candidate in valid
            if candidate.get("max_joint_step_deg") is not None
            and candidate["max_joint_step_deg"] <= ms.MAX_PREFERRED_JOINT_STEP_DEG
        ]
        pool = preferred or valid

        if policy == "as_is":
            # original pipeline rule: prefer candidates with a small joint step
            eligible = pool
        else:
            eligible = []
            for candidate, profile in zip(candidates, profiles):
                if candidate["status"] != "valid":
                    continue
                if profile["max_orientation_error_deg"] is None:
                    continue
                if profile["max_orientation_error_deg"] > tol_ori:
                    continue
                if profile["max_position_deviation_mm"] > tol_pos_mm:
                    continue
                eligible.append(candidate)

        selected = max(eligible, key=ms.candidate_key) if eligible else None

        candidate_rows = []
        for candidate, profile in zip(candidates, profiles):
            candidate_rows.append(
                {
                    "method": candidate.get("method"),
                    "status": candidate.get("status"),
                    "reason": candidate.get("reason"),
                    "sample_count": candidate.get("sample_count"),
                    "min_margin_deg": ms.finite(candidate.get("min_margin_deg")),
                    "max_condition": ms.finite(candidate.get("max_condition")),
                    "min_clearance_m": ms.finite(candidate.get("min_clearance_m")),
                    "min_arm_arm_m": ms.finite(candidate.get("min_arm_arm_m")),
                    "max_joint_step_deg": ms.finite(candidate.get("max_joint_step_deg")),
                    "path_length_deg": ms.finite(candidate.get("path_length_deg")),
                    "collision_free": bool(candidate.get("collision_free")),
                    "within_limits": bool(candidate.get("within_limits")),
                    "max_orientation_error_deg": profile["max_orientation_error_deg"],
                    "max_position_deviation_mm": profile["max_position_deviation_mm"],
                    "accepted": candidate is selected,
                }
            )

        if selected is None:
            transitions.append(
                {
                    "from": start["label"],
                    "to": end["label"],
                    "status": "not_found",
                    "reason": f"no_candidate_passed_{policy}",
                    "candidates": candidate_rows,
                    "samples": [],
                }
            )
            break

        records = []
        for sample in selected["samples"]:
            t = float(sample["t"])
            position = start_pos + (end_pos - start_pos) * t
            records.append(sample_metrics(side, sample, with_position(rotation_target, position), geom))

        def values(key):
            return [record[key] for record in records if record[key] is not None]

        summary = {
            "method": selected.get("method"),
            "sample_count": len(records),
            "max_position_deviation_mm": max(values("position_error_mm")),
            "max_orientation_error_deg": max(values("orientation_error_deg")),
            "min_margin_deg": min(values("min_margin_deg")),
            "max_condition": max(values("condition")),
            "min_clearance_m": min(values("clearance_min_m")),
            "min_arm_bed_m": min(values("arm_bed_m")),
            "min_arm_torso_m": min(values("arm_torso_m")),
            "max_joint_step_deg": float(
                max(ms.q_distance(side, first["q_deg"], second["q_deg"]) for first, second in zip(records, records[1:]))
                if len(records) > 1
                else 0.0
            ),
            "all_collision_free": all(record["collision_free"] for record in records),
            "all_within_limits": all(record["within_limits"] for record in records),
            "endpoint_position_error_mm": records[-1]["position_error_mm"] if records else None,
            "endpoint_orientation_error_deg": records[-1]["orientation_error_deg"] if records else None,
        }
        transitions.append(
            {
                "from": start["label"],
                "to": end["label"],
                "status": "valid",
                "reason": None,
                "selected_method": selected.get("method"),
                "summary": summary,
                "candidates": candidate_rows,
                "samples": records,
            }
        )
        current_q = selected["samples"][-1]["q_deg"]

    status = "valid" if len(transitions) == len(references) - 1 and all(t["status"] == "valid" for t in transitions) else "not_found"
    return {"status": status, "reason": None, "references": references, "transitions": transitions}


def light_path(operation: dict) -> dict:
    samples = []
    for transition in operation["transitions"]:
        for record in transition["samples"]:
            samples.append(
                {
                    "t": record["t"],
                    "q_deg": record["q_deg"],
                    "stage": f"{transition['from']}->{transition['to']}",
                }
            )
    for index, sample in enumerate(samples):
        sample["phase"] = index / max(1, len(samples) - 1)
    return {"samples": samples}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--op", default="P6", choices=("P6", "P7"))
    parser.add_argument("--side", default="L", choices=("L", "R"))
    parser.add_argument("--clearance", type=float, default=0.10)
    parser.add_argument("--height", type=float, default=0.7)
    parser.add_argument("--width", type=float, default=1.0)
    parser.add_argument("--yaw", type=float, default=80.0, help="P7 finger yaw, degrees")
    parser.add_argument("--tol-ori", type=float, default=0.5)
    parser.add_argument("--tol-pos", type=float, default=1.0)
    parser.add_argument("--policies", default="as_is,strict_6d")
    args = parser.parse_args()

    if args.op == "P7":
        yaw = math.radians(args.yaw)
        finger = np.array([math.sin(yaw), math.cos(yaw), 0.0], dtype=float)
    else:
        finger = bed.PLUS_X.copy()

    ms.MODE = "6d"
    geom = bed.bed_geometry("lateral", args.height, args.width, args.clearance)

    policies = [item.strip() for item in args.policies.split(",") if item.strip()]
    started = time.time()
    results = {}
    for policy in policies:
        policy_started = time.time()
        results[policy] = plan_operation(args.op, args.side, geom, policy, args.tol_ori, args.tol_pos)
        results[policy]["seconds"] = round(time.time() - policy_started, 1)
    elapsed = time.time() - started

    tag = f"{args.op}_{args.side}_c{args.clearance:.2f}"
    output = {
        "metadata": {
            "stage": "2B.1",
            "kind": "unilateral_6d",
            "created_utc": datetime.now(timezone.utc).isoformat(),
            "operation": args.op,
            "side": args.side,
            "mode": "6d",
            "approach": "lateral",
            "height_m": args.height,
            "width_m": args.width,
            "clearance_m": args.clearance,
            "lift_m": ms.LIFT_M,
            "finger_yaw_deg": args.yaw if args.op == "P7" else 0.0,
            "finger": [float(value) for value in finger],
            "other_arm": "P0 (parked)",
            "policies": policies,
            "strict_tolerances": {"orientation_deg": args.tol_ori, "position_mm": args.tol_pos},
            "script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            "git_head": subprocess.run(
                ["git", "-C", str(BASE), "rev-parse", "HEAD"], capture_output=True, text=True, check=False
            ).stdout.strip() or None,
            "seconds": round(elapsed, 1),
            "definitions": {
                "position_error_mm": "palm face vs linear start->end reference at the same t",
                "orientation_error_deg": "rotation-vector angle vs the constant commanded rotation",
                "endpoint_*": "t=1 values, i.e. achievement of the transition end pose",
            },
        },
        "operation_status": {policy: results[policy]["status"] for policy in policies},
        "references": [
            {
                "label": reference["label"],
                "position_floor": reference["position_floor"],
                "reachable": reference["reachable"],
                "static": reference["static"],
            }
            for reference in results[policies[0]]["references"]
        ],
        "policies": {
            policy: {
                "status": results[policy]["status"],
                "reason": results[policy].get("reason"),
                "seconds": results[policy]["seconds"],
                "transitions": results[policy]["transitions"],
            }
            for policy in policies
        },
    }

    out_json = HERE / f"stage2b1_unilateral_{tag}.json"
    out_json.write_text(json.dumps(output, indent=2, ensure_ascii=False), encoding="utf-8")

    for policy in policies:
        if results[policy]["status"] == "valid":
            path = light_path(results[policy])
            path["metadata"] = {"operation": args.op, "side": args.side, "policy": policy, "clearance_m": args.clearance}
            (HERE / f"stage2b1_path_{tag}_{policy}.json").write_text(
                json.dumps(path, indent=2, ensure_ascii=False), encoding="utf-8"
            )

    for policy in policies:
        print(f"[{policy}] status={results[policy]['status']} seconds={results[policy]['seconds']}")
        for transition in results[policy]["transitions"]:
            if transition["status"] != "valid":
                print(f"  {transition['from']}->{transition['to']} FAILED: {transition.get('reason')}")
                for row in transition["candidates"]:
                    print(
                        f"     {row['method']:18s} {row['status']:8s} ori={row['max_orientation_error_deg']} "
                        f"pos={row['max_position_deviation_mm']}"
                    )
                continue
            summary = transition["summary"]
            print(
                f"  {transition['from']}->{transition['to']:10s} {summary['method']:18s} "
                f"n={summary['sample_count']:3d} ori={summary['max_orientation_error_deg']:6.3f} "
                f"pos={summary['max_position_deviation_mm']:7.2f}mm margin={summary['min_margin_deg']:6.2f} "
                f"cond={summary['max_condition']:5.2f} clr={summary['min_clearance_m']*1000:6.1f}mm "
                f"end={summary['endpoint_position_error_mm']:.3f}mm/{summary['endpoint_orientation_error_deg']:.4f}deg"
            )

    print(f"written: {out_json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
