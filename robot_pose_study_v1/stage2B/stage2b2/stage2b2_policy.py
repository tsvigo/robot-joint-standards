#!/usr/bin/env python3
"""Stage 2B.2 — corrected IK policy and step validator.

Purpose
-------
1. Remove the uncontrolled null-space drift of ``ik_study.solve_once`` (the
   ``0.03 * N @ limit_gradient`` term keeps pushing while the task error is
   above tolerance and produces a large terminal step when the pose barely
   changes).
2. Keep the useful part of redundancy: a deliberate, bounded null-space move
   toward a chosen anchor configuration.  The step is applied one increment per
   sample and is never applied while the pose is not reached, so drift and
   intentional improvement are cleanly separated.
3. Plan the full closed work cycle with the anchor at the start configuration.
   If the policy works, the cycle closes by q without a terminal corrective
   manoeuvre.
4. Compare it with the old approach (stage 2B.1 strict trajectory followed by
   the terminal self-motion) on six criteria for P7 R, c = 0.05 m.
5. Produce a step validator that records, per interval, the per-joint step,
   the Euclidean norm of dq, the link and palm displacement, the continuity
   classification and the between-sample collision certificate.

The 3-5 deg range is a diagnostic threshold for adaptive refinement, not a
hard ban: any step above it is flagged and checked interval by interval.

Read-only with respect to the .blend, the stage 2A/2B sources and the stage
2B.1 artifacts.  Writes only into this directory.
"""

from __future__ import annotations

import csv
import json
import math
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
sys.path.insert(0, str(HERE))

import bed_study as bed  # noqa: E402
import fk_model  # noqa: E402
import ik_study as ik  # noqa: E402
import motion_study as ms  # noqa: E402
import stage2b2_control_p7r as ctl  # noqa: E402

# Stage 2B.1 unilateral artifacts were built with MODE = "6d"
# (stage2b1_unilateral.py:345, metadata "mode": "6d").  The corrected policy
# must use the same mode, otherwise the static references differ.
ms.MODE = "6d"

OPERATION = "P7"
SIDE = "R"
CLEARANCE = 0.05
HEIGHT = 0.7
WIDTH = 1.0
FINGER_YAW = 80.0
JOINTS = ms.JOINT_KEYS

ARTIFACT = BASE / "stage2B" / "stage2b1" / "stage2b1_unilateral_P7_R_c0.05.json"
OUT_JSON = HERE / "stage2b2_policy_p7r.json"
OUT_STEPS = HERE / "stage2b2_policy_p7r_steps.csv"
OUT_CMP = HERE / "stage2b2_policy_p7r_compare.csv"

# --- policy parameters -----------------------------------------------------
TASK_TOL = 1e-6           # scaled task error accepted at a sample
TASK_ITER_MAX = 60        # task-only iterations per sample
STEP_CAP_DEG = 6.0        # per-iteration task step cap
BIAS_CAP_DEG = 0.35       # max injected null-space motion per sample
BIAS_STEPS = 1            # null-space increments per sample
WAYPOINT_BIAS_STEPS = 3   # increments at a reference waypoint (pose is fixed)
POSE_POS_TOL_MM = 0.01    # pose must stay reached during a null-space move
POSE_ORI_TOL_DEG = 1e-3

# --- validator thresholds (diagnostic, not gate) ---------------------------
DIAG_LOW_DEG = 3.0
DIAG_HIGH_DEG = 5.0


# ---------------------------------------------------------------- helpers
def geometry() -> dict:
    return bed.bed_geometry("lateral", HEIGHT, WIDTH, CLEARANCE)


def finger_vector() -> np.ndarray:
    yaw = math.radians(FINGER_YAW)
    return np.array([math.sin(yaw), math.cos(yaw), 0.0], dtype=float)


def references(geom: dict) -> list[dict]:
    """Static references, same seeds and finger as stage2b1_unilateral.py."""
    sequence = ms.operation_sequence(OPERATION, SIDE, geom)
    rotation = bed.orientation_from_normal_finger(SIDE, bed.DOWN, finger_vector())
    for item in sequence:
        item["target"]["rotation"] = rotation.tolist()
        item["target"]["finger"] = [float(value) for value in finger_vector()]
    out = []
    for item in sequence:
        seed = zlib.crc32(f"{OPERATION}|{SIDE}|{item['label']}|{geom['clearance_m']}".encode("utf-8"))
        result = ms.static_solve(SIDE, item["target"], int(seed))
        chosen = ms.pick_reference(result, SIDE, geom)
        out.append(
            {
                "label": item["label"],
                "target": item["target"],
                "position_floor": item["position_floor"],
                "q_deg": chosen["q_deg"],
                "solutions": [s["q_deg"] for s in result.get("solutions", []) if s.get("converged")],
            }
        )
    return out


def q_array(q_deg: dict) -> np.ndarray:
    return np.array([q_deg[key] for key in JOINTS], dtype=float)


def q_dict(values: np.ndarray) -> dict:
    return {key: float(value) for key, value in zip(JOINTS, values)}


def euclidean(a: dict, b: dict) -> float:
    return float(math.sqrt(sum((float(b[k]) - float(a[k])) ** 2 for k in JOINTS)))


def per_joint(a: dict, b: dict) -> dict:
    return {key: float(b[key]) - float(a[key]) for key in JOINTS}


# ---------------------------------------------------------------- policy
def track_sample(side: str, target: dict, q_seed: dict, tol: float = TASK_TOL) -> tuple[dict, dict, int]:
    """Task-only damped least squares.

    No null-space term at all: the step is ``J^+ e``, the minimum-norm task
    correction, and the loop stops as soon as the scaled task error is below
    ``tol``.  Therefore no self-motion can accumulate between samples.
    """
    q = ik.clip_q(side, q_array(q_seed))
    damping = 1e-3
    qd = q_dict(q)
    best = ik.task_metrics(side, qd, ms.LIFT_M, target, ms.MODE)
    iterations = 0
    for iteration in range(TASK_ITER_MAX):
        iterations = iteration + 1
        if float(best["task_error_norm"]) <= tol:
            break
        jacobian, error, _ = ctl.task_jacobian(side, qd, target)
        _, pinv = ctl.nullspace(jacobian, damping)
        step = pinv @ error
        norm = float(np.linalg.norm(step))
        if norm > math.radians(STEP_CAP_DEG):
            step *= math.radians(STEP_CAP_DEG) / norm
        candidate = ik.clip_q(side, q + np.degrees(step))
        candidate_metrics = ik.task_metrics(side, q_dict(candidate), ms.LIFT_M, target, ms.MODE)
        if float(candidate_metrics["task_error_norm"]) < float(best["task_error_norm"]):
            q = candidate
            qd = q_dict(q)
            best = candidate_metrics
            damping = max(damping * 0.7, 1e-6)
            continue
        damping = min(damping * 2.0, 1e2)
        half = ik.clip_q(side, q + np.degrees(0.5 * step))
        half_metrics = ik.task_metrics(side, q_dict(half), ms.LIFT_M, target, ms.MODE)
        if float(half_metrics["task_error_norm"]) < float(best["task_error_norm"]):
            q = half
            qd = q_dict(q)
            best = half_metrics
            damping = max(damping * 0.7, 1e-6)
            continue
        if damping >= 1e2:
            break
    return qd, best, iterations


def null_bias(
    side: str,
    target: dict,
    q_deg: dict,
    anchor_deg: dict,
    cap_deg: float = BIAS_CAP_DEG,
    steps: int = BIAS_STEPS,
) -> tuple[dict, float]:
    """Bounded null-space move toward the anchor, followed by re-projection.

    The move is applied only when the pose is already reached.  Each increment
    is capped by ``cap_deg`` (max joint change), so the injected self-motion is
    explicit and can be accounted for instead of emerging from the solver.
    """
    current = q_array(q_deg)
    anchor = q_array(anchor_deg)
    injected = 0.0
    for _ in range(steps):
        qd = q_dict(current)
        jacobian, _, task = ctl.task_jacobian(side, qd, target)
        pos_mm = float(np.linalg.norm(task["position_error"]) * 1000.0)
        if pos_mm > POSE_POS_TOL_MM or float(task["orientation_error_deg"]) > POSE_ORI_TOL_DEG:
            break
        projector, _ = ctl.nullspace(jacobian)
        direction = projector @ (anchor - current)
        largest = float(np.max(np.abs(direction)))
        if largest < 1e-9:
            break
        if largest > cap_deg:
            direction *= cap_deg / largest
        candidate = ik.clip_q(side, current + direction)
        corrected, _ = ctl.project_to_pose(side, candidate, target, iterations=4)
        check = ik.task_metrics(side, q_dict(corrected), ms.LIFT_M, target, ms.MODE)
        if float(np.linalg.norm(check["position_error"]) * 1000.0) > POSE_POS_TOL_MM:
            break
        if float(check["orientation_error_deg"]) > POSE_ORI_TOL_DEG:
            break
        moved = float(np.max(np.abs(corrected - current)))
        if moved < 1e-9:
            break
        current = corrected
        injected = max(injected, moved)
    return q_dict(current), injected


def anchor_for(refs: list[dict], segment: int, fraction: float, mode: str) -> dict:
    """Redundancy anchor for one sample.

    ``start``   - fixed start configuration (naive loop closure);
    ``schedule``- linear schedule between the endpoint references of the
                  transition, i.e. the known feasible redundancy plan.
    """
    if mode == "start":
        return refs[0]["q_deg"]
    return ctl.lerp_q(refs[segment]["q_deg"], refs[segment + 1]["q_deg"], fraction)


def plan_cycle(
    side: str,
    refs: list[dict],
    geom: dict,
    anchor_mode: str = "schedule",
    bias: bool = True,
    cap_deg: float = BIAS_CAP_DEG,
) -> list[dict]:
    """Full closed cycle S0 -> S1 -> ... -> S5 with one continuous q path.

    Task-only tracking keeps the pose without null-space drift.  The
    redundancy is then moved explicitly toward the scheduled anchor, one
    bounded increment per sample.  If tracking ever fails, the sample is
    re-projected from the anchor, which is a feasible neighbourhood.
    """
    samples: list[dict] = []
    q_current = {key: float(value) for key, value in refs[0]["q_deg"].items()}
    start = ctl.metrics(side, q_current, refs[0]["target"], geom, ms.P0_Q)
    start.update({"t": 0.0, "segment": 0, "segment_t": 0.0, "injected_deg": 0.0, "iterations": 0,
                  "waypoint": refs[0]["label"], "fallback": False})
    samples.append(start)

    for segment, (first, second) in enumerate(zip(refs, refs[1:])):
        p0 = np.asarray(first["position_floor"], dtype=float)
        p1 = np.asarray(second["position_floor"], dtype=float)
        distance = float(np.linalg.norm(p1 - p0))
        count = max(4, min(ms.MAX_SAMPLES, int(math.ceil(distance / ms.CART_STEP_M))))
        for index in range(1, count + 1):
            fraction = index / count
            target = ms.interp_target(first["target"], second["target"], fraction)
            anchor = anchor_for(refs, segment, fraction, anchor_mode)

            # Seed from the scheduled anchor: the redundancy is planned, not
            # left to the solver.  Tracking then only closes the small gap
            # between the (interpolated) anchor and the actual pose manifold.
            q_track, task, iterations = track_sample(side, target, anchor)
            fallback = False
            pos_mm = float(np.linalg.norm(task["position_error"]) * 1000.0)
            ori_deg = float(task["orientation_error_deg"])
            if pos_mm > POSE_POS_TOL_MM or ori_deg > 1e-2:
                alt, _ = ctl.project_to_pose(side, q_array(anchor), target, iterations=8)
                alt_metrics = ik.task_metrics(side, q_dict(alt), ms.LIFT_M, target, ms.MODE)
                alt_pos = float(np.linalg.norm(alt_metrics["position_error"]) * 1000.0)
                if alt_pos < pos_mm or float(alt_metrics["orientation_error_deg"]) < ori_deg:
                    q_track = q_dict(alt)
                    fallback = True

            # Continuity guard: never accept a jump relative to the previous
            # sample if a nearby solution of the same pose exists.
            step_from_previous = max(
                abs(float(q_track[key]) - float(q_current[key])) for key in JOINTS
            )
            if step_from_previous > DIAG_HIGH_DEG:
                q_alt, _, iterations_alt = track_sample(side, target, q_current)
                alt_metrics = ik.task_metrics(side, q_alt, ms.LIFT_M, target, ms.MODE)
                alt_pos = float(np.linalg.norm(alt_metrics["position_error"]) * 1000.0)
                alt_ori = float(alt_metrics["orientation_error_deg"])
                alt_step = max(abs(float(q_alt[key]) - float(q_current[key])) for key in JOINTS)
                if alt_pos <= POSE_POS_TOL_MM and alt_ori <= 1e-2 and alt_step < step_from_previous:
                    q_track = q_alt
                    iterations = iterations_alt
                    fallback = True

            injected = 0.0
            if bias:
                steps = WAYPOINT_BIAS_STEPS if index == count else BIAS_STEPS
                q_new, injected = null_bias(side, target, q_track, anchor, cap_deg=cap_deg, steps=steps)
            else:
                q_new = q_track
            row = ctl.metrics(side, q_new, target, geom, ms.P0_Q)
            row.update(
                {
                    "t": 0.0,  # filled after the loop with the global arc index
                    "segment": segment,
                    "segment_t": float(fraction),
                    "injected_deg": float(injected),
                    "iterations": int(iterations),
                    "waypoint": second["label"] if index == count else None,
                    "fallback": fallback,
                }
            )
            samples.append(row)
            q_current = q_new

    for index, row in enumerate(samples):
        row["t"] = index / max(1, len(samples) - 1)
    return samples


# ---------------------------------------------------------------- validator
def link_displacement(side: str, q_a: dict, q_b: dict) -> float:
    points_a = ctl.link_points(side, q_a)
    points_b = ctl.link_points(side, q_b)
    return float(np.max(np.linalg.norm(points_b - points_a, axis=1)))


def palm_displacement(side: str, q_a: dict, q_b: dict) -> float:
    palm_a = fk_model.palm_geometry(side, q_a, ms.LIFT_M)["face_floor"]
    palm_b = fk_model.palm_geometry(side, q_b, ms.LIFT_M)["face_floor"]
    return float(np.linalg.norm(np.asarray(palm_b) - np.asarray(palm_a)))


def step_validator(side: str, samples: list[dict]) -> list[dict]:
    """Per-interval diagnostics.

    ``flag`` is diagnostic: ``ok`` below 3 deg, ``refine`` between 3 and 5 deg
    (interval is additionally refined during certification), ``anomaly`` above
    5 deg.  A large step is not forbidden, but it must be explained by task
    motion or by an explicitly injected null-space move.
    """
    rows = []
    for index, (first, second) in enumerate(zip(samples, samples[1:])):
        diff = per_joint(first["q_deg"], second["q_deg"])
        absolute = {key: abs(value) for key, value in diff.items()}
        joint = max(absolute, key=absolute.get)
        step = absolute[joint]
        if step <= DIAG_LOW_DEG:
            flag = "ok"
        elif step <= DIAG_HIGH_DEG:
            flag = "refine"
        else:
            flag = "anomaly"
        injected = float(second.get("injected_deg") or 0.0)
        if injected > 0.0 and step <= injected * 2.0 + 1e-9:
            kind = "injected_self_motion"
        elif second.get("waypoint"):
            kind = "waypoint"
        else:
            kind = "task_tracking"
        rows.append(
            {
                "interval": index,
                "from_t": float(first["t"]),
                "to_t": float(second["t"]),
                "segment": int(second.get("segment", 0)),
                "max_joint": joint,
                "max_joint_step_deg": float(step),
                "euclidean_dq_deg": euclidean(first["q_deg"], second["q_deg"]),
                "link_displacement_m": link_displacement(side, first["q_deg"], second["q_deg"]),
                "palm_displacement_m": palm_displacement(side, first["q_deg"], second["q_deg"]),
                "injected_deg": injected,
                "position_error_before_mm": float(first["position_error_mm"]),
                "position_error_after_mm": float(second["position_error_mm"]),
                "margin_before_deg": float(first["min_margin_deg"]),
                "margin_after_deg": float(second["min_margin_deg"]),
                "flag": flag,
                "kind": kind,
                **{f"dq_{key}_deg": float(diff[key]) for key in JOINTS},
            }
        )
    return rows


def continuity_summary(side: str, samples: list[dict]) -> dict:
    rows = step_validator(side, samples)
    joints = {key: 0.0 for key in JOINTS}
    for row in rows:
        for key in JOINTS:
            joints[key] = max(joints[key], abs(row[f"dq_{key}_deg"]))
    return {
        "interval_count": len(rows),
        "max_joint_step_deg": max((row["max_joint_step_deg"] for row in rows), default=0.0),
        "max_euclidean_dq_deg": max((row["euclidean_dq_deg"] for row in rows), default=0.0),
        "max_joint_by_joint_deg": joints,
        "max_link_displacement_m": max((row["link_displacement_m"] for row in rows), default=0.0),
        "max_palm_displacement_m": max((row["palm_displacement_m"] for row in rows), default=0.0),
        "refine_intervals": sum(1 for row in rows if row["flag"] == "refine"),
        "anomaly_intervals": sum(1 for row in rows if row["flag"] == "anomaly"),
        "max_position_error_mm": max((row["position_error_after_mm"] for row in rows), default=0.0),
        "max_orientation_error_deg": max((sample["orientation_error_deg"] for sample in samples), default=0.0),
        "collision_free": all(bool(sample["collision_free"]) for sample in samples),
        "within_limits": all(bool(sample["within_limits"]) for sample in samples),
        "injected_total_deg": sum(float(row["injected_deg"]) for row in rows),
    }


def path_metrics(side: str, samples: list[dict]) -> dict:
    length = sum(euclidean(a["q_deg"], b["q_deg"]) for a, b in zip(samples, samples[1:]))
    per_joint_length = {
        key: sum(abs(float(b["q_deg"][key]) - float(a["q_deg"][key])) for a, b in zip(samples, samples[1:]))
        for key in JOINTS
    }
    worst_margin = min(samples, key=lambda row: row["min_margin_deg"])
    worst_condition = max(samples, key=lambda row: row["condition"])
    worst_position = max(samples, key=lambda row: row["position_error_mm"])
    worst_orientation = max(samples, key=lambda row: row["orientation_error_deg"])
    worst_clearance = min(samples, key=lambda row: row["clearance_min_m"])
    return {
        "sample_count": len(samples),
        "joint_path_length_deg": length,
        "joint_path_length_by_joint_deg": per_joint_length,
        "min_margin_deg": float(worst_margin["min_margin_deg"]),
        "min_margin_joint": worst_margin["min_margin_joint"],
        "min_margin_at": float(worst_margin["t"]),
        "max_condition": float(worst_condition["condition"]),
        "max_position_deviation_mm": float(worst_position["position_error_mm"]),
        "max_orientation_deviation_deg": float(worst_orientation["orientation_error_deg"]),
        "min_clearance_m": float(worst_clearance["clearance_min_m"]),
        "min_clearance_at": float(worst_clearance["t"]),
        "min_arm_bed_m": min(float(row["arm_bed_m"]) for row in samples),
        "min_arm_arm_m": min(float(row["arm_arm_m"]) for row in samples),
        "min_arm_torso_m": min(float(row["arm_torso_m"]) for row in samples),
        "min_arm_base_m": min(float(row["arm_base_m"]) for row in samples),
        "collision_free": all(bool(row["collision_free"]) for row in samples),
        "within_limits": all(bool(row["within_limits"]) for row in samples),
        "end_margin_deg": float(samples[-1]["min_margin_deg"]),
        "end_margin_joint": samples[-1]["min_margin_joint"],
    }


def closure(side: str, start: dict, end: dict) -> dict:
    diff = per_joint(start, end)
    return {
        "max_abs_deg": max(abs(value) for value in diff.values()),
        "euclidean_deg": euclidean(start, end),
        "delta_deg": diff,
    }


def certify_path(side: str, samples: list[dict], geom: dict) -> dict:
    """Interval certificate; adaptive refinement follows the diagnostic flags."""
    intervals = []
    undefined = 0
    minimum = None
    for index, (first, second) in enumerate(zip(samples, samples[1:])):
        step = max(abs(float(second["q_deg"][key]) - float(first["q_deg"][key])) for key in JOINTS)
        # The certificate refines on its own when the bound is not conclusive;
        # the diagnostic flag only records that this interval was refined.
        depth = 0
        node = ctl.certify_interval(side, first["q_deg"], second["q_deg"], geom, depth)
        leaves: list[dict] = []
        ctl.flatten_certification(node, leaves)
        bad = sum(1 for leaf in leaves if leaf["status"] != "certified")
        undefined += bad
        clearance = min(leaf["clearance_m"] for leaf in leaves)
        bound = max(leaf["bound_m"] for leaf in leaves)
        minimum = clearance if minimum is None else min(minimum, clearance)
        intervals.append(
            {
                "interval": index,
                "from_t": float(first["t"]),
                "to_t": float(second["t"]),
                "max_joint_step_deg": step,
                "initial_depth": depth,
                "leaf_count": len(leaves),
                "clearance_m": float(clearance),
                "bound_m": float(bound),
                "margin_m": float(clearance - bound),
                "status": "certified" if bad == 0 else "undefined",
            }
        )
    return {
        "interval_count": len(intervals),
        "certified": sum(1 for item in intervals if item["status"] == "certified"),
        "undefined": undefined,
        "min_clearance_m": minimum,
        "intervals": intervals,
    }


# ---------------------------------------------------------------- comparison
def old_approach(side: str, refs: list[dict], geom: dict) -> dict:
    """Stage 2B.1 strict trajectory followed by the terminal self-motion."""
    artifact = json.loads(ARTIFACT.read_text())
    strict = artifact["policies"]["strict_6d"]
    samples: list[dict] = []
    for transition in strict["transitions"]:
        for sample in transition["samples"]:
            if samples and abs(float(sample["t"]) - 0.0) < 1e-12:
                continue  # seam duplicate: the previous transition already ended here
            row = dict(sample)
            row["t"] = float(sample["t"])
            row["injected_deg"] = 0.0
            row["waypoint"] = None
            row["segment"] = len(samples)
            samples.append(row)
    start_q = samples[0]["q_deg"]
    end_q = samples[-1]["q_deg"]
    goal_q = refs[5]["q_deg"]
    motion = ctl.self_motion(side, refs[5]["target"], end_q, goal_q, geom)
    motion_samples = motion["samples"]
    for index, row in enumerate(motion_samples):
        row["t"] = 1.0 + index / max(1, len(motion_samples) - 1)
    combined = samples + motion_samples
    for index, row in enumerate(combined):
        row["t"] = index / max(1, len(combined) - 1)

    metrics = path_metrics(side, samples)
    terminal = path_metrics(side, motion_samples)
    total = dict(metrics)
    total["joint_path_length_deg"] = metrics["joint_path_length_deg"] + terminal["joint_path_length_deg"]
    total["joint_path_length_by_joint_deg"] = {
        key: metrics["joint_path_length_by_joint_deg"][key] + terminal["joint_path_length_by_joint_deg"][key]
        for key in JOINTS
    }
    total["min_margin_deg"] = min(metrics["min_margin_deg"], terminal["min_margin_deg"])
    total["max_condition"] = max(metrics["max_condition"], terminal["max_condition"])
    total["max_position_deviation_mm"] = max(
        metrics["max_position_deviation_mm"], terminal["max_position_deviation_mm"]
    )
    total["max_orientation_deviation_deg"] = max(
        metrics["max_orientation_deviation_deg"], terminal["max_orientation_deviation_deg"]
    )
    total["min_clearance_m"] = min(metrics["min_clearance_m"], terminal["min_clearance_m"])
    total["min_arm_bed_m"] = min(metrics["min_arm_bed_m"], terminal["min_arm_bed_m"])
    total["min_arm_arm_m"] = min(metrics["min_arm_arm_m"], terminal["min_arm_arm_m"])
    total["min_arm_torso_m"] = min(metrics["min_arm_torso_m"], terminal["min_arm_torso_m"])
    total["min_arm_base_m"] = min(metrics["min_arm_base_m"], terminal["min_arm_base_m"])
    total["collision_free"] = metrics["collision_free"] and terminal["collision_free"]
    total["within_limits"] = metrics["within_limits"] and terminal["within_limits"]
    total["end_margin_deg"] = float(terminal["end_margin_deg"])
    total["end_margin_joint"] = terminal["end_margin_joint"]
    total["sample_count"] = len(combined)
    return {
        "method": "old_strict_plus_terminal_self_motion",
        "trajectory": total,
        "trajectory_without_terminal": metrics,
        "terminal_self_motion": terminal,
        "terminal_steps": int(motion["steps_used"]),
        "terminal_status": motion["status"],
        "closure_by_q": closure(side, start_q, goal_q),
        "continuity": continuity_summary(side, combined),
        "certificate": certify_path(side, combined, geom),
        "samples": combined,
    }


def write_csv(path: Path, rows: list[dict], fields: list[str]) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row.get(key) for key in fields})


def main() -> int:
    started = time.time()
    geom = geometry()
    refs = references(geom)
    if not all(ref["q_deg"] for ref in refs):
        print("static reference unreachable", file=sys.stderr)
        return 1

    anchor = refs[0]["q_deg"]
    variants: dict[str, dict] = {}
    for mode in ("schedule", "start"):
        samples = plan_cycle(SIDE, refs, geom, anchor_mode=mode, bias=True)
        variants[mode] = {
            "anchor_mode": mode,
            "samples": samples,
            "metrics": path_metrics(SIDE, samples),
            "closure": closure(SIDE, refs[0]["q_deg"], samples[-1]["q_deg"]),
            "continuity": continuity_summary(SIDE, samples),
        }

    def score(variant: dict) -> tuple:
        metrics = variant["metrics"]
        pose_ok = (
            metrics["max_position_deviation_mm"] <= 1.0
            and metrics["max_orientation_deviation_deg"] <= 0.5
        )
        return (pose_ok, metrics["min_margin_deg"], -metrics["joint_path_length_deg"])

    chosen = max(variants, key=lambda key: score(variants[key]))
    new_samples = variants[chosen]["samples"]
    new_metrics = variants[chosen]["metrics"]
    new_closure = variants[chosen]["closure"]
    new_continuity = variants[chosen]["continuity"]

    old = old_approach(SIDE, refs, geom)

    new_certificate = certify_path(SIDE, new_samples, geom)

    waypoint_rows = [
        {
            "label": sample["waypoint"],
            "t": sample["t"],
            "margin_deg": sample["min_margin_deg"],
            "margin_joint": sample["min_margin_joint"],
            "condition": sample["condition"],
            "clearance_m": sample["clearance_min_m"],
            "q_deg": sample["q_deg"],
        }
        for sample in new_samples
        if sample.get("waypoint")
    ]

    payload = {
        "metadata": {
            "stage": "2B.2",
            "kind": "corrected_ik_policy_and_step_validator",
            "created_utc": datetime.now(timezone.utc).isoformat(),
            "operation": OPERATION,
            "side": SIDE,
            "clearance_m": CLEARANCE,
            "height_m": HEIGHT,
            "width_m": WIDTH,
            "finger_yaw_deg": FINGER_YAW,
            "mode": ms.MODE,
            "source_artifact": str(ARTIFACT.relative_to(BASE)),
            "policy": {
                "task_tol": TASK_TOL,
                "task_iter_max": TASK_ITER_MAX,
                "step_cap_deg": STEP_CAP_DEG,
                "bias_cap_deg": BIAS_CAP_DEG,
                "bias_steps_per_sample": BIAS_STEPS,
                "bias_steps_at_waypoint": WAYPOINT_BIAS_STEPS,
                "anchor": "S0_ready static reference",
                "diagnostic_thresholds_deg": [DIAG_LOW_DEG, DIAG_HIGH_DEG],
            },
            "validator": {
                "records": ["per-joint step", "euclidean dq", "link displacement", "palm displacement",
                            "continuity class", "between-sample certificate"],
                "note": "3-5 deg is a diagnostic threshold for adaptive refinement, not a hard ban",
            },
        },
        "references": [
            {"label": ref["label"], "q_deg": ref["q_deg"], "position_floor": ref["position_floor"]}
            for ref in refs
        ],
        "new_cycle": {
            "method": "full_cycle_task_only_with_anchor_bias",
            "anchor_mode": chosen,
            "anchor_mode_selection": "pose tolerance first, then min margin, then shortest joint path",
            "metrics": new_metrics,
            "closure_by_q": new_closure,
            "continuity": new_continuity,
            "waypoints": waypoint_rows,
            "certificate": new_certificate,
            "fallback_samples": sum(1 for sample in new_samples if sample.get("fallback")),
            "variants": {
                mode: {
                    "metrics": variant["metrics"],
                    "closure_by_q": variant["closure"],
                    "continuity": variant["continuity"],
                }
                for mode, variant in variants.items()
            },
        },
        "old_cycle": {
            "method": old["method"],
            "metrics": old["trajectory"],
            "metrics_trajectory_only": old["trajectory_without_terminal"],
            "terminal_self_motion": old["terminal_self_motion"],
            "terminal_steps": old["terminal_steps"],
            "terminal_status": old["terminal_status"],
            "closure_by_q": old["closure_by_q"],
            "continuity": old["continuity"],
            "certificate": old["certificate"],
        },
        "comparison": {
            "criteria": [
                "joint_path_length_deg",
                "min_margin_deg",
                "max_condition",
                "max_position_deviation_mm",
                "min_clearance_m",
                "closure_max_abs_deg",
                "max_joint_step_deg",
                "certified_intervals",
            ],
            "new": {
                "joint_path_length_deg": new_metrics["joint_path_length_deg"],
                "min_margin_deg": new_metrics["min_margin_deg"],
                "max_condition": new_metrics["max_condition"],
                "max_position_deviation_mm": new_metrics["max_position_deviation_mm"],
                "max_orientation_deviation_deg": new_metrics["max_orientation_deviation_deg"],
                "min_clearance_m": new_metrics["min_clearance_m"],
                "closure_max_abs_deg": new_closure["max_abs_deg"],
                "max_joint_step_deg": new_continuity["max_joint_step_deg"],
                "certified_intervals": f"{new_certificate['certified']}/{new_certificate['interval_count']}",
                "terminal_manoeuvre": False,
            },
            "old": {
                "joint_path_length_deg": old["trajectory"]["joint_path_length_deg"],
                "min_margin_deg": old["trajectory"]["min_margin_deg"],
                "max_condition": old["trajectory"]["max_condition"],
                "max_position_deviation_mm": old["trajectory"]["max_position_deviation_mm"],
                "max_orientation_deviation_deg": old["trajectory"]["max_orientation_deviation_deg"],
                "min_clearance_m": old["trajectory"]["min_clearance_m"],
                "closure_max_abs_deg": old["closure_by_q"]["max_abs_deg"],
                "max_joint_step_deg": old["continuity"]["max_joint_step_deg"],
                "certified_intervals": f"{old['certificate']['certified']}/{old['certificate']['interval_count']}",
                "terminal_manoeuvre": True,
                "terminal_steps": old["terminal_steps"],
            },
        },
        "runtime_seconds": time.time() - started,
    }
    OUT_JSON.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")

    step_rows = step_validator(SIDE, new_samples)
    write_csv(
        OUT_STEPS,
        step_rows,
        [
            "interval", "from_t", "to_t", "segment", "max_joint", "max_joint_step_deg",
            "euclidean_dq_deg", "link_displacement_m", "palm_displacement_m", "injected_deg",
            "position_error_before_mm", "position_error_after_mm", "margin_before_deg",
            "margin_after_deg", "flag", "kind",
        ]
        + [f"dq_{key}_deg" for key in JOINTS],
    )

    compare_rows = []
    for name in ("new", "old"):
        row = {"approach": name}
        row.update(payload["comparison"][name])
        compare_rows.append(row)
    fields = ["approach"] + payload["comparison"]["criteria"] + [
        "max_orientation_deviation_deg", "terminal_manoeuvre", "terminal_steps"
    ]
    write_csv(OUT_CMP, compare_rows, [key for key in fields if key in compare_rows[0] or key == "terminal_steps"])

    print(json.dumps(payload["comparison"], indent=2, ensure_ascii=False))
    print(f"runtime {payload['runtime_seconds']:.1f} s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
