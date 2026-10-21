#!/usr/bin/env python3
"""Stage 2B: continuous motion study for P6/P7.

The module plans and validates joint trajectories for the two 7-DOF arms.
Linear joint interpolation is only a candidate; it is accepted only after
limits, collision and conditioning checks along the whole sampled path.
The collision geometry is the simplified stage 2A geometry.  This is not a
safety assessment and does not model a patient, sheet, diaper or forces.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import subprocess
import sys
import zlib
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np

sys.dont_write_bytecode = True

HERE = Path(__file__).resolve().parent
STAGE2A_IK = HERE.parent / "stage2A_ik"
STAGE2A = HERE.parent / "stage2A"
for _path in (HERE, STAGE2A_IK, STAGE2A):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

import fk_model  # noqa: E402
import ik_study as ik  # noqa: E402
import bed_study as bed  # noqa: E402

JOINT_KEYS = tuple(fk_model.ARM_JOINT_KEYS)
P0_Q = {key: 0.0 for key in JOINT_KEYS}
MODE = "5d"
LIFT_M = 0.0
CART_STEP_M = 0.01
JOINT_STEP_DEG = 0.5
MAX_JOINT_JUMP_DEG = 20.0
MAX_PREFERRED_JOINT_STEP_DEG = 6.0
MAX_SAMPLES = 260
IK_MAX_ITER = 170
STATIC_RANDOM_COUNT = 4
STATIC_RETRY_RANDOM_COUNT = 12
BIMANUAL_SAMPLES = 201
BASELINE = {
    "approach": "lateral",
    "height_m": 0.7,
    "width_m": 1.0,
    "clearance_m": 0.10,
}
BLEND_NAME = "robot_scaled_4-2-21-LTS-ARCH-02-MANUAL-01.blend"


# ---------------------------------------------------------------------------
# Helpers


def file_sha256(path: Path) -> str | None:
    if not path.exists():
        return None
    return hashlib.sha256(path.read_bytes()).hexdigest()


def git_head(root: Path) -> str | None:
    try:
        return subprocess.check_output(
            ["git", "-C", str(root), "rev-parse", "HEAD"],
            text=True,
            stderr=subprocess.DEVNULL,
        ).strip()
    except Exception:
        return None


def q_array(side: str, q_deg: dict[str, float]) -> np.ndarray:
    return ik.q_array(side, q_deg)


def q_distance(side: str, first: dict[str, float], second: dict[str, float]) -> float:
    return float(np.linalg.norm(q_array(side, first) - q_array(side, second)))


def finite(value: Any) -> Any:
    if value is None:
        return None
    if isinstance(value, (np.floating, np.integer)):
        value = value.item()
    if isinstance(value, float):
        if not math.isfinite(value):
            return None
        return float(value)
    return value


def safe_min(values: list[Any]) -> Any:
    present = [value for value in values if value is not None]
    return finite(min(present)) if present else None


def safe_max(values: list[Any]) -> Any:
    present = [value for value in values if value is not None]
    return finite(max(present)) if present else None


def target_at(
    x: float,
    y: float,
    z: float,
    side: str,
    finger: np.ndarray,
    geom: dict[str, Any],
) -> dict[str, Any]:
    normal = bed.DOWN
    rotation = bed.orientation_from_normal_finger(side, normal, finger)
    return {
        "position_floor": [float(x), float(y), float(z)],
        "normal": [float(value) for value in bed.normalize(normal)],
        "rotation": rotation.tolist(),
        "bed": geom,
        "source": "stage2b_reference",
    }


def interp_target(
    start: dict[str, Any],
    end: dict[str, Any],
    t: float,
) -> dict[str, Any]:
    p0 = np.asarray(start["position_floor"], dtype=float)
    p1 = np.asarray(end["position_floor"], dtype=float)
    position = p0 + (p1 - p0) * float(t)
    return {
        "position_floor": [float(value) for value in position],
        "normal": list(start["normal"]),
        "rotation": [list(row) for row in start["rotation"]],
        "bed": start["bed"],
        "source": "stage2b_interpolated",
    }


def operation_sequence(operation: str, side: str, geom: dict[str, Any]) -> list[dict[str, Any]]:
    sign = 1.0 if side == "L" else -1.0
    x_near = float(geom["x_near"])
    width = float(geom["width_m"])
    height = float(geom["height_m"])
    if operation == "P6":
        grasp_x = x_near + 0.45 * width
        ready_x = grasp_x - 0.185
        tension_x = grasp_x - 0.05
        z_grasp = height + 0.08
        z_above = height + 0.20
        z_lift = height + 0.18
        z_ready = height + 0.25
        y = 0.35 * sign
        finger = bed.PLUS_X
        points = [
            ("S0_ready", ready_x, y, z_ready),
            ("S1_above", grasp_x, y, z_above),
            ("S2_grasp", grasp_x, y, z_grasp),
            ("S3_tension", tension_x, y, z_grasp),
            ("S4_lift", tension_x, y, z_lift),
            ("S5_ready", ready_x, y, z_ready),
        ]
    elif operation == "P7":
        contact_x = x_near + 0.55 * width
        ready_x = contact_x - 0.185
        z_contact = height + 0.10
        z_above = height + 0.20
        z_ready = height + 0.25
        y_contact = 0.12 * sign
        y_manip = 0.22 * sign
        y_ready = 0.25 * sign
        finger = bed.PLUS_Y
        points = [
            ("S0_ready", ready_x, y_ready, z_ready),
            ("S1_above", contact_x, y_contact, z_above),
            ("S2_contact", contact_x, y_contact, z_contact),
            ("S3_manip", contact_x, y_manip, z_contact),
            ("S4_lift", contact_x, y_manip, z_above),
            ("S5_ready", ready_x, y_ready, z_ready),
        ]
    else:
        raise ValueError(f"unsupported operation: {operation!r}")

    return [
        {
            "label": label,
            "target": target_at(x, y, z, side, finger, geom),
            "position_floor": [float(x), float(y), float(z)],
        }
        for label, x, y, z in points
    ]


# ---------------------------------------------------------------------------
# Static references and sequential IK


def static_solve(side: str, target: dict[str, Any], seed: int) -> dict[str, Any]:
    result = bed.solve_bed_target(
        side,
        target,
        MODE,
        LIFT_M,
        seed,
        random_count=STATIC_RANDOM_COUNT,
    )
    if not result.get("reachable"):
        retry = bed.solve_bed_target(
            side,
            target,
            MODE,
            LIFT_M,
            seed + 1,
            random_count=STATIC_RETRY_RANDOM_COUNT,
        )
        if retry.get("reachable"):
            return retry
    return result


def pick_reference(result: dict[str, Any], side: str, geom: dict[str, Any]) -> dict[str, Any] | None:
    if not result.get("reachable"):
        return None
    candidates = [solution for solution in result.get("solutions", []) if solution.get("converged")]
    if not candidates:
        candidates = [result["best"]]
    for solution in candidates:
        clearances = bed.collision_clearances(side, solution["q_deg"], LIFT_M, geom, other_q=P0_Q)
        if clearances["collision_free"]:
            return solution
    return candidates[0]


def solve_path_point(
    side: str,
    target: dict[str, Any],
    q_seed: dict[str, float],
    seed: int,
) -> dict[str, Any]:
    q0 = q_array(side, q_seed)
    result = ik.solve_once(side, target, MODE, LIFT_M, q0, max_iter=IK_MAX_ITER)
    if result["converged"] and result["within_limits"]:
        return result

    fallback = static_solve(side, target, seed)
    if fallback.get("reachable"):
        candidates = [solution for solution in fallback.get("solutions", []) if solution.get("converged")]
        if candidates:
            return min(candidates, key=lambda solution: q_distance(side, solution["q_deg"], q_seed))
    return result


# ---------------------------------------------------------------------------
# Path evaluation


def evaluate_sample(
    side: str,
    q_deg: dict[str, float],
    target: dict[str, Any],
    geom: dict[str, Any],
    other_q: dict[str, float] | None,
) -> dict[str, Any]:
    metrics = ik.task_metrics(side, q_deg, LIFT_M, target, MODE)
    clearances = bed.collision_clearances(side, q_deg, LIFT_M, geom, other_q=other_q)
    return {
        "q_deg": {key: float(q_deg.get(key, 0.0)) for key in JOINT_KEYS},
        "position_error_mm": float(np.linalg.norm(metrics["position_error"]) * 1000.0),
        "normal_error_deg": float(metrics["normal_error_deg"]),
        "min_margin_deg": float(metrics["min_margin"]["margin_deg"]),
        "condition": finite(metrics["condition"]),
        "within_limits": bool(metrics["within_limits"]),
        "clearance_min_m": finite(clearances["minimum"]),
        "arm_arm_m": finite(clearances["arm_arm"]),
        "arm_bed_m": finite(clearances["arm_bed"]),
        "arm_torso_m": finite(clearances["arm_torso"]),
        "arm_base_m": finite(clearances["arm_base"]),
        "collision_free": bool(clearances["collision_free"]),
    }


def summarize_samples(
    side: str,
    samples: list[dict[str, Any]],
    target: dict[str, Any],
    geom: dict[str, Any],
    other_q: dict[str, float] | None,
) -> dict[str, Any]:
    evaluated = [evaluate_sample(side, sample["q_deg"], target, geom, other_q) for sample in samples]
    for index, sample in enumerate(samples):
        evaluated[index]["t"] = sample.get("t")
    clearances = [sample["clearance_min_m"] for sample in evaluated if sample["clearance_min_m"] is not None]
    margins = [sample["min_margin_deg"] for sample in evaluated]
    conditions = [sample["condition"] for sample in evaluated if sample["condition"] is not None]
    arm_arms = [sample["arm_arm_m"] for sample in evaluated if sample["arm_arm_m"] is not None]
    steps = [
        q_distance(side, first["q_deg"], second["q_deg"])
        for first, second in zip(evaluated, evaluated[1:])
    ]
    critical: dict[str, Any] = {}
    if clearances:
        index = min(
            range(len(evaluated)),
            key=lambda i: evaluated[i]["clearance_min_m"] if evaluated[i]["clearance_min_m"] is not None else math.inf,
        )
        critical["min_clearance"] = {
            "sample_index": index,
            "q_deg": evaluated[index]["q_deg"],
            "value": clearances and min(clearances),
        }
    if margins:
        index = min(range(len(evaluated)), key=lambda i: evaluated[i]["min_margin_deg"])
        critical["min_margin"] = {
            "sample_index": index,
            "q_deg": evaluated[index]["q_deg"],
            "value": min(margins),
        }
    if conditions:
        index = max(
            range(len(evaluated)),
            key=lambda i: evaluated[i]["condition"] if evaluated[i]["condition"] is not None else -math.inf,
        )
        critical["max_condition"] = {
            "sample_index": index,
            "q_deg": evaluated[index]["q_deg"],
            "value": max(conditions),
        }
    return {
        "samples": evaluated,
        "sample_count": len(evaluated),
        "min_clearance_m": safe_min(clearances),
        "min_arm_arm_m": safe_min(arm_arms),
        "min_margin_deg": safe_min(margins),
        "max_condition": safe_max(conditions),
        "max_joint_step_deg": finite(max(steps) if steps else 0.0),
        "path_length_deg": finite(sum(steps)),
        "collision_free": bool(all(sample["collision_free"] for sample in evaluated)),
        "within_limits": bool(all(sample["within_limits"] for sample in evaluated)),
        "critical": critical,
    }


def path_summary(
    side: str,
    samples: list[dict[str, Any]],
    target: dict[str, Any],
    geom: dict[str, Any],
    other_q: dict[str, float] | None,
    method: str,
) -> dict[str, Any]:
    summary = summarize_samples(side, samples, target, geom, other_q)
    summary["method"] = method
    summary["status"] = "valid" if summary["collision_free"] and summary["within_limits"] else "invalid"
    if summary["status"] != "valid":
        reasons = []
        if not summary["collision_free"]:
            reasons.append("collision")
        if not summary["within_limits"]:
            reasons.append("joint_limits")
        summary["reason"] = "+".join(reasons)
    else:
        summary["reason"] = None
    return summary


# ---------------------------------------------------------------------------
# Candidate planners


def plan_task_segment(
    side: str,
    start_target: dict[str, Any],
    end_target: dict[str, Any],
    q_start: dict[str, float],
    geom: dict[str, Any],
    other_q: dict[str, float] | None,
    seed: int,
) -> dict[str, Any]:
    distance = float(
        np.linalg.norm(
            np.asarray(end_target["position_floor"], dtype=float)
            - np.asarray(start_target["position_floor"], dtype=float)
        )
    )
    count = max(4, min(MAX_SAMPLES, int(math.ceil(distance / CART_STEP_M))))
    samples: list[dict[str, Any]] = []
    q_previous = q_start
    for index in range(count + 1):
        t = index / count
        target = interp_target(start_target, end_target, t)
        if index == 0:
            q_current = q_start
            metrics = ik.task_metrics(side, q_current, LIFT_M, target, MODE)
            if not metrics["within_limits"]:
                return {
                    "method": "task_space",
                    "status": "invalid",
                    "reason": "start_outside_limits",
                    "samples": [],
                    "sample_count": 0,
                }
        else:
            result = solve_path_point(side, target, q_previous, seed + index)
            if not result.get("converged"):
                return {
                    "method": "task_space",
                    "status": "invalid",
                    "reason": f"ik_not_converged_at_{index}",
                    "samples": samples,
                    "sample_count": len(samples),
                }
            q_current = result["q_deg"]
        step = q_distance(side, q_previous, q_current) if index else 0.0
        if step > MAX_JOINT_JUMP_DEG:
            return {
                "method": "task_space",
                "status": "invalid",
                "reason": f"joint_jump_{step:.2f}deg_at_{index}",
                "samples": samples,
                "sample_count": len(samples),
            }
        samples.append({"t": float(t), "q_deg": q_current})
        q_previous = q_current
    return path_summary(side, samples, start_target, geom, other_q, "task_space")


def plan_via_segment(
    side: str,
    start_target: dict[str, Any],
    end_target: dict[str, Any],
    q_start: dict[str, float],
    geom: dict[str, Any],
    other_q: dict[str, float] | None,
    seed: int,
) -> dict[str, Any]:
    p0 = np.asarray(start_target["position_floor"], dtype=float)
    p1 = np.asarray(end_target["position_floor"], dtype=float)
    midpoint = 0.5 * (p0 + p1)
    midpoint[2] = max(p0[2], p1[2]) + 0.10
    via = {
        "position_floor": [float(value) for value in midpoint],
        "normal": list(start_target["normal"]),
        "rotation": [list(row) for row in start_target["rotation"]],
        "bed": geom,
        "source": "stage2b_via",
    }
    first = plan_task_segment(side, start_target, via, q_start, geom, other_q, seed + 1)
    if first["status"] != "valid":
        first["method"] = "via_lift"
        first["reason"] = "first_segment:" + str(first.get("reason"))
        return first
    q_mid = first["samples"][-1]["q_deg"]
    second = plan_task_segment(side, via, end_target, q_mid, geom, other_q, seed + 2)
    if second["status"] != "valid":
        second["method"] = "via_lift"
        second["reason"] = "second_segment:" + str(second.get("reason"))
        return second
    combined = first["samples"] + second["samples"][1:]
    for index, sample in enumerate(combined):
        sample["t"] = index / max(1, len(combined) - 1)
    return path_summary(side, combined, start_target, geom, other_q, "via_lift")


def plan_joint_lerp(
    side: str,
    start_target: dict[str, Any],
    end_target: dict[str, Any],
    q_start: dict[str, float],
    q_end: dict[str, float],
    geom: dict[str, Any],
    other_q: dict[str, float] | None,
    method: str = "joint_lerp",
) -> dict[str, Any]:
    start_values = q_array(side, q_start)
    end_values = q_array(side, q_end)
    max_difference = float(np.max(np.abs(end_values - start_values)))
    count = max(4, min(MAX_SAMPLES, int(math.ceil(max_difference / JOINT_STEP_DEG))))
    samples: list[dict[str, Any]] = []
    for index in range(count + 1):
        t = index / count
        values = start_values + (end_values - start_values) * t
        samples.append({"t": float(t), "q_deg": ik.q_dict(values)})
    return path_summary(side, samples, start_target, geom, other_q, method)


def candidate_key(candidate: dict[str, Any]) -> tuple[float, float, float, float]:
    clearance = candidate.get("min_clearance_m")
    margin = candidate.get("min_margin_deg")
    condition = candidate.get("max_condition")
    length = candidate.get("path_length_deg")
    return (
        float(margin) if margin is not None else -math.inf,
        float(clearance) if clearance is not None else -math.inf,
        -float(condition) if condition is not None else -math.inf,
        -float(length) if length is not None else -math.inf,
    )


def plan_transition(
    side: str,
    start_target: dict[str, Any],
    end_target: dict[str, Any],
    q_start: dict[str, float],
    end_candidates: list[dict[str, float]],
    geom: dict[str, Any],
    other_q: dict[str, float] | None,
    seed: int,
) -> dict[str, Any]:
    candidates: list[dict[str, Any]] = [
        plan_task_segment(side, start_target, end_target, q_start, geom, other_q, seed + 10),
        plan_via_segment(side, start_target, end_target, q_start, geom, other_q, seed + 20),
    ]
    primary_end = end_candidates[0] if end_candidates else None
    if primary_end is not None:
        candidates.append(plan_joint_lerp(side, start_target, end_target, q_start, primary_end, geom, other_q))
        alternatives = sorted(end_candidates[1:], key=lambda candidate: q_distance(side, q_start, candidate))[:2]
        for index, alternative in enumerate(alternatives):
            candidates.append(
                plan_joint_lerp(
                    side,
                    start_target,
                    end_target,
                    q_start,
                    alternative,
                    geom,
                    other_q,
                    method=f"joint_lerp_alt{index + 1}",
                )
            )
    valid = [candidate for candidate in candidates if candidate["status"] == "valid"]
    selected = max(valid, key=candidate_key) if valid else None
    return {
        "status": "valid" if selected is not None else "not_found",
        "selected": selected,
        "candidates": [
            {key: value for key, value in candidate.items() if key != "samples"}
            for candidate in candidates
        ],
        "reason": None
        if selected is not None
        else "; ".join(f"{candidate.get('method')}:{candidate.get('reason')}" for candidate in candidates),
    }


# ---------------------------------------------------------------------------
# Operations


def plan_operation(operation: str, side: str, geom: dict[str, Any]) -> dict[str, Any]:
    sequence = operation_sequence(operation, side, geom)
    references: list[dict[str, Any]] = []
    for item in sequence:
        seed = zlib.crc32(f"{operation}|{side}|{item['label']}|{geom['clearance_m']}".encode("utf-8"))
        result = static_solve(side, item["target"], int(seed))
        chosen = pick_reference(result, side, geom)
        references.append(
            {
                "label": item["label"],
                "position_floor": item["position_floor"],
                "target": item["target"],
                "reachable": chosen is not None,
                "q_deg": chosen.get("q_deg") if chosen else None,
                "position_error_mm": finite(chosen.get("position_error_mm")) if chosen else None,
                "normal_error_deg": finite(chosen.get("normal_error_deg")) if chosen else None,
                "min_margin_deg": finite(chosen.get("min_margin_deg")) if chosen else None,
                "condition": finite(chosen.get("condition")) if chosen else None,
                "solutions": [solution["q_deg"] for solution in result.get("solutions", []) if solution.get("converged")],
            }
        )

    if not all(reference["reachable"] for reference in references):
        return {
            "operation": operation,
            "side": side,
            "status": "reference_unreachable",
            "references": [{key: value for key, value in reference.items() if key != "solutions"} for reference in references],
            "transitions": [],
            "summary": None,
        }

    current_q = references[0]["q_deg"]
    transitions: list[dict[str, Any]] = []
    for index in range(len(references) - 1):
        start = references[index]
        end = references[index + 1]
        end_candidates = list(end["solutions"])
        if end["q_deg"] and all(q_distance(side, end["q_deg"], candidate) > 1e-9 for candidate in end_candidates):
            end_candidates.insert(0, end["q_deg"])
        seed = zlib.crc32(f"{operation}|{side}|{start['label']}|{end['label']}|{geom['clearance_m']}".encode("utf-8"))
        transition = plan_transition(
            side,
            start["target"],
            end["target"],
            current_q,
            end_candidates,
            geom,
            P0_Q,
            int(seed),
        )
        transition.update({"from": start["label"], "to": end["label"]})
        transitions.append(transition)
        if transition["status"] != "valid":
            break
        current_q = transition["selected"]["samples"][-1]["q_deg"]

    selected = [transition["selected"] for transition in transitions if transition["status"] == "valid"]
    summary = None
    if len(selected) == len(references) - 1:
        summary = {
            "transition_count": len(selected),
            "min_clearance_m": safe_min([item["min_clearance_m"] for item in selected]),
            "min_arm_arm_m": safe_min([item["min_arm_arm_m"] for item in selected]),
            "min_margin_deg": safe_min([item["min_margin_deg"] for item in selected]),
            "max_condition": safe_max([item["max_condition"] for item in selected]),
            "max_joint_step_deg": safe_max([item["max_joint_step_deg"] for item in selected]),
            "path_length_deg": finite(sum(item["path_length_deg"] for item in selected)),
        }
    return {
        "operation": operation,
        "side": side,
        "status": "valid" if summary is not None else "not_found",
        "references": [{key: value for key, value in reference.items() if key != "solutions"} for reference in references],
        "transitions": transitions,
        "summary": summary,
    }


# ---------------------------------------------------------------------------
# Bimanual checks


def full_arm_path(operation: dict[str, Any]) -> list[dict[str, Any]]:
    samples: list[dict[str, Any]] = []
    for transition in operation["transitions"]:
        if transition["status"] != "valid":
            continue
        current = [dict(sample) for sample in transition["selected"]["samples"]]
        if samples:
            current = current[1:]
        samples.extend(current)
    for index, sample in enumerate(samples):
        sample["t"] = index / max(1, len(samples) - 1)
    return samples


def q_at(path: list[dict[str, Any]], t: float, side: str) -> dict[str, float]:
    times = np.asarray([sample["t"] for sample in path], dtype=float)
    values = np.asarray([q_array(side, sample["q_deg"]) for sample in path], dtype=float)
    clipped = float(np.clip(t, 0.0, 1.0))
    result = np.array([np.interp(clipped, times, values[:, index]) for index in range(values.shape[1])])
    return ik.q_dict(result)


def bimanual_sample(
    left_q: dict[str, float],
    right_q: dict[str, float],
    left_target: dict[str, Any],
    right_target: dict[str, Any],
    geom: dict[str, Any],
) -> dict[str, Any]:
    left_metrics = ik.task_metrics("L", left_q, LIFT_M, left_target, MODE)
    right_metrics = ik.task_metrics("R", right_q, LIFT_M, right_target, MODE)
    left_clearances = bed.collision_clearances("L", left_q, LIFT_M, geom, other_q=right_q)
    right_clearances = bed.collision_clearances("R", right_q, LIFT_M, geom, other_q=left_q)
    clearances = [value for value in (left_clearances["minimum"], right_clearances["minimum"]) if value is not None]
    arm_arms = [value for value in (left_clearances["arm_arm"], right_clearances["arm_arm"]) if value is not None]
    conditions = [value for value in (left_metrics["condition"], right_metrics["condition"]) if value is not None]
    return {
        "left_q_deg": {key: float(left_q.get(key, 0.0)) for key in JOINT_KEYS},
        "right_q_deg": {key: float(right_q.get(key, 0.0)) for key in JOINT_KEYS},
        "min_clearance_m": safe_min(clearances),
        "min_arm_arm_m": safe_min(arm_arms),
        "min_margin_deg": finite(min(left_metrics["min_margin"]["margin_deg"], right_metrics["min_margin"]["margin_deg"])),
        "max_condition": safe_max(conditions),
        "collision_free": bool(left_clearances["collision_free"] and right_clearances["collision_free"]),
        "within_limits": bool(left_metrics["within_limits"] and right_metrics["within_limits"]),
    }


def summarize_bimanual(samples: list[dict[str, Any]], mode: str) -> dict[str, Any]:
    return {
        "mode": mode,
        "status": "valid" if all(sample["collision_free"] and sample["within_limits"] for sample in samples) else "invalid",
        "sample_count": len(samples),
        "min_clearance_m": safe_min([sample["min_clearance_m"] for sample in samples]),
        "min_arm_arm_m": safe_min([sample["min_arm_arm_m"] for sample in samples]),
        "min_margin_deg": safe_min([sample["min_margin_deg"] for sample in samples]),
        "max_condition": safe_max([sample["max_condition"] for sample in samples]),
        "samples": samples,
    }


def plan_bimanual(operation: str, left: dict[str, Any], right: dict[str, Any], geom: dict[str, Any]) -> dict[str, Any]:
    if left["status"] != "valid" or right["status"] != "valid":
        return {"operation": operation, "status": "not_found", "reason": "one_arm_operation_invalid", "candidates": []}
    left_path = full_arm_path(left)
    right_path = full_arm_path(right)
    if not left_path or not right_path:
        return {"operation": operation, "status": "not_found", "reason": "empty_arm_path", "candidates": []}
    left_target = left["references"][0]["target"]
    right_target = right["references"][0]["target"]
    candidates: list[dict[str, Any]] = []

    synchronized = [
        bimanual_sample(
            q_at(left_path, index / (BIMANUAL_SAMPLES - 1), "L"),
            q_at(right_path, index / (BIMANUAL_SAMPLES - 1), "R"),
            left_target,
            right_target,
            geom,
        )
        for index in range(BIMANUAL_SAMPLES)
    ]
    candidates.append(summarize_bimanual(synchronized, "synchronized"))

    sequential = []
    for index in range(BIMANUAL_SAMPLES):
        t = 2.0 * index / (BIMANUAL_SAMPLES - 1)
        if t <= 1.0:
            left_q = q_at(left_path, t, "L")
            right_q = q_at(right_path, 0.0, "R")
        else:
            left_q = q_at(left_path, 1.0, "L")
            right_q = q_at(right_path, t - 1.0, "R")
        sequential.append(bimanual_sample(left_q, right_q, left_target, right_target, geom))
    candidates.append(summarize_bimanual(sequential, "sequential_L_then_R"))

    shifted = [
        bimanual_sample(
            q_at(left_path, min(1.2 * index / (BIMANUAL_SAMPLES - 1), 1.0), "L"),
            q_at(right_path, max(0.0, 1.2 * index / (BIMANUAL_SAMPLES - 1) - 0.2), "R"),
            left_target,
            right_target,
            geom,
        )
        for index in range(BIMANUAL_SAMPLES)
    ]
    candidates.append(summarize_bimanual(shifted, "phase_shift_0.2"))

    valid = [candidate for candidate in candidates if candidate["status"] == "valid"]
    synchronized = [candidate for candidate in valid if candidate["mode"] == "synchronized"]
    if synchronized:
        selected = synchronized[0]
    else:
        selected = max(
            valid,
            key=lambda candidate: (
                candidate["min_clearance_m"] if candidate["min_clearance_m"] is not None else -math.inf,
                candidate["min_arm_arm_m"] if candidate["min_arm_arm_m"] is not None else -math.inf,
            ),
        ) if valid else None
    return {
        "operation": operation,
        "status": "valid" if selected is not None else "not_found",
        "selected": selected,
        "candidates": [{key: value for key, value in candidate.items() if key != "samples"} for candidate in candidates],
        "reason": None if selected is not None else "no_collision_free_bimanual_mode",
    }


# ---------------------------------------------------------------------------
# Output


def write_csv(path: Path, rows: list[dict[str, Any]], fieldnames: list[str]) -> None:
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def write_outputs(payload: dict[str, Any], out_dir: Path) -> None:
    (out_dir / "motion_study.json").write_text(
        json.dumps(payload, indent=2, ensure_ascii=False),
        encoding="utf-8",
    )

    sequence_rows: list[dict[str, Any]] = []
    transition_rows: list[dict[str, Any]] = []
    sample_rows: list[dict[str, Any]] = []
    for operation, sides in payload["operations"].items():
        for side, result in sides.items():
            for reference in result["references"]:
                sequence_rows.append(
                    {
                        "operation": operation,
                        "side": side,
                        "point": reference["label"],
                        "x_m": reference["position_floor"][0],
                        "y_m": reference["position_floor"][1],
                        "z_m": reference["position_floor"][2],
                        "reachable": reference["reachable"],
                        "position_error_mm": reference["position_error_mm"],
                        "normal_error_deg": reference["normal_error_deg"],
                        "min_margin_deg": reference["min_margin_deg"],
                        "condition": reference["condition"],
                        "q_deg": json.dumps(reference["q_deg"], ensure_ascii=False),
                    }
                )
            for transition in result["transitions"]:
                selected = transition.get("selected")
                transition_rows.append(
                    {
                        "operation": operation,
                        "side": side,
                        "from": transition["from"],
                        "to": transition["to"],
                        "status": transition["status"],
                        "method": selected["method"] if selected else "",
                        "samples": selected["sample_count"] if selected else 0,
                        "min_clearance_m": selected["min_clearance_m"] if selected else None,
                        "min_arm_arm_m": selected["min_arm_arm_m"] if selected else None,
                        "min_margin_deg": selected["min_margin_deg"] if selected else None,
                        "max_condition": selected["max_condition"] if selected else None,
                        "max_joint_step_deg": selected["max_joint_step_deg"] if selected else None,
                        "reason": transition.get("reason"),
                    }
                )
                if selected:
                    for index, sample in enumerate(selected["samples"]):
                        sample_rows.append(
                            {
                                "operation": operation,
                                "side": side,
                                "from": transition["from"],
                                "to": transition["to"],
                                "method": selected["method"],
                                "sample": index,
                                "t": sample["t"],
                                "q_deg": json.dumps(sample["q_deg"], ensure_ascii=False),
                                "position_error_mm": sample["position_error_mm"],
                                "normal_error_deg": sample["normal_error_deg"],
                                "min_margin_deg": sample["min_margin_deg"],
                                "condition": sample["condition"],
                                "clearance_min_m": sample["clearance_min_m"],
                                "collision_free": sample["collision_free"],
                            }
                        )

    write_csv(
        out_dir / "motion_sequences.csv",
        sequence_rows,
        ["operation", "side", "point", "x_m", "y_m", "z_m", "reachable", "position_error_mm", "normal_error_deg", "min_margin_deg", "condition", "q_deg"],
    )
    write_csv(
        out_dir / "motion_transitions.csv",
        transition_rows,
        ["operation", "side", "from", "to", "status", "method", "samples", "min_clearance_m", "min_arm_arm_m", "min_margin_deg", "max_condition", "max_joint_step_deg", "reason"],
    )
    write_csv(
        out_dir / "motion_samples.csv",
        sample_rows,
        ["operation", "side", "from", "to", "method", "sample", "t", "q_deg", "position_error_mm", "normal_error_deg", "min_margin_deg", "condition", "clearance_min_m", "collision_free"],
    )

    bimanual_rows = []
    for operation, result in payload["bimanual"].items():
        selected = result.get("selected")
        bimanual_rows.append(
            {
                "operation": operation,
                "status": result["status"],
                "mode": selected["mode"] if selected else "",
                "samples": selected["sample_count"] if selected else 0,
                "min_clearance_m": selected["min_clearance_m"] if selected else None,
                "min_arm_arm_m": selected["min_arm_arm_m"] if selected else None,
                "min_margin_deg": selected["min_margin_deg"] if selected else None,
                "max_condition": selected["max_condition"] if selected else None,
                "reason": result.get("reason"),
            }
        )
    write_csv(
        out_dir / "motion_bimanual.csv",
        bimanual_rows,
        ["operation", "status", "mode", "samples", "min_clearance_m", "min_arm_arm_m", "min_margin_deg", "max_condition", "reason"],
    )

    sensitivity_rows = []
    for key, result in payload.get("sensitivity", {}).items():
        for side, operation_result in result.items():
            summary = operation_result.get("summary")
            sensitivity_rows.append(
                {
                    "case": key,
                    "side": side,
                    "status": operation_result["status"],
                    "min_clearance_m": summary["min_clearance_m"] if summary else None,
                    "min_margin_deg": summary["min_margin_deg"] if summary else None,
                    "max_condition": summary["max_condition"] if summary else None,
                }
            )
    write_csv(
        out_dir / "motion_sensitivity.csv",
        sensitivity_rows,
        ["case", "side", "status", "min_clearance_m", "min_margin_deg", "max_condition"],
    )


# ---------------------------------------------------------------------------
# Main


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out-dir", type=Path, default=HERE)
    parser.add_argument("--no-sensitivity", action="store_true")
    args = parser.parse_args()
    out_dir = args.out_dir.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    geom = bed.bed_geometry(
        BASELINE["approach"],
        BASELINE["height_m"],
        BASELINE["width_m"],
        BASELINE["clearance_m"],
    )
    operations: dict[str, dict[str, Any]] = {}
    for operation in ("P6", "P7"):
        operations[operation] = {}
        for side in ("L", "R"):
            print(f"PLAN {operation} {side}", flush=True)
            operations[operation][side] = plan_operation(operation, side, geom)

    bimanual: dict[str, Any] = {}
    for operation in ("P6", "P7"):
        print(f"BIMANUAL {operation}", flush=True)
        bimanual[operation] = plan_bimanual(operation, operations[operation]["L"], operations[operation]["R"], geom)

    sensitivity: dict[str, Any] = {}
    if not args.no_sensitivity:
        for operation, side in (("P6", "L"), ("P7", "L")):
            for clearance in (0.05, 0.15):
                key = f"{operation}_{side}_c{clearance:.2f}"
                print(f"SENSITIVITY {key}", flush=True)
                sensitivity[key] = {
                    side: plan_operation(
                        operation,
                        side,
                        bed.bed_geometry(
                            BASELINE["approach"],
                            BASELINE["height_m"],
                            BASELINE["width_m"],
                            clearance,
                        ),
                    )
                }

    source_json = STAGE2A_IK / "workspace_search.json"
    source_csv = STAGE2A_IK / "workspace_search.csv"
    source_rows = json.loads(source_json.read_text(encoding="utf-8"))["rows"]
    payload = {
        "metadata": {
            "generated_at": datetime.now(timezone.utc).isoformat(),
            "git_head": git_head(HERE.parents[1]),
            "blend_sha256": file_sha256(HERE.parents[1] / BLEND_NAME),
            "source_json_sha256": file_sha256(source_json),
            "source_csv_sha256": file_sha256(source_csv),
            "source_counts": {
                "json_rows": len(source_rows),
                "csv_rows": sum(1 for _ in source_csv.open(encoding="utf-8")) - 1,
            },
            "baseline": BASELINE,
            "mode": MODE,
            "lift_m": LIFT_M,
            "cart_step_m": CART_STEP_M,
            "joint_step_deg": JOINT_STEP_DEG,
            "max_joint_jump_deg": MAX_JOINT_JUMP_DEG,
            "ik_max_iter": IK_MAX_ITER,
        },
        "operations": operations,
        "bimanual": bimanual,
        "sensitivity": sensitivity,
    }
    write_outputs(payload, out_dir)
    print(f"WROTE {out_dir / 'motion_study.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
