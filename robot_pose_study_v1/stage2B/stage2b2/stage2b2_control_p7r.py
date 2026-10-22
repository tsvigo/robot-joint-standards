#!/usr/bin/env python3
"""Stage 2B.2 — control analysis of P7 R at bed clearance c = 0.05 m.

Read-only with respect to the .blend, the stage 2A/2B sources and the stage
2B.1 artifacts.  Writes only into this directory.

Questions answered
------------------
A. Why does the strict 6D policy end at a 1.56 deg joint margin while the
   reference solution of the same pose has 11.89 deg?
B. Is the trajectory free of IK discontinuities between samples?
C. Does the work cycle close by the joint vector q (not only by the palm
   pose)?
D. Is there a continuous admissible path from the 1.56 deg configuration to
   the 11.89 deg configuration, and what does it cost in 6D deviation?
E. Can the terminal configuration be improved at the fixed terminal pose
   (self-motion) without leaving the target pose?
F. Are the intervals between consecutive samples collision free under a
   conservative swept bound?

Definitions
-----------
``margin``
    Distance of the nearest joint to its limit, degrees.
``bound``
    Conservative upper bound of the displacement of any point of a link over
    one joint-space interval: sum_j |dq_j| * max|dp/dq_j|, where the
    gradient is evaluated numerically at both ends and at the midpoint and
    multiplied by a 1.05 safety factor.  The distance between two bodies is
    1-Lipschitz in their point positions, so
    ``min(clearance at both ends) - bound > 0`` certifies the interval.  It
    is a numerical certificate, not a formal proof.
"""

from __future__ import annotations

import hashlib
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

import bed_study as bed  # noqa: E402
import ik_study as ik  # noqa: E402
import motion_study as ms  # noqa: E402

STAGE2B1 = BASE / "stage2B" / "stage2b1"
ARTIFACT = STAGE2B1 / "stage2b1_unilateral_P7_R_c0.05.json"
OUT = HERE / "stage2b2_control_p7r.json"

OPERATION = "P7"
SIDE = "R"
CLEARANCE = 0.05
HEIGHT = 0.7
WIDTH = 1.0
FINGER_YAW = 80.0
JOINTS = ms.JOINT_KEYS
BOUND_SAFETY = 1.05
SUBSTEPS = 60
MAX_SPLIT_DEPTH = 4
SELF_MOTION_STEP_DEG = 0.5
SELF_MOTION_MAX_STEPS = 200


def geometry() -> dict:
    return bed.bed_geometry("lateral", HEIGHT, WIDTH, CLEARANCE)


def finger_vector() -> np.ndarray:
    yaw = math.radians(FINGER_YAW)
    return np.array([math.sin(yaw), math.cos(yaw), 0.0], dtype=float)


def references(geom: dict) -> list[dict]:
    """Rebuild the static references exactly as stage2b1_unilateral.py does."""
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


def metrics(side: str, q_deg: dict, target: dict, geom: dict, other_q: dict | None) -> dict:
    task = ik.task_metrics(side, q_deg, ms.LIFT_M, target, ms.MODE)
    clear = bed.collision_clearances(side, q_deg, ms.LIFT_M, geom, other_q=other_q)
    margin = task["min_margin"]
    return {
        "q_deg": {key: float(q_deg[key]) for key in JOINTS},
        "position_error_mm": float(np.linalg.norm(task["position_error"]) * 1000.0),
        "orientation_error_deg": float(task["orientation_error_deg"]),
        "normal_error_deg": float(task["normal_error_deg"]),
        "min_margin_deg": float(margin["margin_deg"]),
        "min_margin_joint": margin["joint"],
        "min_margin_nearest": margin["nearest"],
        "condition": float(task["condition"]),
        "within_limits": bool(task["within_limits"]),
        "clearance_min_m": float(clear["minimum"]),
        "arm_arm_m": clear["arm_arm"],
        "arm_bed_m": clear["arm_bed"],
        "arm_torso_m": clear["arm_torso"],
        "arm_base_m": clear["arm_base"],
        "collision_free": bool(clear["collision_free"]),
    }


def delta_q(a: dict, b: dict) -> dict:
    return {key: float(b[key] - a[key]) for key in JOINTS}


def max_step(a: dict, b: dict) -> tuple[float, str]:
    diffs = {key: abs(float(b[key] - a[key])) for key in JOINTS}
    joint = max(diffs, key=diffs.get)
    return diffs[joint], joint


def link_points(side: str, q_deg: dict) -> np.ndarray:
    segments = bed.arm_segments(side, q_deg, ms.LIFT_M)
    return np.array([point for segment in segments for point in (segment["p0"], segment["p1"])], dtype=float)


def joint_gradients(side: str, q_deg: dict | None, step_deg: float = 0.05) -> dict[str, float]:
    """max|dp/dq_j| in metres per radian, over the sampled points of the arm."""
    if q_deg is None:
        return {key: 0.0 for key in JOINTS}
    base = link_points(side, q_deg)
    out = {}
    for key in JOINTS:
        shifted = dict(q_deg)
        shifted[key] = float(shifted[key]) + step_deg
        delta = np.linalg.norm(link_points(side, shifted) - base, axis=1)
        out[key] = float(np.max(delta)) / math.radians(step_deg)
    return out


def swept_bound(side: str, q_a: dict, q_b: dict, q_mid: dict | None) -> float:
    grads = [joint_gradients(side, q_a), joint_gradients(side, q_b)]
    if q_mid is not None:
        grads.append(joint_gradients(side, q_mid))
    bound = 0.0
    for key in JOINTS:
        worst = max(g[key] for g in grads)
        bound += abs(math.radians(float(q_b[key]) - float(q_a[key]))) * worst
    return bound * BOUND_SAFETY


def lerp_q(q_a: dict, q_b: dict, t: float) -> dict:
    return {key: float(q_a[key] + (q_b[key] - q_a[key]) * t) for key in JOINTS}


def evaluate_sequence(side: str, samples: list[dict], start_target: dict, end_target: dict, geom: dict) -> list[dict]:
    out = []
    for sample in samples:
        t = float(sample["t"])
        target = ms.interp_target(start_target, end_target, t)
        row = metrics(side, sample["q_deg"], target, geom, ms.P0_Q)
        row["t"] = t
        out.append(row)
    return out


def summarize(rows: list[dict]) -> dict:
    def values(key):
        return [row[key] for row in rows if row[key] is not None]

    steps = []
    for first, second in zip(rows, rows[1:]):
        step, joint = max_step(first["q_deg"], second["q_deg"])
        steps.append({"step_deg": step, "joint": joint, "t": second["t"]})
    return {
        "sample_count": len(rows),
        "max_position_deviation_mm": max(values("position_error_mm")),
        "max_orientation_error_deg": max(values("orientation_error_deg")),
        "min_margin_deg": min(values("min_margin_deg")),
        "min_margin_joint": min(rows, key=lambda row: row["min_margin_deg"])["min_margin_joint"],
        "max_condition": max(values("condition")),
        "min_clearance_m": min(values("clearance_min_m")),
        "max_joint_step_deg": max(step["step_deg"] for step in steps) if steps else 0.0,
        "collision_free": all(row["collision_free"] for row in rows),
        "within_limits": all(row["within_limits"] for row in rows),
        "endpoint_position_error_mm": rows[-1]["position_error_mm"],
        "endpoint_orientation_error_deg": rows[-1]["orientation_error_deg"],
        "endpoint_margin_deg": rows[-1]["min_margin_deg"],
        "endpoint_margin_joint": rows[-1]["min_margin_joint"],
    }


# ---------------------------------------------------------------- kinematics


def task_jacobian(side: str, q_deg: dict, target: dict) -> tuple[np.ndarray, np.ndarray, dict]:
    task = ik.task_metrics(side, q_deg, ms.LIFT_M, target, ms.MODE)
    jacobian = np.asarray(task["scaled_jacobian"], dtype=float)
    error = np.asarray(task["task_error"], dtype=float) * np.array(
        [1.0 / ik.L_NORM_M] * 3 + [1.0] * (jacobian.shape[0] - 3), dtype=float
    )
    return jacobian, error, task


def nullspace(jacobian: np.ndarray, damping: float = 1e-3) -> np.ndarray:
    normal = jacobian @ jacobian.T + (damping**2) * np.eye(jacobian.shape[0])
    try:
        pinv = np.linalg.solve(normal, jacobian).T
    except np.linalg.LinAlgError:
        pinv = np.linalg.pinv(jacobian, rcond=1e-12)
    return np.eye(jacobian.shape[1]) - pinv @ jacobian, pinv


def project_to_pose(side: str, values: np.ndarray, target: dict, iterations: int = 3) -> tuple[np.ndarray, float]:
    """Task-space correction only: no null-space term, so no self-motion jump."""
    current = values.copy()
    worst = 0.0
    for _ in range(iterations):
        q_deg = {key: float(value) for key, value in zip(JOINTS, current)}
        jacobian, error, task = task_jacobian(side, q_deg, target)
        worst = float(np.linalg.norm(task["position_error"]))
        if worst < 1e-9 and float(task["orientation_error_deg"]) < 1e-6:
            break
        _, pinv = nullspace(jacobian)
        step = pinv @ error
        current = ik.clip_q(side, current + np.degrees(step))
    return current, worst


def self_motion(side: str, target: dict, q_start: dict, q_goal: dict, geom: dict) -> dict:
    """Advance along the redundancy manifold at a fixed pose.

    The direction is the projection of (q_goal - q) onto the task null space,
    limited to ``SELF_MOTION_STEP_DEG`` per step.  Every step is corrected by
    a task-space-only projection, so the palm stays at the target and the step
    size stays bounded.
    """
    values = np.asarray([q_start[key] for key in JOINTS], dtype=float)
    goal = np.asarray([q_goal[key] for key in JOINTS], dtype=float)
    rows = []
    stalled = False
    for index in range(SELF_MOTION_MAX_STEPS + 1):
        q_deg = {key: float(value) for key, value in zip(JOINTS, values)}
        row = metrics(side, q_deg, target, geom, ms.P0_Q)
        row["step"] = index
        row["t"] = float(index)
        rows.append(row)
        difference = goal - values
        if float(np.max(np.abs(difference))) < 0.05:
            break
        jacobian, _, _ = task_jacobian(side, q_deg, target)
        projector, _ = nullspace(jacobian)
        direction = projector @ difference
        largest = float(np.max(np.abs(direction)))
        if largest < 1e-9:
            stalled = True
            break
        if largest > SELF_MOTION_STEP_DEG:
            direction *= SELF_MOTION_STEP_DEG / largest
        candidate = ik.clip_q(side, values + direction)
        corrected, _ = project_to_pose(side, candidate, target, iterations=3)
        if float(np.max(np.abs(corrected - values))) < 1e-6:
            stalled = True
            break
        values = corrected
    summary = summarize(rows)
    summary["status"] = "stalled" if stalled else "valid"
    summary["steps_used"] = len(rows) - 1
    summary["goal_distance_deg"] = float(
        max(abs(rows[-1]["q_deg"][key] - q_goal[key]) for key in JOINTS)
    )
    summary["start_margin_deg"] = rows[0]["min_margin_deg"]
    summary["end_margin_deg"] = rows[-1]["min_margin_deg"]
    summary["start_q"] = rows[0]["q_deg"]
    summary["end_q"] = rows[-1]["q_deg"]
    summary["samples"] = rows
    return summary


def hybrid_switch(
    side: str,
    prefix: list[dict],
    index: int,
    q_goal: dict,
    start_target: dict,
    end_target: dict,
    geom: dict,
) -> dict:
    """Straight-line prefix up to ``index``, then a joint-space blend to q_goal."""
    head = [{"t": float(sample["t"]), "q_deg": sample["q_deg"]} for sample in prefix[: index + 1]]
    q_start = head[-1]["q_deg"]
    distance = max(abs(float(q_goal[key]) - float(q_start[key])) for key in JOINTS)
    count = max(4, int(math.ceil(distance / ms.JOINT_STEP_DEG)))
    t0 = head[-1]["t"]
    tail = []
    for step in range(1, count + 1):
        fraction = step / count
        tail.append({"t": float(t0 + (1.0 - t0) * fraction), "q_deg": lerp_q(q_start, q_goal, fraction)})
    rows = evaluate_sequence(side, head + tail, start_target, end_target, geom)
    summary = summarize(rows)
    summary["switch_index"] = index
    summary["switch_t"] = t0
    summary["blend_samples"] = count
    return summary


def joint_path_check(
    side: str,
    q_from: dict,
    q_to: dict,
    geom: dict,
    target: dict,
) -> dict:
    """Pure joint-space move: limits, margins and collisions (no path envelope)."""
    distance = max(abs(float(q_to[key]) - float(q_from[key])) for key in JOINTS)
    count = max(4, int(math.ceil(distance / ms.JOINT_STEP_DEG)))
    rows = []
    for step in range(count + 1):
        fraction = step / count
        q_deg = lerp_q(q_from, q_to, fraction)
        row = metrics(side, q_deg, target, geom, ms.P0_Q)
        row["t"] = fraction
        rows.append(row)
    summary = summarize(rows)
    summary["sample_count"] = len(rows)
    summary["joint_distance_deg"] = distance
    return summary


def certify_interval(
    side: str,
    q_a: dict,
    q_b: dict,
    geom: dict,
    depth: int,
) -> dict:
    """Recursive conservative certification of one joint-space interval."""
    if depth == 0:
        clear_a = bed.collision_clearances(side, q_a, ms.LIFT_M, geom, other_q=ms.P0_Q)
        clear_b = bed.collision_clearances(side, q_b, ms.LIFT_M, geom, other_q=ms.P0_Q)
        minimal = min(clear_a["minimum"], clear_b["minimum"])
    else:
        clear_a = bed.collision_clearances(side, q_a, ms.LIFT_M, geom, other_q=ms.P0_Q)
        clear_b = bed.collision_clearances(side, q_b, ms.LIFT_M, geom, other_q=ms.P0_Q)
        minimal = min(clear_a["minimum"], clear_b["minimum"])
    q_mid = lerp_q(q_a, q_b, 0.5)
    bound = swept_bound(side, q_a, q_b, q_mid)
    if minimal - bound > 0.0:
        return {"status": "certified", "depth": depth, "clearance_m": minimal, "bound_m": bound}
    if depth >= MAX_SPLIT_DEPTH:
        return {
            "status": "undefined",
            "depth": depth,
            "clearance_m": minimal,
            "bound_m": bound,
            "deficit_m": bound - minimal,
        }
    left = certify_interval(side, q_a, q_mid, geom, depth + 1)
    right = certify_interval(side, q_mid, q_b, geom, depth + 1)
    certified = [item for item in (left, right) if item["status"] == "certified"]
    undefined = [item for item in (left, right) if item["status"] == "undefined"]
    return {
        "status": "certified" if not undefined else "undefined",
        "depth": depth,
        "clearance_m": min(clear_a["minimum"], clear_b["minimum"]),
        "bound_m": bound,
        "children": [left, right],
        "undefined_children": len(undefined),
        "certified_children": len(certified),
    }


def flatten_certification(node: dict, out: list[dict]) -> None:
    if "children" not in node:
        out.append(node)
        return
    for child in node["children"]:
        flatten_certification(child, out)


def endpoint_jump(transition: dict) -> dict | None:
    samples = transition["samples"]
    if len(samples) < 2:
        return None
    first, second = samples[-2], samples[-1]
    step, joint = max_step(first["q_deg"], second["q_deg"])
    diffs = delta_q(first["q_deg"], second["q_deg"])
    return {
        "from_t": first["t"],
        "to_t": second["t"],
        "max_step_deg": step,
        "joint": joint,
        "delta_deg": diffs,
        "euclidean_deg": float(math.sqrt(sum(value * value for value in diffs.values()))),
        "margin_before_deg": first["min_margin_deg"],
        "margin_after_deg": second["min_margin_deg"],
        "position_error_before_mm": first["position_error_mm"],
        "position_error_after_mm": second["position_error_mm"],
    }


def main() -> int:
    started = time.time()
    ms.MODE = "6d"
    geom = geometry()
    artifact = json.loads(ARTIFACT.read_text(encoding="utf-8"))
    refs = references(geom)
    labels = [item["label"] for item in refs]
    end_reference = refs[-1]
    start_reference = refs[0]
    end_target = end_reference["target"]
    before_end_target = refs[-2]["target"]

    payload: dict = {
        "metadata": {
            "stage": "2B.2",
            "kind": "control_analysis",
            "created_utc": datetime.now(timezone.utc).isoformat(),
            "operation": OPERATION,
            "side": SIDE,
            "clearance_m": CLEARANCE,
            "height_m": HEIGHT,
            "width_m": WIDTH,
            "finger_yaw_deg": FINGER_YAW,
            "mode": ms.MODE,
            "source_artifact": str(ARTIFACT.relative_to(BASE)),
            "source_sha256": hashlib.sha256(ARTIFACT.read_bytes()).hexdigest(),
            "script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            "bound_safety": BOUND_SAFETY,
            "self_motion_step_deg": SELF_MOTION_STEP_DEG,
            "max_split_depth": MAX_SPLIT_DEPTH,
        }
    }

    # ---------------------------------------------------------------- A
    trad = artifact["policies"]["strict_6d"]["transitions"]
    asis = artifact["policies"]["as_is"]["transitions"]
    strict_end = trad[-1]["samples"][-1]["q_deg"]
    asis_end = asis[-1]["samples"][-1]["q_deg"]
    static_rows = []
    for index, solution in enumerate(end_reference["solutions"]):
        row = metrics(SIDE, solution, end_target, geom, ms.P0_Q)
        row["index"] = index
        row["distance_to_strict_end_deg"] = float(max_step(solution, strict_end)[0])
        row["distance_to_as_is_end_deg"] = float(max_step(solution, asis_end)[0])
        static_rows.append(row)
    static_rows.sort(key=lambda row: -row["min_margin_deg"])
    payload["terminal_pose_manifold"] = {
        "label": labels[-1],
        "reference_margin_deg": metrics(SIDE, end_reference["q_deg"], end_target, geom, ms.P0_Q)["min_margin_deg"],
        "solutions": static_rows,
        "strict_end": metrics(SIDE, strict_end, end_target, geom, ms.P0_Q),
        "as_is_end": metrics(SIDE, asis_end, end_target, geom, ms.P0_Q),
    }

    # ---------------------------------------------------------------- B
    continuity = {}
    for policy, transitions in (("as_is", asis), ("strict_6d", trad)):
        transitions_rows = []
        for transition in transitions:
            samples = transition["samples"]
            steps = []
            for first, second in zip(samples, samples[1:]):
                step, joint = max_step(first["q_deg"], second["q_deg"])
                steps.append({"t": second["t"], "step_deg": step, "joint": joint})
            values = sorted(item["step_deg"] for item in steps)
            median = values[len(values) // 2] if values else 0.0
            transitions_rows.append(
                {
                    "from": transition["from"],
                    "to": transition["to"],
                    "method": transition["selected_method"],
                    "sample_count": len(samples),
                    "steps": steps,
                    "max_step_deg": values[-1] if values else 0.0,
                    "median_step_deg": median,
                    "p95_step_deg": values[min(len(values) - 1, int(0.95 * len(values)))] if values else 0.0,
                    "anomalies": [
                        item for item in steps if median > 0 and item["step_deg"] > max(3.0 * median, 3.0)
                    ],
                    "endpoint_jump": endpoint_jump(transition),
                }
            )
        continuity[policy] = {
            "transitions": transitions_rows,
            "boundary_steps": [
                {
                    "at": transitions_rows[index]["to"],
                    "step_deg": max_step(transitions[index]["samples"][-1]["q_deg"], transitions[index + 1]["samples"][0]["q_deg"])[0],
                }
                for index in range(len(transitions) - 1)
            ],
        }
    payload["continuity"] = continuity

    # ---------------------------------------------------------------- C
    loop = {}
    for policy, transitions in (("as_is", asis), ("strict_6d", trad)):
        q_start = transitions[0]["samples"][0]["q_deg"]
        q_end = transitions[-1]["samples"][-1]["q_deg"]
        diffs = delta_q(q_start, q_end)
        loop[policy] = {
            "q_start": q_start,
            "q_end": q_end,
            "delta_deg": diffs,
            "max_abs_delta_deg": max(abs(value) for value in diffs.values()),
            "joint_of_max_delta": max(diffs, key=lambda key: abs(diffs[key])),
            "euclidean_delta_deg": float(math.sqrt(sum(value * value for value in diffs.values()))),
        }
    payload["loop_closure"] = loop

    # ---------------------------------------------------------------- D
    best_static = max(static_rows, key=lambda row: row["min_margin_deg"])
    worst_static = min(static_rows, key=lambda row: row["min_margin_deg"])
    q_best = best_static["q_deg"]
    payload["self_motion_terminal"] = self_motion(SIDE, end_target, strict_end, q_best, geom)
    payload["self_motion_manifold"] = {
        "from_index": worst_static["index"],
        "from_margin_deg": worst_static["min_margin_deg"],
        "to_index": best_static["index"],
        "to_margin_deg": best_static["min_margin_deg"],
        "path": self_motion(SIDE, end_target, worst_static["q_deg"], q_best, geom),
    }

    # ---------------------------------------------------------------- E
    switches = []
    prefix = trad[-1]["samples"]
    for fraction in (0.5, 0.6, 0.7, 0.8, 0.9, 0.95):
        index = min(len(prefix) - 1, max(0, int(round(fraction * (len(prefix) - 1)))))
        switches.append(
            hybrid_switch(SIDE, prefix, index, q_best, before_end_target, end_target, geom)
        )
    full_lerp = next(
        (candidate for candidate in trad[-1]["candidates"] if candidate["method"] == "joint_lerp"),
        None,
    )
    payload["branch_switch_cost"] = {
        "goal_q": q_best,
        "goal_margin_deg": best_static["min_margin_deg"],
        "switches": switches,
        "full_joint_lerp": full_lerp,
    }

    # ---------------------------------------------------------------- F
    swept = {}
    for policy, transitions in (("as_is", asis), ("strict_6d", trad)):
        per_transition = []
        for transition in transitions:
            samples = transition["samples"]
            intervals = []
            for first, second in zip(samples, samples[1:]):
                node = certify_interval(SIDE, first["q_deg"], second["q_deg"], geom, 0)
                leaves: list[dict] = []
                flatten_certification(node, leaves)
                undefined = [leaf for leaf in leaves if leaf["status"] == "undefined"]
                intervals.append(
                    {
                        "from_t": first["t"],
                        "to_t": second["t"],
                        "status": node["status"],
                        "bound_m": node["bound_m"],
                        "leaf_count": len(leaves),
                        "certified_leaf_count": len(leaves) - len(undefined),
                        "min_certified_clearance_m": min(
                            (leaf["clearance_m"] for leaf in leaves if leaf["status"] == "certified"),
                            default=None,
                        ),
                        "worst_deficit_m": max((leaf.get("deficit_m", 0.0) for leaf in undefined), default=0.0),
                    }
                )
            bounds = [item["bound_m"] for item in intervals]
            per_transition.append(
                {
                    "from": transition["from"],
                    "to": transition["to"],
                    "interval_count": len(intervals),
                    "certified_intervals": sum(1 for item in intervals if item["status"] == "certified"),
                    "undefined_intervals": sum(1 for item in intervals if item["status"] == "undefined"),
                    "min_certified_clearance_m": min(
                        (item["min_certified_clearance_m"] for item in intervals if item["min_certified_clearance_m"] is not None),
                        default=None,
                    ),
                    "max_bound_m": max(bounds) if bounds else None,
                    "mean_bound_m": float(np.mean(bounds)) if bounds else None,
                    "worst_deficit_m": max((item["worst_deficit_m"] for item in intervals), default=0.0),
                    "intervals": intervals,
                }
            )
        swept[policy] = per_transition
    payload["swept_check"] = swept

    # ---------------------------------------------------------------- G
    payload["return_path"] = {
        "from": "strict_6d end (S5_ready)",
        "to": "start reference (S0_ready)",
        "path": joint_path_check(SIDE, strict_end, start_reference["q_deg"], geom, end_target),
    }

    payload["runtime_seconds"] = round(time.time() - started, 1)
    OUT.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")

    # ------------------------------------------------------------ report
    print(f"written: {OUT} ({payload['runtime_seconds']} s)")
    print()
    print("A. Статические решения позы S5_ready (многообразие избыточности)")
    for row in static_rows:
        print(
            f"   #{row['index']:2d} margin={row['min_margin_deg']:6.3f} ({row['min_margin_joint']} {row['min_margin_nearest']})"
            f" cond={row['condition']:5.2f} clr={row['clearance_min_m']:.3f}"
            f" |Δ|до strict={row['distance_to_strict_end_deg']:6.2f} до as_is={row['distance_to_as_is_end_deg']:6.2f}"
        )
    print(f"   strict end: margin={payload['terminal_pose_manifold']['strict_end']['min_margin_deg']:.3f}"
          f" ({payload['terminal_pose_manifold']['strict_end']['min_margin_joint']})")
    print(f"   as_is  end: margin={payload['terminal_pose_manifold']['as_is_end']['min_margin_deg']:.3f}"
          f" ({payload['terminal_pose_manifold']['as_is_end']['min_margin_joint']})")
    print()
    print("B. Непрерывность (максимальный шаг сустава)")
    for policy, data in continuity.items():
        for row in data["transitions"]:
            jump = row["endpoint_jump"]
            extra = ""
            if jump and jump["max_step_deg"] > 3.0:
                extra = (
                    f"  СКАЧОК на конце: {jump['max_step_deg']:.2f}° ({jump['joint']}),"
                    f" |Δq|={jump['euclidean_deg']:.2f}°, margin {jump['margin_before_deg']:.2f}->{jump['margin_after_deg']:.2f}"
                )
            print(
                f"   {policy:9s} {row['from']:>10}->{row['to']:<10} n={row['sample_count']:3d}"
                f" max={row['max_step_deg']:6.2f}° median={row['median_step_deg']:5.3f}° p95={row['p95_step_deg']:5.2f}°{extra}"
            )
        print(f"   {policy:9s} стыки переходов: " + ", ".join(
            f"{item['at']}={item['step_deg']:.2f}°" for item in data["boundary_steps"]) )
    print()
    print("C. Замкнутость цикла по q")
    for policy, data in loop.items():
        print(
            f"   {policy:9s} max|Δq|={data['max_abs_delta_deg']:6.3f}° ({data['joint_of_max_delta']})"
            f" |Δq|={data['euclidean_delta_deg']:6.3f}°"
        )
    print()
    print("D. Самодвижение в конечной позе (фиксированная поза, только нуль-пространство)")
    for name in ("self_motion_terminal", "self_motion_manifold"):
        item = payload[name]["path"] if name == "self_motion_manifold" else payload[name]
        print(
            f"   {name}: {item['start_margin_deg']:.3f}° -> {item['end_margin_deg']:.3f}°"
            f" ({item['endpoint_margin_joint']}), шагов={item['steps_used']},"
            f" max|Δq|шаг={item['max_joint_step_deg']:.3f}°, невязка позы={item['max_position_deviation_mm']:.6f} мм"
            f" / {item['max_orientation_error_deg']:.6f}°, min_clr={item['min_clearance_m']:.3f} м,"
            f" не достигнуто={item['goal_distance_deg']:.3f}°, status={item['status']}"
        )
    print()
    print("E. Стоимость перехода на лучшую ветвь вдоль траектории (переключение в точке t0)")
    for row in switches:
        print(
            f"   t0={row['switch_t']:.2f} dev={row['max_position_deviation_mm']:7.2f} мм"
            f" ori={row['max_orientation_error_deg']:6.2f}° margin={row['min_margin_deg']:6.2f}°"
            f" clr={row['min_clearance_m']:.3f} шаг={row['max_joint_step_deg']:.2f}°"
        )
    if full_lerp:
        print(
            f"   полный joint_lerp (артефакт): dev={full_lerp.get('max_position_deviation_mm'):.2f} мм"
            f" ori={full_lerp.get('max_orientation_error_deg'):.2f}° margin={full_lerp.get('min_margin_deg'):.2f}°"
        )
    print()
    print("F. Межсэмпловая сертификация (консервативные границы)")
    for policy, rows in swept.items():
        for row in rows:
            print(
                f"   {policy:9s} {row['from']:>10}->{row['to']:<10}"
                f" интервалов={row['interval_count']:3d} серт.={row['certified_intervals']:3d}"
                f" неопр.={row['undefined_intervals']:3d} min_clr={row['min_certified_clearance_m']}"
                f" bound_mean={row['mean_bound_m']:.4f} bound_max={row['max_bound_m']:.4f}"
            )
    print()
    print("G. Возврат из конечной конфигурации в стартовую")
    row = payload["return_path"]["path"]
    print(
        f"   путь {row['joint_distance_deg']:.2f}° по q, шаг={row['max_joint_step_deg']:.2f}°"
        f" margin={row['min_margin_deg']:.2f}° clr={row['min_clearance_m']:.3f} м"
        f" столкновений={not row['collision_free']} пределы={row['within_limits']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
