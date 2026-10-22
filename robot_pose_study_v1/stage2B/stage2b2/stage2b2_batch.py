#!/usr/bin/env python3
"""Batch runner for corrected IK policy on multiple (operation, side, clearance) combos."""

from __future__ import annotations

import sys
from pathlib import Path

# Path setup MUST come before any local imports
HERE = Path(__file__).resolve().parent
BASE = HERE.parents[1]  # robot_pose_study_v1
for _sub in ("stage2B", "stage2A", "stage2A_ik"):
    sys.path.insert(0, str(BASE / _sub))
sys.path.insert(0, str(HERE))

import csv
import json
import math
import time
import zlib
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np

import bed_study as bed  # noqa: E402
import fk_model  # noqa: E402
import ik_study as ik  # noqa: E402
import motion_study as ms  # noqa: E402
import stage2b2_control_p7r as ctl  # noqa: E402

# --- policy parameters (identical for all runs) ---
TASK_TOL = 1e-6
TASK_ITER_MAX = 60
STEP_CAP_DEG = 6.0
BIAS_CAP_DEG = 0.35
BIAS_STEPS = 1
WAYPOINT_BIAS_STEPS = 3
POSE_POS_TOL_MM = 0.01
POSE_ORI_TOL_DEG = 1e-3

DIAG_LOW_DEG = 3.0
DIAG_HIGH_DEG = 5.0


def run_one(
    operation: str,
    side: str,
    clearance: float,
    height: float = 0.7,
    width: float = 1.0,
    finger_yaw_deg: float = 80.0,
) -> dict:
    """Run corrected policy for one configuration."""
    started = time.time()

    # --- setup ---
    ms.MODE = "6d"
    JOINTS = ms.JOINT_KEYS

    def geometry() -> dict:
        return bed.bed_geometry("lateral", height, width, clearance)

    def finger_vector() -> np.ndarray:
        if operation == "P6":
            return bed.PLUS_X.copy()
        yaw = math.radians(finger_yaw_deg)
        return np.array([math.sin(yaw), math.cos(yaw), 0.0], dtype=float)

    def references(geom: dict) -> list[dict]:
        sequence = ms.operation_sequence(operation, side, geom)
        rotation = bed.orientation_from_normal_finger(side, bed.DOWN, finger_vector())
        for item in sequence:
            item["target"]["rotation"] = rotation.tolist()
            item["target"]["finger"] = [float(v) for v in finger_vector()]
        out = []
        s0_result = None
        s0_chosen = None
        for idx, item in enumerate(sequence):
            if item["label"] == "S5_ready":
                # S5_ready is the same pose as S0_ready; reuse S0's target and solution exactly
                target = out[0]["target"]
                q_deg = {k: v for k, v in out[0]["q_deg"].items()}
                solutions = out[0]["solutions"]
            else:
                seed = zlib.crc32(f"{operation}|{side}|{item['label']}|{geom['clearance_m']}".encode("utf-8"))
                result = ms.static_solve(side, item["target"], int(seed))
                chosen = ms.pick_reference(result, side, geom)
                target = item["target"]
                q_deg = chosen["q_deg"]
                solutions = [s["q_deg"] for s in result.get("solutions", []) if s.get("converged")]
                if item["label"] == "S0_ready":
                    s0_chosen = chosen
            out.append(
                {
                    "label": item["label"],
                    "target": target,
                    "position_floor": item["position_floor"],
                    "q_deg": q_deg,
                    "solutions": solutions,
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

    # --- policy ---
    def track_sample(side: str, target: dict, q_seed: dict, tol: float = TASK_TOL) -> tuple[dict, dict, int]:
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

                step_from_previous = max(
                    abs(float(q_track[key]) - float(q_current[key])) for key in JOINTS
                )
                is_waypoint = (index == count)
                if not is_waypoint and step_from_previous > DIAG_HIGH_DEG:
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
                        "t": 0.0,
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

    # --- metrics & validator ---
    def euclidean(a: dict, b: dict) -> float:
        return float(math.sqrt(sum((float(b[k]) - float(a[k])) ** 2 for k in JOINTS)))

    def per_joint(a: dict, b: dict) -> dict:
        return {key: float(b[key]) - float(a[key]) for key in JOINTS}

    def link_displacement(side: str, q_a: dict, q_b: dict) -> float:
        points_a = ctl.link_points(side, q_a)
        points_b = ctl.link_points(side, q_b)
        return float(np.max(np.linalg.norm(points_b - points_a, axis=1)))

    def palm_displacement(side: str, q_a: dict, q_b: dict) -> float:
        palm_a = fk_model.palm_geometry(side, q_a, ms.LIFT_M)["face_floor"]
        palm_b = fk_model.palm_geometry(side, q_b, ms.LIFT_M)["face_floor"]
        return float(np.linalg.norm(np.asarray(palm_b) - np.asarray(palm_a)))

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

    def continuity_summary(side: str, samples: list[dict]) -> dict:
        max_step = 0.0
        max_euclid = 0.0
        joints = {key: 0.0 for key in JOINTS}
        max_link = 0.0
        max_palm = 0.0
        max_pos_err = 0.0
        max_ori_err = 0.0
        injected_total = 0.0
        for a, b in zip(samples, samples[1:]):
            diff = {k: float(b["q_deg"][k]) - float(a["q_deg"][k]) for k in JOINTS}
            for k, v in diff.items():
                av = abs(v)
                joints[k] = max(joints[k], av)
            step = max(abs(v) for v in diff.values())
            max_step = max(max_step, step)
            max_euclid = max(max_euclid, euclidean(a["q_deg"], b["q_deg"]))
            max_link = max(max_link, link_displacement(side, a["q_deg"], b["q_deg"]))
            max_palm = max(max_palm, palm_displacement(side, a["q_deg"], b["q_deg"]))
            max_pos_err = max(max_pos_err, float(b["position_error_mm"]))
            max_ori_err = max(max_ori_err, float(b["orientation_error_deg"]))
            injected_total += float(b.get("injected_deg") or 0.0)
        return {
            "interval_count": len(samples) - 1,
            "max_joint_step_deg": max_step,
            "max_euclidean_dq_deg": max_euclid,
            "max_joint_by_joint_deg": joints,
            "max_link_displacement_m": max_link,
            "max_palm_displacement_m": max_palm,
            "max_position_error_mm": max_pos_err,
            "max_orientation_error_deg": max_ori_err,
            "collision_free": all(bool(s["collision_free"]) for s in samples),
            "within_limits": all(bool(s["within_limits"]) for s in samples),
            "injected_total_deg": injected_total,
        }

    def certify_path(side: str, samples: list[dict], geom: dict) -> dict:
        intervals = []
        undefined = 0
        minimum = None
        for index, (first, second) in enumerate(zip(samples, samples[1:])):
            step = max(abs(float(second["q_deg"][key]) - float(first["q_deg"][key])) for key in JOINTS)
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

    # --- old approach from artifact ---
    def old_approach(side: str, refs: list[dict], geom: dict) -> dict:
        artifact_path = BASE / "stage2B" / "stage2b1" / f"stage2b1_unilateral_{operation}_{side}_c{clearance:.2f}.json"
        if not artifact_path.exists():
            return {"error": f"artifact not found: {artifact_path}"}
        artifact = json.loads(artifact_path.read_text())
        strict = artifact["policies"]["strict_6d"]
        samples: list[dict] = []
        for transition in strict["transitions"]:
            for sample in transition["samples"]:
                if samples and abs(float(sample["t"]) - 0.0) < 1e-12:
                    continue
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

        def path_metrics_old(side: str, samples: list[dict]) -> dict:
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

        metrics = path_metrics_old(side, samples)
        terminal = path_metrics_old(side, motion_samples)
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
            "closure_by_q": {
                "max_abs_deg": max(abs(float(goal_q[k]) - float(start_q[k])) for k in JOINTS),
                "euclidean_deg": euclidean(start_q, goal_q),
                "delta_deg": {k: float(goal_q[k]) - float(start_q[k]) for k in JOINTS},
            },
            "continuity": continuity_summary(side, combined),
            "certificate": certify_path(side, combined, geom),
        }

    def euclidean(a: dict, b: dict) -> float:
        return float(math.sqrt(sum((float(b[k]) - float(a[k])) ** 2 for k in JOINTS)))

    def per_joint(a: dict, b: dict) -> dict:
        return {key: float(b[key]) - float(a[key]) for key in JOINTS}

    def link_displacement(side: str, q_a: dict, q_b: dict) -> float:
        points_a = ctl.link_points(side, q_a)
        points_b = ctl.link_points(side, q_b)
        return float(np.max(np.linalg.norm(points_b - points_a, axis=1)))

    def palm_displacement(side: str, q_a: dict, q_b: dict) -> float:
        palm_a = fk_model.palm_geometry(side, q_a, ms.LIFT_M)["face_floor"]
        palm_b = fk_model.palm_geometry(side, q_b, ms.LIFT_M)["face_floor"]
        return float(np.linalg.norm(np.asarray(palm_b) - np.asarray(palm_a)))

    def certify_path(side: str, samples: list[dict], geom: dict) -> dict:
        intervals = []
        undefined = 0
        minimum = None
        for index, (first, second) in enumerate(zip(samples, samples[1:])):
            step = max(abs(float(second["q_deg"][key]) - float(first["q_deg"][key])) for key in JOINTS)
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

    # --- main execution ---
    geom = geometry()
    refs = references(geom)
    if not all(ref["q_deg"] for ref in refs):
        return {"error": "static reference unreachable"}

    variants: dict[str, dict] = {}
    for mode in ("schedule", "start"):
        samples = plan_cycle(side, refs, geom, anchor_mode=mode, bias=True)
        variants[mode] = {
            "anchor_mode": mode,
            "samples": samples,
            "metrics": path_metrics(side, samples),
            "closure": {k: v for k, v in closure(side, refs[0]["q_deg"], samples[-1]["q_deg"]).items()},
            "continuity": continuity_summary(side, samples),
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
    new_metrics = path_metrics(side, new_samples)
    new_closure = closure(side, refs[0]["q_deg"], new_samples[-1]["q_deg"])
    new_continuity = continuity_summary(side, new_samples)

    old = old_approach(side, refs, geom)
    if "error" in old:
        return {"error": old["error"]}

    new_certificate = certify_path(side, new_samples, geom)

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
            "kind": "corrected_ik_policy_batch",
            "created_utc": datetime.now(timezone.utc).isoformat(),
            "operation": operation,
            "side": side,
            "clearance_m": clearance,
            "height_m": height,
            "width_m": width,
            "finger_yaw_deg": finger_yaw_deg if operation == "P7" else 0.0,
            "mode": ms.MODE,
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
            "runtime_seconds": time.time() - started,
        },
        "references": [
            {"label": ref["label"], "q_deg": ref["q_deg"], "position_floor": ref["position_floor"]}
            for ref in refs
        ],
        "new_cycle": {
            "method": "full_cycle_task_only_with_anchor_bias",
            "anchor_mode": "schedule",
            "metrics": new_metrics,
            "closure_by_q": new_closure,
            "continuity": continuity_summary(side, new_samples),
            "certificate": certify_path(side, new_samples, geom),
            "fallback_samples": sum(1 for s in new_samples if s.get("fallback")),
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
    return payload


def main():
    # Configurations to run
    configs = [
        # c=0.10 both arms
        ("P6", "R", 0.10, 80.0),
        ("P7", "R", 0.10, 80.0),
        ("P6", "L", 0.10, 80.0),
        ("P7", "L", 0.10, 80.0),
        # c=0.05 left arm
        ("P6", "L", 0.05, 0.0),
        ("P7", "L", 0.05, 80.0),
    ]

    results = {}
    for op, side, clr, yaw in configs:
        print(f"\n=== {op} {side} c={clr} yaw={yaw} ===")
        payload = run_one(op, side, clr, finger_yaw_deg=yaw)
        if "error" in payload:
            print(f"  ERROR: {payload['error']}")
            results[f"{op}_{side}_c{clr:.2f}"] = {"error": payload["error"]}
            continue

        key = f"{op}_{side}_c{clr:.2f}"
        results[key] = payload

        # Save individual JSON
        out_dir = Path(__file__).resolve().parent
        out_json = out_dir / f"stage2b2_policy_{op}_{side}_c{clr:.2f}.json"
        out_json.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")

        # Print comparison
        cmp = payload["comparison"]
        print(f"  NEW: path {cmp['new']['joint_path_length_deg']:.1f}°, margin {cmp['new']['min_margin_deg']:.3f}°, "
              f"maxstep {cmp['new']['max_joint_step_deg']:.2f}°, closure {cmp['new']['closure_max_abs_deg']:.2f}°, "
              f"cert {cmp['new']['certified_intervals']}, terminal={cmp['new']['terminal_manoeuvre']}")
        print(f"  OLD: path {cmp['old']['joint_path_length_deg']:.1f}°, margin {cmp['old']['min_margin_deg']:.3f}°, "
              f"maxstep {cmp['old']['max_joint_step_deg']:.2f}°, closure {cmp['old']['closure_max_abs_deg']:.2f}°, "
              f"cert {cmp['old']['certified_intervals']}, terminal={cmp['old']['terminal_manoeuvre']} ({cmp['old']['terminal_steps']} steps)")

    # Save summary
    out_dir = Path(__file__).resolve().parent
    summary_path = out_dir / "stage2b2_batch_summary.json"
    summary_path.write_text(json.dumps(results, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"\nSummary saved to {summary_path}")

    # Print summary table
    print("\n=== SUMMARY TABLE ===")
    print(f"{'Case':<20} {'New Path':>8} {'Old Path':>8} {'New Margin':>10} {'Old Margin':>10} {'New Step':>9} {'Old Step':>9} {'Closure':>8}")
    for key, val in results.items():
        if "error" in val:
            print(f"{key:<20} ERROR: {val['error']}")
            continue
        cmp = val["comparison"]
        print(f"{key:<20} {cmp['new']['joint_path_length_deg']:>7.1f} {cmp['old']['joint_path_length_deg']:>7.1f} "
              f"{cmp['new']['min_margin_deg']:>9.3f} {cmp['old']['min_margin_deg']:>9.3f} "
              f"{cmp['new']['max_joint_step_deg']:>8.2f} {cmp['old']['max_joint_step_deg']:>8.2f} "
              f"{cmp['new']['closure_max_abs_deg']:>7.2f}")

if __name__ == "__main__":
    main()