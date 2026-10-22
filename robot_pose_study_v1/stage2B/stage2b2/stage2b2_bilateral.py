#!/usr/bin/env python3
"""Stage 2B.2 — bilateral synchronous checks with separate clearance categories.

Runs synchronous two-arm trajectories for P6/P7 at c=0.05 and c=0.10.
Clearance categories:
- arm_arm: capsules of left arm vs right arm
- palm_palm: palm boxes of both hands
- arm_bed: arm capsules vs bed box
- arm_torso: arm capsules vs torso box
- arm_base: arm capsules vs base box
"""

from __future__ import annotations

import json
import math
import sys
import time
import zlib
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

# Path setup
HERE = Path(__file__).resolve().parent
BASE = HERE.parents[1]
for _sub in ("stage2B", "stage2A", "stage2A_ik"):
    sys.path.insert(0, str(BASE / _sub))
sys.path.insert(0, str(HERE))

import bed_study as bed
import fk_model
import ik_study as ik
import motion_study as ms
import stage2b2_control_p7r as ctl

# --- policy parameters (same as unilateral) ---
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

# --- geometry helpers ---
def geometry(clearance: float) -> dict:
    return bed.bed_geometry("lateral", 0.7, 1.0, clearance)

def finger_vector(op: str) -> np.ndarray:
    if op == "P6":
        return bed.PLUS_X.copy()
    yaw = math.radians(80.0)
    return np.array([math.sin(yaw), math.cos(yaw), 0.0], dtype=float)

def references(op: str, side: str, geom: dict) -> list[dict]:
    """Static references with S5_ready == S0_ready."""
    sequence = ms.operation_sequence(op, side, geom)
    rotation = bed.orientation_from_normal_finger(side, bed.DOWN, finger_vector(op))
    for item in sequence:
        item["target"]["rotation"] = rotation.tolist()
        item["target"]["finger"] = [float(v) for v in finger_vector(op)]
    out = []
    for idx, item in enumerate(sequence):
        if item["label"] == "S5_ready":
            target = out[0]["target"]
            q_deg = {k: v for k, v in out[0]["q_deg"].items()}
            solutions = out[0]["solutions"]
        else:
            seed = zlib.crc32(f"{op}|{side}|{item['label']}|{geom['clearance_m']}".encode("utf-8"))
            result = ms.static_solve(side, item["target"], int(seed))
            chosen = ms.pick_reference(result, side, geom)
            target = item["target"]
            q_deg = chosen["q_deg"]
            solutions = [s["q_deg"] for s in result.get("solutions", []) if s.get("converged")]
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
    return np.array([q_deg[key] for key in ms.JOINT_KEYS], dtype=float)

def q_dict(values: np.ndarray) -> dict:
    return {key: float(value) for key, value in zip(ms.JOINT_KEYS, values)}

# --- unilateral policy (same as batch) ---
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

def anchor_for(refs: list[dict], segment: int, fraction: float) -> dict:
    return ctl.lerp_q(refs[segment]["q_deg"], refs[segment + 1]["q_deg"], fraction)

def plan_cycle(side: str, refs: list[dict], geom: dict) -> list[dict]:
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
            anchor = anchor_for(refs, segment, fraction)

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
                abs(float(q_track[key]) - float(q_current[key])) for key in ms.JOINT_KEYS
            )
            is_waypoint = (index == count)
            if not is_waypoint and step_from_previous > DIAG_HIGH_DEG:
                q_alt, _, iterations_alt = track_sample(side, target, q_current)
                alt_metrics = ik.task_metrics(side, q_alt, ms.LIFT_M, target, ms.MODE)
                alt_pos = float(np.linalg.norm(alt_metrics["position_error"]) * 1000.0)
                alt_ori = float(alt_metrics["orientation_error_deg"])
                alt_step = max(abs(float(q_alt[key]) - float(q_current[key])) for key in ms.JOINT_KEYS)
                if alt_pos <= POSE_POS_TOL_MM and alt_ori <= 1e-2 and alt_step < step_from_previous:
                    q_track = q_alt
                    iterations = iterations_alt
                    fallback = True

            injected = 0.0
            if True:  # bias always on
                steps = WAYPOINT_BIAS_STEPS if is_waypoint else BIAS_STEPS
                q_new, injected = null_bias(side, target, q_track, anchor, cap_deg=BIAS_CAP_DEG, steps=steps)
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
                    "waypoint": second["label"] if is_waypoint else None,
                    "fallback": fallback,
                }
            )
            samples.append(row)
            q_current = q_new

    for index, row in enumerate(samples):
        row["t"] = index / max(1, len(samples) - 1)
    return samples

# --- bilateral clearance checking ---
def get_arm_capsules(side: str, q_deg: dict) -> list[tuple[np.ndarray, np.ndarray, float]]:
    """Return list of (p0, p1, radius) for arm capsules in world frame."""
    segs = bed.arm_segments(side, q_deg, ms.LIFT_M)
    capsules = []
    for seg in segs:
        p0 = np.array(seg["p0"], dtype=float)
        p1 = np.array(seg["p1"], dtype=float)
        r = float(seg["radius"])
        capsules.append((p0, p1, r))
    return capsules

def capsule_distance(p0_a, p1_a, r_a, p0_b, p1_b, r_b) -> float:
    """Min distance between two capsules (segment-segment distance - radii)."""
    # Segment-segment distance in 3D
    # From "Distance between two segments in 3D" - standard algorithm
    u = p1_a - p0_a
    v = p1_b - p0_b
    w0 = p0_a - p0_b
    a = float(np.dot(u, u))
    b = float(np.dot(u, v))
    c = float(np.dot(v, v))
    d = float(np.dot(u, w0))
    e = float(np.dot(v, w0))
    D = a * c - b * b
    sc, sN, sD = D, D, D
    tc, tN, tD = D, D, D
    SMALL_NUM = 1e-12
    if D < SMALL_NUM:
        sN = 0.0
        sD = 1.0
        tN = e
        tD = c
    else:
        sN = (b * e - c * d)
        tN = (a * e - b * d)
        if sN < 0.0:
            sN = 0.0
            tN = e
            tD = c
        elif sN > sD:
            sN = sD
            tN = e + b
            tD = c
    if tN < 0.0:
        tN = 0.0
        if -d < 0.0:
            sN = 0.0
        elif -d > a:
            sN = sD
        else:
            sN = -d
            sD = a
    elif tN > tD:
        tN = tD
        if (-d + b) < 0.0:
            sN = 0.0
        elif (-d + b) > a:
            sN = sD
        else:
            sN = (-d + b)
            sD = a
    sc = 0.0 if abs(sN) < SMALL_NUM else sN / sD
    tc = 0.0 if abs(tN) < SMALL_NUM else tN / tD
    dP = w0 + sc * u - tc * v
    dist = float(np.linalg.norm(dP))
    return max(0.0, dist - r_a - r_b)

def palm_box(side: str, q_deg: dict) -> tuple[np.ndarray, np.ndarray]:
    """Return (center, half_extents) of palm box in world frame."""
    geom = fk_model.palm_geometry(side, q_deg, ms.LIFT_M)
    center = np.array(geom["face_floor"], dtype=float)
    # Approximate palm box: use local bbox transformed
    # For simplicity, use a fixed half-extents based on known palm size
    half_ext = np.array([0.04, 0.05, 0.02], dtype=float)  # x, y, z
    return center, half_ext

def box_distance(c1, h1, c2, h2) -> float:
    """Min distance between two AABBs (center + half_extents)."""
    diff = np.abs(c2 - c1) - (h1 + h2)
    return float(np.max(np.maximum(diff, 0.0)))

def bed_box(geom: dict) -> tuple[np.ndarray, np.ndarray]:
    """Bed as AABB: center and half-extents."""
    # bed_geometry returns x_near, x_far, y_near, y_far, z_bottom, z_top
    x_near = geom["x_near"]
    x_far = geom["x_far"]
    y_near = geom.get("y_near", -geom["width_m"]/2)
    y_far = geom.get("y_far", geom["width_m"]/2)
    z_bottom = geom.get("z_bottom", 0.0)
    z_top = geom.get("z_top", geom["height_m"])
    center = np.array([(x_near + x_far)/2, (y_near + y_far)/2, (z_bottom + z_top)/2])
    half = np.array([(x_far - x_near)/2, (y_far - y_near)/2, (z_top - z_bottom)/2])
    return center, half

def torso_box() -> tuple[np.ndarray, np.ndarray]:
    """Torso as AABB (approximate)."""
    center = np.array([0.0, 0.0, 0.6])
    half = np.array([0.25, 0.15, 0.3])
    return center, half

def base_box() -> tuple[np.ndarray, np.ndarray]:
    """Base as AABB (approximate)."""
    center = np.array([0.0, 0.0, 0.1])
    half = np.array([0.3, 0.2, 0.1])
    return center, half

def check_bilateral_clearances(qL: dict, qR: dict, geom: dict) -> dict:
    """Check all clearance categories between two arms and environment."""
    capsL = get_arm_capsules("L", qL)
    capsR = get_arm_capsules("R", qR)
    
    # arm-arm: min capsule distance
    arm_arm = float('inf')
    for p0_a, p1_a, r_a in capsL:
        for p0_b, p1_b, r_b in capsR:
            d = capsule_distance(p0_a, p1_a, r_a, p0_b, p1_b, r_b)
            arm_arm = min(arm_arm, d)
    
    # palm-palm
    palmL_c, palmL_h = palm_box("L", qL)
    palmR_c, palmR_h = palm_box("R", qR)
    palm_palm = box_distance(palmL_c, palmL_h, palmR_c, palmR_h)
    
    # arm-bed
    bed_c, bed_h = bed_box(geom)
    arm_bed = float('inf')
    for caps in (capsL, capsR):
        for p0, p1, r in caps:
            # Approximate: segment vs box
            # Simplified: check endpoints distance to box
            for p in (p0, p1):
                diff = np.abs(p - bed_c) - bed_h
                d = float(np.max(np.maximum(diff, 0.0)))
                arm_bed = min(arm_bed, max(0.0, d - r))
    
    # arm-torso
    torso_c, torso_h = torso_box()
    arm_torso = float('inf')
    for caps in (capsL, capsR):
        for p0, p1, r in caps:
            for p in (p0, p1):
                diff = np.abs(p - torso_c) - torso_h
                d = float(np.max(np.maximum(diff, 0.0)))
                arm_torso = min(arm_torso, max(0.0, d - r))
    
    # arm-base
    base_c, base_h = base_box()
    arm_base = float('inf')
    for caps in (capsL, capsR):
        for p0, p1, r in caps:
            for p in (p0, p1):
                diff = np.abs(p - base_c) - base_h
                d = float(np.max(np.maximum(diff, 0.0)))
                arm_base = min(arm_base, max(0.0, d - r))
    
    return {
        "arm_arm": arm_arm,
        "palm_palm": palm_palm,
        "arm_bed": arm_bed,
        "arm_torso": arm_torso,
        "arm_base": arm_base,
    }

def run_bilateral(op: str, clearance: float) -> dict:
    geom = geometry(clearance)
    refs_L = references(op, "L", geom)
    refs_R = references(op, "R", geom)
    
    # Plan cycles
    samples_L = plan_cycle("L", refs_L, geom)
    samples_R = plan_cycle("R", refs_R, geom)
    
    # Check clearances at each synchronized sample
    min_clearances = {k: float('inf') for k in ["arm_arm", "palm_palm", "arm_bed", "arm_torso", "arm_base"]}
    for sL, sR in zip(samples_L, samples_R):
        clr = check_bilateral_clearances(sL["q_deg"], sR["q_deg"], geom)
        for k, v in clr.items():
            min_clearances[k] = min(min_clearances[k], v)
    
    # Also check unilateral clearances for each arm
    def unilateral_min(samples):
        return {k: min(s[k] for s in samples) for k in ["arm_bed_m", "arm_arm_m", "arm_torso_m", "arm_base_m"]}
    
    return {
        "operation": op,
        "clearance": clearance,
        "bilateral_min": min_clearances,
        "unilateral_L_min": unilateral_min(samples_L),
        "unilateral_R_min": unilateral_min(samples_R),
        "samples_L_count": len(samples_L),
        "samples_R_count": len(samples_R),
    }

def main():
    configs = [
        ("P6", 0.05),
        ("P7", 0.05),
        ("P6", 0.10),
        # ("P7", 0.10),  # wait for decision
    ]
    
    results = {}
    for op, clr in configs:
        print(f"\n=== {op} c={clr} bilateral ===")
        res = run_bilateral(op, clr)
        results[f"{op}_c{clr:.2f}"] = res
        print(f"  bilateral mins: { {k: round(v,6) for k,v in res['bilateral_min'].items()} }")
        print(f"  unilateral L: { {k: round(v,6) for k,v in res['unilateral_L_min'].items()} }")
        print(f"  unilateral R: { {k: round(v,6) for k,v in res['unilateral_R_min'].items()} }")
    
    # Save
    out_dir = Path(__file__).resolve().parent
    out_json = out_dir / "stage2b2_bilateral.json"
    out_json.write_text(json.dumps(results, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"\nSaved to {out_json}")

if __name__ == "__main__":
    main()