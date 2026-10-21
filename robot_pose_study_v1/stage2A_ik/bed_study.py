"""Stage 2A continuation: parametric bed, first reachability sample and simplified collisions.

The script does not modify the .blend.  It uses the validated FK/IK model from
``stage2A_ik/ik_study.py`` and a temporary, explicitly simplified collision
geometry.  Results are research data, not a safety classification.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path
from typing import Any

import numpy as np

sys.dont_write_bytecode = True

HERE = Path(__file__).resolve().parent
STAGE2A = HERE.parent / "stage2A"
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))
if str(STAGE2A) not in sys.path:
    sys.path.insert(0, str(STAGE2A))

import fk_model  # noqa: E402
import ik_study as ik  # noqa: E402

# Robot base geometry relative to the floor origin O (base centre projected
# to the floor).  The values are taken from model_key_facts.json.
BASE_HALF_X = 0.335
BASE_HALF_Y = 0.335
BASE_Z_MIN = 0.0466
BASE_Z_MAX = 0.2226

BED_LENGTH_M = 2.0
BED_HEIGHTS_M = (0.6, 0.7, 0.8, 0.9)
BED_WIDTHS_M = (0.9, 1.0, 1.2)
CLEARANCES_M = (0.05, 0.10, 0.15, 0.20)
BASELINE_CLEARANCE_M = 0.10
BASELINE_HEIGHT_M = 0.7
BASELINE_WIDTH_M = 1.0

# Contact-face approach heights above the bed surface for the first sample.
APPROACH_Z_M = 0.08
DIAPER_Z_M = 0.10
FEEDING_Z_M = 0.35

DOWN = np.array([0.0, 0.0, -1.0], dtype=float)
PLUS_X = np.array([1.0, 0.0, 0.0], dtype=float)
PLUS_Y = np.array([0.0, 1.0, 0.0], dtype=float)


def normalize(vector: np.ndarray) -> np.ndarray:
    value = np.asarray(vector, dtype=float)
    norm = float(np.linalg.norm(value))
    if norm < 1e-15:
        raise ValueError("cannot normalize a zero vector")
    return value / norm


def orientation_from_normal_finger(side: str, normal: np.ndarray, finger: np.ndarray) -> np.ndarray:
    """Construct a proper palm rotation from normal and finger directions."""
    n = normalize(normal)
    f = np.asarray(finger, dtype=float)
    f = f - n * float(np.dot(f, n))
    if float(np.linalg.norm(f)) < 1e-9:
        t1, _t2 = ik.tangent_basis(n)
        f = t1
    f = normalize(f)
    if side == "L":
        col1 = -n
        col2 = f
    else:
        col1 = n
        col2 = -f
    col3 = np.cross(col1, col2)
    rotation = np.column_stack([col1, col2, col3])
    return fk_model.orthonormalize(rotation)


def bed_geometry(
    approach: str,
    height_m: float,
    width_m: float,
    clearance_m: float,
    length_m: float = BED_LENGTH_M,
) -> dict[str, Any]:
    """Axis-aligned parametric bed in front of the robot.

    Lateral approach: bed length along Y, bed width (depth) along X.
    Frontal approach: bed length along X, bed width along Y.
    """
    x_near = BASE_HALF_X + float(clearance_m)
    if approach == "lateral":
        x_far = x_near + float(width_m)
        y_min = -float(length_m) / 2.0
        y_max = float(length_m) / 2.0
    elif approach == "frontal":
        x_far = x_near + float(length_m)
        y_min = -float(width_m) / 2.0
        y_max = float(width_m) / 2.0
    else:
        raise ValueError(f"unknown approach: {approach!r}")
    return {
        "approach": approach,
        "height_m": float(height_m),
        "width_m": float(width_m),
        "length_m": float(length_m),
        "clearance_m": float(clearance_m),
        "base_front_x_m": BASE_HALF_X,
        "x_near": x_near,
        "x_far": x_far,
        "y_min": y_min,
        "y_max": y_max,
        "z_top": float(height_m),
        "base_bed_clearance_m": x_near - BASE_HALF_X,
    }


def make_targets(approach: str, bed: dict[str, Any], side: str) -> dict[str, dict[str, Any]]:
    """Bed-relative placeholder targets.  The patient is not modelled."""
    sign = 1.0 if side == "L" else -1.0
    height = bed["z_top"]
    x_near = bed["x_near"]
    x_far = bed["x_far"]
    y_min = bed["y_min"]
    y_max = bed["y_max"]
    width = bed["width_m"]
    length = bed["length_m"]

    if approach == "lateral":
        y_head = y_max - 0.25
        raw = {
            "P4": (x_near + 0.12, sign * 0.25, height + APPROACH_Z_M, DOWN, PLUS_X),
            "P5": (x_far - 0.12, sign * 0.25, height + APPROACH_Z_M, DOWN, PLUS_X),
            "P6": (x_near + 0.45 * width, sign * 0.35, height + APPROACH_Z_M, DOWN, PLUS_X),
            "P7": (x_near + 0.55 * width, sign * 0.12, height + DIAPER_Z_M, DOWN, PLUS_Y),
            "P9": (x_near + 0.30 * width, y_head + sign * 0.05, height + FEEDING_Z_M, DOWN, PLUS_Y),
        }
    else:
        raw = {
            "P4": (x_near + 0.12, sign * 0.20, height + APPROACH_Z_M, DOWN, PLUS_X),
            "P5": (x_far - 0.12, sign * 0.20, height + APPROACH_Z_M, DOWN, PLUS_X),
            "P6": (x_near + 0.50, sign * 0.20, height + APPROACH_Z_M, DOWN, PLUS_X),
            "P7": (x_near + 0.80, sign * 0.10, height + DIAPER_Z_M, DOWN, PLUS_X),
            "P9": (x_far - 0.30, sign * 0.10, height + FEEDING_Z_M, DOWN, PLUS_X),
        }

    targets: dict[str, dict[str, Any]] = {}
    for pose, (x, y, z, normal, finger) in raw.items():
        rotation = orientation_from_normal_finger(side, normal, finger)
        targets[pose] = {
            "pose": pose,
            "side": side,
            "approach": approach,
            "position_floor": [float(x), float(y), float(z)],
            "normal": [float(value) for value in normalize(normal)],
            "finger": [float(value) for value in normalize(finger)],
            "rotation": rotation.tolist(),
            "bed": {
                "height_m": bed["height_m"],
                "width_m": bed["width_m"],
                "clearance_m": bed["clearance_m"],
                "x_near": bed["x_near"],
                "x_far": bed["x_far"],
                "y_min": bed["y_min"],
                "y_max": bed["y_max"],
                "length_m": length,
            },
        }
    return targets


# ---------------------------------------------------------------------------
# Temporary simplified collision geometry


def box_distance_point(point: np.ndarray, box: dict[str, float]) -> float:
    p = np.asarray(point, dtype=float)
    dx = max(box["xmin"] - p[0], 0.0, p[0] - box["xmax"])
    dy = max(box["ymin"] - p[1], 0.0, p[1] - box["ymax"])
    dz = max(box["zmin"] - p[2], 0.0, p[2] - box["zmax"])
    if dx == 0.0 and dy == 0.0 and dz == 0.0:
        return -min(
            p[0] - box["xmin"],
            box["xmax"] - p[0],
            p[1] - box["ymin"],
            box["ymax"] - p[1],
            p[2] - box["zmin"],
            box["zmax"] - p[2],
        )
    return float(math.sqrt(dx * dx + dy * dy + dz * dz))


def oriented_box_distance_point(
    point: np.ndarray,
    center: np.ndarray,
    rotation: np.ndarray,
    half: np.ndarray,
) -> float:
    local = np.asarray(rotation, dtype=float).T @ (np.asarray(point, dtype=float) - center)
    h = np.asarray(half, dtype=float)
    delta = np.maximum(np.abs(local) - h, 0.0)
    if float(np.linalg.norm(delta)) == 0.0:
        return -float(np.min(h - np.abs(local)))
    return float(np.linalg.norm(delta))


def segment_box_clearance(
    p0: np.ndarray,
    p1: np.ndarray,
    box: dict[str, float],
    radius: float,
    samples: int = 21,
    t_min: float = 0.0,
) -> float:
    best = float("inf")
    for index in range(samples):
        t = t_min + (1.0 - t_min) * index / max(samples - 1, 1)
        point = p0 + t * (p1 - p0)
        best = min(best, box_distance_point(point, box))
    return best - float(radius)


def segment_oriented_box_clearance(
    p0: np.ndarray,
    p1: np.ndarray,
    center: np.ndarray,
    rotation: np.ndarray,
    half: np.ndarray,
    radius: float,
    samples: int = 21,
    t_min: float = 0.0,
) -> float:
    best = float("inf")
    for index in range(samples):
        t = t_min + (1.0 - t_min) * index / max(samples - 1, 1)
        point = p0 + t * (p1 - p0)
        best = min(best, oriented_box_distance_point(point, center, rotation, half))
    return best - float(radius)


def segment_segment_distance(p1: np.ndarray, q1: np.ndarray, p2: np.ndarray, q2: np.ndarray) -> float:
    """Shortest distance between two 3D segments."""
    u = q1 - p1
    v = q2 - p2
    w = p1 - p2
    a = float(np.dot(u, u))
    b = float(np.dot(u, v))
    c = float(np.dot(v, v))
    d = float(np.dot(u, w))
    e = float(np.dot(v, w))
    epsilon = 1e-12

    if a <= epsilon and c <= epsilon:
        return float(np.linalg.norm(p1 - p2))
    if a <= epsilon:
        s = 0.0
        t = float(np.clip(e / c, 0.0, 1.0))
    elif c <= epsilon:
        t = 0.0
        s = float(np.clip(-d / a, 0.0, 1.0))
    else:
        denominator = a * c - b * b
        if abs(denominator) > epsilon:
            s = float(np.clip((b * e - c * d) / denominator, 0.0, 1.0))
        else:
            s = 0.0
        t = (b * s + e) / c
        if t < 0.0:
            t = 0.0
            s = float(np.clip(-d / a, 0.0, 1.0))
        elif t > 1.0:
            t = 1.0
            s = float(np.clip((b - d) / a, 0.0, 1.0))
    closest_1 = p1 + s * u
    closest_2 = p2 + t * v
    return float(np.linalg.norm(closest_1 - closest_2))


def arm_segments(side: str, q_deg: dict[str, float], lift_m: float) -> list[dict[str, Any]]:
    mats = fk_model.fk(side, q_deg, lift_m)
    prefix = "left" if side == "L" else "right"
    names = {
        "shoulder3": f"{prefix}_shoulder_joint_empty_3",
        "shoulder1": f"{prefix}_shoulder_joint_empty_1",
        "elbow": f"{prefix}_elbow_joint_empty",
        "wrist1": f"{prefix}_wrist_joint_1_empty",
        "palm": f"{prefix}_palm_link",
    }
    points = {key: mats[name][:3, 3].copy() for key, name in names.items()}
    return [
        {"label": "shoulder", "p0": points["shoulder3"], "p1": points["shoulder1"], "radius": 0.060},
        {"label": "upper_arm", "p0": points["shoulder1"], "p1": points["elbow"], "radius": 0.047},
        {"label": "forearm", "p0": points["elbow"], "p1": points["wrist1"], "radius": 0.047},
        {"label": "hand", "p0": points["wrist1"], "p1": points["palm"], "radius": 0.030},
    ]


def base_box() -> dict[str, float]:
    return {
        "xmin": -BASE_HALF_X,
        "xmax": BASE_HALF_X,
        "ymin": -BASE_HALF_Y,
        "ymax": BASE_HALF_Y,
        "zmin": BASE_Z_MIN,
        "zmax": BASE_Z_MAX,
    }


def bed_box(bed: dict[str, Any]) -> dict[str, float]:
    return {
        "xmin": bed["x_near"],
        "xmax": bed["x_far"],
        "ymin": bed["y_min"],
        "ymax": bed["y_max"],
        "zmin": 0.0,
        "zmax": bed["z_top"],
    }


def torso_box(q_deg: dict[str, float], lift_m: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    mats = fk_model.fk("L", q_deg, lift_m)
    matrix = mats["torso_link"]
    center = matrix[:3, 3].copy()
    rotation = fk_model.orthonormalize(matrix)
    half = np.array([0.16, 0.13, 0.05], dtype=float)
    return center, rotation, half


def collision_clearances(
    side: str,
    q_deg: dict[str, float],
    lift_m: float,
    bed: dict[str, Any],
    other_q: dict[str, float] | None = None,
) -> dict[str, Any]:
    """Simplified capsule/box clearances.  Negative values mean overlap."""
    segments = arm_segments(side, q_deg, lift_m)
    torso_center, torso_rotation, torso_half = torso_box(q_deg, lift_m)
    base = base_box()
    bed_volume = bed_box(bed)

    torso_values = []
    for segment in segments:
        if segment["label"] == "shoulder":
            continue
        t_min = 0.20 if segment["label"] == "upper_arm" else 0.0
        torso_values.append(
            segment_oriented_box_clearance(
                segment["p0"], segment["p1"], torso_center, torso_rotation, torso_half,
                segment["radius"], t_min=t_min,
            )
        )

    base_values = [
        segment_box_clearance(segment["p0"], segment["p1"], base, segment["radius"])
        for segment in segments
    ]
    bed_values = [
        segment_box_clearance(segment["p0"], segment["p1"], bed_volume, segment["radius"])
        for segment in segments
        if segment["label"] in ("upper_arm", "forearm", "hand")
    ]

    arm_arm = None
    if other_q is not None:
        other_segments = arm_segments("R" if side == "L" else "L", other_q, lift_m)
        arm_arm_values = []
        for first in segments:
            for second in other_segments:
                distance = segment_segment_distance(first["p0"], first["p1"], second["p0"], second["p1"])
                arm_arm_values.append(distance - first["radius"] - second["radius"])
        arm_arm = min(arm_arm_values)

    values = {
        "arm_torso": min(torso_values) if torso_values else None,
        "arm_base": min(base_values) if base_values else None,
        "arm_bed": min(bed_values) if bed_values else None,
        "arm_arm": arm_arm,
    }
    numeric = [value for value in values.values() if value is not None]
    values["minimum"] = min(numeric) if numeric else None
    values["collision_free"] = bool(all(value >= 0.0 for value in numeric)) if numeric else True
    return values


# ---------------------------------------------------------------------------
# Bed-target IK


def seed_poses(side: str) -> dict[str, dict[str, float]]:
    sign = -1.0 if side == "L" else 1.0
    return {
        "P0": {},
        "forward_bent": {"sh2": 75.0 * sign, "el": 45.0},
        "folded": {"sh2": 75.0 * sign, "el": 150.0},
    }


def solve_bed_target(
    side: str,
    target: dict[str, Any],
    mode: str,
    lift_m: float,
    seed: int,
    random_count: int = 6,
) -> dict[str, Any]:
    rng = np.random.default_rng(seed)
    seeds = seed_poses(side)
    starts: list[tuple[str, np.ndarray]] = []
    for label, q_deg in seeds.items():
        starts.append((label, ik.q_array(side, q_deg)))
    starts.append(("zero", np.zeros(len(ik.JOINT_KEYS), dtype=float)))

    lower = ik.LOWER[side]
    upper = ik.UPPER[side]
    span = upper - lower
    for index in range(random_count):
        values = lower + 0.05 * span + rng.random(len(ik.JOINT_KEYS)) * 0.90 * span
        values[3] = rng.uniform(20.0, 120.0)
        starts.append((f"random{index}", values))

    for label, q_deg in seeds.items():
        values = ik.q_array(side, q_deg)
        for delta in (-120.0, -60.0, 60.0, 120.0):
            candidate = values.copy()
            candidate[5] = float(np.clip(candidate[5] + delta, lower[5], upper[5]))
            starts.append((f"{label}_w1{delta:+.0f}", candidate))

    raw: list[dict[str, Any]] = []
    for label, start_values in starts:
        result = ik.solve_once(side, target, mode, lift_m, start_values, max_iter=220)
        result["start"] = label
        raw.append(result)

    valid = [solution for solution in raw if solution["converged"]]
    anchor = min(valid or raw, key=ik.solution_sort_key)
    anchor_values = ik.q_array(side, anchor["q_deg"])
    anchor_metrics = ik.task_metrics(side, ik.q_dict(anchor_values), lift_m, target, mode)
    directions = ik.nullspace_directions(anchor_metrics["analysis_scaled_jacobian"])
    for column in range(directions.shape[1]):
        for delta_deg in (-6.0, -3.0, 3.0, 6.0):
            start_values = ik.clip_q(side, anchor_values + delta_deg * directions[:, column])
            result = ik.solve_once(side, target, mode, lift_m, start_values, max_iter=220)
            result["start"] = f"null{column}{delta_deg:+.0f}"
            raw.append(result)

    valid = ik.dedupe_solutions([solution for solution in raw if solution["converged"]])
    valid.sort(key=ik.solution_sort_key)
    if valid:
        best = valid[0]
    else:
        best = min(raw, key=lambda solution: (solution["task_error_norm"], solution["position_error_mm"]))

    # Prefer a collision-free valid solution if one exists.
    chosen = best
    if valid:
        for solution in valid[:8]:
            values = solution["q_deg"]
            clearances = collision_clearances(side, values, lift_m, target["bed"], other_q=seeds["P0"])
            if clearances["collision_free"]:
                chosen = solution
                break

    clearances = collision_clearances(side, chosen["q_deg"], lift_m, target["bed"], other_q=seeds["P0"])
    return {
        "side": side,
        "mode": mode,
        "target": target,
        "best": chosen,
        "solutions": valid,
        "solution_count": len(valid),
        "raw_solution_count": len(raw),
        "reachable": bool(chosen["converged"]),
        "collision_free": bool(clearances["collision_free"]),
        "clearances": clearances,
        "chosen_has_collision_free_alternative": bool(
            chosen is not best or clearances["collision_free"]
        ),
    }


# ---------------------------------------------------------------------------
# Sample configuration


def sample_configurations() -> list[dict[str, Any]]:
    configs: list[dict[str, Any]] = []
    seen: set[tuple[Any, ...]] = set()

    def add(approach: str, height: float, width: float, clearance: float, poses: tuple[str, ...], modes: tuple[str, ...]) -> None:
        key = (approach, height, width, clearance, poses, modes)
        if key in seen:
            return
        seen.add(key)
        configs.append(
            {
                "approach": approach,
                "height_m": height,
                "width_m": width,
                "clearance_m": clearance,
                "poses": poses,
                "modes": modes,
            }
        )

    add("lateral", BASELINE_HEIGHT_M, BASELINE_WIDTH_M, BASELINE_CLEARANCE_M,
        ("P4", "P5", "P6", "P7", "P9"), ("6d", "5d"))
    add("frontal", BASELINE_HEIGHT_M, BASELINE_WIDTH_M, BASELINE_CLEARANCE_M,
        ("P4", "P5", "P6", "P7", "P9"), ("6d", "5d"))

    for height in BED_HEIGHTS_M:
        for width in BED_WIDTHS_M:
            add("lateral", height, width, BASELINE_CLEARANCE_M,
                ("P4", "P5"), ("5d",))
    for clearance in CLEARANCES_M:
        add("lateral", BASELINE_HEIGHT_M, BASELINE_WIDTH_M, clearance,
            ("P4", "P5"), ("5d",))
    return configs


def run_sample() -> dict[str, Any]:
    rows: list[dict[str, Any]] = []
    configs = sample_configurations()
    target_records: list[dict[str, Any]] = []
    seed_counter = 20261021

    for config_index, config in enumerate(configs):
        bed = bed_geometry(
            config["approach"], config["height_m"], config["width_m"], config["clearance_m"]
        )
        for side_index, side in enumerate(("L", "R")):
            targets = make_targets(config["approach"], bed, side)
            for pose_index, pose in enumerate(config["poses"]):
                target = targets[pose]
                target_records.append(target)
                for mode_index, mode in enumerate(config["modes"]):
                    seed = seed_counter + config_index * 1000 + side_index * 100 + pose_index * 10 + mode_index
                    result = solve_bed_target(side, target, mode, 0.0, seed)
                    best = result["best"]
                    clearances = result["clearances"]
                    rows.append(
                        {
                            "approach": config["approach"],
                            "height_m": config["height_m"],
                            "width_m": config["width_m"],
                            "clearance_m": config["clearance_m"],
                            "pose": pose,
                            "side": side,
                            "mode": mode,
                            "reachable": result["reachable"],
                            "collision_free": result["collision_free"],
                            "position_error_mm": best["position_error_mm"],
                            "orientation_error_deg": best["orientation_error_deg"],
                            "normal_error_deg": best["normal_error_deg"],
                            "min_margin_deg": best["min_margin_deg"],
                            "condition": best["condition"],
                            "rank": best["rank"],
                            "solution_count": result["solution_count"],
                            "raw_solution_count": result["raw_solution_count"],
                            "clearance_arm_torso_m": clearances["arm_torso"],
                            "clearance_arm_base_m": clearances["arm_base"],
                            "clearance_arm_bed_m": clearances["arm_bed"],
                            "clearance_arm_arm_m": clearances["arm_arm"],
                            "clearance_min_m": clearances["minimum"],
                            "q_deg": best["q_deg"],
                            "target_position_floor": target["position_floor"],
                            "target_normal": target["normal"],
                        }
                    )

    summary: dict[str, Any] = {
        "rows": len(rows),
        "configurations": len(configs),
        "reachable": sum(1 for row in rows if row["reachable"]),
        "collision_free": sum(1 for row in rows if row["collision_free"]),
        "reachable_and_collision_free": sum(
            1 for row in rows if row["reachable"] and row["collision_free"]
        ),
        "by_approach": {},
        "by_pose": {},
        "by_mode": {},
        "by_side": {},
        "unreachable": [],
        "reachable_but_collision": [],
    }
    for row in rows:
        for key, bucket in (
            ("approach", summary["by_approach"]),
            ("pose", summary["by_pose"]),
            ("mode", summary["by_mode"]),
            ("side", summary["by_side"]),
        ):
            value = row[key]
            bucket.setdefault(value, {"rows": 0, "reachable": 0, "collision_free": 0})
            bucket[value]["rows"] += 1
            bucket[value]["reachable"] += int(row["reachable"])
            bucket[value]["collision_free"] += int(row["collision_free"])
        if not row["reachable"]:
            summary["unreachable"].append(
                {
                    "approach": row["approach"],
                    "height_m": row["height_m"],
                    "width_m": row["width_m"],
                    "clearance_m": row["clearance_m"],
                    "pose": row["pose"],
                    "side": row["side"],
                    "mode": row["mode"],
                    "position_error_mm": row["position_error_mm"],
                    "orientation_error_deg": row["orientation_error_deg"],
                }
            )
        if row["reachable"] and not row["collision_free"]:
            summary["reachable_but_collision"].append(
                {
                    "approach": row["approach"],
                    "height_m": row["height_m"],
                    "width_m": row["width_m"],
                    "clearance_m": row["clearance_m"],
                    "pose": row["pose"],
                    "side": row["side"],
                    "mode": row["mode"],
                    "clearance_min_m": row["clearance_min_m"],
                    "clearance_arm_torso_m": row["clearance_arm_torso_m"],
                    "clearance_arm_base_m": row["clearance_arm_base_m"],
                    "clearance_arm_bed_m": row["clearance_arm_bed_m"],
                    "clearance_arm_arm_m": row["clearance_arm_arm_m"],
                }
            )

    payload = {
        "meta": {
            "purpose": "first small-sample reachability and simplified-collision check over a parametric bed",
            "not_a_safety_classification": True,
            "patient_model": "absent; bed-relative placeholder targets",
            "bed": {
                "length_m": BED_LENGTH_M,
                "heights_m": list(BED_HEIGHTS_M),
                "widths_m": list(BED_WIDTHS_M),
                "clearances_m": list(CLEARANCES_M),
                "length_assumption": "2.0 m is an assumption, not a measured bed",
            },
            "robot_base": {
                "half_x_m": BASE_HALF_X,
                "half_y_m": BASE_HALF_Y,
                "z_min_m": BASE_Z_MIN,
                "z_max_m": BASE_Z_MAX,
                "clearance_definition": "gap between base front face and bed near side",
            },
            "target_heights": {
                "approach_above_surface_m": APPROACH_Z_M,
                "diaper_above_surface_m": DIAPER_Z_M,
                "feeding_above_surface_m": FEEDING_Z_M,
            },
            "collision_model": {
                "type": "temporary capsules for arm links, oriented box for torso, AABB for base and bed",
                "radii_m": {"shoulder": 0.060, "upper_arm": 0.047, "forearm": 0.047, "hand": 0.030},
                "torso_half_dims_m": [0.16, 0.13, 0.05],
                "sampling": "21 points per capsule segment for box distances",
                "limitations": [
                    "not the original collision meshes",
                    "no patient, bedding, rails, frame or soft contacts",
                    "arm-arm check uses the other arm at P0, not a joint solution",
                    "negative clearance means overlap in the simplified model",
                ],
            },
            "ik": {
                "source": "stage2A_ik/ik_study.py",
                "modes": ["6d", "5d"],
                "random_starts_per_target": 6,
                "seed_poses": ["P0", "forward_bent", "folded", "zero"],
                "nullspace_sampling": "+/-3 and +/-6 degrees along each analysis null-space direction",
            },
        },
        "summary": summary,
        "rows": rows,
    }

    (HERE / "bed_model.json").write_text(
        json.dumps(
            {
                "meta": payload["meta"],
                "configurations": configs,
                "targets": target_records,
            },
            indent=2,
            ensure_ascii=False,
        ),
        encoding="utf-8",
    )
    (HERE / "small_sample.json").write_text(
        json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8"
    )

    csv_path = HERE / "small_sample.csv"
    fieldnames = [
        "approach", "height_m", "width_m", "clearance_m", "pose", "side", "mode",
        "reachable", "collision_free", "position_error_mm", "orientation_error_deg",
        "normal_error_deg", "min_margin_deg", "condition", "rank", "solution_count",
        "raw_solution_count", "clearance_arm_torso_m", "clearance_arm_base_m",
        "clearance_arm_bed_m", "clearance_arm_arm_m", "clearance_min_m",
        *[f"q_{key}" for key in ik.JOINT_KEYS],
    ]
    with csv_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            flat = {key: row.get(key) for key in fieldnames if key in row}
            for key in ik.JOINT_KEYS:
                flat[f"q_{key}"] = row["q_deg"].get(key)
            writer.writerow(flat)

    lines = [
        "# Stage 2A: first bed reachability sample",
        "",
        "This is kinematic reachability and a simplified collision check, not a safety classification.",
        "Patient is absent; targets are bed-relative placeholders. Bed length 2.0 m is an assumption.",
        "Collision geometry is temporary capsules/boxes, not the original collision meshes.",
        "",
        "## Summary",
        "",
        f"- rows: {summary['rows']}",
        f"- reachable: {summary['reachable']}",
        f"- collision-free in simplified model: {summary['collision_free']}",
        f"- reachable and collision-free: {summary['reachable_and_collision_free']}",
        "",
        "### By approach",
        "",
        "| approach | rows | reachable | collision-free |",
        "|---|---:|---:|---:|",
    ]
    for key, item in sorted(summary["by_approach"].items()):
        lines.append(f"| {key} | {item['rows']} | {item['reachable']} | {item['collision_free']} |")
    lines.extend(
        [
            "",
            "### By pose",
            "",
            "| pose | rows | reachable | collision-free |",
            "|---|---:|---:|---:|",
        ]
    )
    for key, item in sorted(summary["by_pose"].items()):
        lines.append(f"| {key} | {item['rows']} | {item['reachable']} | {item['collision_free']} |")
    lines.extend(
        [
            "",
            "### By mode",
            "",
            "| mode | rows | reachable | collision-free |",
            "|---|---:|---:|---:|",
        ]
    )
    for key, item in sorted(summary["by_mode"].items()):
        lines.append(f"| {key} | {item['rows']} | {item['reachable']} | {item['collision_free']} |")
    lines.extend(
        [
            "",
            "### By side",
            "",
            "| side | rows | reachable | collision-free |",
            "|---|---:|---:|---:|",
        ]
    )
    for key, item in sorted(summary["by_side"].items()):
        lines.append(f"| {key} | {item['rows']} | {item['reachable']} | {item['collision_free']} |")
    lines.extend(
        [
            "",
            "## First rows",
            "",
            "| approach | H | W | clearance | pose | side | mode | reachable | collision-free | pos err mm | ori err deg | min margin | cond | min clearance |",
            "|---|---:|---:|---:|---|---|---|---:|---:|---:|---:|---:|---:|---:|",
        ]
    )
    for row in rows[:40]:
        condition = "inf" if row["condition"] is None else f"{row['condition']:.2e}"
        clearance = "n/a" if row["clearance_min_m"] is None else f"{row['clearance_min_m']:.3f}"
        lines.append(
            f"| {row['approach']} | {row['height_m']:.1f} | {row['width_m']:.1f} | "
            f"{row['clearance_m']:.2f} | {row['pose']} | {row['side']} | {row['mode']} | "
            f"{'yes' if row['reachable'] else 'no'} | {'yes' if row['collision_free'] else 'no'} | "
            f"{row['position_error_mm']:.3f} | {row['orientation_error_deg']:.3f} | "
            f"{row['min_margin_deg']:.1f} | {condition} | {clearance} |"
        )
    lines.extend(
        [
            "",
            "## Unreachable rows",
            "",
            f"Count: {len(summary['unreachable'])}",
            "",
            "## Reachable but simplified-collision rows",
            "",
            f"Count: {len(summary['reachable_but_collision'])}",
        ]
    )
    (HERE / "small_sample.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    return payload


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", action="store_true", help="run the small sample")
    args = parser.parse_args()
    if not args.run:
        print("use --run to execute the small sample")
        return
    payload = run_sample()
    summary = payload["summary"]
    print(
        "SAMPLE",
        f"rows={summary['rows']}",
        f"reachable={summary['reachable']}",
        f"collision_free={summary['collision_free']}",
        f"reachable_and_collision_free={summary['reachable_and_collision_free']}",
        f"unreachable={len(summary['unreachable'])}",
        f"reachable_but_collision={len(summary['reachable_but_collision'])}",
    )


if __name__ == "__main__":
    main()
