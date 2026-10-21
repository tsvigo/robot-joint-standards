"""Stage 2A continuation: numerical IK for the two 7-DOF arms.

The module implements two task modes:

* ``6d``: contact-face position + full palm orientation (6 constraints);
* ``5d``: contact-face position + palm normal direction (5 constraints),
  leaving rotation about the normal free.

The solver uses damped least squares in a scaled task space, joint-limit
clamping, a null-space term that pushes joints away from limits, multiple
initial guesses, and null-space sampling to collect several solutions.

It does not modify the .blend, does not use forces, and does not classify
poses as safe.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any

import numpy as np

sys.dont_write_bytecode = True

HERE = Path(__file__).resolve().parent
STAGE2A = HERE.parent / "stage2A"
if str(STAGE2A) not in sys.path:
    sys.path.insert(0, str(STAGE2A))

import fk_model  # noqa: E402

FLOOR_ORIGIN = fk_model.FLOOR_ORIGIN
PALM_GEOMETRY = fk_model.load_palm_geometry()
JOINT_KEYS = fk_model.ARM_JOINT_KEYS
L_NORM_M = 0.5  # characteristic length for dimensionless 6D/5D Jacobians
FD_STEP_RAD = 1e-6
FD_ABS_TOL = 1e-4
FD_REL_TOL = 1e-4
IK_ACCEPT_TOL = 1e-5  # scaled task error, dimensionless

LOWER = {
    side: np.array([joint[3] for joint in fk_model.JOINTS[side]], dtype=float)
    for side in ("L", "R")
}
UPPER = {
    side: np.array([joint[4] for joint in fk_model.JOINTS[side]], dtype=float)
    for side in ("L", "R")
}


# ---------------------------------------------------------------------------
# Small linear-algebra helpers


def normalize(vector: np.ndarray) -> np.ndarray:
    value = np.asarray(vector, dtype=float)
    norm = float(np.linalg.norm(value))
    if norm < 1e-15:
        raise ValueError("cannot normalize a zero vector")
    return value / norm


def skew(vector: np.ndarray) -> np.ndarray:
    x, y, z = np.asarray(vector, dtype=float)
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]], dtype=float)


def vee(matrix: np.ndarray) -> np.ndarray:
    return np.array([matrix[2, 1], matrix[0, 2], matrix[1, 0]], dtype=float)


def rotation_vector(rotation: np.ndarray) -> np.ndarray:
    """World-frame rotation vector of a proper rotation matrix."""
    matrix = fk_model.orthonormalize(rotation)
    cosine = float(np.clip((np.trace(matrix) - 1.0) / 2.0, -1.0, 1.0))
    angle = math.acos(cosine)
    if angle < 1e-8:
        return 0.5 * vee(matrix - matrix.T)
    if angle > math.pi - 1e-6:
        values, vectors = np.linalg.eig(matrix)
        index = int(np.argmin(np.abs(values - 1.0)))
        axis = np.real(vectors[:, index])
        axis = normalize(axis)
        candidate = vee(matrix - matrix.T)
        if float(np.dot(axis, candidate)) < 0.0:
            axis = -axis
        return axis * angle
    axis = vee(matrix - matrix.T) / (2.0 * math.sin(angle))
    return axis * angle


def tangent_basis(normal: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    n = normalize(normal)
    seed = np.array([1.0, 0.0, 0.0]) if abs(float(n[0])) < 0.9 else np.array([0.0, 1.0, 0.0])
    t1 = normalize(np.cross(n, seed))
    t2 = np.cross(n, t1)
    return t1, t2


def q_array(side: str, q_deg: dict[str, float]) -> np.ndarray:
    return np.array([float(q_deg.get(key, 0.0)) for key in JOINT_KEYS], dtype=float)


def q_dict(values: np.ndarray) -> dict[str, float]:
    return {key: float(value) for key, value in zip(JOINT_KEYS, values)}


def clip_q(side: str, values: np.ndarray) -> np.ndarray:
    return np.minimum(np.maximum(np.asarray(values, dtype=float), LOWER[side]), UPPER[side])


def limit_gradient(side: str, values: np.ndarray) -> np.ndarray:
    """Dimensionless push toward joint centres, stronger near limits."""
    lower = LOWER[side]
    upper = UPPER[side]
    centre = 0.5 * (lower + upper)
    span = upper - lower
    margin = np.minimum(values - lower, upper - values)
    direction = np.where(values < centre, 1.0, -1.0)
    gradient = 0.02 * (centre - values) / span
    near = margin < 10.0
    gradient[near] += direction[near] * (10.0 - margin[near]) / 10.0
    return gradient


# ---------------------------------------------------------------------------
# Task error and Jacobian


def target_from_q(side: str, q_deg: dict[str, float], lift_m: float = 0.0) -> dict[str, Any]:
    """Model-generated target: exact for the rigidified FK."""
    geometry = fk_model.palm_geometry(side, q_deg, lift_m, geometry=PALM_GEOMETRY)
    return {
        "position_floor": [float(value) for value in geometry["face_floor"]],
        "rotation": fk_model.orthonormalize(geometry["matrix"]).tolist(),
        "normal": [float(value) for value in geometry["normal"]],
        "source": "model",
    }


def target_from_gt(gt: dict[str, Any], pose: str, side: str) -> dict[str, Any]:
    """Blender GT target for a control pose, expressed relative to the floor."""
    matrix = np.asarray(gt["poses"][pose][side], dtype=float)
    item = PALM_GEOMETRY[side]
    bbox = np.asarray(item["local_bbox"], dtype=float)
    x_face = bbox[0, 0] if side == "L" else bbox[1, 0]
    y_mid = 0.5 * (bbox[0, 1] + bbox[1, 1])
    z_mid = 0.5 * (bbox[0, 2] + bbox[1, 2])
    face_local = np.array([x_face, y_mid, z_mid], dtype=float)
    face_world = (matrix @ np.r_[face_local, 1.0])[:3]
    rotation = fk_model.orthonormalize(matrix)
    normal = rotation @ np.asarray(item["normal_local"], dtype=float)
    return {
        "position_floor": [float(value) for value in (face_world - FLOOR_ORIGIN)],
        "rotation": rotation.tolist(),
        "normal": [float(value) for value in normalize(normal)],
        "source": "gt",
    }


def task_metrics(
    side: str,
    q_deg: dict[str, float],
    lift_m: float,
    target: dict[str, Any],
    mode: str,
) -> dict[str, Any]:
    """Task error, scaled Jacobian, conditioning and limit metrics."""
    geometry = fk_model.palm_geometry(side, q_deg, lift_m, geometry=PALM_GEOMETRY)
    jacobian, _axes, _point = fk_model.jacobian(
        side, q_deg, lift_m, point_local=geometry["face_local"]
    )

    position_error = np.asarray(target["position_floor"], dtype=float) - geometry["face_floor"]
    normal_target = normalize(np.asarray(target["normal"], dtype=float))
    normal_current = geometry["normal"]
    normal_angle_deg = math.degrees(
        math.acos(float(np.clip(np.dot(normal_target, normal_current), -1.0, 1.0)))
    )

    if mode == "6d":
        rotation_target = fk_model.orthonormalize(np.asarray(target["rotation"], dtype=float))
        rotation_current = fk_model.orthonormalize(geometry["matrix"])
        orientation_error = rotation_vector(rotation_target @ rotation_current.T)
        task_jacobian = np.vstack([jacobian[:3, :], jacobian[3:, :]])
        error = np.r_[position_error, orientation_error]
        scale = np.array([1.0 / L_NORM_M] * 3 + [1.0] * 3, dtype=float)
        orientation_error_deg = math.degrees(float(np.linalg.norm(orientation_error)))
    elif mode == "5d":
        t1, t2 = tangent_basis(normal_target)
        normal_error = normal_target - normal_current
        tangent_error = np.array([float(t1 @ normal_error), float(t2 @ normal_error)])
        normal_jacobian = np.vstack([t1, t2]) @ (-skew(normal_current)) @ jacobian[3:, :]
        task_jacobian = np.vstack([jacobian[:3, :], normal_jacobian])
        error = np.r_[position_error, tangent_error]
        scale = np.array([1.0 / L_NORM_M] * 3 + [1.0, 1.0], dtype=float)
        orientation_error_deg = normal_angle_deg
    else:
        raise ValueError(f"unknown IK mode: {mode!r}")

    scaled_jacobian = task_jacobian * scale[:, None]
    scaled_error = error * scale
    singular = np.linalg.svd(scaled_jacobian, compute_uv=False)
    sigma_max = float(singular[0]) if len(singular) else 0.0
    sigma_min = float(singular[-1]) if len(singular) else 0.0
    rank = int(np.sum(singular > 1e-9 * sigma_max)) if sigma_max > 0.0 else 0
    condition = float(sigma_max / sigma_min) if sigma_min > 1e-15 else None

    margins = fk_model.limit_margins(side, q_deg)
    min_margin = min(margins, key=lambda item: item["margin_deg"])

    return {
        "geometry": geometry,
        "position_error": position_error,
        "orientation_error_deg": orientation_error_deg,
        "normal_error_deg": normal_angle_deg,
        "task_error": error,
        "task_error_norm": float(np.linalg.norm(scaled_error)),
        "task_jacobian": task_jacobian,
        "scaled_jacobian": scaled_jacobian,
        "singular_values_scaled": [float(value) for value in singular],
        "sigma_min_scaled": sigma_min,
        "sigma_max_scaled": sigma_max,
        "rank": rank,
        "condition": condition,
        "min_margin": min_margin,
        "margins": margins,
        "within_limits": fk_model.within_limits(side, q_deg),
    }


# ---------------------------------------------------------------------------
# Solver


def solve_once(
    side: str,
    target: dict[str, Any],
    mode: str,
    lift_m: float,
    q0_deg: np.ndarray,
    max_iter: int = 300,
    tol: float = 1e-7,
) -> dict[str, Any]:
    """Damped least-squares IK from one initial guess."""
    q = clip_q(side, q0_deg)
    damping = 1e-3
    no_improve = 0
    best_q = q.copy()
    best_metrics = task_metrics(side, q_dict(q), lift_m, target, mode)
    best_norm = best_metrics["task_error_norm"]

    for _iteration in range(max_iter):
        metrics = task_metrics(side, q_dict(q), lift_m, target, mode)
        error_norm = metrics["task_error_norm"]
        if error_norm < best_norm:
            best_norm = error_norm
            best_q = q.copy()
            best_metrics = metrics
        if error_norm <= tol:
            break

        jacobian = metrics["scaled_jacobian"]
        error = metrics["task_error"] * np.array(
            [1.0 / L_NORM_M] * 3 + [1.0] * (jacobian.shape[0] - 3), dtype=float
        )
        normal_matrix = jacobian @ jacobian.T + (damping**2) * np.eye(jacobian.shape[0])
        try:
            jacobian_pinv = np.linalg.solve(normal_matrix, jacobian).T
        except np.linalg.LinAlgError:
            jacobian_pinv = np.linalg.pinv(jacobian, rcond=1e-12)
        task_step = jacobian_pinv @ error

        nullspace = np.eye(len(JOINT_KEYS)) - jacobian_pinv @ jacobian
        null_step = 0.03 * (nullspace @ limit_gradient(side, q))
        step = task_step + null_step
        step_norm = float(np.linalg.norm(step))
        max_step = math.radians(12.0)
        if step_norm > max_step:
            step *= max_step / step_norm

        candidate = clip_q(side, q + np.degrees(step))
        candidate_metrics = task_metrics(side, q_dict(candidate), lift_m, target, mode)
        candidate_norm = candidate_metrics["task_error_norm"]

        if candidate_norm < error_norm:
            q = candidate
            damping = max(damping * 0.7, 1e-6)
            no_improve = 0
            continue

        damping = min(damping * 2.0, 1e2)
        no_improve += 1
        half = clip_q(side, q + np.degrees(0.5 * step))
        half_metrics = task_metrics(side, q_dict(half), lift_m, target, mode)
        if half_metrics["task_error_norm"] < error_norm:
            q = half
            no_improve = 0
            continue
        if no_improve > 12:
            break

    final_metrics = task_metrics(side, q_dict(best_q), lift_m, target, mode)
    return result_from_metrics(side, best_q, lift_m, target, mode, final_metrics, "unknown")


def result_from_metrics(
    side: str,
    q_values: np.ndarray,
    lift_m: float,
    target: dict[str, Any],
    mode: str,
    metrics: dict[str, Any],
    start: str,
) -> dict[str, Any]:
    position_error_mm = float(np.linalg.norm(metrics["position_error"]) * 1000.0)
    return {
        "side": side,
        "mode": mode,
        "start": start,
        "q_deg": q_dict(q_values),
        "position_error_mm": position_error_mm,
        "orientation_error_deg": float(metrics["orientation_error_deg"]),
        "normal_error_deg": float(metrics["normal_error_deg"]),
        "task_error_norm": float(metrics["task_error_norm"]),
        "converged": bool(metrics["task_error_norm"] <= IK_ACCEPT_TOL),
        "within_limits": bool(metrics["within_limits"]),
        "min_margin": metrics["min_margin"],
        "min_margin_deg": float(metrics["min_margin"]["margin_deg"]),
        "condition": metrics["condition"],
        "sigma_min_scaled": float(metrics["sigma_min_scaled"]),
        "rank": int(metrics["rank"]),
        "singular_values_scaled": metrics["singular_values_scaled"],
        "lift_m": float(lift_m),
        "target_source": target.get("source", "unknown"),
    }


def result_from_q(
    side: str,
    q_deg: dict[str, float],
    lift_m: float,
    target: dict[str, Any],
    mode: str,
    start: str,
) -> dict[str, Any]:
    values = clip_q(side, q_array(side, q_deg))
    metrics = task_metrics(side, q_dict(values), lift_m, target, mode)
    return result_from_metrics(side, values, lift_m, target, mode, metrics, start)


def dedupe_solutions(solutions: list[dict[str, Any]], tol_deg: float = 0.2) -> list[dict[str, Any]]:
    kept: list[dict[str, Any]] = []
    for solution in solutions:
        values = q_array("L" if solution["side"] == "L" else "R", solution["q_deg"])
        duplicate = False
        for existing in kept:
            existing_values = q_array("L" if existing["side"] == "L" else "R", existing["q_deg"])
            if float(np.max(np.abs(values - existing_values))) <= tol_deg:
                duplicate = True
                break
        if not duplicate:
            kept.append(solution)
    return kept


def solution_sort_key(solution: dict[str, Any]) -> tuple[float, float, float, float]:
    condition = solution["condition"] if solution["condition"] is not None else 1e300
    return (
        -solution["min_margin_deg"],
        condition,
        solution["position_error_mm"],
        solution["orientation_error_deg"],
    )


def nullspace_directions(scaled_jacobian: np.ndarray, tol: float = 1e-9) -> np.ndarray:
    _u, singular, vt = np.linalg.svd(scaled_jacobian)
    sigma_max = float(singular[0]) if len(singular) else 0.0
    rank = int(np.sum(singular > tol * sigma_max)) if sigma_max > 0.0 else 0
    return vt[rank:].T


def make_starts(
    side: str,
    known_q: dict[str, float],
    rng: np.random.Generator,
    count: int = 10,
) -> list[tuple[str, np.ndarray]]:
    starts: list[tuple[str, np.ndarray]] = [
        ("known", q_array(side, known_q)),
        ("zero", np.zeros(len(JOINT_KEYS), dtype=float)),
    ]
    lower = LOWER[side]
    upper = UPPER[side]
    span = upper - lower
    for index in range(count):
        values = lower + 0.05 * span + rng.random(len(JOINT_KEYS)) * 0.90 * span
        values[3] = rng.uniform(20.0, 120.0)  # elbow bias
        starts.append((f"random{index}", values))
    return starts


def solve_target(
    side: str,
    target: dict[str, Any],
    mode: str,
    lift_m: float,
    known_q: dict[str, float],
    seed: int,
) -> dict[str, Any]:
    rng = np.random.default_rng(seed)
    starts = make_starts(side, known_q, rng)
    raw_solutions: list[dict[str, Any]] = []
    for label, start_values in starts:
        result = solve_once(side, target, mode, lift_m, start_values)
        result["start"] = label
        raw_solutions.append(result)

    known_result = result_from_q(side, known_q, lift_m, target, mode, "known-exact")
    raw_solutions.append(known_result)

    anchor = min(raw_solutions, key=solution_sort_key)
    anchor_values = q_array(side, anchor["q_deg"])
    anchor_metrics = task_metrics(side, q_dict(anchor_values), lift_m, target, mode)
    directions = nullspace_directions(anchor_metrics["scaled_jacobian"])
    for column in range(directions.shape[1]):
        for delta_deg in (-6.0, -3.0, 3.0, 6.0):
            start_values = clip_q(side, anchor_values + delta_deg * directions[:, column])
            result = solve_once(side, target, mode, lift_m, start_values)
            result["start"] = f"null{column}{delta_deg:+.0f}"
            raw_solutions.append(result)

    solutions = dedupe_solutions(raw_solutions)
    solutions.sort(key=solution_sort_key)
    random_converged = sum(
        1 for solution in raw_solutions
        if solution["start"].startswith("random") and solution["converged"]
    )
    best = solutions[0] if solutions else None
    return {
        "side": side,
        "mode": mode,
        "target": target,
        "known_q_deg": {key: float(known_q.get(key, 0.0)) for key in JOINT_KEYS},
        "known_result": known_result,
        "best": best,
        "solutions": solutions,
        "solution_count": len(solutions),
        "raw_solution_count": len(raw_solutions),
        "random_converged": random_converged,
        "random_start_count": len(starts) - 2,
    }


# ---------------------------------------------------------------------------
# Jacobian finite-difference check


def fd_jacobian(
    side: str,
    q_deg: dict[str, float],
    lift_m: float,
    mode: str,
    target: dict[str, Any],
) -> np.ndarray:
    geometry = fk_model.palm_geometry(side, q_deg, lift_m, geometry=PALM_GEOMETRY)
    jacobian, _axes, _point = fk_model.jacobian(
        side, q_deg, lift_m, point_local=geometry["face_local"]
    )
    rows = 6 if mode == "6d" else 5
    finite = np.zeros((rows, len(JOINT_KEYS)), dtype=float)
    normal_target = normalize(np.asarray(target["normal"], dtype=float))
    t1, t2 = tangent_basis(normal_target)
    step_deg = math.degrees(FD_STEP_RAD)

    for column, key in enumerate(JOINT_KEYS):
        plus = dict(q_deg)
        minus = dict(q_deg)
        plus[key] = float(plus.get(key, 0.0)) + step_deg
        minus[key] = float(minus.get(key, 0.0)) - step_deg
        geometry_plus = fk_model.palm_geometry(side, plus, lift_m, geometry=PALM_GEOMETRY)
        geometry_minus = fk_model.palm_geometry(side, minus, lift_m, geometry=PALM_GEOMETRY)
        finite[:3, column] = (
            geometry_plus["face_floor"] - geometry_minus["face_floor"]
        ) / (2.0 * FD_STEP_RAD)
        if mode == "6d":
            rotation_plus = fk_model.orthonormalize(geometry_plus["matrix"])
            rotation_minus = fk_model.orthonormalize(geometry_minus["matrix"])
            finite[3:, column] = rotation_vector(rotation_plus @ rotation_minus.T) / (
                2.0 * FD_STEP_RAD
            )
        else:
            normal_derivative = (
                geometry_plus["normal"] - geometry_minus["normal"]
            ) / (2.0 * FD_STEP_RAD)
            finite[3, column] = float(t1 @ normal_derivative)
            finite[4, column] = float(t2 @ normal_derivative)
    return finite


def jacobian_configs() -> list[tuple[str, str, dict[str, float]]]:
    configs: list[tuple[str, str, dict[str, float]]] = []
    for side in ("L", "R"):
        sign = -1.0 if side == "L" else 1.0
        configs.extend(
            [
                (side, "P0", {}),
                (side, "palmdown0", {"sh3": 45.0 * sign, "rad": 45.0 * sign}),
                (side, "forward", {"sh2": 90.0 * sign}),
                (side, "forward_bent", {"sh2": 75.0 * sign, "el": 45.0}),
                (side, "el90", {"el": 90.0}),
                (side, "folded", {"sh2": 75.0 * sign, "el": 150.0}),
            ]
        )
        rng = np.random.default_rng(20261021 + (0 if side == "L" else 1))
        lower = LOWER[side] + 0.1 * (UPPER[side] - LOWER[side])
        upper = UPPER[side] - 0.1 * (UPPER[side] - LOWER[side])
        random_values = lower + rng.random(len(JOINT_KEYS)) * (upper - lower)
        configs.append((side, "random", q_dict(random_values)))
    return configs


def run_jacobian_check() -> dict[str, Any]:
    rows: list[dict[str, Any]] = []
    for side, name, q_deg in jacobian_configs():
        target = target_from_q(side, q_deg, 0.0)
        for mode in ("6d", "5d"):
            metrics = task_metrics(side, q_deg, 0.0, target, mode)
            finite = fd_jacobian(side, q_deg, 0.0, mode, target)
            analytic = metrics["task_jacobian"]
            difference = analytic - finite
            max_abs = float(np.max(np.abs(difference)))
            relative = float(np.linalg.norm(difference) / max(np.linalg.norm(finite), 1e-15))
            rows.append(
                {
                    "side": side,
                    "config": name,
                    "mode": mode,
                    "q_deg": {key: float(q_deg.get(key, 0.0)) for key in JOINT_KEYS},
                    "rows": int(analytic.shape[0]),
                    "max_abs_error": max_abs,
                    "relative_frobenius_error": relative,
                    "passed": bool(max_abs <= FD_ABS_TOL and relative <= FD_REL_TOL),
                }
            )

    payload = {
        "meta": {
            "purpose": "finite-difference check of the analytic geometric Jacobian before IK optimization",
            "finite_difference": "central difference, h = 1e-6 rad",
            "position_units": "m/rad",
            "orientation_units": "rad/rad",
            "normal_units": "dimensionless/rad after projection onto the normal tangent plane",
            "condition_number": "sigma_max/sigma_min of the scaled task Jacobian",
            "condition_scaling": {
                "position_rows": "divided by L_norm = 0.5 m",
                "orientation_rows": "unchanged (rad/rad)",
                "normal_rows": "unchanged after tangent projection",
            },
            "numerical_tolerance": {
                "max_abs": FD_ABS_TOL,
                "relative_frobenius": FD_REL_TOL,
                "note": "numerical verification tolerance, not a safety or performance threshold",
            },
        },
        "checks": rows,
        "summary": {
            "count": len(rows),
            "passed": sum(1 for row in rows if row["passed"]),
            "max_abs_error": max(row["max_abs_error"] for row in rows),
            "max_relative_error": max(row["relative_frobenius_error"] for row in rows),
        },
    }
    (HERE / "ik_jacobian_check.json").write_text(
        json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8"
    )
    lines = [
        "# IK Jacobian finite-difference check",
        "",
        "Central difference, h = 1e-6 rad. Position rows: m/rad; orientation rows: rad/rad.",
        "Conditioning uses the scaled Jacobian: position rows divided by L_norm = 0.5 m.",
        "",
        "| side | config | mode | max abs error | relative Frobenius | pass |",
        "|---|---|---|---:|---:|---|",
    ]
    for row in rows:
        lines.append(
            f"| {row['side']} | {row['config']} | {row['mode']} | "
            f"{row['max_abs_error']:.3e} | {row['relative_frobenius_error']:.3e} | "
            f"{'yes' if row['passed'] else 'no'} |"
        )
    (HERE / "ik_jacobian_check.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    return payload


# ---------------------------------------------------------------------------
# Control-pose IK validation


def run_control_validation() -> dict[str, Any]:
    gt = json.loads((STAGE2A / "control_pose_gt.json").read_text(encoding="utf-8"))
    pose_definitions = gt["pose_definitions"]
    lift_mm = gt["lift_mm"]
    rows: list[dict[str, Any]] = []

    for pose_index, (pose, sides) in enumerate(pose_definitions.items()):
        for side_index, side in enumerate(("L", "R")):
            known_q = sides[side]
            lift_m = float(lift_mm.get(pose, 0.0)) / 1000.0
            target = target_from_q(side, known_q, lift_m)
            for mode_index, mode in enumerate(("6d", "5d")):
                seed = 20261021 + pose_index * 100 + side_index * 10 + mode_index
                result = solve_target(side, target, mode, lift_m, known_q, seed)
                known = result["known_result"]
                best = result["best"]
                rows.append(
                    {
                        "pose": pose,
                        "side": side,
                        "mode": mode,
                        "lift_mm": float(lift_mm.get(pose, 0.0)),
                        "known_q_deg": result["known_q_deg"],
                        "known_result": known,
                        "best": best,
                        "solution_count": result["solution_count"],
                        "raw_solution_count": result["raw_solution_count"],
                        "random_converged": result["random_converged"],
                        "random_start_count": result["random_start_count"],
                        "target": target,
                        "solutions": result["solutions"],
                    }
                )

    known_exact = sum(
        1 for row in rows
        if row["known_result"]["position_error_mm"] <= 1e-6
        and row["known_result"]["orientation_error_deg"] <= 1e-6
    )
    best_position = max(row["best"]["position_error_mm"] for row in rows if row["best"])
    best_orientation = max(row["best"]["orientation_error_deg"] for row in rows if row["best"])
    random_success = sum(1 for row in rows if row["random_converged"] > 0)
    min_margin = min(row["best"]["min_margin_deg"] for row in rows if row["best"])
    total_solutions = sum(row["solution_count"] for row in rows)
    max_solutions = max(row["solution_count"] for row in rows)

    payload = {
        "meta": {
            "purpose": "IK validation on the 12 Blender-validated control poses, both arms, 6D and 5D",
            "target_source": "rigidified FK from the known control-pose joint angles",
            "modes": {
                "6d": "contact-face position + full palm orientation",
                "5d": "contact-face position + palm normal; rotation about the normal is free",
            },
            "condition_scaling": {
                "position_rows": "divided by L_norm = 0.5 m",
                "orientation_rows": "unchanged (rad/rad)",
                "normal_rows": "unchanged after tangent projection",
            },
            "acceptance": {
                "scaled_task_error": IK_ACCEPT_TOL,
                "note": "acceptance for the solver; not a safety or performance threshold",
            },
            "random_starts_per_target": 10,
            "nullspace_sampling": "anchor solution, +/-3 and +/-6 degrees along each null-space direction",
        },
        "summary": {
            "rows": len(rows),
            "poses": len(pose_definitions),
            "known_exact": known_exact,
            "random_success": random_success,
            "best_max_position_error_mm": best_position,
            "best_max_orientation_error_deg": best_orientation,
            "min_best_margin_deg": min_margin,
            "total_solutions": total_solutions,
            "max_solutions_per_target": max_solutions,
        },
        "rows": rows,
    }
    (HERE / "ik_control_validation.json").write_text(
        json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8"
    )

    lines = [
        "# IK validation on control poses",
        "",
        "Targets are generated by the rigidified FK from the known control-pose joint angles.",
        "6D = position + full orientation; 5D = position + palm normal.",
        "Conditioning uses the scaled Jacobian; position rows divided by L_norm = 0.5 m.",
        "",
        "| pose | side | mode | known pos mm | known ori deg | best pos mm | best ori deg | solutions | random converged | best margin | best cond |",
        "|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for row in rows:
        best = row["best"]
        condition = "inf" if best["condition"] is None else f"{best['condition']:.2e}"
        lines.append(
            f"| {row['pose']} | {row['side']} | {row['mode']} | "
            f"{row['known_result']['position_error_mm']:.4f} | "
            f"{row['known_result']['orientation_error_deg']:.4f} | "
            f"{best['position_error_mm']:.4f} | {best['orientation_error_deg']:.4f} | "
            f"{row['solution_count']} | {row['random_converged']}/{row['random_start_count']} | "
            f"{best['min_margin_deg']:.1f} | {condition} |"
        )
    (HERE / "ik_control_validation.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    return payload


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("jacobian", "control", "all"), default="all")
    args = parser.parse_args()

    if args.mode in ("jacobian", "all"):
        payload = run_jacobian_check()
        print(
            "JACOBIAN",
            f"checks={payload['summary']['count']}",
            f"passed={payload['summary']['passed']}",
            f"max_abs={payload['summary']['max_abs_error']:.3e}",
            f"max_rel={payload['summary']['max_relative_error']:.3e}",
        )
    if args.mode in ("control", "all"):
        payload = run_control_validation()
        print(
            "CONTROL",
            f"rows={payload['summary']['rows']}",
            f"known_exact={payload['summary']['known_exact']}",
            f"random_success={payload['summary']['random_success']}",
            f"best_pos_mm={payload['summary']['best_max_position_error_mm']:.4f}",
            f"best_ori_deg={payload['summary']['best_max_orientation_error_deg']:.4f}",
            f"total_solutions={payload['summary']['total_solutions']}",
            f"max_solutions={payload['summary']['max_solutions_per_target']}",
        )


if __name__ == "__main__":
    main()
