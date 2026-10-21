"""Broad parametric workspace search for the Stage 2A robot arms.

This is a research script.  It does not modify the .blend and does not
replace the validated IK implementation.  It reuses ``bed_study`` for the
parametric bed, targets, simplified collision geometry and seed poses.

The search is intentionally split into independent groups:

* 5D: full grid over height, width, base-to-bed clearance, approach, side,
  pose and lift=0.
* 6D: orientation sweep at selected bed configurations, using four finger
  directions for each target.
* lift: selected poses at lift 0, 25, 50, 75 and 100 mm.
* joint: both arms solved for the same pose, followed by an arm-arm check.
* P8: two geometric contact zones (shoulder and pelvis), solved
  independently and, if both reachable, checked for arm-arm clearance.

Everything is kinematic reachability and a temporary simplified collision
model.  It is not a safety, force or patient-contact assessment.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
import time
import zlib
from collections import Counter, defaultdict
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path
from typing import Any

# Avoid oversubscribing BLAS inside worker processes.  This must happen before
# numpy is imported.
os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("MKL_NUM_THREADS", "1")
os.environ.setdefault("NUMEXPR_NUM_THREADS", "1")

import numpy as np  # noqa: E402

sys.dont_write_bytecode = True

HERE = Path(__file__).resolve().parent
STAGE2A = HERE.parent / "stage2A"
for path in (HERE, STAGE2A):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

import bed_study as bed  # noqa: E402
import ik_study as ik  # noqa: E402

APPROACHES = ("lateral", "frontal")
SIDES = ("L", "R")
POSES = ("P4", "P5", "P6", "P7", "P9")
GRID_HEIGHTS = (0.6, 0.7, 0.8, 0.9)
GRID_WIDTHS = (0.9, 1.0, 1.2)
GRID_CLEARANCES = (0.05, 0.10, 0.15, 0.20)
LIFT_VALUES = (0.0, 0.025, 0.050, 0.075, 0.100)
ORIENTATIONS = ("+X", "-X", "+Y", "-Y")

FINGER_VECTORS = {
    "+X": np.array([1.0, 0.0, 0.0]),
    "-X": np.array([-1.0, 0.0, 0.0]),
    "+Y": np.array([0.0, 1.0, 0.0]),
    "-Y": np.array([0.0, -1.0, 0.0]),
}


def stable_seed(*parts: Any) -> int:
    text = "|".join(str(part) for part in parts)
    return int(zlib.crc32(text.encode("utf-8")) & 0x7FFFFFFF)


def make_target(
    approach: str,
    height_m: float,
    width_m: float,
    clearance_m: float,
    side: str,
    pose: str,
    finger_label: str | None = None,
) -> dict[str, Any]:
    bed_geom = bed.bed_geometry(approach, height_m, width_m, clearance_m)
    target = bed.make_targets(approach, bed_geom, side)[pose].copy()
    if finger_label is not None:
        finger = FINGER_VECTORS[finger_label]
        target["finger"] = [float(value) for value in bed.normalize(finger)]
        target["rotation"] = bed.orientation_from_normal_finger(
            side, np.asarray(target["normal"], dtype=float), finger
        ).tolist()
    return target


def target_bed_dict(target: dict[str, Any]) -> dict[str, Any]:
    return target["bed"]


def solve_fast(
    side: str,
    target: dict[str, Any],
    mode: str,
    lift_m: float,
    seed: int,
    prev_q: dict[str, float] | None = None,
    random_count: int = 3,
    max_iter: int = 180,
) -> dict[str, Any]:
    """Small start set plus a fallback; deliberately cheaper than solve_bed_target."""
    rng = np.random.default_rng(seed)
    lower = ik.LOWER[side]
    upper = ik.UPPER[side]
    span = upper - lower

    starts: list[tuple[str, np.ndarray]] = []
    for label, q_deg in bed.seed_poses(side).items():
        starts.append((label, ik.q_array(side, q_deg)))
    starts.append(("zero", np.zeros(len(ik.JOINT_KEYS), dtype=float)))
    if prev_q is not None:
        starts.append(("prev", ik.q_array(side, prev_q)))
    for index in range(random_count):
        values = lower + 0.05 * span + rng.random(len(ik.JOINT_KEYS)) * 0.90 * span
        values[3] = rng.uniform(20.0, 120.0)
        starts.append((f"random{index}", values))

    raw: list[dict[str, Any]] = []
    for label, q0 in starts:
        result = ik.solve_once(side, target, mode, lift_m, q0, max_iter=max_iter)
        result["start"] = label
        raw.append(result)

    if not any(result["converged"] for result in raw):
        # A few more random starts only for targets that failed the first pass.
        for index in range(random_count + 2):
            values = lower + 0.05 * span + rng.random(len(ik.JOINT_KEYS)) * 0.90 * span
            values[3] = rng.uniform(20.0, 120.0)
            result = ik.solve_once(side, target, mode, lift_m, values, max_iter=max_iter)
            result["start"] = f"fallback{index}"
            raw.append(result)

    valid = ik.dedupe_solutions([result for result in raw if result["converged"]])
    valid.sort(key=ik.solution_sort_key)
    if valid:
        best = valid[0]
    else:
        best = min(raw, key=lambda result: (result["task_error_norm"], result["position_error_mm"]))

    chosen = best
    if valid:
        for solution in valid[:8]:
            clearances = bed.collision_clearances(
                side,
                solution["q_deg"],
                lift_m,
                target_bed_dict(target),
                other_q=bed.seed_poses(side)["P0"],
            )
            if clearances["collision_free"]:
                chosen = solution
                break

    clearances = bed.collision_clearances(
        side,
        chosen["q_deg"],
        lift_m,
        target_bed_dict(target),
        other_q=bed.seed_poses(side)["P0"],
    )
    return {
        "best": chosen,
        "solutions": valid,
        "solution_count": len(valid),
        "raw_solution_count": len(raw),
        "reachable": bool(chosen["converged"]),
        "clearances": clearances,
        "collision_free": bool(clearances["collision_free"]),
    }


def row_from_result(
    kind: str,
    approach: str,
    height_m: float,
    width_m: float,
    clearance_m: float,
    pose: str,
    side: str,
    mode: str,
    lift_m: float,
    finger_label: str | None,
    result: dict[str, Any],
) -> dict[str, Any]:
    best = result["best"]
    clearances = result["clearances"]
    return {
        "kind": kind,
        "approach": approach,
        "height_m": float(height_m),
        "width_m": float(width_m),
        "clearance_m": float(clearance_m),
        "pose": pose,
        "side": side,
        "mode": mode,
        "lift_m": float(lift_m),
        "finger": finger_label,
        "reachable": bool(result["reachable"]),
        "collision_free": bool(result["collision_free"]),
        "position_error_mm": float(best["position_error_mm"]),
        "orientation_error_deg": float(best["orientation_error_deg"]),
        "normal_error_deg": float(best["normal_error_deg"]),
        "min_margin_deg": float(best["min_margin_deg"]),
        "condition": float(best["condition"]) if best["condition"] is not None else None,
        "rank": int(best["rank"]),
        "solution_count": int(result["solution_count"]),
        "raw_solution_count": int(result["raw_solution_count"]),
        "q_deg": best["q_deg"],
        "clearance_arm_torso_m": clearances["arm_torso"],
        "clearance_arm_base_m": clearances["arm_base"],
        "clearance_arm_bed_m": clearances["arm_bed"],
        "clearance_arm_arm_m": clearances["arm_arm"],
        "clearance_min_m": clearances["minimum"],
    }


def baseline_q() -> dict[tuple[str, str, str], dict[str, float] | None]:
    """Baseline 5D solution per approach/side/pose at H=0.7, W=1.0, c=0.1."""
    result: dict[tuple[str, str, str], dict[str, float] | None] = {}
    total = len(APPROACHES) * len(SIDES) * len(POSES)
    index = 0
    for approach in APPROACHES:
        for side in SIDES:
            for pose in POSES:
                index += 1
                target = make_target(approach, 0.7, 1.0, 0.1, side, pose)
                solved = solve_fast(
                    side,
                    target,
                    "5d",
                    0.0,
                    stable_seed("baseline", approach, side, pose),
                    prev_q=None,
                    random_count=6,
                    max_iter=220,
                )
                result[(approach, side, pose)] = (
                    solved["best"]["q_deg"] if solved["reachable"] else None
                )
                print(
                    f"baseline {index}/{total}: {approach} {side} {pose} "
                    f"reachable={solved['reachable']} "
                    f"pos={solved['best']['position_error_mm']:.3f}mm",
                    flush=True,
                )
    return result


def task_5d(payload: tuple[Any, ...]) -> dict[str, Any]:
    approach, height_m, width_m, clearance_m, pose, side, prev_q = payload
    target = make_target(approach, height_m, width_m, clearance_m, side, pose)
    solved = solve_fast(
        side,
        target,
        "5d",
        0.0,
        stable_seed("5d", approach, height_m, width_m, clearance_m, pose, side),
        prev_q=prev_q,
        random_count=3,
        max_iter=180,
    )
    return row_from_result(
        "5d_grid", approach, height_m, width_m, clearance_m, pose, side, "5d", 0.0, None, solved
    )


def task_6d(payload: tuple[Any, ...]) -> dict[str, Any]:
    approach, height_m, width_m, clearance_m, pose, side, finger_label, prev_q = payload
    target = make_target(
        approach, height_m, width_m, clearance_m, side, pose, finger_label=finger_label
    )
    solved = solve_fast(
        side,
        target,
        "6d",
        0.0,
        stable_seed(
            "6d", approach, height_m, width_m, clearance_m, pose, side, finger_label
        ),
        prev_q=prev_q,
        random_count=3,
        max_iter=200,
    )
    return row_from_result(
        "6d_orientation",
        approach,
        height_m,
        width_m,
        clearance_m,
        pose,
        side,
        "6d",
        0.0,
        finger_label,
        solved,
    )


def task_lift(payload: tuple[Any, ...]) -> dict[str, Any]:
    approach, pose, side, lift_m, prev_q = payload
    target = make_target(approach, 0.7, 1.0, 0.1, side, pose)
    solved = solve_fast(
        side,
        target,
        "5d",
        lift_m,
        stable_seed("lift", approach, pose, side, lift_m),
        prev_q=prev_q,
        random_count=3,
        max_iter=180,
    )
    return row_from_result(
        "lift", approach, 0.7, 1.0, 0.1, pose, side, "5d", lift_m, None, solved
    )


def joint_clearances(
    approach: str,
    height_m: float,
    width_m: float,
    clearance_m: float,
    lift_m: float,
    left_q: dict[str, float],
    right_q: dict[str, float],
) -> dict[str, Any]:
    bed_geom = bed.bed_geometry(approach, height_m, width_m, clearance_m)
    left = bed.collision_clearances("L", left_q, lift_m, bed_geom, other_q=right_q)
    right = bed.collision_clearances("R", right_q, lift_m, bed_geom, other_q=left_q)
    values = {
        "arm_torso": min(
            value for value in (left["arm_torso"], right["arm_torso"]) if value is not None
        )
        if left["arm_torso"] is not None or right["arm_torso"] is not None
        else None,
        "arm_base": min(
            value for value in (left["arm_base"], right["arm_base"]) if value is not None
        )
        if left["arm_base"] is not None or right["arm_base"] is not None
        else None,
        "arm_bed": min(
            value for value in (left["arm_bed"], right["arm_bed"]) if value is not None
        )
        if left["arm_bed"] is not None or right["arm_bed"] is not None
        else None,
        "arm_arm": min(
            value for value in (left["arm_arm"], right["arm_arm"]) if value is not None
        )
        if left["arm_arm"] is not None or right["arm_arm"] is not None
        else None,
    }
    numeric = [value for value in values.values() if value is not None]
    values["minimum"] = min(numeric) if numeric else None
    values["collision_free"] = bool(all(value >= 0.0 for value in numeric)) if numeric else True
    return values


def task_joint(payload: tuple[Any, ...]) -> dict[str, Any]:
    approach, pose, mode, left_prev, right_prev = payload
    targets = bed.make_targets(
        approach, bed.bed_geometry(approach, 0.7, 1.0, 0.1), "L"
    )
    target_l = targets[pose]
    target_r = bed.make_targets(
        approach, bed.bed_geometry(approach, 0.7, 1.0, 0.1), "R"
    )[pose]
    left = solve_fast(
        "L",
        target_l,
        mode,
        0.0,
        stable_seed("joint", approach, pose, mode, "L"),
        prev_q=left_prev,
        random_count=4,
        max_iter=220,
    )
    right = solve_fast(
        "R",
        target_r,
        mode,
        0.0,
        stable_seed("joint", approach, pose, mode, "R"),
        prev_q=right_prev,
        random_count=4,
        max_iter=220,
    )
    both = bool(left["reachable"] and right["reachable"])
    clearances = None
    if both:
        clearances = joint_clearances(
            approach,
            0.7,
            1.0,
            0.1,
            0.0,
            left["best"]["q_deg"],
            right["best"]["q_deg"],
        )
    return {
        "kind": "joint_pose",
        "approach": approach,
        "height_m": 0.7,
        "width_m": 1.0,
        "clearance_m": 0.1,
        "pose": pose,
        "mode": mode,
        "left_reachable": bool(left["reachable"]),
        "right_reachable": bool(right["reachable"]),
        "both_reachable": both,
        "left_q_deg": left["best"]["q_deg"],
        "right_q_deg": right["best"]["q_deg"],
        "left_position_error_mm": float(left["best"]["position_error_mm"]),
        "right_position_error_mm": float(right["best"]["position_error_mm"]),
        "left_condition": float(left["best"]["condition"]) if left["best"]["condition"] is not None else None,
        "right_condition": float(right["best"]["condition"]) if right["best"]["condition"] is not None else None,
        "left_min_margin_deg": float(left["best"]["min_margin_deg"]),
        "right_min_margin_deg": float(right["best"]["min_margin_deg"]),
        "collision_free": bool(clearances["collision_free"]) if clearances else None,
        "clearance_arm_torso_m": clearances["arm_torso"] if clearances else None,
        "clearance_arm_base_m": clearances["arm_base"] if clearances else None,
        "clearance_arm_bed_m": clearances["arm_bed"] if clearances else None,
        "clearance_arm_arm_m": clearances["arm_arm"] if clearances else None,
        "clearance_min_m": clearances["minimum"] if clearances else None,
    }


def p8_zones(approach: str, height_m: float, width_m: float, clearance_m: float) -> dict[str, dict[str, Any]]:
    """Two geometric zones for P8: shoulder and pelvis.  No patient is modelled."""
    bed_geom = bed.bed_geometry(approach, height_m, width_m, clearance_m)
    z = float(bed_geom["z_top"]) + bed.APPROACH_Z_M
    if approach == "lateral":
        x = float(bed_geom["x_near"]) + 0.35 * float(width_m)
        shoulder = (x, 0.35, z)
        pelvis = (x, -0.35, z)
    else:
        shoulder = (float(bed_geom["x_near"]) + 0.55, 0.10, z)
        pelvis = (float(bed_geom["x_far"]) - 0.55, -0.10, z)

    result: dict[str, dict[str, Any]] = {}
    for label, position in (("shoulder", shoulder), ("pelvis", pelvis)):
        rotation = bed.orientation_from_normal_finger("L", bed.DOWN, bed.PLUS_X)
        result[label] = {
            "pose": f"P8_{label}",
            "side": "L",
            "approach": approach,
            "position_floor": [float(value) for value in position],
            "normal": [0.0, 0.0, -1.0],
            "finger": [1.0, 0.0, 0.0],
            "rotation": rotation.tolist(),
            "bed": {
                "height_m": bed_geom["height_m"],
                "z_top": bed_geom["z_top"],
                "width_m": bed_geom["width_m"],
                "clearance_m": bed_geom["clearance_m"],
                "x_near": bed_geom["x_near"],
                "x_far": bed_geom["x_far"],
                "y_min": bed_geom["y_min"],
                "y_max": bed_geom["y_max"],
                "length_m": bed_geom["length_m"],
            },
        }
    return result


def task_p8(payload: tuple[Any, ...]) -> dict[str, Any]:
    approach, assignment = payload
    zones = p8_zones(approach, 0.7, 1.0, 0.1)
    left_zone = "shoulder" if assignment == "L_shoulder" else "pelvis"
    right_zone = "pelvis" if left_zone == "shoulder" else "shoulder"
    left = solve_fast(
        "L",
        zones[left_zone],
        "5d",
        0.0,
        stable_seed("p8", approach, assignment, "L"),
        prev_q=None,
        random_count=5,
        max_iter=220,
    )
    right = solve_fast(
        "R",
        zones[right_zone],
        "5d",
        0.0,
        stable_seed("p8", approach, assignment, "R"),
        prev_q=None,
        random_count=5,
        max_iter=220,
    )
    both = bool(left["reachable"] and right["reachable"])
    clearances = None
    if both:
        clearances = joint_clearances(
            approach,
            0.7,
            1.0,
            0.1,
            0.0,
            left["best"]["q_deg"],
            right["best"]["q_deg"],
        )
    return {
        "kind": "p8_geometric",
        "approach": approach,
        "height_m": 0.7,
        "width_m": 1.0,
        "clearance_m": 0.1,
        "pose": "P8",
        "mode": "5d",
        "assignment": assignment,
        "left_zone": left_zone,
        "right_zone": right_zone,
        "left_reachable": bool(left["reachable"]),
        "right_reachable": bool(right["reachable"]),
        "both_reachable": both,
        "left_position_error_mm": float(left["best"]["position_error_mm"]),
        "right_position_error_mm": float(right["best"]["position_error_mm"]),
        "left_q_deg": left["best"]["q_deg"],
        "right_q_deg": right["best"]["q_deg"],
        "collision_free": bool(clearances["collision_free"]) if clearances else None,
        "clearance_arm_arm_m": clearances["arm_arm"] if clearances else None,
        "clearance_min_m": clearances["minimum"] if clearances else None,
    }


def summarize(rows: list[dict[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for kind in sorted({row["kind"] for row in rows}):
        subset = [row for row in rows if row["kind"] == kind]
        result[kind] = {
            "rows": len(subset),
            "reachable": sum(1 for row in subset if row.get("reachable") is True),
            "collision_free": sum(1 for row in subset if row.get("collision_free") is True),
            "by_approach": dict(Counter(row.get("approach") for row in subset)),
            "by_side": dict(Counter(row.get("side") for row in subset)),
            "by_mode": dict(Counter(row.get("mode") for row in subset)),
        }
    return result


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    fields = [
        "kind", "approach", "height_m", "width_m", "clearance_m", "pose", "side",
        "mode", "lift_m", "finger", "assignment", "reachable", "collision_free",
        "position_error_mm", "orientation_error_deg", "normal_error_deg",
        "min_margin_deg", "condition", "rank", "solution_count", "raw_solution_count",
        "q_deg", "left_q_deg", "right_q_deg", "left_reachable", "right_reachable",
        "both_reachable", "left_condition", "right_condition", "left_min_margin_deg",
        "right_min_margin_deg", "clearance_arm_torso_m", "clearance_arm_base_m",
        "clearance_arm_bed_m", "clearance_arm_arm_m", "clearance_min_m",
    ]
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            flat = dict(row)
            for key in ("q_deg", "left_q_deg", "right_q_deg"):
                if key in flat and flat[key] is not None:
                    flat[key] = json.dumps(flat[key], ensure_ascii=False, sort_keys=True)
            writer.writerow(flat)


def md_table(header: list[str], rows: list[list[Any]]) -> str:
    lines = ["| " + " | ".join(header) + " |", "|" + "|".join(["---"] * len(header)) + "|"]
    for row in rows:
        lines.append("| " + " | ".join(str(value) for value in row) + " |")
    return "\n".join(lines)


def write_report(path: Path, rows: list[dict[str, Any]], summary: dict[str, Any]) -> None:
    lines: list[str] = []
    lines.append("# Stage 2A — широкий поиск рабочих поз над кроватью")
    lines.append("")
    lines.append("**Статус:** кинематическая достижимость и упрощённая проверка зазоров; не оценка безопасности, прочности или допустимости операции.")
    lines.append("")
    lines.append("## 1. Что просчитано")
    lines.append("")
    lines.append("- 5D: высоты 0.6/0.7/0.8/0.9 м; ширины 0.9/1.0/1.2 м; зазоры 0.05/0.10/0.15/0.20 м; боковой и фронтальный подходы; обе руки; P4/P5/P6/P7/P9; лифт 0.")
    lines.append("- 6D: выбранные конфигурации H=0.7/0.9 м, W=1.0/1.2 м, зазор 0.10 м; четыре направления пальцев (+X/−X/+Y/−Y).")
    lines.append("- Лифт: 0/25/50/75/100 мм для P4/P6/P7/P9 на базовой конфигурации.")
    lines.append("- Совместные позы: обе руки в одной позе; проверка «рука-рука».")
    lines.append("- P8: две геометрические зоны (плечо и таз), без пациента; проверка достижимости и «рука-рука».")
    lines.append("")
    lines.append("## 2. Сводка")
    lines.append("")
    summary_rows = []
    for kind, item in summary.items():
        summary_rows.append([kind, item["rows"], item["reachable"], item["collision_free"]])
    lines.append(md_table(["группа", "строк", "достижимо", "collision-free"], summary_rows))
    lines.append("")
    lines.append("## 3. 5D по позам и высотам")
    lines.append("")
    grid = [row for row in rows if row["kind"] == "5d_grid"]
    for approach in APPROACHES:
        lines.append(f"### {approach}")
        lines.append("")
        table = []
        for pose in POSES:
            values = []
            for height in GRID_HEIGHTS:
                subset = [row for row in grid if row["approach"] == approach and row["pose"] == pose and row["height_m"] == height]
                values.append(f"{sum(1 for row in subset if row['reachable'])}/{len(subset)}")
            table.append([pose] + values)
        lines.append(md_table(["поза"] + [f"H={h}" for h in GRID_HEIGHTS], table))
        lines.append("")
    lines.append("## 4. 5D по ширине и зазору")
    lines.append("")
    for approach in APPROACHES:
        lines.append(f"### {approach}")
        lines.append("")
        table = []
        for width in GRID_WIDTHS:
            values = []
            for clearance in GRID_CLEARANCES:
                subset = [row for row in grid if row["approach"] == approach and row["width_m"] == width and row["clearance_m"] == clearance]
                values.append(f"{sum(1 for row in subset if row['reachable'])}/{len(subset)}")
            table.append([f"W={width}"] + values)
        lines.append(md_table(["ширина"] + [f"c={c}" for c in GRID_CLEARANCES], table))
        lines.append("")
    lines.append("## 5. 6D по ориентациям")
    lines.append("")
    six = [row for row in rows if row["kind"] == "6d_orientation"]
    table = []
    for pose in ("P4", "P6", "P7", "P9"):
        for approach in APPROACHES:
            for side in SIDES:
                values = []
                for orientation in ORIENTATIONS:
                    subset = [row for row in six if row["pose"] == pose and row["approach"] == approach and row["side"] == side and row["finger"] == orientation]
                    values.append(f"{sum(1 for row in subset if row['reachable'])}/{len(subset)}")
                table.append([pose, approach, side] + values)
    lines.append(md_table(["поза", "подход", "рука"] + list(ORIENTATIONS), table))
    lines.append("")
    lines.append("## 6. Лифт")
    lines.append("")
    lift = [row for row in rows if row["kind"] == "lift"]
    table = []
    for pose in ("P4", "P6", "P7", "P9"):
        for side in SIDES:
            values = []
            for lift_value in LIFT_VALUES:
                subset = [row for row in lift if row["pose"] == pose and row["side"] == side and row["lift_m"] == lift_value]
                values.append(f"{sum(1 for row in subset if row['reachable'])}/{len(subset)}")
            table.append([pose, side] + values)
    lines.append(md_table(["поза", "рука"] + [f"{int(value*1000)} мм" for value in LIFT_VALUES], table))
    lines.append("")
    lines.append("## 7. Совместные позы")
    lines.append("")
    joint = [row for row in rows if row["kind"] == "joint_pose"]
    table = []
    for row in joint:
        table.append([
            row["approach"], row["pose"], row["mode"], row["left_reachable"], row["right_reachable"],
            row["both_reachable"], row.get("collision_free"),
            row.get("clearance_arm_arm_m"), row.get("clearance_min_m"),
        ])
    lines.append(md_table(["подход", "поза", "режим", "L", "R", "обе", "collision-free", "рука-рука, м", "мин. зазор, м"], table))
    lines.append("")
    lines.append("## 8. P8 — геометрические зоны")
    lines.append("")
    p8 = [row for row in rows if row["kind"] == "p8_geometric"]
    table = []
    for row in p8:
        table.append([
            row["approach"], row["assignment"], row["left_zone"], row["right_zone"],
            row["left_reachable"], row["right_reachable"], row["both_reachable"],
            row.get("clearance_arm_arm_m"), row.get("clearance_min_m"),
        ])
    lines.append(md_table(["подход", "назначение", "L-зона", "R-зона", "L", "R", "обе", "рука-рука, м", "мин. зазор, м"], table))
    lines.append("")
    lines.append("## 9. Крайние значения")
    lines.append("")
    reachable = [row for row in rows if row.get("reachable") is True]
    if reachable:
        best_cond = min(reachable, key=lambda row: row["condition"] if row["condition"] is not None else 1e300)
        worst_cond = max(reachable, key=lambda row: row["condition"] if row["condition"] is not None else -1)
        min_margin = min(reachable, key=lambda row: row["min_margin_deg"])
        min_clearance = min(
            [row for row in reachable if row.get("clearance_min_m") is not None],
            key=lambda row: row["clearance_min_m"],
            default=None,
        )
        lines.append(f"- Лучший `cond`: {best_cond['condition']:.3f} ({best_cond['kind']} {best_cond['approach']} {best_cond['pose']} {best_cond['side']}).")
        lines.append(f"- Худший `cond` среди достижимых: {worst_cond['condition']:.3f} ({worst_cond['kind']} {worst_cond['approach']} {worst_cond['pose']} {worst_cond['side']}).")
        lines.append(f"- Минимальный запас до лимита: {min_margin['min_margin_deg']:.4f}° ({min_margin['kind']} {min_margin['approach']} {min_margin['pose']} {min_margin['side']}).")
        if min_clearance:
            lines.append(f"- Минимальный упрощённый зазор: {min_clearance['clearance_min_m']:.4f} м ({min_clearance['kind']} {min_clearance['approach']} {min_clearance['pose']} {min_clearance['side']}).")
    lines.append("")
    lines.append("## 10. Недостижимые зоны")
    lines.append("")
    unreachable = [row for row in rows if row.get("reachable") is False]
    lines.append(f"Всего недостижимых строк: {len(unreachable)}.")
    counts = Counter((row["kind"], row["approach"], row["pose"], row["side"]) for row in unreachable)
    table = [[key[0], key[1], key[2], key[3], value] for key, value in counts.most_common(30)]
    lines.append(md_table(["группа", "подход", "поза", "рука", "строк"], table))
    lines.append("")
    lines.append("## 11. Ограничения")
    lines.append("")
    lines.append("- Упрощённая collision-геометрия: капсулы звеньев, oriented box торса, AABB базы и кровати; нет пациента, постели, поручней и исходных collision-мешей.")
    lines.append("- P8 — только две геометрические зоны; силовой контакт и поворот пациента не моделируются.")
    lines.append("- Лифт 0…+100 мм — тестовый диапазон; по существующему AABB-аудиту первый контакт механических элементов ожидается около +427.3 мм, но это не подтверждённый рабочий диапазон.")
    lines.append("- 6D-отказы зависят от выбранного направления пальцев; таблица не доказывает недостижимость при любой ориентации.")
    lines.append("- Это не оценка безопасности, устойчивости, прочности или допустимости операции.")
    lines.append("")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--jobs", type=int, default=8)
    parser.add_argument("--quick", action="store_true")
    parser.add_argument("--no-6d", action="store_true")
    parser.add_argument("--out", type=Path, default=HERE / "workspace_search.json")
    args = parser.parse_args()

    if args.quick:
        heights = (0.7,)
        widths = (1.0,)
        clearances = (0.1,)
        poses = ("P4", "P6")
        do_6d = False
    else:
        heights = GRID_HEIGHTS
        widths = GRID_WIDTHS
        clearances = GRID_CLEARANCES
        poses = POSES
        do_6d = not args.no_6d

    print("baseline pass", flush=True)
    baseline = baseline_q()

    payloads_5d = []
    for approach in APPROACHES:
        for height in heights:
            for width in widths:
                for clearance in clearances:
                    for pose in poses:
                        for side in SIDES:
                            payloads_5d.append(
                                (
                                    approach, height, width, clearance, pose, side,
                                    baseline.get((approach, side, pose)),
                                )
                            )
    print(f"5D targets: {len(payloads_5d)}", flush=True)

    payloads_6d = []
    if do_6d:
        for approach in APPROACHES:
            for height in (0.7, 0.9):
                for width in (1.0, 1.2):
                    for pose in ("P4", "P6", "P7", "P9"):
                        for side in SIDES:
                            for orientation in ORIENTATIONS:
                                payloads_6d.append(
                                    (
                                        approach, height, width, 0.1, pose, side,
                                        orientation, baseline.get((approach, side, pose)),
                                    )
                                )
    print(f"6D targets: {len(payloads_6d)}", flush=True)

    payloads_lift = []
    for approach in APPROACHES:
        for pose in ("P4", "P6", "P7", "P9"):
            for side in SIDES:
                for lift_value in LIFT_VALUES:
                    payloads_lift.append(
                        (approach, pose, side, lift_value, baseline.get((approach, side, pose)))
                    )
    print(f"lift targets: {len(payloads_lift)}", flush=True)

    payloads_joint = []
    for approach in APPROACHES:
        for pose in ("P6", "P7", "P9"):
            for mode in ("5d", "6d"):
                payloads_joint.append(
                    (
                        approach, pose, mode,
                        baseline.get((approach, "L", pose)),
                        baseline.get((approach, "R", pose)),
                    )
                )
    print(f"joint targets: {len(payloads_joint)}", flush=True)

    payloads_p8 = [(approach, assignment) for approach in APPROACHES for assignment in ("L_shoulder", "R_shoulder")]
    print(f"P8 targets: {len(payloads_p8)}", flush=True)

    all_payloads: list[tuple[str, tuple[Any, ...]]] = []
    all_payloads.extend(("5d", payload) for payload in payloads_5d)
    all_payloads.extend(("6d", payload) for payload in payloads_6d)
    all_payloads.extend(("lift", payload) for payload in payloads_lift)
    all_payloads.extend(("joint", payload) for payload in payloads_joint)
    all_payloads.extend(("p8", payload) for payload in payloads_p8)

    task_map = {
        "5d": task_5d,
        "6d": task_6d,
        "lift": task_lift,
        "joint": task_joint,
        "p8": task_p8,
    }

    rows: list[dict[str, Any]] = []
    started = time.time()
    total = len(all_payloads)
    print(f"total tasks: {total}", flush=True)

    with ProcessPoolExecutor(max_workers=max(1, args.jobs)) as executor:
        futures = {
            executor.submit(task_map[kind], payload): (kind, payload)
            for kind, payload in all_payloads
        }
        for index, future in enumerate(as_completed(futures), start=1):
            kind, payload = futures[future]
            try:
                rows.append(future.result())
            except Exception as exc:  # keep the rest of the run alive
                rows.append(
                    {
                        "kind": kind,
                        "payload": repr(payload),
                        "reachable": False,
                        "error": f"{type(exc).__name__}: {exc}",
                    }
                )
                print(f"ERROR {kind} {payload}: {exc}", flush=True)
            if index % 25 == 0 or index == total:
                elapsed = time.time() - started
                print(
                    f"progress {index}/{total} elapsed={elapsed:.1f}s "
                    f"rows={len(rows)}",
                    flush=True,
                )

    summary = summarize(rows)
    output = {
        "generated_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "method": "kinematic IK 5D/6D + temporary simplified collision geometry",
        "limits": {
            "heights_m": list(heights),
            "widths_m": list(widths),
            "clearances_m": list(clearances),
            "poses": list(poses),
            "lift_m": list(LIFT_VALUES),
            "orientations": list(ORIENTATIONS),
        },
        "baseline": {f"{key[0]}|{key[1]}|{key[2]}": value for key, value in baseline.items()},
        "summary": summary,
        "rows": rows,
    }
    args.out.write_text(json.dumps(output, ensure_ascii=False, indent=2), encoding="utf-8")
    csv_path = args.out.with_suffix(".csv")
    write_csv(csv_path, rows)
    report_path = args.out.with_name("workspace_report.md")
    write_report(report_path, rows, summary)

    print("done", flush=True)
    print(json.dumps(summary, ensure_ascii=False, indent=2), flush=True)
    print(f"json={args.out}", flush=True)
    print(f"csv={csv_path}", flush=True)
    print(f"report={report_path}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
