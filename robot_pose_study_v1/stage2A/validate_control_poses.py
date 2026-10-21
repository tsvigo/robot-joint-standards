"""Validate the rigidified FK model against Blender control-pose GT.

Reads ``control_pose_gt.json``, compares 12 poses x 2 sides, computes
limit margins, 6x7 Jacobian singular values, condition numbers and shoulder
block ranks, then writes:

    control_pose_results.json
    control_pose_table.csv
    control_pose_table.md

No .blend access and no writes outside this directory.
"""

from __future__ import annotations

import csv
import json
import math
import sys
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

sys.dont_write_bytecode = True

import fk_model

HERE = Path(__file__).resolve().parent
GT_PATH = HERE / "control_pose_gt.json"
JSON_PATH = HERE / "control_pose_results.json"
CSV_PATH = HERE / "control_pose_table.csv"
MD_PATH = HERE / "control_pose_table.md"

POS_TOL_MM = 1.2
ROT_TOL_DEG = 0.02

POSE_NOTES = {
    "P0": "Исходная; ладони ±X; правый локоть почти выпрямлен.",
    "palmdown0": "Ладонь −Z; roll распределён sh3/rad ±45; правый локоть почти выпрямлен.",
    "forward": "sh2 на пределе ±90; плечевой gimbal lock; полный 6x7 ранг 6.",
    "backward": "sh2 на пределе ±90; обратный вылет; плечевой gimbal lock.",
    "forward_bent": "Компромисс sh2 ±75, el 45; лучше обусловлен; ориентация не решена.",
    "el90": "Лучшая обусловленность; ладонь ±Y, не контактная ориентация.",
    "lowered_sym": "Симметричное опускание L sh1 −21 / R sh1 +21; ладони ±X; ориентация не решена.",
    "p4pos_old": "Старый кандидат P4; правая рука поднята; ладонь не вниз.",
    "p6pos_old": "Старый кандидат P6; не ладонь вниз; w1 на лимите.",
    "folded": "Складная; el на верхнем лимите; столкновения не проверены.",
    "lift100": "Тестовая точка лифта +100 мм; рабочий диапазон не задан.",
    "liftm50": "Тестовая точка −50 мм; на 2.4 мм ниже первого контакта placeholder; не рабочая поза.",
}


def rotation_error_deg(a: np.ndarray, b: np.ndarray) -> float:
    ra = fk_model.orthonormalize(a)
    rb = fk_model.orthonormalize(b)
    cosine = (float(np.trace(ra.T @ rb)) - 1.0) / 2.0
    return math.degrees(math.acos(float(np.clip(cosine, -1.0, 1.0))))


def vec3(values: np.ndarray) -> list[float]:
    return [float(value) for value in values]


def fmt_vec(values: np.ndarray) -> str:
    return f"({values[0]:.2f},{values[1]:.2f},{values[2]:.2f})"


def main() -> None:
    gt = json.loads(GT_PATH.read_text(encoding="utf-8"))
    pose_defs = gt["pose_definitions"]
    lift_mm = gt["lift_mm"]

    rows = []
    max_pos = 0.0
    max_rot = 0.0
    for pose_name, sides in pose_defs.items():
        lift = float(lift_mm.get(pose_name, 0.0)) / 1000.0
        for side in ("L", "R"):
            q = sides[side]
            model_matrix = fk_model.palm_matrix(side, q, lift)
            gt_matrix = np.asarray(gt["poses"][pose_name][side], dtype=float)
            pos_error_mm = float(np.linalg.norm(model_matrix[:3, 3] - gt_matrix[:3, 3]) * 1000.0)
            rot_error_deg = rotation_error_deg(model_matrix, gt_matrix)
            max_pos = max(max_pos, pos_error_mm)
            max_rot = max(max_rot, rot_error_deg)

            geom = fk_model.palm_geometry(side, q, lift)
            jac, _axes, _point = fk_model.jacobian(side, q, lift)
            singular = np.linalg.svd(jac, compute_uv=False)
            rank = int(np.linalg.matrix_rank(jac, tol=1e-6))
            cond = float(singular[0] / singular[-1]) if singular[-1] > 1e-15 else None

            face_jac, _face_axes, _face_point = fk_model.jacobian(
                side, q, lift, point_local=geom["face_local"]
            )
            face_singular = np.linalg.svd(face_jac, compute_uv=False)
            face_cond = (
                float(face_singular[0] / face_singular[-1])
                if face_singular[-1] > 1e-15
                else None
            )

            shoulder_singular, shoulder_rank = fk_model.shoulder_singular_values(side, q)
            margins = fk_model.limit_margins(side, q)
            min_margin = min(margins, key=lambda item: item["margin_deg"])

            rows.append(
                {
                    "pose": pose_name,
                    "side": side,
                    "lift_mm": float(lift_mm.get(pose_name, 0.0)),
                    "q_deg": {key: float(value) for key, value in q.items()},
                    "palm_center_floor_m": vec3(geom["center_floor"]),
                    "contact_face_center_floor_m": vec3(geom["face_floor"]),
                    "normal": vec3(geom["normal"]),
                    "finger": vec3(geom["finger"]),
                    "angle_to_down_deg": float(geom["angle_to_down_deg"]),
                    "min_margin": {
                        "joint": min_margin["joint"],
                        "q_deg": float(min_margin["q_deg"]),
                        "margin_deg": float(min_margin["margin_deg"]),
                        "nearest": min_margin["nearest"],
                    },
                    "margins": margins,
                    "s": [float(value) for value in singular],
                    "s_min": float(singular[-1]),
                    "s_max": float(singular[0]),
                    "cond": cond,
                    "rank": rank,
                    "face_cond": face_cond,
                    "shoulder_s": [float(value) for value in shoulder_singular],
                    "shoulder_rank": shoulder_rank,
                    "gt_position_error_mm": pos_error_mm,
                    "gt_rotation_error_deg": rot_error_deg,
                    "within_limits": fk_model.within_limits(side, q),
                    "notes": POSE_NOTES.get(pose_name, ""),
                }
            )

    print("=== VALIDATION vs BLENDER ===")
    print(f"{'pose':14s} {'side':2s} {'pos_mm':>9s} {'rot_deg':>9s}")
    for row in rows:
        print(
            f"{row['pose']:14s} {row['side']:2s} "
            f"{row['gt_position_error_mm']:9.4f} {row['gt_rotation_error_deg']:9.4f}"
        )
    print(f"max position error: {max_pos:.4f} mm")
    print(f"max rotation error: {max_rot:.4f} deg")
    if max_pos > POS_TOL_MM or max_rot > ROT_TOL_DEG:
        raise SystemExit(
            f"validation failed: tolerance {POS_TOL_MM} mm / {ROT_TOL_DEG} deg"
        )

    print("\n=== KEY METRICS ===")
    print(f"{'pose':14s} {'side':2s} {'palm_z':>7s} {'normal':>22s} {'margin':>12s} {'cond':>10s} {'rank':>4s} {'sr':>2s} {'down':>5s}")
    for row in rows:
        palm_z = row["palm_center_floor_m"][2]
        normal = np.asarray(row["normal"])
        margin = f"{row['min_margin']['joint']}={row['min_margin']['margin_deg']:.0f}"
        cond = f"{row['cond']:.2e}" if row["cond"] is not None else "inf"
        print(
            f"{row['pose']:14s} {row['side']:2s} {palm_z:7.3f} "
            f"{fmt_vec(normal):>22s} {margin:>12s} {cond:>10s} "
            f"{row['rank']:4d} {row['shoulder_rank']:2d} "
            f"{row['angle_to_down_deg']:5.1f}"
        )

    joint_limits = {
        side: {
            key: [float(lower), float(upper)]
            for key, _empty, _axis, lower, upper in fk_model.JOINTS[side]
        }
        for side in ("L", "R")
    }
    results = {
        "meta": {
            "stage": "2A",
            "scope": "kinematics, limits, singular values only; no forces, no collisions, no safety classification",
            "generated_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
            "source_file": gt.get("source_file"),
            "source_sha256": gt.get("source_sha256"),
            "blender_version": gt.get("blender_version"),
            "fk_model": Path(fk_model.__file__).name,
            "floor_origin_m": vec3(fk_model.FLOOR_ORIGIN),
            "base_origin_height_m": float(fk_model.BASE_ORIGIN_HEIGHT_M),
            "joint_limits_deg": joint_limits,
            "validation_tolerance": {
                "position_mm": POS_TOL_MM,
                "rotation_deg": ROT_TOL_DEG,
            },
        },
        "validation": {
            "poses": len(pose_defs),
            "rows": len(rows),
            "max_position_error_mm": max_pos,
            "max_rotation_error_deg": max_rot,
            "passed": True,
        },
        "poses": rows,
    }
    JSON_PATH.write_text(json.dumps(results, indent=2, ensure_ascii=False), encoding="utf-8")

    csv_fields = [
        "pose", "side", "lift_mm", "q_deg",
        "palm_x_m", "palm_y_m", "palm_z_m",
        "face_x_m", "face_y_m", "face_z_m",
        "normal_x", "normal_y", "normal_z",
        "finger_x", "finger_y", "finger_z",
        "angle_to_down_deg", "min_margin_joint", "min_margin_deg",
        "s_min", "s_max", "cond", "rank", "face_cond",
        "shoulder_rank", "shoulder_s_min",
        "gt_pos_err_mm", "gt_rot_err_deg", "within_limits", "notes",
    ]
    with CSV_PATH.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=csv_fields)
        writer.writeheader()
        for row in rows:
            palm = row["palm_center_floor_m"]
            face = row["contact_face_center_floor_m"]
            normal = row["normal"]
            finger = row["finger"]
            writer.writerow(
                {
                    "pose": row["pose"],
                    "side": row["side"],
                    "lift_mm": f"{row['lift_mm']:.1f}",
                    "q_deg": json.dumps(row["q_deg"], ensure_ascii=False, separators=(",", ":")),
                    "palm_x_m": f"{palm[0]:.6f}",
                    "palm_y_m": f"{palm[1]:.6f}",
                    "palm_z_m": f"{palm[2]:.6f}",
                    "face_x_m": f"{face[0]:.6f}",
                    "face_y_m": f"{face[1]:.6f}",
                    "face_z_m": f"{face[2]:.6f}",
                    "normal_x": f"{normal[0]:.6f}",
                    "normal_y": f"{normal[1]:.6f}",
                    "normal_z": f"{normal[2]:.6f}",
                    "finger_x": f"{finger[0]:.6f}",
                    "finger_y": f"{finger[1]:.6f}",
                    "finger_z": f"{finger[2]:.6f}",
                    "angle_to_down_deg": f"{row['angle_to_down_deg']:.4f}",
                    "min_margin_joint": row["min_margin"]["joint"],
                    "min_margin_deg": f"{row['min_margin']['margin_deg']:.4f}",
                    "s_min": f"{row['s_min']:.6e}",
                    "s_max": f"{row['s_max']:.6e}",
                    "cond": "" if row["cond"] is None else f"{row['cond']:.6e}",
                    "rank": row["rank"],
                    "face_cond": "" if row["face_cond"] is None else f"{row['face_cond']:.6e}",
                    "shoulder_rank": row["shoulder_rank"],
                    "shoulder_s_min": f"{min(row['shoulder_s']):.6e}",
                    "gt_pos_err_mm": f"{row['gt_position_error_mm']:.6f}",
                    "gt_rot_err_deg": f"{row['gt_rotation_error_deg']:.6f}",
                    "within_limits": row["within_limits"],
                    "notes": row["notes"],
                }
            )

    md_lines = [
        "# Stage 2A control poses",
        "",
        "Coordinates relative to floor. `palm_z` is the palm-origin height; the contact face is offset along the palm normal.",
        "",
        "| pose | side | lift mm | palm z, m | normal | min margin | s_min | cond | rank | shoulder rank | angle to down | GT err mm/deg | note |",
        "|---|---:|---:|---:|---|---:|---:|---:|---:|---:|---:|---:|---|",
    ]
    for row in rows:
        normal = np.asarray(row["normal"])
        cond = f"{row['cond']:.2e}" if row["cond"] is not None else "inf"
        md_lines.append(
            f"| {row['pose']} | {row['side']} | {row['lift_mm']:.0f} | "
            f"{row['palm_center_floor_m'][2]:.3f} | {fmt_vec(normal)} | "
            f"{row['min_margin']['joint']}={row['min_margin']['margin_deg']:.0f}° | "
            f"{row['s_min']:.2e} | {cond} | {row['rank']} | {row['shoulder_rank']} | "
            f"{row['angle_to_down_deg']:.1f}° | "
            f"{row['gt_position_error_mm']:.3f}/{row['gt_rotation_error_deg']:.4f} | "
            f"{row['notes']} |"
        )
    MD_PATH.write_text("\n".join(md_lines) + "\n", encoding="utf-8")

    print(f"\nWROTE {JSON_PATH}")
    print(f"WROTE {CSV_PATH}")
    print(f"WROTE {MD_PATH}")


if __name__ == "__main__":
    main()
