#!/usr/bin/env python3
"""Stage 2B.1 — P5 on the far side of the bed.

Variant E (approach from the opposite side)
    If the robot drives around the bed, the far-side point becomes a near-edge
    point.  In the robot frame the bed then spans the same [x_near, x_far]
    interval, and the P5 point sits at x_near + 0.12 -- the same geometry as
    P4.  Two palm orientations are tested, because the geometry alone does not
    decide the hand orientation:

    ``local``      finger = +X, i.e. the palm points away from the robot across
                   the bed (the natural convention of every other pose);
    ``world``      finger = -X, which preserves the *world* direction of the
                   original P5 finger after the 180 deg drive-around.

Variant F (longer manipulator) is evaluated parametrically only: the script
searches along the line from the P5 point toward the shoulder for the closest
point that is solvable in 6D, and reports the required arm extension.  The
Blender model is not modified.

Read-only relative to the .blend and to the stage 2A/2B sources.  Writes one
JSON next to this file.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
import time
import zlib
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
BASE = HERE.parents[1]
for _sub in ("stage2B", "stage2A", "stage2A_ik"):
    sys.path.insert(0, str(BASE / _sub))

import motion_study as ms  # noqa: E402
import ik_study as ik  # noqa: E402
import bed_study as bed  # noqa: E402

CLEARANCES = (0.05, 0.10, 0.12, 0.15, 0.20)
FINGERS = {"local": bed.PLUS_X, "world": -bed.PLUS_X}


def make_target(side: str, x: float, y: float, z: float, finger, geom: dict) -> dict:
    rotation = bed.orientation_from_normal_finger(side, bed.DOWN, finger)
    return {
        "position_floor": [float(x), float(y), float(z)],
        "normal": [float(value) for value in bed.normalize(bed.DOWN)],
        "finger": [float(value) for value in finger],
        "rotation": rotation.tolist(),
        "bed": geom,
        "source": "stage2b1_p5",
    }


def quick_solutions(side: str, target: dict) -> list[dict]:
    starts = [value for value in bed.seed_poses(side).values()]
    starts.append({key: 0.0 for key in ik.JOINT_KEYS})
    results = []
    for start in starts:
        result = ik.solve_once(side, target, "6d", ms.LIFT_M, ik.q_array(side, start), max_iter=220)
        if result.get("converged"):
            results.append(result)
    return results


def best_solution(side: str, target: dict, geom: dict) -> dict | None:
    candidates = quick_solutions(side, target)
    if not candidates:
        return None
    candidates.sort(key=ik.solution_sort_key)
    best = None
    for candidate in candidates:
        clearances = bed.collision_clearances(side, candidate["q_deg"], ms.LIFT_M, geom, other_q=ms.P0_Q)
        metrics = ik.task_metrics(side, candidate["q_deg"], ms.LIFT_M, target, "6d")
        row = {
            "q_deg": candidate["q_deg"],
            "position_error_mm": float(np.linalg.norm(metrics["position_error"]) * 1000.0),
            "orientation_error_deg": float(metrics["orientation_error_deg"]),
            "min_margin_deg": float(metrics["min_margin"]["margin_deg"]),
            "min_margin_joint": metrics["min_margin"]["joint"],
            "condition": float(metrics["condition"]) if metrics["condition"] is not None else None,
            "within_limits": bool(metrics["within_limits"]),
            "arm_bed_m": clearances["arm_bed"],
            "arm_torso_m": clearances["arm_torso"],
            "arm_base_m": clearances["arm_base"],
            "arm_arm_m": clearances["arm_arm"],
            "collision_free": bool(clearances["collision_free"]),
            "usable": bool(
                clearances["collision_free"]
                and metrics["within_limits"]
                and np.linalg.norm(metrics["position_error"]) * 1000.0 <= 1.0
                and metrics["orientation_error_deg"] <= 0.5
            ),
        }
        if row["usable"]:
            return row
        if best is None:
            best = row
    return best


def full_static(side: str, target: dict, geom: dict, seed_text: str) -> dict:
    seed = int(zlib.crc32(seed_text.encode("utf-8")))
    result = ms.static_solve(side, target, seed)
    if not result.get("reachable"):
        # Keep the residual of the best attempt: it separates "the point is out of
        # reach" from "the point is inside the workspace but the palm orientation
        # is infeasible", which is the whole point of the P5 palm-orientation check.
        best = result.get("best") or {}
        clearances = result.get("clearances") or {}
        return {
            "reachable": False,
            "reason": "no_converged_solution",
            "diagnostics": {
                "converged": bool(best.get("converged")),
                "position_error_mm": best.get("position_error_mm"),
                "orientation_error_deg": best.get("orientation_error_deg"),
                "normal_error_deg": best.get("normal_error_deg"),
                "within_limits": best.get("within_limits"),
                "min_margin_deg": best.get("min_margin_deg"),
                "min_margin_joint": best.get("min_margin_joint"),
                "collision_free": clearances.get("collision_free"),
                "minimum_clearance_m": clearances.get("minimum"),
                "converged_solution_count": sum(
                    1 for solution in result.get("solutions", []) if solution.get("converged")
                ),
                "raw_solution_count": result.get("raw_solution_count"),
            },
            "distance_from_shoulder_m": float(
                np.linalg.norm(np.asarray(target["position_floor"], dtype=float) - shoulder_point(side))
            ),
        }
    chosen = ms.pick_reference(result, side, geom)
    if chosen is None:
        return {"reachable": False, "reason": "no_collision_free_solution"}
    metrics = ik.task_metrics(side, chosen["q_deg"], ms.LIFT_M, target, "6d")
    clearances = bed.collision_clearances(side, chosen["q_deg"], ms.LIFT_M, geom, other_q=ms.P0_Q)
    return {
        "reachable": True,
        "q_deg": chosen["q_deg"],
        "position_error_mm": float(np.linalg.norm(metrics["position_error"]) * 1000.0),
        "orientation_error_deg": float(metrics["orientation_error_deg"]),
        "min_margin_deg": float(metrics["min_margin"]["margin_deg"]),
        "min_margin_joint": metrics["min_margin"]["joint"],
        "condition": float(metrics["condition"]) if metrics["condition"] is not None else None,
        "min_clearance_m": clearances["minimum"],
        "collision_free": bool(clearances["collision_free"]),
        "distance_from_shoulder_m": float(
            np.linalg.norm(np.asarray(target["position_floor"], dtype=float) - shoulder_point(side))
        ),
    }


def shoulder_point(side: str) -> np.ndarray:
    return bed.arm_segments(side, ms.P0_Q, ms.LIFT_M)[0]["p0"]


def required_extension(side: str, target_point: np.ndarray, finger, geom: dict, seed_text: str) -> dict:
    shoulder = shoulder_point(side)
    direction = shoulder - target_point
    distance = float(np.linalg.norm(direction))
    if distance < 1e-9:
        return {"required_extension_m": None, "reason": "degenerate"}
    unit = direction / distance

    scan: list[dict] = []

    def ok(offset: float) -> bool:
        point = target_point + unit * offset
        target = make_target(side, point[0], point[1], point[2], finger, geom)
        row = best_solution(side, target, geom)
        usable = bool(row and row["usable"])
        scan.append(
            {
                "offset_m": round(float(offset), 4),
                "floor_position": [round(float(value), 4) for value in point],
                "usable": usable,
                "reason": None if usable else (row or {}).get("reason", "no_usable_solution"),
                "position_error_mm": (row or {}).get("position_error_mm"),
                "orientation_error_deg": (row or {}).get("orientation_error_deg"),
                "min_margin_deg": (row or {}).get("min_margin_deg"),
                "collision_free": (row or {}).get("collision_free"),
            }
        )
        return usable

    if ok(0.0):
        return {
            "required_extension_m": 0.0,
            "search_distance_m": distance,
            "found_at_m": 0.0,
            "scan": scan,
            "method": "6d, first usable point from the original P5 point toward the shoulder",
        }

    # Reachability along the line is NOT monotone (a point closer to the shoulder
    # can sit inside the torso/base exclusion zone), so the far end can be
    # unusable while an intermediate offset is fine.  Scan outward and take the
    # first usable offset instead of gating on the far end.
    low, high = 0.0, distance * 0.9
    steps = 24
    found = None
    previous = 0.0
    for index in range(1, steps + 1):
        offset = high * index / steps
        if ok(offset):
            found = offset
            low = previous
            high = offset
            break
        previous = offset

    if found is None:
        return {
            "required_extension_m": None,
            "search_distance_m": distance,
            "reason": "not_reachable_along_line",
            "scan": scan,
            "method": "6d, first usable point from the original P5 point toward the shoulder",
        }

    for _ in range(8):
        middle = 0.5 * (low + high)
        if ok(middle):
            high = middle
        else:
            low = middle
    return {
        "required_extension_m": high,
        "search_distance_m": distance,
        "found_at_m": high,
        "scan": scan,
        "method": "6d, first usable point from the original P5 point toward the shoulder",
    }


def approach_trajectory(side: str, ready_q: dict, ready_target: dict, target: dict, geom: dict) -> dict:
    seed = int(zlib.crc32(f"p5E|{side}|{geom['clearance_m']}".encode("utf-8")))

    segments = []

    joint_samples = []
    steps = 40
    for index in range(steps + 1):
        fraction = index / steps
        q = {
            key: ms.P0_Q[key] + (ready_q[key] - ms.P0_Q[key]) * fraction
            for key in ms.JOINT_KEYS
        }
        joint_samples.append({"t": fraction, "q_deg": q})
    joint_summary = []
    previous = None
    for sample in joint_samples:
        metrics = ik.task_metrics(side, sample["q_deg"], ms.LIFT_M, ready_target, "6d")
        clearances = bed.collision_clearances(side, sample["q_deg"], ms.LIFT_M, geom, other_q=ms.P0_Q)
        joint_summary.append(
            {
                "t": sample["t"],
                "q_deg": sample["q_deg"],
                "min_margin_deg": float(metrics["min_margin"]["margin_deg"]),
                "condition": float(metrics["condition"]) if metrics["condition"] is not None else None,
                "clearance_min_m": clearances["minimum"],
                "collision_free": bool(clearances["collision_free"]),
                "joint_step_deg": float(ms.q_distance(side, previous, sample["q_deg"])) if previous else 0.0,
            }
        )
        previous = sample["q_deg"]
    segments.append({"name": "P0_joint_to_ready", "samples": joint_summary})

    approach = ms.plan_task_segment(side, ready_target, target, ready_q, geom, ms.P0_Q, seed + 10)
    if approach["status"] == "valid":
        start_position = np.asarray(ready_target["position_floor"], dtype=float)
        end_position = np.asarray(target["position_floor"], dtype=float)
        records = []
        for sample in approach["samples"]:
            t = float(sample["t"])
            position = start_position + (end_position - start_position) * t
            reference = dict(ready_target)
            reference["position_floor"] = [float(value) for value in position]
            metrics = ik.task_metrics(side, sample["q_deg"], ms.LIFT_M, reference, "6d")
            clearances = bed.collision_clearances(side, sample["q_deg"], ms.LIFT_M, geom, other_q=ms.P0_Q)
            records.append(
                {
                    "t": t,
                    "q_deg": sample["q_deg"],
                    "position_error_mm": float(np.linalg.norm(metrics["position_error"]) * 1000.0),
                    "orientation_error_deg": float(metrics["orientation_error_deg"]),
                    "min_margin_deg": float(metrics["min_margin"]["margin_deg"]),
                    "condition": float(metrics["condition"]) if metrics["condition"] is not None else None,
                    "clearance_min_m": clearances["minimum"],
                    "collision_free": bool(clearances["collision_free"]),
                }
            )
        segments.append(
            {
                "name": "ready_to_target",
                "status": approach["status"],
                "sample_count": len(records),
                "max_position_deviation_mm": max(record["position_error_mm"] for record in records),
                "max_orientation_error_deg": max(record["orientation_error_deg"] for record in records),
                "min_margin_deg": min(record["min_margin_deg"] for record in records),
                "max_condition": max(record["condition"] for record in records if record["condition"] is not None),
                "min_clearance_m": min(record["clearance_min_m"] for record in records),
                "all_collision_free": all(record["collision_free"] for record in records),
                "samples": records,
            }
        )
    else:
        segments.append({"name": "ready_to_target", "status": approach["status"], "reason": approach.get("reason")})

    return {
        "segments": segments,
        "joint_segment_min_margin_deg": min(sample["min_margin_deg"] for sample in joint_summary),
        "joint_segment_all_collision_free": all(sample["collision_free"] for sample in joint_summary),
        "joint_segment_max_step_deg": max(sample["joint_step_deg"] for sample in joint_summary),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--height", type=float, default=0.7)
    parser.add_argument("--width", type=float, default=1.0)
    args = parser.parse_args()

    ms.MODE = "6d"
    started = time.time()
    payload = {
        "metadata": {
            "stage": "2B.1",
            "kind": "p5_far_side",
            "created_utc": datetime.now(timezone.utc).isoformat(),
            "mode": "6d",
            "approach": "lateral",
            "height_m": args.height,
            "width_m": args.width,
            "clearances_m": list(CLEARANCES),
            "script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            "git_head": subprocess.run(
                ["git", "-C", str(BASE), "rev-parse", "HEAD"], capture_output=True, text=True, check=False
            ).stdout.strip() or None,
            "definitions": {
                "original": "P5 as published: x_far - 0.12, finger +X",
                "variant_E_local": "approach from the opposite side, finger +X (palm points across the bed)",
                "variant_E_world": "approach from the opposite side, finger -X (keeps the world direction of the old finger)",
                "required_extension_m": "arm length that would be needed for the original P5 point with the original orientation",
            },
        },
        "original": {},
        "variant_E": {},
        "variant_F": {},
        "variant_E_trajectory": None,
    }

    for clearance in CLEARANCES:
        geom = bed.bed_geometry("lateral", args.height, args.width, clearance)
        x_near = float(geom["x_near"])
        x_far = float(geom["x_far"])
        height = float(geom["height_m"])
        z_target = height + bed.APPROACH_Z_M

        payload["original"][f"{clearance:.2f}"] = {}
        payload["variant_F"][f"{clearance:.2f}"] = {}
        for side in ("L", "R"):
            sign = 1.0 if side == "L" else -1.0
            target = make_target(side, x_far - 0.12, sign * 0.25, z_target, bed.PLUS_X, geom)
            payload["original"][f"{clearance:.2f}"][side] = full_static(
                side, target, geom, f"p5|{side}|{clearance:.2f}"
            )
            extension = required_extension(
                side,
                np.asarray(target["position_floor"], dtype=float),
                bed.PLUS_X,
                geom,
                f"p5F|{side}|{clearance:.2f}",
            )
            payload["variant_F"][f"{clearance:.2f}"][side] = extension
            print(
                f"original P5 {side} c={clearance:.2f}: reachable={payload['original'][f'{clearance:.2f}'][side].get('reachable')} "
                f"required extension={extension.get('required_extension_m')}"
            )

        payload["variant_E"][f"{clearance:.2f}"] = {}
        for finger_name, finger in FINGERS.items():
            payload["variant_E"][f"{clearance:.2f}"][finger_name] = {}
            for side in ("L", "R"):
                sign = 1.0 if side == "L" else -1.0
                target = make_target(side, x_near + 0.12, sign * 0.25, z_target, finger, geom)
                payload["variant_E"][f"{clearance:.2f}"][finger_name][side] = full_static(
                    side, target, geom, f"p5E|{finger_name}|{side}|{clearance:.2f}"
                )
                print(
                    f"variant E {finger_name} {side} c={clearance:.2f}: "
                    f"reachable={payload['variant_E'][f'{clearance:.2f}'][finger_name][side].get('reachable')}"
                )

    # trajectory for the most promising variant E configuration
    best = None
    for clearance in CLEARANCES:
        for finger_name in FINGERS:
            for side in ("L", "R"):
                row = payload["variant_E"][f"{clearance:.2f}"][finger_name][side]
                if not row.get("reachable") or not row.get("collision_free"):
                    continue
                score = (finger_name == "local", -clearance, row["min_margin_deg"])
                if best is None or score > best[0]:
                    best = (score, clearance, finger_name, side)

    if best is not None:
        _, clearance, finger_name, side = best
        geom = bed.bed_geometry("lateral", args.height, args.width, clearance)
        sign = 1.0 if side == "L" else -1.0
        x_near = float(geom["x_near"])
        height = float(geom["height_m"])
        finger = FINGERS[finger_name]
        target = make_target(side, x_near + 0.12, sign * 0.25, height + bed.APPROACH_Z_M, finger, geom)
        ready = make_target(side, x_near + 0.12, sign * 0.25, height + bed.APPROACH_Z_M + 0.25, finger, geom)
        ready_row = full_static(side, ready, geom, f"p5E_ready|{side}|{clearance:.2f}")
        if ready_row.get("reachable"):
            payload["variant_E_trajectory"] = {
                "clearance_m": clearance,
                "finger_variant": finger_name,
                "side": side,
                "ready_target": ready,
                "target": target,
                "ready_q_deg": ready_row["q_deg"],
                "result": approach_trajectory(side, ready_row["q_deg"], ready, target, geom),
            }
            print(f"trajectory: c={clearance:.2f} {finger_name} {side} computed")
        else:
            payload["variant_E_trajectory"] = {
                "clearance_m": clearance,
                "finger_variant": finger_name,
                "side": side,
                "status": "ready_pose_unreachable",
            }

    payload["metadata"]["seconds"] = round(time.time() - started, 1)
    out = HERE / "stage2b1_p5.json"
    out.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")

    print("\nclearance | E local L/R | E world L/R | required extension L/R (mm)")
    for clearance in CLEARANCES:
        local = payload["variant_E"][f"{clearance:.2f}"]["local"]
        world = payload["variant_E"][f"{clearance:.2f}"]["world"]
        ext = payload["variant_F"][f"{clearance:.2f}"]
        def mark(row):
            if not row.get("reachable"):
                return "no"
            return "OK" if row.get("collision_free") else "col"
        ext_l = ext["L"].get("required_extension_m")
        ext_r = ext["R"].get("required_extension_m")
        print(
            f"{clearance:9.2f} | {mark(local['L']):>2s}/{mark(local['R']):<2s}      | "
            f"{mark(world['L']):>2s}/{mark(world['R']):<2s}      | "
            f"{ext_l*1000 if ext_l is not None else float('nan'):7.1f}/"
            f"{ext_r*1000 if ext_r is not None else float('nan'):<7.1f}"
        )

    print(f"\nwritten: {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
