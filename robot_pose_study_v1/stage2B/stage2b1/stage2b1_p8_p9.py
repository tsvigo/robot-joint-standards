#!/usr/bin/env python3
"""Stage 2B.1 — P8 (two contact zones at once) and P9 (feeding zone).

P8
    Two geometric zones, shoulder and pelvis, reached *simultaneously* by the
    two arms.  Both assignments (L-shoulder / L-pelvis) are solved in 6D, the
    static pair is checked for mutual clearance, and a synchronized joint-space
    approach from a ready pose to the contact zones is sampled so that both
    arms move at the same time.

P9
    Bringing an object to the feeding zone, in 5D and in 6D.  The object is not
    modelled; collisions with the patient are NOT checked and are reported as
    unverified.

Read-only relative to the .blend and to the stage 2A/2B sources.
"""

from __future__ import annotations

import argparse
import contextlib
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

DISTAL = ("forearm", "hand")


@contextlib.contextmanager
def mode_scope(mode: str):
    """`motion_study.static_solve` reads the module-level MODE, so pin it here."""
    previous = ms.MODE
    ms.MODE = mode
    try:
        yield mode
    finally:
        ms.MODE = previous


def target(side: str, x: float, y: float, z: float, finger, geom: dict) -> dict:
    rotation = bed.orientation_from_normal_finger(side, bed.DOWN, finger)
    return {
        "position_floor": [float(x), float(y), float(z)],
        "normal": [float(value) for value in bed.normalize(bed.DOWN)],
        "finger": [float(value) for value in finger],
        "rotation": rotation.tolist(),
        "bed": geom,
        "source": "stage2b1_p8_p9",
    }


def state_row(side: str, point: dict, geom: dict, mode: str, seed_text: str) -> dict:
    seed = int(zlib.crc32(seed_text.encode("utf-8")))
    with mode_scope(mode):
        result = ms.static_solve(side, point, seed)
    if not result.get("reachable"):
        best = result.get("best") or {}
        clearances = result.get("clearances") or {}
        return {
            "reachable": False,
            "mode": mode,
            "reason": "no_converged_solution",
            "diagnostics": {
                "converged": bool(best.get("converged")),
                "position_error_mm": best.get("position_error_mm"),
                "orientation_error_deg": best.get("orientation_error_deg"),
                "normal_error_deg": best.get("normal_error_deg"),
                "within_limits": best.get("within_limits"),
                "min_margin_deg": best.get("min_margin_deg"),
                "collision_free": clearances.get("collision_free"),
                "minimum_clearance_m": clearances.get("minimum"),
                "converged_solution_count": sum(
                    1 for solution in result.get("solutions", []) if solution.get("converged")
                ),
                "raw_solution_count": result.get("raw_solution_count"),
            },
        }
    chosen = ms.pick_reference(result, side, geom)
    if chosen is None:
        return {"reachable": False, "mode": mode, "reason": "no_collision_free_solution"}
    metrics = ik.task_metrics(side, chosen["q_deg"], ms.LIFT_M, point, mode)
    clearances = bed.collision_clearances(side, chosen["q_deg"], ms.LIFT_M, geom, other_q=ms.P0_Q)
    return {
        "reachable": True,
        "mode": mode,
        "q_deg": chosen["q_deg"],
        "position_error_mm": float(np.linalg.norm(metrics["position_error"]) * 1000.0),
        "orientation_error_deg": float(metrics["orientation_error_deg"]),
        "normal_error_deg": float(metrics["normal_error_deg"]),
        "min_margin_deg": float(metrics["min_margin"]["margin_deg"]),
        "min_margin_joint": metrics["min_margin"]["joint"],
        "condition": float(metrics["condition"]) if metrics["condition"] is not None else None,
        "min_clearance_m": clearances["minimum"],
        "collision_free": bool(clearances["collision_free"]),
        "converged_solution_count": sum(
            1 for solution in result.get("solutions", []) if solution.get("converged")
        ),
    }


def distal_clearance(left_q: dict, right_q: dict) -> float:
    return distal_pair(left_q, right_q)[0]


def distal_pair(left_q: dict, right_q: dict) -> tuple[float, str]:
    left = [s for s in bed.arm_segments("L", left_q, ms.LIFT_M) if s["label"] in DISTAL]
    right = [s for s in bed.arm_segments("R", right_q, ms.LIFT_M) if s["label"] in DISTAL]
    best = (float("inf"), "")
    for first in left:
        for second in right:
            distance = bed.segment_segment_distance(first["p0"], first["p1"], second["p0"], second["p1"])
            value = distance - first["radius"] - second["radius"]
            if value < best[0]:
                best = (value, f"L:{first['label']}|R:{second['label']}")
    return best


def arm_arm_pair(left_q: dict, right_q: dict) -> tuple[float, str]:
    left = bed.arm_segments("L", left_q, ms.LIFT_M)
    right = bed.arm_segments("R", right_q, ms.LIFT_M)
    best = (float("inf"), "")
    for first in left:
        for second in right:
            distance = bed.segment_segment_distance(first["p0"], first["p1"], second["p0"], second["p1"])
            value = distance - first["radius"] - second["radius"]
            if value < best[0]:
                best = (value, f"L:{first['label']}|R:{second['label']}")
    return best


def pair_clearance(left_q: dict, right_q: dict, geom: dict) -> dict:
    left = bed.collision_clearances("L", left_q, ms.LIFT_M, geom, other_q=right_q)
    right = bed.collision_clearances("R", right_q, ms.LIFT_M, geom, other_q=left_q)
    distal, distal_label = distal_pair(left_q, right_q)
    arm_arm, arm_arm_label = arm_arm_pair(left_q, right_q)
    return {
        "arm_arm_m": min(value for value in (left["arm_arm"], right["arm_arm"]) if value is not None),
        "arm_arm_pair": arm_arm_label,
        "distal_clearance_m": distal,
        "distal_pair": distal_label,
        "collision_free": bool(left["collision_free"] and right["collision_free"]),
    }


def segment_metrics(side: str, mode: str, samples: list[dict], start_target: dict, end_target: dict, geom: dict) -> dict:
    start_position = np.asarray(start_target["position_floor"], dtype=float)
    end_position = np.asarray(end_target["position_floor"], dtype=float)
    records = []
    for sample in samples:
        t = float(sample["t"])
        reference = dict(start_target)
        reference["position_floor"] = [
            float(value) for value in start_position + (end_position - start_position) * t
        ]
        metrics = ik.task_metrics(side, sample["q_deg"], ms.LIFT_M, reference, mode)
        clearances = bed.collision_clearances(side, sample["q_deg"], ms.LIFT_M, geom, other_q=ms.P0_Q)
        records.append(
            {
                "t": t,
                "q_deg": sample["q_deg"],
                "position_error_mm": float(np.linalg.norm(metrics["position_error"]) * 1000.0),
                "orientation_error_deg": float(metrics["orientation_error_deg"]),
                "normal_error_deg": float(metrics["normal_error_deg"]),
                "min_margin_deg": float(metrics["min_margin"]["margin_deg"]),
                "min_margin_joint": metrics["min_margin"]["joint"],
                "condition": float(metrics["condition"]) if metrics["condition"] is not None else None,
                "min_clearance_m": clearances["minimum"],
                "collision_free": bool(clearances["collision_free"]),
            }
        )
    tightest = min(records, key=lambda record: record["min_margin_deg"])
    return {
        "sample_count": len(records),
        "max_position_deviation_mm": max(record["position_error_mm"] for record in records),
        "max_orientation_error_deg": max(record["orientation_error_deg"] for record in records),
        "max_normal_error_deg": max(record["normal_error_deg"] for record in records),
        "min_margin_deg": tightest["min_margin_deg"],
        "min_margin_joint": tightest["min_margin_joint"],
        "max_condition": max(record["condition"] for record in records if record["condition"] is not None),
        "min_clearance_m": min(record["min_clearance_m"] for record in records),
        "all_collision_free": all(record["collision_free"] for record in records),
        "endpoint_position_error_mm": records[-1]["position_error_mm"],
        "endpoint_orientation_error_deg": records[-1]["orientation_error_deg"],
        "samples": records,
    }


def plan_segment(side: str, mode: str, start: dict, end: dict, q_start: dict, geom: dict, seed_text: str) -> dict:
    seed = int(zlib.crc32(seed_text.encode("utf-8")))
    with mode_scope(mode):
        primary = ms.plan_task_segment(side, start, end, q_start, geom, ms.P0_Q, seed + 10)
        selected = primary
        method = "task_space"
        if primary["status"] != "valid":
            via = ms.plan_via_segment(side, start, end, q_start, geom, ms.P0_Q, seed + 20)
            if via["status"] == "valid":
                selected = via
                method = "via_lift"
            else:
                end_solutions = ms.static_solve(side, end, seed + 30)
                candidates = [solution["q_deg"] for solution in end_solutions.get("solutions", []) if solution.get("converged")]
                if end_solutions.get("reachable") and candidates:
                    lerp = ms.plan_joint_lerp(side, start, end, q_start, candidates[0], geom, ms.P0_Q)
                    if lerp["status"] == "valid":
                        selected = lerp
                        method = "joint_lerp"
    if selected["status"] != "valid":
        return {"status": "not_found", "reason": selected.get("reason"), "method": method}
    summary = segment_metrics(side, mode, selected["samples"], start, end, geom)
    return {"status": "valid", "method": method, "summary": summary}


def shoulder_point(side: str) -> np.ndarray:
    return bed.arm_segments(side, ms.P0_Q, ms.LIFT_M)[0]["p0"]


def p8_zones(geom: dict, width: float) -> dict:
    z = float(geom["z_top"]) + bed.APPROACH_Z_M
    x = float(geom["x_near"]) + 0.35 * width
    return {"shoulder": (x, 0.35, z), "pelvis": (x, -0.35, z)}


def run_p8(geom: dict, width: float) -> dict:
    zones = p8_zones(geom, width)
    out: dict[str, dict] = {}
    for mode in ("5d", "6d"):
        out[mode] = {}
        for assignment in ("L_shoulder", "L_pelvis"):
            left_zone = "shoulder" if assignment == "L_shoulder" else "pelvis"
            right_zone = "pelvis" if left_zone == "shoulder" else "shoulder"
            left_point = target("L", *zones[left_zone], bed.PLUS_X, geom)
            right_point = target("R", *zones[right_zone], bed.PLUS_X, geom)
            left = state_row("L", left_point, geom, mode, f"p8|{mode}|{assignment}|L|{geom['clearance_m']}")
            right = state_row("R", right_point, geom, mode, f"p8|{mode}|{assignment}|R|{geom['clearance_m']}")
            left["target_distance_from_shoulder_m"] = float(
                np.linalg.norm(np.asarray(left_point["position_floor"], dtype=float) - shoulder_point("L"))
            )
            right["target_distance_from_shoulder_m"] = float(
                np.linalg.norm(np.asarray(right_point["position_floor"], dtype=float) - shoulder_point("R"))
            )
            row = {
                "assignment": assignment,
                "mode": mode,
                "left_zone": left_zone,
                "right_zone": right_zone,
                "left": left,
                "right": right,
                "both_reachable": bool(left.get("reachable") and right.get("reachable")),
            }
            if row["both_reachable"]:
                row["pair"] = pair_clearance(left["q_deg"], right["q_deg"], geom)

                left_ready_point = target("L", zones[left_zone][0], zones[left_zone][1], zones[left_zone][2] + 0.25, bed.PLUS_X, geom)
                right_ready_point = target("R", zones[right_zone][0], zones[right_zone][1], zones[right_zone][2] + 0.25, bed.PLUS_X, geom)
                left_ready = state_row("L", left_ready_point, geom, mode, f"p8ready|{mode}|{assignment}|L|{geom['clearance_m']}")
                right_ready = state_row("R", right_ready_point, geom, mode, f"p8ready|{mode}|{assignment}|R|{geom['clearance_m']}")
                if left_ready.get("reachable") and right_ready.get("reachable"):
                    steps = 40
                    samples = []
                    for index in range(steps + 1):
                        fraction = index / steps
                        left_q = {
                            key: left_ready["q_deg"][key] + (left["q_deg"][key] - left_ready["q_deg"][key]) * fraction
                            for key in ms.JOINT_KEYS
                        }
                        right_q = {
                            key: right_ready["q_deg"][key] + (right["q_deg"][key] - right_ready["q_deg"][key]) * fraction
                            for key in ms.JOINT_KEYS
                        }
                        pair = pair_clearance(left_q, right_q, geom)
                        left_metrics = ik.task_metrics("L", left_q, ms.LIFT_M, left_point, mode)
                        right_metrics = ik.task_metrics("R", right_q, ms.LIFT_M, right_point, mode)
                        samples.append(
                            {
                                "phase": fraction,
                                **pair,
                                "left_position_error_mm": float(np.linalg.norm(left_metrics["position_error"]) * 1000.0),
                                "left_orientation_error_deg": float(left_metrics["orientation_error_deg"]),
                                "left_min_margin_deg": float(left_metrics["min_margin"]["margin_deg"]),
                                "right_position_error_mm": float(np.linalg.norm(right_metrics["position_error"]) * 1000.0),
                                "right_orientation_error_deg": float(right_metrics["orientation_error_deg"]),
                                "right_min_margin_deg": float(right_metrics["min_margin"]["margin_deg"]),
                            }
                        )
                    last = samples[-1]
                    row["synchronized_approach"] = {
                        "method": "joint_lerp_from_ready_pose",
                        "samples": steps + 1,
                        "min_arm_arm_m": min(sample["arm_arm_m"] for sample in samples),
                        "min_distal_clearance_m": min(sample["distal_clearance_m"] for sample in samples),
                        "all_collision_free": all(sample["collision_free"] for sample in samples),
                        "max_position_error_mm": max(
                            max(sample["left_position_error_mm"], sample["right_position_error_mm"]) for sample in samples
                        ),
                        "endpoint_position_error_mm": max(last["left_position_error_mm"], last["right_position_error_mm"]),
                        "endpoint_orientation_error_deg": max(
                            last["left_orientation_error_deg"], last["right_orientation_error_deg"]
                        ),
                        "min_margin_deg": min(
                            min(sample["left_min_margin_deg"], sample["right_min_margin_deg"]) for sample in samples
                        ),
                        "cartesian_ok": bool(
                            max(last["left_position_error_mm"], last["right_position_error_mm"]) <= 1.0
                            and max(last["left_orientation_error_deg"], last["right_orientation_error_deg"]) <= 0.5
                        ),
                        "trajectory": samples,
                    }
                else:
                    row["synchronized_approach"] = {
                        "status": "ready_pose_unreachable",
                        "left": left_ready,
                        "right": right_ready,
                    }
            out[mode][assignment] = row
    return out


def p9_ready_candidates(x_feed: float, sign: float, z_feed: float, z_ready: float) -> dict:
    """Ready-pose candidates for P9.

    P9 has no ready pose in `operation_sequence`, so the first candidate copies
    the P6/P7 convention (0.185 m back, y = 0.25, z = table + 0.25).  The other
    two are higher/lower fallbacks.  Which one was used is recorded in the JSON.
    """
    return {
        "study_convention": (x_feed - 0.185, sign * 0.25, z_ready),
        "behind_above": (x_feed - 0.20, sign * 0.25, z_feed + 0.20),
        "close_high": (x_feed - 0.15, sign * 0.20, z_feed + 0.30),
        "far_above": (x_feed - 0.30, sign * 0.15, z_feed + 0.15),
    }


def run_p9(geom: dict, width: float) -> dict:
    x_feed = float(geom["x_near"]) + 0.30 * width
    z_feed = float(geom["height_m"]) + bed.FEEDING_Z_M
    fingers = {"defined_plus_y": bed.PLUS_Y, "alternative_plus_x": bed.PLUS_X}
    out = {}
    for mode in ("5d", "6d"):
        ms.MODE = mode
        out[mode] = {}
        for side in ("L", "R"):
            sign = 1.0 if side == "L" else -1.0
            attempts = []
            accepted = None
            for finger_name, finger in fingers.items():
                z_ready = float(geom["height_m"]) + 0.25
                for ready_label, (rx, ry, rz) in p9_ready_candidates(x_feed, sign, z_feed, z_ready).items():
                    points = {
                        "S0_ready": target(side, rx, ry, rz, finger, geom),
                        "S1_above": target(side, x_feed, sign * 0.05, z_feed + 0.10, finger, geom),
                        "S2_feed": target(side, x_feed, sign * 0.05, z_feed, finger, geom),
                    }
                    states = {}
                    for label, point in points.items():
                        states[label] = state_row(
                            side, point, geom, mode, f"p9|{mode}|{side}|{finger_name}|{ready_label}|{label}|{geom['clearance_m']}"
                        )
                    unreachable = [label for label, state in states.items() if not state.get("reachable")]
                    if unreachable:
                        attempts.append(
                            {
                                "finger": finger_name,
                                "ready_point": ready_label,
                                "status": "static_state_unreachable",
                                "unreachable_states": unreachable,
                            }
                        )
                        continue
                    row = {
                        "finger": finger_name,
                        "finger_vector": [float(value) for value in finger],
                        "ready_point": ready_label,
                        "points": {label: point["position_floor"] for label, point in points.items()},
                        "states": states,
                        "segments": [],
                    }
                    q_current = states["S0_ready"]["q_deg"]
                    failed = False
                    for first, second in (("S0_ready", "S1_above"), ("S1_above", "S2_feed")):
                        segment = plan_segment(
                            side, mode, points[first], points[second], q_current, geom,
                            f"p9|{mode}|{side}|{finger_name}|{ready_label}|{first}|{second}|{geom['clearance_m']}",
                        )
                        row["segments"].append({"from": first, "to": second, **segment})
                        if segment["status"] != "valid":
                            attempts.append(
                                {
                                    "finger": finger_name,
                                    "ready_point": ready_label,
                                    "status": "planning_failed",
                                    "reason": segment.get("reason"),
                                    "failed_segment": f"{first}->{second}",
                                }
                            )
                            failed = True
                            break
                        q_current = segment["summary"]["samples"][-1]["q_deg"]
                    if failed:
                        continue
                    accepted = row
                    break
                if accepted is not None:
                    break
            if accepted is None:
                accepted = {"attempts": attempts, "segments": [], "status": "not_found"}
            else:
                accepted["attempts"] = attempts
            out[mode][side] = accepted
    ms.MODE = "6d"
    return out


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--height", type=float, default=0.7)
    parser.add_argument("--width", type=float, default=1.0)
    parser.add_argument("--clearance", type=float, default=0.10)
    parser.add_argument("--only", choices=("all", "p8", "p9"), default="all")
    parser.add_argument("--out", default="stage2b1_p8_p9.json")
    args = parser.parse_args()

    started = time.time()
    geom = bed.bed_geometry("lateral", args.height, args.width, args.clearance)

    p8 = run_p8(geom, args.width) if args.only in ("all", "p8") else {}
    p9 = run_p9(geom, args.width) if args.only in ("all", "p9") else {}

    payload = {
        "metadata": {
            "stage": "2B.1",
            "kind": "p8_p9",
            "created_utc": datetime.now(timezone.utc).isoformat(),
            "approach": "lateral",
            "height_m": args.height,
            "width_m": args.width,
            "clearance_m": args.clearance,
            "script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            "git_head": subprocess.run(
                ["git", "-C", str(BASE), "rev-parse", "HEAD"], capture_output=True, text=True, check=False
            ).stdout.strip() or None,
            "seconds": round(time.time() - started, 1),
            "definitions": {
                "p8": "two geometric zones reached simultaneously by the two arms; no patient, no force",
                "p9": "object brought to the feeding zone; the object is not modelled",
                "p9_patient_collision": "NOT CHECKED — no patient geometry exists",
            },
        },
        "p8": p8,
        "p9": p9,
    }

    out = HERE / args.out
    out.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")

    for mode, assignments in p8.items():
        for assignment, row in assignments.items():
            status = "both" if row["both_reachable"] else "no"
            pair = row.get("pair", {})
            approach = row.get("synchronized_approach", {})
            left_margin = row["left"].get("min_margin_deg", 0.0) or 0.0
            right_margin = row["right"].get("min_margin_deg", 0.0) or 0.0
            print(
                f"P8 {mode} {assignment:10s} both={status:4s} "
                f"ori_L/R={row['left'].get('orientation_error_deg', float('nan')):.2f}/"
                f"{row['right'].get('orientation_error_deg', float('nan')):.2f} "
                f"margin={min(left_margin, right_margin):.2f} "
                f"(L {left_margin:.2f}/R {right_margin:.2f}) "
                f"arm_arm={pair.get('arm_arm_m', float('nan'))*1000:.1f}mm "
                f"distal={pair.get('distal_clearance_m', float('nan'))*1000:.1f}mm "
                f"sync_min_distal={approach.get('min_distal_clearance_m', float('nan'))*1000:.1f}mm "
                f"sync_end_ori={approach.get('endpoint_orientation_error_deg', float('nan')):.2f}deg "
                f"sync_cart_ok={approach.get('cartesian_ok')}"
            )
            for side_name in ("left", "right"):
                state = row[side_name]
                if not state.get("reachable"):
                    diag = state.get("diagnostics", {})
                    print(
                        f"      {side_name}: unreachable ({state.get('reason')}) "
                        f"dist={state.get('target_distance_from_shoulder_m', float('nan')):.3f} m "
                        f"best pos={diag.get('position_error_mm')} mm "
                        f"ori={diag.get('orientation_error_deg')} deg "
                        f"converged={diag.get('converged')} "
                        f"within_limits={diag.get('within_limits')} "
                        f"collision_free={diag.get('collision_free')}"
                    )
    for mode, sides in p9.items():
        for side, row in sides.items():
            segments = row.get("segments", [])
            if not segments:
                print(
                    f"P9 {mode} {side}: not_found; attempts={len(row.get('attempts', []))}"
                )
                for attempt in row.get("attempts", []):
                    print(
                        f"      {attempt['finger']}/{attempt['ready_point']}: {attempt['status']} "
                        f"{attempt.get('unreachable_states') or attempt.get('failed_segment')}"
                    )
                continue
            last = segments[-1]
            if last.get("status") == "valid":
                summary = last["summary"]
                print(
                    f"P9 {mode} {side}: 2 segments finger={row['finger']} ready={row['ready_point']} "
                    f"method={last['method']} ori={summary['max_orientation_error_deg']:.3f} "
                    f"pos_dev={summary['max_position_deviation_mm']:.2f} "
                    f"margin={summary['min_margin_deg']:.2f} ({summary['min_margin_joint']}) "
                    f"cond={summary['max_condition']:.2f} clr={summary['min_clearance_m']*1000:.1f}"
                )
            else:
                print(f"P9 {mode} {side}: {last.get('from')}->{last.get('to')} FAILED {last.get('reason')}")

    print(f"written: {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
