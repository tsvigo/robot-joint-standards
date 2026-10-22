#!/usr/bin/env python3
"""Stage 2B.1 — synchronized two-arm 6D check.

Consumes the light path files produced by ``stage2b1_unilateral.py`` and
compares two synchronization schemes at every sample:

``phase``
    Both arms are normalized over their whole operation (this is what
    ``motion_study.plan_bimanual`` does).  Equal phase does not mean equal
    Cartesian speed and does not even mean that both arms are in the same
    motion stage, because the stages have different lengths on each side.
``stage``
    Both arms are aligned by stage index and by the fraction inside that
    stage, so both arms start and finish each of the five stages together.

For every sample the script records joint coordinates, joint-limit margins,
conditioning, clearances, the coarse arm-arm distance, a distal
(forearm/hand) hand-to-hand distance and the palm-to-palm distance.

Writes one JSON next to this file.  Read-only relative to the .blend and to
the stage 2A/2B sources.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
import time
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
import fk_model  # noqa: E402

DISTAL = ("forearm", "hand")


def qdict(q: dict[str, float]) -> dict[str, float]:
    return {key: float(q.get(key, 0.0)) for key in ms.JOINT_KEYS}


def palm_pose(side: str, q: dict[str, float]) -> dict:
    return fk_model.palm_geometry(side, q, ms.LIFT_M)


def self_target(geometry: dict) -> dict:
    return {
        "position_floor": [float(value) for value in geometry["face_floor"]],
        "normal": [float(value) for value in geometry["normal"]],
        "rotation": [[float(value) for value in row] for row in geometry["matrix"]],
    }


def distal_clearance(left_q: dict[str, float], right_q: dict[str, float]) -> float | None:
    left = [s for s in bed.arm_segments("L", left_q, ms.LIFT_M) if s["label"] in DISTAL]
    right = [s for s in bed.arm_segments("R", right_q, ms.LIFT_M) if s["label"] in DISTAL]
    values = []
    for first in left:
        for second in right:
            distance = bed.segment_segment_distance(first["p0"], first["p1"], second["p0"], second["p1"])
            values.append(distance - first["radius"] - second["radius"])
    return min(values) if values else None


def resample_side(side: str, q_list: list[dict], phase: float) -> dict:
    times = np.linspace(0.0, 1.0, len(q_list))
    values = np.asarray([ms.q_array(side, q) for q in q_list], dtype=float)
    clipped = float(np.clip(phase, 0.0, 1.0))
    return ik.q_dict(np.array([np.interp(clipped, times, values[:, index]) for index in range(values.shape[1])]))


def load_path(operation: str, side: str, clearance: float, policy: str) -> dict:
    path = HERE / f"stage2b1_path_{operation}_{side}_c{clearance:.2f}_{policy}.json"
    if not path.exists():
        raise SystemExit(f"missing path file: {path} (run stage2b1_unilateral.py first)")
    return json.loads(path.read_text(encoding="utf-8"))


def group_stages(path: dict) -> list[dict]:
    stages: list[dict] = []
    for sample in path["samples"]:
        if not stages or stages[-1]["stage"] != sample["stage"]:
            stages.append({"stage": sample["stage"], "q": []})
        stages[-1]["q"].append(sample["q_deg"])
    return stages


def build_phase_samples(left_path: dict, right_path: dict, count: int):
    left_q = [sample["q_deg"] for sample in left_path["samples"]]
    right_q = [sample["q_deg"] for sample in right_path["samples"]]
    out = []
    for index in range(count):
        phase = index / (count - 1)
        out.append(
            {
                "phase": phase,
                "mode": "phase",
                "left_q": resample_side("L", left_q, phase),
                "right_q": resample_side("R", right_q, phase),
                "left_stage": left_path["samples"][min(len(left_q) - 1, round(phase * (len(left_q) - 1)))]["stage"],
                "right_stage": right_path["samples"][min(len(right_q) - 1, round(phase * (len(right_q) - 1)))]["stage"],
            }
        )
    return out


def build_stage_samples(left_path: dict, right_path: dict, count: int):
    left_stages = group_stages(left_path)
    right_stages = group_stages(right_path)
    total = min(len(left_stages), len(right_stages))
    out = []
    for index in range(count):
        phase = index / (count - 1)
        position = phase * total
        stage_index = min(total - 1, int(position))
        local = min(1.0, max(0.0, position - stage_index))
        out.append(
            {
                "phase": phase,
                "mode": "stage",
                "left_q": resample_side("L", left_stages[stage_index]["q"], local),
                "right_q": resample_side("R", right_stages[stage_index]["q"], local),
                "left_stage": left_stages[stage_index]["stage"],
                "right_stage": right_stages[stage_index]["stage"],
            }
        )
    return out


def measure(geom: dict, item: dict, previous: dict | None) -> dict:
    left_q = item["left_q"]
    right_q = item["right_q"]
    left_geometry = palm_pose("L", left_q)
    right_geometry = palm_pose("R", right_q)
    left_metrics = ik.task_metrics("L", left_q, ms.LIFT_M, self_target(left_geometry), "6d")
    right_metrics = ik.task_metrics("R", right_q, ms.LIFT_M, self_target(right_geometry), "6d")
    left_clearances = bed.collision_clearances("L", left_q, ms.LIFT_M, geom, other_q=right_q)
    right_clearances = bed.collision_clearances("R", right_q, ms.LIFT_M, geom, other_q=left_q)
    left_palm = np.asarray(left_geometry["face_floor"], dtype=float)
    right_palm = np.asarray(right_geometry["face_floor"], dtype=float)

    record = {
        "phase": item["phase"],
        "mode": item["mode"],
        "left_stage": item["left_stage"],
        "right_stage": item["right_stage"],
        "stage_match": item["left_stage"] == item["right_stage"],
        "left_q_deg": qdict(left_q),
        "right_q_deg": qdict(right_q),
        "left_min_margin_deg": float(left_metrics["min_margin"]["margin_deg"]),
        "right_min_margin_deg": float(right_metrics["min_margin"]["margin_deg"]),
        "left_condition": float(left_metrics["condition"]) if left_metrics["condition"] is not None else None,
        "right_condition": float(right_metrics["condition"]) if right_metrics["condition"] is not None else None,
        "left_within_limits": bool(left_metrics["within_limits"]),
        "right_within_limits": bool(right_metrics["within_limits"]),
        "arm_arm_m": min(value for value in (left_clearances["arm_arm"], right_clearances["arm_arm"]) if value is not None),
        "distal_clearance_m": distal_clearance(left_q, right_q),
        "palm_distance_m": float(np.linalg.norm(left_palm - right_palm)),
        "arm_bed_m": min(value for value in (left_clearances["arm_bed"], right_clearances["arm_bed"]) if value is not None),
        "arm_torso_m": min(
            value for value in (left_clearances["arm_torso"], right_clearances["arm_torso"]) if value is not None
        ),
        "arm_base_m": min(value for value in (left_clearances["arm_base"], right_clearances["arm_base"]) if value is not None),
        "collision_free": bool(left_clearances["collision_free"] and right_clearances["collision_free"]),
        "left_palm": left_palm.tolist(),
        "right_palm": right_palm.tolist(),
    }

    if previous is None:
        record.update(
            {
                "left_joint_step_deg": 0.0,
                "right_joint_step_deg": 0.0,
                "left_speed_m_per_phase": 0.0,
                "right_speed_m_per_phase": 0.0,
            }
        )
    else:
        delta = max(1e-9, item["phase"] - previous["phase"])
        record["left_joint_step_deg"] = float(ms.q_distance("L", previous["left_q_deg"], left_q))
        record["right_joint_step_deg"] = float(ms.q_distance("R", previous["right_q_deg"], right_q))
        record["left_speed_m_per_phase"] = float(
            np.linalg.norm(left_palm - np.asarray(previous["left_palm"], dtype=float)) / delta
        )
        record["right_speed_m_per_phase"] = float(
            np.linalg.norm(right_palm - np.asarray(previous["right_palm"], dtype=float)) / delta
        )
    return record


def summarise(records: list[dict], key: str | None = None) -> dict:
    subset = records if key is None else [record for record in records if record.get("mode") == key]

    def values(field):
        return [record[field] for record in subset if record.get(field) is not None]

    left_speeds = values("left_speed_m_per_phase")[1:]
    right_speeds = values("right_speed_m_per_phase")[1:]
    return {
        "sample_count": len(subset),
        "min_arm_arm_m": min(values("arm_arm_m")) if values("arm_arm_m") else None,
        "min_distal_clearance_m": min(values("distal_clearance_m")) if values("distal_clearance_m") else None,
        "min_palm_distance_m": min(values("palm_distance_m")) if values("palm_distance_m") else None,
        "min_margin_deg": min(values("left_min_margin_deg") + values("right_min_margin_deg")),
        "min_left_margin_deg": min(values("left_min_margin_deg")),
        "min_right_margin_deg": min(values("right_min_margin_deg")),
        "max_condition": max(values("left_condition") + values("right_condition")),
        "min_arm_bed_m": min(values("arm_bed_m")),
        "min_arm_torso_m": min(values("arm_torso_m")),
        "min_arm_base_m": min(values("arm_base_m")),
        "max_left_joint_step_deg": max(values("left_joint_step_deg")),
        "max_right_joint_step_deg": max(values("right_joint_step_deg")),
        "max_left_speed_m_per_phase": max(left_speeds) if left_speeds else None,
        "max_right_speed_m_per_phase": max(right_speeds) if right_speeds else None,
        "speed_max_ratio": (max(left_speeds) / max(right_speeds))
        if left_speeds and right_speeds and max(right_speeds) > 0
        else None,
        "all_collision_free": all(record["collision_free"] for record in subset),
        "all_within_limits": all(
            record["left_within_limits"] and record["right_within_limits"] for record in subset
        ),
        "stage_mismatch_samples": sum(1 for record in subset if not record["stage_match"]),
        "stage_mismatch_fraction": (sum(1 for record in subset if not record["stage_match"]) / len(subset))
        if subset
        else None,
        "left_path_length_m": float(
            sum(
                np.linalg.norm(np.asarray(subset[index]["left_palm"]) - np.asarray(subset[index - 1]["left_palm"]))
                for index in range(1, len(subset))
            )
        ),
        "right_path_length_m": float(
            sum(
                np.linalg.norm(np.asarray(subset[index]["right_palm"]) - np.asarray(subset[index - 1]["right_palm"]))
                for index in range(1, len(subset))
            )
        ),
    }


def per_stage(records: list[dict], mode: str) -> list[dict]:
    rows = []
    subset = [record for record in records if record["mode"] == mode]
    names = []
    for record in subset:
        if record["left_stage"] not in names:
            names.append(record["left_stage"])
    for name in names:
        block = [record for record in subset if record["left_stage"] == name]
        if not block:
            continue
        rows.append(
            {
                "stage": name,
                "sample_count": len(block),
                "min_distal_clearance_m": min(
                    record["distal_clearance_m"] for record in block if record["distal_clearance_m"] is not None
                )
                if any(record["distal_clearance_m"] is not None for record in block)
                else None,
                "min_arm_arm_m": min(record["arm_arm_m"] for record in block if record["arm_arm_m"] is not None),
                "min_palm_distance_m": min(record["palm_distance_m"] for record in block),
                "min_margin_deg": min(
                    min(record["left_min_margin_deg"], record["right_min_margin_deg"]) for record in block
                ),
                "max_condition": max(
                    max(
                        record["left_condition"] or 0.0,
                        record["right_condition"] or 0.0,
                    )
                    for record in block
                ),
                "collision_free": all(record["collision_free"] for record in block),
                "stage_match": all(record["stage_match"] for record in block),
            }
        )
    return rows


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--op", default="P6", choices=("P6", "P7"))
    parser.add_argument("--policy", default="as_is", choices=("as_is", "strict_6d"))
    parser.add_argument("--clearance", type=float, default=0.10)
    parser.add_argument("--height", type=float, default=0.7)
    parser.add_argument("--width", type=float, default=1.0)
    parser.add_argument("--samples", type=int, default=201)
    args = parser.parse_args()

    ms.MODE = "6d"
    geom = bed.bed_geometry("lateral", args.height, args.width, args.clearance)

    started = time.time()
    left_path = load_path(args.op, "L", args.clearance, args.policy)
    right_path = load_path(args.op, "R", args.clearance, args.policy)

    count = max(2, args.samples)
    phase_items = build_phase_samples(left_path, right_path, count)
    stage_items = build_stage_samples(left_path, right_path, count)

    results = {}
    for name, items in (("phase", phase_items), ("stage", stage_items)):
        records = []
        previous = None
        for item in items:
            record = measure(geom, item, previous)
            records.append(record)
            previous = record
        results[name] = records

    tag = f"{args.op}_c{args.clearance:.2f}_{args.policy}"
    output = {
        "metadata": {
            "stage": "2B.1",
            "kind": "bilateral_6d",
            "created_utc": datetime.now(timezone.utc).isoformat(),
            "operation": args.op,
            "policy": args.policy,
            "mode": "6d",
            "approach": "lateral",
            "height_m": args.height,
            "width_m": args.width,
            "clearance_m": args.clearance,
            "samples": count,
            "sync_modes": ["phase", "stage"],
            "script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            "git_head": subprocess.run(
                ["git", "-C", str(BASE), "rev-parse", "HEAD"], capture_output=True, text=True, check=False
            ).stdout.strip() or None,
            "seconds": round(time.time() - started, 1),
            "definitions": {
                "phase": "both arms normalized over the whole operation (pipeline behaviour)",
                "stage": "both arms aligned by stage index and by fraction inside the stage",
                "palm_distance_m": "distance between the two palm face points",
                "distal_clearance_m": "min capsule clearance over forearm/hand pairs, radii subtracted",
                "arm_arm_m": "coarse all-segment arm-arm clearance, may be dominated by proximal links",
            },
        },
        "left_stages": [stage["stage"] for stage in group_stages(left_path)],
        "right_stages": [stage["stage"] for stage in group_stages(right_path)],
        "summary": {name: summarise(results[name]) for name in results},
        "per_stage": {name: per_stage(results[name], name) for name in results},
        "samples": {name: results[name] for name in results},
    }

    out = HERE / f"stage2b1_bilateral_{tag}.json"
    out.write_text(json.dumps(output, indent=2, ensure_ascii=False), encoding="utf-8")

    for name in ("phase", "stage"):
        s = output["summary"][name]
        print(
            f"[{name}] distal={s['min_distal_clearance_m']:.4f} m palm_dist={s['min_palm_distance_m']:.4f} m "
            f"arm_arm={s['min_arm_arm_m']:.4f} m margin={s['min_margin_deg']:.2f} deg cond={s['max_condition']:.2f} "
            f"speed_ratio={s['speed_max_ratio']:.2f} stage_mismatch={s['stage_mismatch_fraction']*100:.0f}% "
            f"collision_free={s['all_collision_free']}"
        )
        print(
            f"        path L/R={s['left_path_length_m']:.3f}/{s['right_path_length_m']:.3f} m "
            f"steps L/R={s['max_left_joint_step_deg']:.2f}/{s['max_right_joint_step_deg']:.2f} deg"
        )
    print(f"written: {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
