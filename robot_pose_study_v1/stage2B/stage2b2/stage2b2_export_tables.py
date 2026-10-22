#!/usr/bin/env python3
"""Export the stage 2B.2 control tables (P7 R, c = 0.05 m) to CSV.

Reads ``stage2b2_control_p7r.json`` produced by ``stage2b2_control_p7r.py``.
Writes only into this directory.

Tables
------
``stage2b2_p7r_continuity.csv``
    One row per planned transition and policy: joint-step statistics, the
    largest step, the step between the last two samples (endpoint jump) and
    the step across the transition boundary.
``stage2b2_p7r_manifold.csv``
    One row per static IK solution of the terminal pose S5_ready.
"""

from __future__ import annotations

import csv
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
SOURCE = HERE / "stage2b2_control_p7r.json"
CONTINUITY = HERE / "stage2b2_p7r_continuity.csv"
MANIFOLD = HERE / "stage2b2_p7r_manifold.csv"
SELF_MOTION = HERE / "stage2b2_p7r_selfmotion.csv"


def main() -> int:
    payload = json.loads(SOURCE.read_text(encoding="utf-8"))

    boundary = {}
    for policy, data in payload["continuity"].items():
        boundary[policy] = {item["at"]: item["step_deg"] for item in data["boundary_steps"]}

    rows = []
    for policy in ("as_is", "strict_6d"):
        data = payload["continuity"][policy]
        for index, transition in enumerate(data["transitions"]):
            steps = transition["steps"]
            largest = max(steps, key=lambda item: item["step_deg"]) if steps else {"step_deg": 0.0, "joint": "", "t": 0.0}
            jump = transition["endpoint_jump"] or {}
            rows.append(
                {
                    "policy": policy,
                    "from": transition["from"],
                    "to": transition["to"],
                    "method": transition["method"],
                    "samples": transition["sample_count"],
                    "max_step_deg": round(transition["max_step_deg"], 3),
                    "max_step_joint": largest["joint"],
                    "max_step_t": round(largest["t"], 4),
                    "median_step_deg": round(transition["median_step_deg"], 3),
                    "p95_step_deg": round(transition["p95_step_deg"], 3),
                    "endpoint_jump_deg": round(jump.get("max_step_deg", 0.0), 3),
                    "endpoint_jump_joint": jump.get("joint", ""),
                    "endpoint_jump_euclid_deg": round(jump.get("euclidean_deg", 0.0), 3),
                    "endpoint_margin_before_deg": round(jump.get("margin_before_deg", float("nan")), 3),
                    "endpoint_margin_after_deg": round(jump.get("margin_after_deg", float("nan")), 3),
                    "boundary_step_deg": round(
                        boundary[policy].get(transition["to"], 0.0) if index < len(data["transitions"]) - 1 else 0.0,
                        3,
                    ),
                }
            )

    with CONTINUITY.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    manifold_rows = []
    for row in payload["terminal_pose_manifold"]["solutions"]:
        manifold_rows.append(
            {
                "index": row["index"],
                "min_margin_deg": round(row["min_margin_deg"], 3),
                "min_margin_joint": row["min_margin_joint"],
                "condition": round(row["condition"], 3),
                "clearance_min_m": round(row["clearance_min_m"], 4),
                "distance_to_strict_end_deg": round(row["distance_to_strict_end_deg"], 3),
                "distance_to_as_is_end_deg": round(row["distance_to_as_is_end_deg"], 3),
                "q_sh3_deg": round(row["q_deg"]["sh3"], 3),
                "q_sh2_deg": round(row["q_deg"]["sh2"], 3),
                "q_sh1_deg": round(row["q_deg"]["sh1"], 3),
                "q_el_deg": round(row["q_deg"]["el"], 3),
                "q_rad_deg": round(row["q_deg"]["rad"], 3),
                "q_w1_deg": round(row["q_deg"]["w1"], 3),
                "q_w2_deg": round(row["q_deg"]["w2"], 3),
            }
        )
    with MANIFOLD.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(manifold_rows[0]))
        writer.writeheader()
        writer.writerows(manifold_rows)

    self_motion_rows = []
    for name in ("self_motion_terminal", "self_motion_manifold"):
        item = payload[name]["path"] if name == "self_motion_manifold" else payload[name]
        for row in item.get("samples", []):
            self_motion_rows.append(
                {
                    "path": name,
                    "step": row["t"],
                    "min_margin_deg": round(row["min_margin_deg"], 4),
                    "min_margin_joint": row["min_margin_joint"],
                    "position_error_mm": round(row["position_error_mm"], 6),
                    "orientation_error_deg": round(row["orientation_error_deg"], 6),
                    "clearance_min_m": round(row["clearance_min_m"], 5),
                    "condition": round(row["condition"], 4),
                    **{f"q_{key}_deg": round(row["q_deg"][key], 4) for key in ("sh3", "sh2", "sh1", "el", "rad", "w1", "w2")},
                }
            )
    with SELF_MOTION.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(self_motion_rows[0]))
        writer.writeheader()
        writer.writerows(self_motion_rows)

    print(f"written: {CONTINUITY} ({len(rows)} rows)")
    print(f"written: {MANIFOLD} ({len(manifold_rows)} rows)")
    print(f"written: {SELF_MOTION} ({len(self_motion_rows)} rows)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
