#!/usr/bin/env python3
"""Stage 2B.1 — extract the configurations that matter for later force analysis.

Read-only with respect to the study inputs: it only reads the artifacts and
writes one JSON with the worst configurations per operation/side/clearance.

Three groups are collected:
  * static          — worst static states (fixed-seed reference branch);
  * trajectory      — worst sample of the planned unilateral motion;
  * synchronized    — worst sample of the synchronized two-arm motion (both
                      joint vectors of the same instant, so force analysis can
                      use them together).
"""

from __future__ import annotations

import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
OUT = HERE / "stage2b1_force_candidates.json"


def load(name: str):
    path = HERE / name
    return json.loads(path.read_text(encoding="utf-8")) if path.exists() else None


def static_candidates(sweep: dict, per_case: int = 2) -> list[dict]:
    out = []
    for case in sweep["cases"]:
        reachable = [state for state in case["states"] if state.get("reachable")]
        if not reachable:
            continue
        worst = sorted(reachable, key=lambda state: state["min_margin_deg"])[:per_case]
        for state in worst:
            out.append(
                {
                    "group": "static",
                    "operation": case["operation"],
                    "side": case["side"],
                    "clearance_m": case["clearance_m"],
                    "label": state["label"],
                    "q_deg": state["q_deg"],
                    "position_floor": state["position_floor"],
                    "min_margin_deg": state["min_margin_deg"],
                    "min_margin_joint": state["min_margin_joint"],
                    "condition": state["condition"],
                    "arm_bed_m": state.get("arm_bed_m"),
                    "arm_torso_m": state.get("arm_torso_m"),
                    "arm_base_m": state.get("arm_base_m"),
                    "collision_free_parked": state.get("collision_free_parked"),
                }
            )
    return out


def trajectory_candidates(per_file: int = 1) -> list[dict]:
    out = []
    for path in sorted(HERE.glob("stage2b1_unilateral_*_c*.json")):
        data = json.loads(path.read_text(encoding="utf-8"))
        block = data["policies"].get("strict_6d")
        if not block:
            continue
        samples = []
        for transition in block["transitions"]:
            if transition["status"] != "valid":
                continue
            for sample in transition["samples"]:
                samples.append((transition, sample))
        if not samples:
            continue
        samples.sort(key=lambda item: item[1]["min_margin_deg"])
        for transition, sample in samples[:per_file]:
            out.append(
                {
                    "group": "trajectory",
                    "operation": data["metadata"]["operation"],
                    "side": data["metadata"]["side"],
                    "clearance_m": data["metadata"]["clearance_m"],
                    "transition": f"{transition['from']}->{transition['to']}",
                    "method": transition["selected_method"],
                    "t": sample["t"],
                    "q_deg": sample["q_deg"],
                    "min_margin_deg": sample["min_margin_deg"],
                    "min_margin_joint": sample["min_margin_joint"],
                    "condition": sample["condition"],
                    "position_error_mm": sample["position_error_mm"],
                    "orientation_error_deg": sample["orientation_error_deg"],
                }
            )
    return out


def synchronized_candidates(per_file: int = 1) -> list[dict]:
    out = []
    for path in sorted(HERE.glob("stage2b1_bilateral_*_c*_strict_6d.json")):
        data = json.loads(path.read_text(encoding="utf-8"))
        samples = data["samples"]["phase"]
        if not samples:
            continue
        ranked = sorted(
            samples,
            key=lambda sample: min(sample["left_min_margin_deg"], sample["right_min_margin_deg"]),
        )
        for sample in ranked[:per_file]:
            out.append(
                {
                    "group": "synchronized",
                    "operation": data["metadata"]["operation"],
                    "clearance_m": data["metadata"]["clearance_m"],
                    "phase": sample["phase"],
                    "left_q_deg": sample["left_q_deg"],
                    "right_q_deg": sample["right_q_deg"],
                    "left_palm": sample["left_palm"],
                    "right_palm": sample["right_palm"],
                    "left_min_margin_deg": sample["left_min_margin_deg"],
                    "right_min_margin_deg": sample["right_min_margin_deg"],
                    "left_condition": sample["left_condition"],
                    "right_condition": sample["right_condition"],
                    "distal_clearance_m": sample["distal_clearance_m"],
                    "palm_distance_m": sample["palm_distance_m"],
                    "arm_bed_m": sample["arm_bed_m"],
                    "collision_free": sample["collision_free"],
                }
            )
    return out


def markdown(entries: list[dict]) -> None:
    print("| группа | операция | рука | c, м | поза / переход | запас, ° | сустав | cond | зазор до кровати, мм |")
    print("|---|---|---|---|---|---|---|---|---|")
    for entry in entries:
        side = entry.get("side") or "L+R"
        if entry.get("label"):
            label = entry["label"]
        elif entry.get("transition"):
            label = f"{entry['transition']} t={entry['t']:.2f}"
        else:
            label = f"phase={entry['phase']:.3f}"
        clearance = entry.get("arm_bed_m")
        margin = entry.get("min_margin_deg")
        if margin is None:
            margin = min(entry["left_min_margin_deg"], entry["right_min_margin_deg"])
        joint = entry.get("min_margin_joint") or "—"
        cond = entry.get("condition")
        if cond is None:
            cond = max(entry["left_condition"] or 0.0, entry["right_condition"] or 0.0)
        print(
            f"| {entry['group']} | {entry['operation']} | {side} | {entry['clearance_m']:.2f} | {label} | "
            f"{margin:.2f} | {joint} | {cond:.2f} | "
            f"{(clearance * 1000 if clearance is not None else float('nan')):.1f} |"
        )


def main() -> int:
    sweep = load("stage2b1_static_sweep.json")
    if not sweep:
        print("static sweep artifact missing")
        return 0
    entries = static_candidates(sweep) + trajectory_candidates() + synchronized_candidates()
    payload = {
        "metadata": {
            "source": "stage2b1_static_sweep.json + stage2b1_unilateral_*_c*.json + stage2b1_bilateral_*_c*_strict_6d.json",
            "note": (
                "Критичные конфигурации для последующих силовых расчётов. "
                "Силовые расчёты в этом этапе не выполнялись."
            ),
        },
        "candidates": entries,
    }
    OUT.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    markdown(entries)
    print(f"\nwritten: {OUT} ({len(entries)} entries)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
