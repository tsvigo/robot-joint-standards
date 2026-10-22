#!/usr/bin/env python3
"""Stage 2B.1 — collect every artifact into compact tables.

Read-only: prints tables to stdout, writes nothing.
"""

from __future__ import annotations

import json
from pathlib import Path

HERE = Path(__file__).resolve().parent


def load(name: str):
    path = HERE / name
    if not path.exists():
        return None
    return json.loads(path.read_text(encoding="utf-8"))


def unilateral_table() -> None:
    print("== UNILATERAL 6D ==")
    header = (
        f"{'file':28s} {'policy':10s} {'status':9s} {'max ori':>8s} {'max pos':>8s} "
        f"{'min mar':>8s} {'max cond':>8s} {'min clr':>8s} {'end ori':>8s}"
    )
    print(header)
    for path in sorted(HERE.glob("stage2b1_unilateral_*.json")):
        data = json.loads(path.read_text(encoding="utf-8"))
        for policy, block in data["policies"].items():
            transitions = block["transitions"]
            valid = [item for item in transitions if item["status"] == "valid"]
            if not valid:
                print(f"{path.stem[:28]:28s} {policy:10s} {block['status']:9s} (no valid transitions)")
                continue
            max_ori = max(item["summary"]["max_orientation_error_deg"] for item in valid)
            max_pos = max(item["summary"]["max_position_deviation_mm"] for item in valid)
            min_mar = min(item["summary"]["min_margin_deg"] for item in valid)
            max_cond = max(item["summary"]["max_condition"] for item in valid)
            min_clr = min(item["summary"]["min_clearance_m"] for item in valid)
            end_ori = max(item["summary"]["endpoint_orientation_error_deg"] for item in valid)
            print(
                f"{path.stem[:28]:28s} {policy:10s} {block['status']:9s} {max_ori:8.3f} {max_pos:8.2f} "
                f"{min_mar:8.2f} {max_cond:8.2f} {min_clr*1000:8.1f} {end_ori:8.4f}"
            )


def bilateral_table() -> None:
    print("\n== BILATERAL 6D ==")
    header = (
        f"{'file':44s} {'sync':6s} {'distal':>8s} {'palm d':>8s} {'armarm':>8s} "
        f"{'margin':>7s} {'cond':>6s} {'ratio':>6s} {'mismatch':>9s} {'free':>5s}"
    )
    print(header)
    for path in sorted(HERE.glob("stage2b1_bilateral_*.json")):
        data = json.loads(path.read_text(encoding="utf-8"))
        for mode, summary in data["summary"].items():
            print(
                f"{path.stem[:44]:44s} {mode:6s} {summary['min_distal_clearance_m']:8.4f} "
                f"{summary['min_palm_distance_m']:8.4f} {summary['min_arm_arm_m']:8.4f} "
                f"{summary['min_margin_deg']:7.2f} {summary['max_condition']:6.2f} "
                f"{summary['speed_max_ratio']:6.2f} {summary['stage_mismatch_fraction']*100:8.0f}% "
                f"{str(summary['all_collision_free']):>5s}"
            )


def static_table() -> None:
    data = load("stage2b1_static_sweep.json")
    if not data:
        return
    print("\n== STATIC SWEEP ==")
    print(f"{'c':>5s} {'op':>3s} {'reach L/R':>10s} {'min margin':>10s} {'max cond':>8s} {'min bed mm':>10s} {'pair armarm mm':>14s}")
    clearances = data["metadata"]["clearances_m"]
    for clearance in clearances:
        for operation in data["metadata"]["operations"]:
            cases = [
                case for case in data["cases"]
                if case["operation"] == operation and abs(case["clearance_m"] - clearance) < 1e-9
            ]
            reach = []
            margins = []
            conditions = []
            beds = []
            for case in cases:
                reach.append(sum(1 for state in case["states"] if state.get("reachable")))
                margins += [state["min_margin_deg"] for state in case["states"] if state.get("reachable")]
                conditions += [state["condition"] for state in case["states"] if state.get("reachable") and state["condition"]]
                beds += [state["arm_bed_m"] for state in case["states"] if state.get("reachable") and state["arm_bed_m"] is not None]
            pairs = [
                item for item in data["simultaneous"]
                if item["operation"] == operation and abs(item["clearance_m"] - clearance) < 1e-9 and item.get("reachable_pair")
            ]
            pair_min = min(item["arm_arm_m"] for item in pairs) if pairs else None
            print(
                f"{clearance:5.2f} {operation:>3s} {reach[0]:4d}/{reach[1]:<4d} {min(margins):10.2f} "
                f"{max(conditions):8.2f} {min(beds)*1000:10.1f} "
                f"{(pair_min*1000 if pair_min is not None else float('nan')):14.1f}"
            )


def p5_table() -> None:
    data = load("stage2b1_p5.json")
    if not data:
        return
    print("\n== P5 ==")
    print(f"{'c':>5s} {'orig L/R':>10s} {'E local L/R':>12s} {'E world L/R':>12s} {'ext L mm':>9s} {'ext R mm':>9s}")
    for clearance in data["metadata"]["clearances_m"]:
        key = f"{clearance:.2f}"

        def mark(row):
            if not row.get("reachable"):
                return "no"
            return "ok" if row.get("collision_free") else "col"

        origin = data["original"][key]
        local = data["variant_E"][key]["local"]
        world = data["variant_E"][key]["world"]
        ext_l = data["variant_F"][key]["L"].get("required_extension_m")
        ext_r = data["variant_F"][key]["R"].get("required_extension_m")
        print(
            f"{clearance:5.2f} {mark(origin['L']):>4s}/{mark(origin['R']):<4s} "
            f"{mark(local['L']):>5s}/{mark(local['R']):<4s} "
            f"{mark(world['L']):>5s}/{mark(world['R']):<4s} "
            f"{(ext_l*1000 if ext_l is not None else float('nan')):9.1f} "
            f"{(ext_r*1000 if ext_r is not None else float('nan')):9.1f}"
        )
    trajectory = data.get("variant_E_trajectory")
    for clearance in data["metadata"]["clearances_m"]:
        key = f"{clearance:.2f}"
        for side in ("L", "R"):
            row = data["variant_F"][key][side]
            if row.get("required_extension_m") is None:
                print(f"  variant F c={clearance:.2f} {side}: {row.get('reason')}")
    if trajectory:
        result = trajectory.get("result")
        if result:
            print(
                f"E trajectory: c={trajectory['clearance_m']:.2f} {trajectory['finger_variant']} {trajectory['side']} "
                f"joint segment margin={result['joint_segment_min_margin_deg']:.2f} deg "
                f"max step={result['joint_segment_max_step_deg']:.2f} deg"
            )
            for segment in result["segments"]:
                if segment["name"] == "ready_to_target" and segment.get("status") == "valid":
                    print(
                        f"  ready->target: n={segment['sample_count']} ori={segment['max_orientation_error_deg']:.3f} "
                        f"pos_dev={segment['max_position_deviation_mm']:.2f} mm margin={segment['min_margin_deg']:.2f} "
                        f"cond={segment['max_condition']:.2f} clr={segment['min_clearance_m']*1000:.1f} mm "
                        f"free={segment['all_collision_free']}"
                    )


def p8_p9_table() -> None:
    data = load("stage2b1_p8_p9.json")
    if not data:
        return
    print("\n== P8 / P9 ==")
    for mode, assignments in data["p8"].items():
        for assignment, row in assignments.items():
            pair = row.get("pair", {})
            approach = row.get("synchronized_approach", {})
            print(
                f"P8 {mode} {assignment:10s} both={str(row['both_reachable']):5s} "
                f"L margin={(row['left'].get('min_margin_deg') or float('nan')):.2f} "
                f"R margin={(row['right'].get('min_margin_deg') or float('nan')):.2f} "
                f"L ori={(row['left'].get('orientation_error_deg') or float('nan')):.2f} "
                f"R ori={(row['right'].get('orientation_error_deg') or float('nan')):.2f} "
                f"arm_arm={(pair.get('arm_arm_m') or float('nan'))*1000:.1f} mm "
                f"distal={(pair.get('distal_clearance_m') or float('nan'))*1000:.1f} mm "
                f"sync distal={(approach.get('min_distal_clearance_m') or float('nan'))*1000:.1f} mm "
                f"sync cart_ok={approach.get('cartesian_ok')}"
            )
    for mode, sides in data["p9"].items():
        for side, row in sides.items():
            segments = row.get("segments", [])
            if not segments:
                print(f"P9 {mode} {side}: {row.get('status')}")
                continue
            last = segments[-1]
            if last.get("status") == "valid":
                summary = last["summary"]
                print(
                    f"P9 {mode} {side}: {len(segments)} segments method={last['method']} "
                    f"ori={summary['max_orientation_error_deg']:.3f} pos_dev={summary['max_position_deviation_mm']:.2f} "
                    f"margin={summary['min_margin_deg']:.2f} cond={summary['max_condition']:.2f} "
                    f"clr={summary['min_clearance_m']*1000:.1f}"
                )
            else:
                print(f"P9 {mode} {side}: FAILED {last.get('from')}->{last.get('to')} {last.get('reason')}")


def clearance_table() -> None:
    """Per-c static 6D summary over both arms and both operations."""
    data = load("stage2b1_static_sweep.json")
    if not data:
        print("\n== BASE DISTANCE DECISION == static sweep missing")
        return
    print("\n== BASE DISTANCE DECISION (static 6D, both arms, P6 and P7) ==")
    print(
        f"{'c':>5s} {'op':>3s} {'reach L/R':>10s} {'min mar':>8s} {'joint':>5s} "
        f"{'max cond':>9s} {'min bed':>8s} {'min pair':>9s}"
    )
    compromise = []
    for clearance in data["metadata"]["clearances_m"]:
        per_operation = {}
        for operation in ("P6", "P7"):
            per_side = []
            for side in ("L", "R"):
                case = next(
                    case for case in data["cases"]
                    if case["operation"] == operation and case["side"] == side
                    and abs(case["clearance_m"] - clearance) < 1e-9
                )
                ok = [state for state in case["states"] if state.get("reachable")]
                per_side.append((side, case, ok))
            reach = "/".join(f"{len(ok)}/{len(case['states'])}" for _, case, ok in per_side)
            states = [state for _, _, ok in per_side for state in ok]
            worst = min(states, key=lambda state: state["min_margin_deg"])
            max_cond = max((state["condition"] for state in states if state.get("condition")), default=0.0)
            min_bed = min((state["arm_bed_m"] for state in states if state.get("arm_bed_m") is not None), default=0.0)
            pairs = [
                item for item in data.get("simultaneous", [])
                if item["operation"] == operation and abs(item["clearance_m"] - clearance) < 1e-9
                and item.get("reachable_pair")
            ]
            pair = min((item["arm_arm_m"] for item in pairs), default=None)
            per_operation[operation] = {
                "margin": worst["min_margin_deg"],
                "condition": max_cond,
            }
            print(
                f"{clearance:5.2f} {operation:>3s} {reach:>10s} {worst['min_margin_deg']:8.2f} "
                f"{worst['min_margin_joint']:>5s} {max_cond:9.2f} {min_bed * 1000:7.1f} "
                f"{(pair * 1000 if pair is not None else float('nan')):9.1f}"
            )
        worst_case = min(per_operation.values(), key=lambda item: item["margin"])
        compromise.append(
            (
                clearance,
                worst_case["margin"],
                max(item["condition"] for item in per_operation.values()),
                per_operation["P6"]["margin"],
                per_operation["P7"]["margin"],
            )
        )
    print("\n  compromise: max over operations of the worst-arm margin")
    for clearance, margin, condition, p6_margin, p7_margin in compromise:
        print(
            f"    c={clearance:.2f}  min(P6,P7)={margin:6.2f}  P6={p6_margin:6.2f}  P7={p7_margin:6.2f}  "
            f"max cond={condition:6.2f}"
        )
    best = max(compromise, key=lambda item: item[1])
    print(f"    best by worst-operation margin: c={best[0]:.2f} ({best[1]:.2f} deg)")


def trajectory_table() -> None:
    """Unilateral and synchronized two-arm trajectory metrics per clearance."""
    print("\n== TRAJECTORIES: UNILATERAL (strict_6d, both arms) ==")
    print(f"{'c':>5s} {'op':>3s} {'valid':>6s} {'max ori':>8s} {'max pos':>8s} {'min mar':>8s} {'max cond':>9s} {'min clr':>8s}")
    for path in sorted(HERE.glob("stage2b1_unilateral_*_c*.json")):
        data = json.loads(path.read_text(encoding="utf-8"))
        block = data["policies"].get("strict_6d")
        if not block:
            continue
        valid = [item for item in block["transitions"] if item["status"] == "valid"]
        clearance = data["metadata"]["clearance_m"]
        operation = data["metadata"]["operation"]
        side = data["metadata"]["side"]
        if not valid:
            print(f"{clearance:5.2f} {operation:>3s}{side:1s}  0/5")
            continue
        print(
            f"{clearance:5.2f} {operation:>3s}{side:1s} {len(valid):5d}/5 "
            f"{max(item['summary']['max_orientation_error_deg'] for item in valid):8.2f} "
            f"{max(item['summary']['max_position_deviation_mm'] for item in valid):8.2f} "
            f"{min(item['summary']['min_margin_deg'] for item in valid):8.2f} "
            f"{max(item['summary']['max_condition'] for item in valid):9.2f} "
            f"{min(item['summary']['min_clearance_m'] for item in valid) * 1000:8.1f}"
        )
    print("\n== TRAJECTORIES: SYNCHRONIZED TWO-ARM (strict_6d, phase scheme) ==")
    print(f"{'c':>5s} {'op':>3s} {'distal':>8s} {'palm':>8s} {'min mar':>8s} {'max cond':>9s} {'max step':>9s} {'mismatch':>9s}")
    for path in sorted(HERE.glob("stage2b1_bilateral_*_c*_strict_6d.json")):
        data = json.loads(path.read_text(encoding="utf-8"))
        samples = data["samples"]["phase"]
        clearance = data["metadata"]["clearance_m"]
        operation = data["metadata"]["operation"]
        print(
            f"{clearance:5.2f} {operation:>3s} "
            f"{min(s['distal_clearance_m'] for s in samples) * 1000:8.1f} "
            f"{min(s['palm_distance_m'] for s in samples) * 1000:8.1f} "
            f"{min(min(s['left_min_margin_deg'], s['right_min_margin_deg']) for s in samples):8.2f} "
            f"{max(max(s['left_condition'] or 0, s['right_condition'] or 0) for s in samples):9.2f} "
            f"{max(max(s['left_joint_step_deg'], s['right_joint_step_deg']) for s in samples):9.2f} "
            f"{100.0 * sum(1 for s in samples if not s['stage_match']) / len(samples):8.1f}%"
        )


def main() -> int:
    unilateral_table()
    bilateral_table()
    static_table()
    clearance_table()
    trajectory_table()
    p5_table()
    p8_p9_table()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
