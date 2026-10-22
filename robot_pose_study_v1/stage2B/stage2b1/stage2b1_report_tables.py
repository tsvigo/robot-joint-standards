#!/usr/bin/env python3
"""Stage 2B.1 — render the report tables as Markdown straight from the artifacts.

Read-only.  Prints Markdown to stdout so the final report never contains a
hand-copied number.
"""

from __future__ import annotations

import json
from pathlib import Path

HERE = Path(__file__).resolve().parent


def load(name: str):
    path = HERE / name
    return json.loads(path.read_text(encoding="utf-8")) if path.exists() else None


def unilateral_markdown() -> None:
    print("### Одноручные 6D-траектории (политика strict_6d)\n")
    print("| c, м | операция | рука | valid | max ori, ° | max откл., мм | min запас, ° | max cond | min зазор, мм | max шаг, ° |")
    print("|---|---|---|---|---|---|---|---|---|---|")
    rows = []
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
            rows.append((clearance, operation, side, 0.0, None, None, None, None, None, None))
            continue
        rows.append(
            (
                clearance,
                operation,
                side,
                len(valid),
                max(item["summary"]["max_orientation_error_deg"] for item in valid),
                max(item["summary"]["max_position_deviation_mm"] for item in valid),
                min(item["summary"]["min_margin_deg"] for item in valid),
                max(item["summary"]["max_condition"] for item in valid),
                min(item["summary"]["min_clearance_m"] for item in valid) * 1000.0,
                max(item["summary"]["max_joint_step_deg"] for item in valid),
            )
        )
    for clearance, operation, side, count, ori, pos, margin, cond, clr, step in sorted(rows):
        if ori is None:
            print(f"| {clearance:.2f} | {operation} | {side} | 0/5 | — | — | — | — | — | — |")
        else:
            print(
                f"| {clearance:.2f} | {operation} | {side} | {count}/5 | {ori:.2f} | {pos:.2f} | "
                f"{margin:.2f} | {cond:.2f} | {clr:.1f} | {step:.1f} |"
            )
    print()


def bilateral_markdown() -> None:
    print("### Синхронные двухручные 6D-траектории (политика strict_6d)\n")
    print("| c, м | операция | схема | min зазор кистей, мм | min между кистями, мм | min запас, ° | max cond | max шаг, ° | рассинхрон стадий | столкновения |")
    print("|---|---|---|---|---|---|---|---|---|---|")
    for path in sorted(HERE.glob("stage2b1_bilateral_*_c*_strict_6d.json")):
        data = json.loads(path.read_text(encoding="utf-8"))
        clearance = data["metadata"]["clearance_m"]
        operation = data["metadata"]["operation"]
        for scheme in ("phase", "stage"):
            samples = data["samples"][scheme]
            mismatch = 100.0 * sum(1 for s in samples if not s["stage_match"]) / len(samples)
            print(
                f"| {clearance:.2f} | {operation} | {scheme} | "
                f"{min(s['distal_clearance_m'] for s in samples) * 1000:.1f} | "
                f"{min(s['palm_distance_m'] for s in samples) * 1000:.1f} | "
                f"{min(min(s['left_min_margin_deg'], s['right_min_margin_deg']) for s in samples):.2f} | "
                f"{max(max(s['left_condition'] or 0, s['right_condition'] or 0) for s in samples):.2f} | "
                f"{max(max(s['left_joint_step_deg'], s['right_joint_step_deg']) for s in samples):.2f} | "
                f"{mismatch:.1f}% | {'нет' if all(s['collision_free'] for s in samples) else 'ДА'} |"
            )
    print()


def clearance_markdown() -> None:
    data = load("stage2b1_static_sweep.json")
    if not data:
        return
    print("### Статическая 6D-достижимость и запасы по расстоянию базы\n")
    print("| c, м | операция | достижимо L/R | min запас, ° | сустав | max cond | min зазор до кровати, мм |")
    print("|---|---|---|---|---|---|---|")
    for clearance in data["metadata"]["clearances_m"]:
        for operation in ("P6", "P7"):
            sides = []
            for side in ("L", "R"):
                case = next(
                    case for case in data["cases"]
                    if case["operation"] == operation and case["side"] == side
                    and abs(case["clearance_m"] - clearance) < 1e-9
                )
                sides.append(case)
            reach = "/".join(
                f"{sum(1 for state in case['states'] if state.get('reachable'))}/{len(case['states'])}"
                for case in sides
            )
            states = [state for case in sides for state in case["states"] if state.get("reachable")]
            worst = min(states, key=lambda state: state["min_margin_deg"])
            print(
                f"| {clearance:.2f} | {operation} | {reach} | {worst['min_margin_deg']:.2f} | "
                f"{worst['min_margin_joint']} | "
                f"{max((state['condition'] for state in states if state.get('condition')), default=0.0):.2f} | "
                f"{min((state['arm_bed_m'] for state in states if state.get('arm_bed_m') is not None), default=0.0) * 1000:.1f} |"
            )
    print()


def decision_markdown() -> None:
    """Combined per-c evidence: static reachability, trajectory margins, synchronized work."""
    sweep = load("stage2b1_static_sweep.json")
    if not sweep:
        return
    unilateral = {}
    for path in HERE.glob("stage2b1_unilateral_*_c*.json"):
        data = json.loads(path.read_text(encoding="utf-8"))
        block = data["policies"].get("strict_6d")
        if not block:
            continue
        valid = [item for item in block["transitions"] if item["status"] == "valid"]
        key = (round(data["metadata"]["clearance_m"], 3), data["metadata"]["operation"], data["metadata"]["side"])
        if valid:
            unilateral[key] = {
                "n": len(valid),
                "margin": min(item["summary"]["min_margin_deg"] for item in valid),
                "cond": max(item["summary"]["max_condition"] for item in valid),
                "ori": max(item["summary"]["max_orientation_error_deg"] for item in valid),
                "pos": max(item["summary"]["max_position_deviation_mm"] for item in valid),
            }
        else:
            unilateral[key] = {"n": 0}
    bilateral = {}
    for path in HERE.glob("stage2b1_bilateral_*_c*_strict_6d.json"):
        data = json.loads(path.read_text(encoding="utf-8"))
        samples = data["samples"]["phase"]
        bilateral[(round(data["metadata"]["clearance_m"], 3), data["metadata"]["operation"])] = {
            "margin": min(min(s["left_min_margin_deg"], s["right_min_margin_deg"]) for s in samples),
            "cond": max(max(s["left_condition"] or 0, s["right_condition"] or 0) for s in samples),
            "distal": min(s["distal_clearance_m"] for s in samples) * 1000.0,
            "palm": min(s["palm_distance_m"] for s in samples) * 1000.0,
            "collfree": all(s["collision_free"] for s in samples),
        }
    print("### Матрица выбора расстояния базы (статика + траектории + синхронная работа)\n")
    print(
        "| c, м | P6 стат. | P7 стат. | P6 траект.: запас / cond | P7 траект.: запас / cond | "
        "синхр. P6: запас, ° | синхр. P7: запас, ° / зазор кистей, мм | столкновения |"
    )
    print("|---|---|---|---|---|---|---|---|")
    for clearance in sweep["metadata"]["clearances_m"]:
        reach = {}
        for operation in ("P6", "P7"):
            counts = []
            for side in ("L", "R"):
                case = next(
                    case for case in sweep["cases"]
                    if case["operation"] == operation and case["side"] == side
                    and abs(case["clearance_m"] - clearance) < 1e-9
                )
                counts.append(f"{sum(1 for state in case['states'] if state.get('reachable'))}/{len(case['states'])}")
            reach[operation] = "/".join(counts)

        def traj(operation, side):
            row = unilateral.get((round(clearance, 3), operation, side))
            if not row:
                return "—"
            if row["n"] == 0:
                return "0/5"
            return f"{row['margin']:.2f} / {row['cond']:.1f}"

        def pair(operation):
            row = bilateral.get((round(clearance, 3), operation))
            if not row:
                return "—"
            return f"{row['margin']:.2f}"

        p7pair = bilateral.get((round(clearance, 3), "P7"))
        p7text = (
            f"{p7pair['margin']:.2f} / {p7pair['distal']:.0f}"
            if p7pair else "—"
        )
        collisions = "—"
        rows = [bilateral.get((round(clearance, 3), op)) for op in ("P6", "P7")]
        rows = [row for row in rows if row]
        if rows:
            collisions = "нет" if all(row["collfree"] for row in rows) else "ДА"
        print(
            f"| {clearance:.2f} | {reach['P6']} | {reach['P7']} | "
            f"{traj('P6', 'L')} (L) / {traj('P6', 'R')} (R) | "
            f"{traj('P7', 'L')} (L) / {traj('P7', 'R')} (R) | "
            f"{pair('P6')} | {p7text} | {collisions} |"
        )
    print()


def p5_markdown() -> None:
    data = load("stage2b1_p5.json")
    if not data:
        return
    print("### P5: исходная точка и вариант E\n")
    print("| c, м | исходная P5 L/R | E local L/R | E world L/R | запас E, ° | требуемое удлинение L/R, мм |")
    print("|---|---|---|---|---|---|")
    for clearance in data["metadata"]["clearances_m"]:
        key = f"{clearance:.2f}"
        flags = {}
        for finger in ("local", "world"):
            for side in ("L", "R"):
                row = data["variant_E"][key][finger][side]
                flags[(finger, side)] = row.get("reachable", False)
        margins = [
            row["min_margin_deg"]
            for finger in ("local", "world")
            for side in ("L", "R")
            for row in [data["variant_E"][key][finger][side]]
            if row.get("reachable")
        ]
        original = data["original"][key]
        ext = {
            side: data["variant_F"][key][side].get("required_extension_m")
            for side in ("L", "R")
        }

        def mark(value: bool) -> str:
            return "да" if value else "нет"

        def ext_text(value) -> str:
            return f"{value * 1000:.0f}" if value is not None else "—"

        print(
            f"| {clearance:.2f} | {mark(original['L']['reachable'])}/{mark(original['R']['reachable'])} | "
            f"{mark(flags[('local', 'L')])}/{mark(flags[('local', 'R')])} | "
            f"{mark(flags[('world', 'L')])}/{mark(flags[('world', 'R')])} | "
            f"{(min(margins) if margins else float('nan')):.2f} | "
            f"{ext_text(ext['L'])}/{ext_text(ext['R'])} |"
        )
    print()


if __name__ == "__main__":
    unilateral_markdown()
    bilateral_markdown()
    clearance_markdown()
    decision_markdown()
    p5_markdown()
