#!/usr/bin/env python3
"""Stage 2B.1 — completeness audit of the machine-readable artifacts.

Read-only.  Prints, for every artifact produced in this stage, whether the
fields required by the objective are present and how many samples were stored:

* joint coordinates (``q_deg``) for every sample,
* position and orientation error,
* joint-limit margin (and the limiting joint where recorded),
* clearance against bed / torso / base / other arm,
* mutual arm-arm and distal clearance for the two-arm runs.

Exit code 1 if any required field is missing.
"""

from __future__ import annotations

import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
JOINT_KEYS = ("sh3", "sh2", "sh1", "el", "rad", "w1", "w2")

failures: list[str] = []


def check(condition: bool, message: str) -> None:
    if not condition:
        failures.append(message)
        print(f"  FAIL {message}")


def has_joints(q) -> bool:
    return isinstance(q, dict) and all(key in q for key in JOINT_KEYS)


def audit_unilateral() -> None:
    print("== UNILATERAL ==")
    for path in sorted(HERE.glob("stage2b1_unilateral_*.json")):
        data = json.loads(path.read_text(encoding="utf-8"))
        for policy, block in data["policies"].items():
            samples = 0
            for transition in block["transitions"]:
                if transition["status"] != "valid":
                    continue
                for sample in transition["samples"]:
                    samples += 1
                    check(has_joints(sample.get("q_deg")), f"{path.name} {policy} q_deg")
                    for field in (
                        "position_error_mm",
                        "orientation_error_deg",
                        "min_margin_deg",
                        "min_margin_joint",
                        "clearance_min_m",
                        "arm_bed_m",
                        "arm_torso_m",
                        "arm_base_m",
                        "collision_free",
                    ):
                        check(field in sample, f"{path.name} {policy} {field}")
                    check(
                        sample.get("condition") is not None,
                        f"{path.name} {policy} condition",
                    )
            print(f"  {path.name:48s} {policy:10s} samples={samples}")


def audit_bilateral() -> None:
    print("== BILATERAL ==")
    for path in sorted(HERE.glob("stage2b1_bilateral_*.json")):
        data = json.loads(path.read_text(encoding="utf-8"))
        for scheme, samples in data["samples"].items():
            check(len(samples) > 0, f"{path.name} {scheme} empty")
            for sample in samples:
                check(has_joints(sample.get("left_q_deg")), f"{path.name} {scheme} left_q_deg")
                check(has_joints(sample.get("right_q_deg")), f"{path.name} {scheme} right_q_deg")
                for field in (
                    "left_min_margin_deg",
                    "right_min_margin_deg",
                    "left_condition",
                    "right_condition",
                    "arm_arm_m",
                    "distal_clearance_m",
                    "palm_distance_m",
                    "collision_free",
                    "left_joint_step_deg",
                    "right_joint_step_deg",
                ):
                    check(field in sample, f"{path.name} {scheme} {field}")
            print(f"  {path.name:48s} {scheme:6s} samples={len(samples)}")


def audit_static() -> None:
    path = HERE / "stage2b1_static_sweep.json"
    if not path.exists():
        print("== STATIC SWEEP == missing")
        return
    data = json.loads(path.read_text(encoding="utf-8"))
    print("== STATIC SWEEP ==")
    reachable = 0
    total = 0
    for case in data["cases"]:
        for state in case["states"]:
            total += 1
            if not state.get("reachable"):
                continue
            reachable += 1
            check(has_joints(state.get("q_deg")), f"static {case['operation']} {case['side']} {state['label']} q_deg")
            for field in ("position_error_mm", "orientation_error_deg", "min_margin_deg", "condition", "arm_bed_m"):
                check(field in state, f"static {case['operation']} {state['label']} {field}")
    print(f"  cases={len(data['cases'])} states={total} reachable={reachable}")
    print(f"  simultaneous pairs={len(data.get('simultaneous', []))}")


def audit_p5() -> None:
    path = HERE / "stage2b1_p5.json"
    if not path.exists():
        print("== P5 == missing")
        return
    data = json.loads(path.read_text(encoding="utf-8"))
    print("== P5 ==")
    for clearance in data["metadata"]["clearances_m"]:
        key = f"{clearance:.2f}"
        for finger in ("local", "world"):
            for side in ("L", "R"):
                row = data["variant_E"][key][finger][side]
                if row.get("reachable"):
                    check(has_joints(row.get("q_deg")), f"p5 E {key} {finger} {side} q_deg")
                    for field in ("position_error_mm", "orientation_error_deg", "min_margin_deg", "min_clearance_m"):
                        check(field in row, f"p5 E {key} {finger} {side} {field}")
    trajectory = data.get("variant_E_trajectory") or {}
    result = trajectory.get("result") or {}
    for segment in result.get("segments", []):
        for sample in segment.get("samples", []):
            check(has_joints(sample.get("q_deg")), f"p5 trajectory {segment['name']} q_deg")
    print(
        f"  variant F entries={len(data['variant_F'])} "
        f"trajectory={'yes' if result else 'no'}"
    )


def audit_p8_p9() -> None:
    path = HERE / "stage2b1_p8_p9.json"
    if not path.exists():
        print("== P8/P9 == missing")
        return
    data = json.loads(path.read_text(encoding="utf-8"))
    print("== P8 / P9 ==")
    for mode, assignments in data["p8"].items():
        if not isinstance(assignments, dict) or "left" in assignments:
            print(f"  P8 section has the pre-fix schema at key {mode!r}; re-run the script")
            continue
        for assignment, row in assignments.items():
            for side in ("left", "right"):
                state = row[side]
                if state.get("reachable"):
                    check(has_joints(state.get("q_deg")), f"p8 {mode} {assignment} {side} q_deg")
                    for field in ("position_error_mm", "orientation_error_deg", "min_margin_deg", "condition"):
                        check(field in state, f"p8 {mode} {assignment} {side} {field}")
            approach = row.get("synchronized_approach") or {}
            for sample in approach.get("trajectory", []):
                for field in (
                    "left_position_error_mm",
                    "left_orientation_error_deg",
                    "left_min_margin_deg",
                    "right_position_error_mm",
                    "right_orientation_error_deg",
                    "right_min_margin_deg",
                    "distal_clearance_m",
                ):
                    check(field in sample, f"p8 sync {mode} {assignment} {field}")
            print(f"  P8 {mode} {assignment:10s} both={row['both_reachable']} sync={bool(approach.get('trajectory'))}")
    for mode, sides in data["p9"].items():
        for side, row in sides.items():
            print(f"  P9 {mode} {side}: status={row.get('status') or 'ok'} segments={len(row.get('segments', []))}")


def main() -> int:
    audit_unilateral()
    audit_bilateral()
    audit_static()
    audit_p5()
    audit_p8_p9()
    print()
    if failures:
        print(f"FAILURES: {len(failures)}")
        return 1
    print("ALL REQUIRED FIELDS PRESENT")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
