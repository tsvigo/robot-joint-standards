"""Временный разведочный расчёт (удалить/не включать в итоги).

Цель: понять, какая ось даёт максимум момента в позах вылета,
и сверить порядок величин с аудитом arm_actuator_feasibility (1203.5 Н·м).
"""
from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import numpy as np

BASE = Path("/home/v/projects/робот/robot_pose_study_v1/"
            "arm_architecture_comparison_v1")
sys.dont_write_bytecode = True


def _load(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


M = _load(BASE / "arm_architecture_comparison.py", "probe_main")
V2 = _load(BASE / "audit_v2w_2" / "audit_v2w2.py", "probe_v2")

KEYS = ["sh3", "sh2", "sh1", "el", "rad", "w1", "w2"]
POSES = {
    "all0": dict.fromkeys(KEYS, 0.0),
    "sh2+90": {k: 0.0 for k in KEYS} | {"sh2": 90.0},
    "sh2+85": {k: 0.0 for k in KEYS} | {"sh2": 85.0},
    "sh1+90": {k: 0.0 for k in KEYS} | {"sh1": 90.0},
    "P6_S3": M.POSES["P6_L:S3_tension"]["q_deg"],
    "P6_S0": M.POSES["P6_L:S0_ready"]["q_deg"],
}

pts = V2.build_points_v2("L", "nominal", "A")
print(f"{'pose':10s} " + " ".join(f"{k:>7s}" for k in KEYS) + "   |Tg|max")
for name, q in POSES.items():
    tg = M.tg_jacobian("L", q, pts)
    print(f"{name:10s} " + " ".join(f"{v:7.1f}" for v in tg)
          + f"   {np.abs(tg).max():7.1f}")

print()
print("=== + Text (груз 100 кг на ладонь) ===")
for name, q in POSES.items():
    tg = M.tg_jacobian("L", q, pts)
    tx, _ = M.external_wrench("L", q, np.zeros(3), 100.0)
    s = tg + tx
    print(f"{name:10s} " + " ".join(f"{v:7.1f}" for v in s)
          + f"   {np.abs(s).max():7.1f}")

print()
print("=== лимиты осей (joint_limits_audit) ===")
for line in Path("/home/v/projects/робот/joint_limits_audit.txt").read_text(
        encoding="utf-8").splitlines():
    if line.startswith("left_shoulder") or line.startswith("left_elbow") \
            or line.startswith("left_wrist") or line.startswith("left_prox"):
        print(" ", line)
