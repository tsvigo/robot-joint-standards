"""Точная проверка момента w2 в P6_L:S3_tension при 25 кг."""
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


M = _load(BASE / "arm_architecture_comparison.py", "w_main")
V2 = _load(BASE / "audit_v2w_2" / "audit_v2w2.py", "w_v2")
KEYS = ["sh3", "sh2", "sh1", "el", "rad", "w1", "w2"]

q = M.POSES["P6_L:S3_tension"]["q_deg"]
pts = V2.build_points_v2("L", "nominal", "A")

tg = M.tg_jacobian("L", q, pts)
tx, _ = M.external_wrench("L", q, np.zeros(3), 25.0)
fr = M.joint_frames("L", q)
palm = M.FK.fk("L", q)[M.chain_for("L")[-1]][:3, 3]

print("q_deg:", {k: round(v, 2) for k, v in q.items()})
print(f"palm = {np.round(palm, 4)}")
print()
for i, k in enumerate(KEYS):
    o, a = fr[i]
    print(f"{k:4s} ось={np.round(a, 4)} начало={np.round(o, 4)} "
          f"Tg={tg[i]:8.2f} Text={tx[i]:8.2f}")

print()
o6, a6 = fr[6]
r = palm - o6
F = np.array([0.0, 0.0, -25.0 * 9.81])
print(f"w2: о начало={np.round(o6,4)} ось={np.round(a6,4)}")
print(f"    r = palm - о = {np.round(r, 6)}")
print(f"    r×F = {np.round(np.cross(r, F), 4)}")
print(f"    (r×F)·a = {float(np.cross(r, F) @ a6):.6f}")
print(f"    (a×r)·F = {float(np.cross(a6, r) @ F):.6f}   "
      f"[это то, что даёт point_jacobian]")
print(f"    J[:,6] (first 3) = {np.round(M.point_jacobian(fr, 15, palm)[ :3, 6], 6)}")
print()
# Хват пальцев: где в модели приложена нагрузка?
print("Проверка: как external_wrench получает J")
J = M.point_jacobian(fr, len(M.chain_for("L")) - 1, palm)
print("J[:3,6] =", np.round(J[:3, 6], 6))
print("J^T F  =", np.round(J[:3].T @ F, 4))