"""Независимая проверка максимумов статики из joint_requirements.json."""
from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path

import numpy as np

BASE = Path("/home/v/projects/робот/robot_pose_study_v1/"
            "arm_architecture_comparison_v1")
HERE = Path(__file__).resolve().parent
sys.dont_write_bytecode = True


def _load(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


M = _load(BASE / "arm_architecture_comparison.py", "v_main")
V2 = _load(BASE / "audit_v2w_2" / "audit_v2w2.py", "v_v2")

d = json.loads((HERE / "joint_requirements.json").read_text(encoding="utf-8"))
KEYS = ["sh3", "sh2", "sh1", "el", "rad", "w1", "w2"]

print("=== пересчёт осевых моментов из поз JSON (Tg + Text, 100 кг) ===")
for side in ("L", "R"):
    pts = V2.build_points_v2(side, "nominal", "A")
    g = d["detail"][side]["grid_100kg"]
    print(f"-- {side}")
    for k in KEYS:
        q = g[k]["pose"]
        tg = M.tg_jacobian(side, q, pts)
        tx, _ = M.external_wrench(side, q, np.zeros(3), 100.0)
        i = KEYS.index(k)
        expected = g[k]["value"]
        got = float(tg[i] + tx[i])
        ok = abs(got - expected) < 0.05
        print(f"   {k:4s} в JSON {expected:9.2f} | пересчёт {got:9.2f} "
              f"{'OK' if ok else 'РАСХОЖДЕНИЕ'}")

print("\n=== независимый расчёт двух 'подозрительных' точек sh3/sh2 (L) ===")
pts = V2.build_points_v2("L", "nominal", "A")
for label, q in [
    ("A: sh3=0, sh2=-90", {"sh3": 0, "sh2": -90, "sh1": 0, "el": 0,
                           "rad": 0, "w1": 0, "w2": 0}),
    ("B: sh3=-90, sh2=0", {"sh3": -90, "sh2": 0, "sh1": 0, "el": 0,
                           "rad": 0, "w1": 0, "w2": 0}),
]:
    tg = M.tg_jacobian("L", q, pts)
    tx, _ = M.external_wrench("L", q, np.zeros(3), 100.0)
    tot = tg + tx
    print(f" {label}:")
    for i, k in enumerate(KEYS):
        print(f"     {k:4s} Tg={tg[i]:9.2f}  Text={tx[i]:9.2f}  "
              f"сумма={tot[i]:9.2f}")

print("\n=== симметрия: совпадают ли значения шарнирных осей? ===")
# sh3 и sh2 в этих позах дают одинаковую величину — объяснение:
qA = {"sh3": 0, "sh2": -90, "sh1": 0, "el": 0, "rad": 0, "w1": 0, "w2": 0}
qB = {"sh3": -90, "sh2": 0, "sh1": 0, "el": 0, "rad": 0, "w1": 0, "w2": 0}
frA = M.joint_frames("L", qA)
frB = M.joint_frames("L", qB)
for i in (0, 1, 2):
    print(f"  {KEYS[i]}: ось A={np.round(frA[i][1],3)} "
          f"начало A={np.round(frA[i][0],3)}")
    print(f"        ось B={np.round(frB[i][1],3)} "
          f"начало B={np.round(frB[i][0],3)}")
