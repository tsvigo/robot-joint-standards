"""Временный замер скорости статического расчёта (удалить после настройки)."""
from __future__ import annotations

import importlib.util
import sys
import time
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


M = _load(BASE / "arm_architecture_comparison.py", "tm_main")
V = _load(BASE / "audit_v2w_2" / "audit_v2w2.py", "tm_v2")

pts = V.build_points_v2("L", "nominal", "A")
q = {"sh3": -90.0, "sh2": 45.0, "sh1": 30.0, "el": 75.0,
     "rad": 0.0, "w1": 0.0, "w2": 0.0}

N = 50
t0 = time.time()
for _ in range(N):
    tg = M.tg_jacobian("L", q, pts)
    tx, _ = M.external_wrench("L", q, np.zeros(3), 100.0)
per = (time.time() - t0) / N * 1000
print(f"одна поза (tg+text): {per:.2f} мс")

for step in (30, 20, 15, 10):
    n = 1
    for lo, hi in ((-180, 60), (-90, 90), (-90, 130), (-10, 150)):
        n *= len(np.arange(lo, hi + 1e-9, step))
    print(f"  шаг {step:3d}° -> {n:7d} поз; "
          f"оценка на 1 сторону (обе нагрузки в одном проходе): "
          f"{n * per / 1000:6.1f} с")
