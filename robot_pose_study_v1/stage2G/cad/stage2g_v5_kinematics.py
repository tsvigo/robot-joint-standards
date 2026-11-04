# -*- coding: utf-8 -*-
"""Этап 2Г — stage2g_v5_kinematics.py (только Python).

Независимый кинематический аудит четырёхколёсных Mecanum-схем на круглом основании.

Схемы:
  R — радиальная (v4): 4 колеса на 0/90/180/270°, оси вращения РАДИАЛЬНЫЕ (|| пол).
  P — параллельная (альтернатива): 4 колеса на диагоналях FL/FR/RL/RR (45/315/135/225°),
      оси ПАРАЛЛЕЛЬНЫ (|| оси Y платформы), стандартная X-ориентация роликов.

Вывод уравнения (не перенос формулы обычной Mecanum-платформы, а вывод из условий):
  Ось колеса â_w, направление качения t̂ = ẑ × â_w. Спин колеса ω_i даёт скорость
  точки контакта ω_i·r·t̂. Ось ролика в точке контакта â_r = (â_w + h·t̂)/√2 (h = ±1 — рука).
  Ролик свободен вдоль â_r, качение без проскальзывания ⊥ â_r:
      (V_i + ω_i·r·t̂) · â_r = 0   =>   V_i·â_w + h·V_i·t̂ = −h·r·ω_i
  где V_i = (Vx − ωz·y_i, Vy + ωz·x_i) — скорость платформы в точке контакта.

Для каждой схемы и всех 16 раскладок рук (h_i = ±1):
  M·[Vx, Vy, ωz]^T = b(ω) — матрица 4×3;
  rank(M) — управляемые степени;
  null(M)  — свободные (неуправляемые) движения платформы;
  null(Mᵀ) — режимы проскальзывания (линейные связи на скорости колёс).

Результат: stage2g_v5_kinematics.json + печать.
"""
import json
import math
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))

R_ROLL = 76.2      # радиус качения (пример D152.4) — ранги от него не зависят
R_C = 250.0        # радиус позиций колёс (схема R, пример)
U_DIAG = 175.0     # полусторона квадрата диагональных позиций (схема P, пример)


def mat_radial(hands, rc):
    """Схема R: строки [cosθ − h·sinθ, sinθ + h·cosθ, h·Rc] при θ = 0/90/180/270."""
    rows = []
    for th, h in zip((0.0, 90.0, 180.0, 270.0), hands):
        t = math.radians(th)
        rows.append([math.cos(t) - h * math.sin(t),
                     math.sin(t) + h * math.cos(t),
                     h * rc])
    return np.array(rows, dtype=float)


def mat_parallel(hands, u):
    """Схема P: оси || Y; строки [1, −h, −(y + h·x)]; позиции FL/FR/RL/RR."""
    pos = [(u, u), (u, -u), (-u, u), (-u, -u)]   # FL, FR, RL, RR
    rows = []
    for (x, y), h in zip(pos, hands):
        rows.append([1.0, -float(h), -(y + h * x)])
    return np.array(rows, dtype=float)


def analyze(mat, label, positions, axes, hands):
    rank = int(np.linalg.matrix_rank(mat))
    # свободные движения: null(M)
    _, _, vt = np.linalg.svd(mat)
    null_m = [list(np.round(v, 4)) for v in vt[rank:]]
    # режимы проскальзывания: null(Mᵀ) -> λᵀM = 0
    _, _, vt2 = np.linalg.svd(mat.T)
    null_mt = [list(np.round(v, 4)) for v in vt2[rank:]]
    # направления осей роликов â_r = (â_w + h·t̂)/√2
    roller_dirs = []
    for (x, y), aw, h in zip(positions, axes, hands):
        aw = np.array(aw, dtype=float)
        tz = np.array([-aw[1], aw[0], 0.0])           # t̂ = ẑ × â_w (в плоскости)
        ar = (aw + h * tz) / math.sqrt(2.0)
        roller_dirs.append([round(float(ar[0]), 3), round(float(ar[1]), 3)])
    return {
        "label": label,
        "matrix": [[round(float(v), 3) for v in row] for row in mat],
        "rank": rank,
        "null_M_free_motions": null_m,
        "null_MT_slip_modes": null_mt,
        "roller_axis_dirs_xy": roller_dirs,
        "hands": list(hands),
    }


# ---------------- Схема R: все 16 раскладок рук ----------------
print("== СХЕМА R (радиальные оси, колёса на 0/90/180/270°) ==")
print("строки матрицы: [cosθ − h·sinθ, sinθ + h·cosθ, h·Rc]; столбцы: Vx, Vy, ωz")
radial_sweep = []
radial_cases = {}
for mask in range(16):
    hands = tuple(1 if (mask >> i) & 1 == 0 else -1 for i in range(4))
    mat = mat_radial(hands, R_C)
    rank = int(np.linalg.matrix_rank(mat))
    radial_sweep.append({"hands": "".join("L" if h > 0 else "R" for h in hands),
                         "rank": rank})
    tag = "".join("L" if h > 0 else "R" for h in hands)
    if tag in ("LRLR", "RLRL", "LLLL", "LLRR"):
        pos = [(R_C, 0.0), (0.0, R_C), (-R_C, 0.0), (0.0, -R_C)]
        axes = [(1, 0, 0), (0, 1, 0), (-1, 0, 0), (0, -1, 0)]
        radial_cases[tag] = analyze(mat, f"R/{tag}", pos, axes, hands)

print(f"{'руки':>6} {'ранг':>4}")
for s in radial_sweep:
    mark = "  <-- СИНГУЛЯРНАЯ" if s["rank"] < 3 else ""
    print(f"{s['hands']:>6} {s['rank']:>4}{mark}")

# ---------------- Схема P: все 16 раскладок рук ----------------
print("\n== СХЕМА P (параллельные оси || Y, колёса FL/FR/RL/RR на диагоналях) ==")
print("строки матрицы: [1, −h, −(y + h·x)]; столбцы: Vx, Vy, ωz")
parallel_sweep = []
parallel_cases = {}
pos_p = [(U_DIAG, U_DIAG), (U_DIAG, -U_DIAG), (-U_DIAG, U_DIAG), (-U_DIAG, -U_DIAG)]
axes_p = [(0, 1, 0)] * 4
for mask in range(16):
    hands = tuple(1 if (mask >> i) & 1 == 0 else -1 for i in range(4))
    mat = mat_parallel(hands, U_DIAG)
    rank = int(np.linalg.matrix_rank(mat))
    tag = "".join("L" if h > 0 else "R" for h in hands)   # порядок FL, FR, RL, RR
    parallel_sweep.append({"hands_FL_FR_RL_RR": tag, "rank": rank})
    if tag in ("LRRR", "LRRL", "LLRL"):   # X-конфигурация: FL+RR = L, FR+RL = R
        parallel_cases[tag] = analyze(mat, f"P/{tag}", pos_p, axes_p, hands)

print(f"{'руки FL/FR/RL/RR':>18} {'ранг':>4}")
for s in parallel_sweep:
    mark = ""
    if s["hands_FL_FR_RL_RR"] == "LRRL":
        mark = "  <-- стандартная X-конфигурация"
    elif s["rank"] < 3:
        mark = "  <-- СИНГУЛЯРНАЯ"
    print(f"{s['hands_FL_FR_RL_RR']:>18} {s['rank']:>4}{mark}")

# ---------------- Ключевые случаи: подробно ----------------
print("\n== КЛЮЧЕВЫЕ МАТРИЦЫ ==")
key = {}
for tag, case in radial_cases.items():
    print(f"\n-- Схема R, руки {tag}: rank = {case['rank']}")
    for row in case["matrix"]:
        print("   ", row)
    print("    свободные движения null(M):", case["null_M_free_motions"])
    print("    режимы проскальзывания null(Mᵀ):", case["null_MT_slip_modes"])
    print("    оси роликов â_r (xy):", case["roller_axis_dirs_xy"])
    key[f"R_{tag}"] = case
for tag, case in parallel_cases.items():
    print(f"\n-- Схема P, руки FL/FR/RL/RR = {tag}: rank = {case['rank']}")
    for row in case["matrix"]:
        print("   ", row)
    print("    свободные движения null(M):", case["null_M_free_motions"])
    print("    режимы проскальзывания null(Mᵀ):", case["null_MT_slip_modes"])
    print("    оси роликов â_r (xy):", case["roller_axis_dirs_xy"])
    key[f"P_{tag}"] = case

# обратная кинематика схемы P при стандартной X-раскладке (LRRL)
hands_x = (1, -1, -1, 1)     # FL=L, FR=R, RL=R, RR=L
mat_x = mat_parallel(hands_x, U_DIAG)
inv = np.linalg.pinv(mat_x)
omega_expr = [[round(float(v), 5) for v in row] for row in inv]

out = {
    "derivation": "V_i·â_w + h·V_i·t̂ = −h·r·ω_i, t̂ = ẑ×â_w, â_r = (â_w+h·t̂)/√2",
    "scheme_R": {"positions_deg": [0, 90, 180, 270], "axes": "радиальные",
                 "r_roll_example": R_ROLL, "R_c_example": R_C,
                 "sweep": radial_sweep},
    "scheme_P": {"positions": ["FL(+u,+u)", "FR(+u,−u)", "RL(−u,+u)", "RR(−u,−u)"],
                 "axes": "параллельные || Y", "u_example": U_DIAG,
                 "sweep": parallel_sweep},
    "key_cases": key,
    "P_X_inverse_kinematics_omega_per_r": omega_expr,
    "notes": [
        "Ранг M = 3 => полное управление Vx, Vy, ωz (единственный режим проскальзывания — общий для 4-колёсных Mecanum).",
        "Ранг M = 2 => одно свободное скольжение (неуправляемое направление) и два режима проскальзывания.",
        "Схема R с X-раскладкой рук (чередование L/R по кругу) — СИНГУЛЯРНА: все 4 роликовые оси лежат на одной прямой.",
        "Схема R управляема только при НЕ-X раскладке (например, все 4 колеса одной руки) — это не стандартная схема.",
        "Схема P со стандартной X-раскладкой (FL+RR = L, FR+RL = R) — ранг 3, стандартная кинематика Mecanum.",
    ],
}
path = os.path.join(HERE, "stage2g_v5_kinematics.json")
with open(path, "w", encoding="utf-8") as f:
    json.dump(out, f, ensure_ascii=False, indent=1)
print("\nJSON:", path)
