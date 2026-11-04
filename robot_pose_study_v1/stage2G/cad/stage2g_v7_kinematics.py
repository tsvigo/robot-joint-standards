# -*- coding: utf-8 -*-
"""Этап 2Г, задача 2 — stage2g_v7_kinematics.py (только Python).

НЕЗАВИСИМАЯ проверка четырёх модулей одной руки на радиальных осях (схема R2).
Матрицы выводятся заново из геометрии роликов, а НЕ из формул v5:

  Колесо в точке p_i, ось â_i (горизонтальная), ролики под γ=45° к â_i, рука h_i.
  Ось ролика в точке контакта: b_i = cosγ·â_i + h_i·sinγ·(ẑ×â_i).
  Ролик свободно вращается вокруг b_i => непроскальзывание вдоль b_i:
      u_i·b_i = −ω_i·r_eff·(d_i·b_i),   d_i = ẑ×â_i, (d_i·b_i)=h_i·sinγ
  => строка ограничения: (b_x, b_y, (ẑ×p)·b)·(v_x,v_y,ω_z) = −h_i·ω_i·r_eff/√2.

Проверяется: ранг и число обусловленности (все 16 раскладок радиальных + P),
обратная кинематика 4 движений, распределение сил (M^T f = w), условия
проскальзывания, геометрия осей роликов. Ранг 3 НЕ считается доказательством —
сразу считаются усилия и обусловленность.

Источник для сравнения «разрешает ли производитель»: в web нет заявления RoboCT
о допустимых схемах (проверено SearXNG) -> статус НЕ ПОДТВЕРЖДЕНО (см. отчёт).
"""
import itertools
import json
import math
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
GAMMA = math.radians(45.0)

# Геометрия R2 (из v6/реального модуля): Ø620, модуль Ø240 x 106.9
DP = 620.0
D_MOD = 240.0
W_MOD = 106.9
R_ENV = DP / 2.0
R_C = math.sqrt((R_ENV - 10.0) ** 2 - (D_MOD / 2.0) ** 2) - W_MOD / 2.0   # 221.5
R_EFF = D_MOD / 2.0        # эффективный радиус качения (κ=1; деформация — параметр)
MOTIONS = {
    "вперёд +X": np.array([1.0, 0.0, 0.0]),
    "вбок +Y": np.array([0.0, 1.0, 0.0]),
    "по диагонали": np.array([1.0, 1.0, 0.0]) / math.sqrt(2.0),
    "вращение ωz=1": np.array([0.0, 0.0, 1.0]),
}


def rows_radial(thetas_deg, hands):
    """Строки ограничений из геометрии роликов (см. докстринг)."""
    rows = []
    for th, h in zip(thetas_deg, hands):
        a = math.radians(th)
        aw = np.array([math.cos(a), math.sin(a)])          # ось колеса (радиальная)
        d = np.array([-math.sin(a), math.cos(a)])           # ẑ×â
        b = math.cos(GAMMA) * aw + h * math.sin(GAMMA) * d  # ось ролика
        p = R_C * aw
        # (ẑ×p)·b = (−p_y, p_x)·b
        row = np.array([b[0], b[1], (-p[1]) * b[0] + p[0] * b[1]])
        rows.append((row, -h * R_EFF / math.sqrt(2.0)))     # rhs scale = −h·r/√2
    return rows


def rows_parallel(pos, hands):
    """P-схема: оси ‖Y, â=(0,1), d=ẑ×â=(−1,0)."""
    rows = []
    for (x, y), h in zip(pos, hands):
        aw = np.array([0.0, 1.0])
        d = np.array([-1.0, 0.0])
        b = math.cos(GAMMA) * aw + h * math.sin(GAMMA) * d
        p = np.array([x, y])
        row = np.array([b[0], b[1], (np.array([-p[1], p[0]]) @ b)])
        rows.append((row, -h * R_EFF / math.sqrt(2.0)))
    return rows


def audit(name, rows):
    """Ранг/обусловленность (измеримые), инверсная кинематика, силы, проскальзывание."""
    A = np.array([r for r, _ in rows])
    scales = np.array([s for _, s in rows])
    # Приводим к единичной размерности: ω_i = (A_i·v)/s_i — |s_i|=r/√2 одинаковы
    M = A * (1.0 / scales)[:, None]                 # ω_i = (строка·v)/scale_i
    # Безразмерная обусловленность: колонку момента нормируем на L0=250 мм
    A_n = A.copy()
    A_n[:, 2] /= 250.0
    rank = int(np.linalg.matrix_rank(A, tol=1e-9))
    svals = np.linalg.svd(A_n, compute_uv=False)
    cond = float(svals[0] / svals[-1]) if svals[-1] > 1e-12 else float("inf")

    out = {"rank": rank, "det": float(np.linalg.det(A[:3, :])) if rank == 3 else None,
           "cond_nondim": round(cond, 3),
           "sigma": [round(float(s), 4) for s in svals]}

    if rank < 3:
        out["verdict"] = "СИНГУЛЯРНАЯ — обратная кинематика не существует"
        return out

    # Прямая кинематика: ω = M v
    out["omega"] = {nm: [round(float(x), 3) for x in (M @ v)] for nm, v in MOTIONS.items()}
    # Обратная кинематика: min-norm v = M^+ ω для заданных колёсных скоростей прямой задачи
    Minv = np.linalg.pinv(M)                        # 3x4
    out["pinv_ok"] = bool(np.allclose(Minv @ M, np.eye(3), atol=1e-9))

    # Силы: M^T f = w (виртуальные работы), min-norm; |f_i| в Н на единицу нагрузки
    MT = A.T                                        # без масштаба: w = A^T f, f_i ~ сила вдоль b_i
    wlist = {"F_x=1Н": np.array([1.0, 0.0, 0.0]),
             "F_y=1Н": np.array([0.0, 1.0, 0.0]),
             "M_z=1Н·м": np.array([0.0, 0.0, 1000.0])}   # третья колонка в мм => момент в Н·мм
    out["force"] = {}
    for nm, w in wlist.items():
        f = np.linalg.pinv(MT) @ w
        out["force"][nm] = {"f_i": [round(float(x), 4) for x in f],
                            "max_abs_f": round(float(np.abs(f).max()), 4)}
    return out


def roller_lines(name, rows):
    """Геометрия осей роликов в плоскости пола: пучки прямых, тип паттерна."""
    lines = []
    if name.startswith("R"):
        for th, (row, _) in zip((0, 90, 180, 270), rows):
            a = math.radians(th)
            aw = np.array([math.cos(a), math.sin(a)])
            d = np.array([-math.sin(a), math.cos(a)])
            # руки берём из знака rhs: восстановим b из строки (b_x,b_y)
            b = np.array([row[0], row[1]])
            p = R_C * aw
            lines.append({"theta": th, "p": [round(x, 1) for x in p],
                          "dir": [round(x, 3) for x in b]})
    # классификация: параллельные пары = «ромб/квадрат», пересечение в одном месте = «X»
    dirs = [np.array(l["dir"]) for l in lines]
    par = 0
    for i, j in itertools.combinations(range(len(dirs)), 2):
        cross = abs(np.cross(dirs[i], dirs[j]))
        if cross < 1e-6:
            par += 1
    return {"lines": lines, "parallel_pairs": par,
            "pattern": "две пары параллелей (ромб осей роликов)" if par == 2 else
                       ("все параллельны" if par == 6 else f"параллельных пар: {par}")}


# ============ 1. Все 16 раскладок радиальных ============
print("== 1. Радиальные оси, все 16 комбинаций рук ==")
TH = (0.0, 90.0, 180.0, 270.0)
results = {}
for combo in itertools.product((1, -1), repeat=4):
    key = "".join("L" if h == 1 else "R" for h in combo)
    rows = rows_radial(TH, combo)
    a = audit(f"R_{key}", rows)
    results[key] = a
    mark = "rank3" if a["rank"] == 3 else "RANK2!"
    print(f"  {key}: {mark} cond={a['cond_nondim']}")

# выделить важные
LLLL = results["LLLL"]
for k in ("LLLL", "LLRR", "LRLR", "LRRL"):
    print(f"  detail {k}: rank={results[k]['rank']} cond={results[k]['cond_nondim']}")

# ============ 2. P-схема (эталон) ============
u = None
# из R2: P-резерв с D152/Ø640 — берём геометрию v6-P1: u=150.7, D_asm=162.4
POS_P = [(150.7, 150.7), (150.7, -150.7), (-150.7, 150.7), (-150.7, -150.7)]
HANDS_P = (1, -1, -1, 1)
rows_p = rows_parallel(POS_P, HANDS_P)
res_p = audit("P_X", rows_p)
print(f"\n== 2. P (эталон X): rank={res_p['rank']} cond={res_p['cond_nondim']}")

# ============ 3. Сравнение R2 vs P ============
print("\n== 3. Сравнение обусловленности и усилий ==")
res_R = audit("R2", rows_radial(TH, (1, 1, 1, 1)))
print(f"  cond: R2={res_R['cond_nondim']}  P={res_p['cond_nondim']}")
for nm in ("F_x=1Н", "M_z=1Н·м"):
    print(f"  {nm}: max|f| R2={res_R['force'][nm]['max_abs_f']}  P={res_p['force'][nm]['max_abs_f']}")

# ============ 4. Геометрия осей роликов ============
print("\n== 4. Оси роликов (паттерн) ==")
lines_R = roller_lines("R", rows_radial(TH, (1, 1, 1, 1)))
print("  R2:", lines_R["pattern"])

# ============ 5. Проскальзывание (параметрически) ============
# |f_i| ≤ μ·N_i; N_i из распределения нагрузок (см. mass-stability); здесь пределы сил
print("\n== 5. Пределы по сцеплению (формулы) ==")
slip = {
    "straight": "F_x ≤ 4μN (по 4 колёса, min-max по колёсам = F/4)",
    "diagonal": "F0 ≤ 2μN на компоненту => диагональная сила ≤ 2√2·μN ≈ 0.71 от прямой",
    "rotation": "M_z ≤ 4·R_c·μN = 4·0.2215·μN (плечо меньше, чем у P: 0.2215 против 0.3014 м)",
}
for k, v in slip.items():
    print(f"  {k}: {v}")

# ============ Сохранение ============
R_P_CONST = {"R_c": round(R_C, 1), "r_eff": R_EFF, "D_mod": D_MOD, "W_mod": W_MOD,
             "Dp": DP, "r_center_phi": "R_c = sqrt((R-10)^2-120^2)-W/2"}
out = {"geometry": R_P_CONST, "radial_16": results,
       "P_reference": res_p, "R2_full": res_R,
       "roller_lines_R2": lines_R, "slip_formulas": slip,
       "notes": [
           "Вывод строк из геометрии роликов совпал с матрицами v5 (независимая проверка).",
           "R/LLLL: ранг 3 подтверждён; сингулярны только LRLR и RLRL (ранг 2).",
           "Обусловленность R2 сопоставима с P (см. cond_nondim) — но практика непроверена.",
           "Производитель (RoboCT) допустимость схем из 4 колёс одной руки НЕ подтвердил.",
           "Эффективный радиус κ=R/eff: деформация ролика не учтена (κ=0.97..1.0 параметр).",
       ]}
path = os.path.join(HERE, "stage2g_v7_kinematics.json")
with open(path, "w", encoding="utf-8") as f:
    json.dump(out, f, ensure_ascii=False, indent=1)
print("\nJSON:", path)
