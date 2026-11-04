# -*- coding: utf-8 -*-
"""Этап 2Г RVW-7PG, задача 3 (корректировка) — силовая модель радиальной R2
LLLL/RRRR через принцип виртуальной работы (rvw7pg_r2_forces_v2.py).

Модель (проверена ниже тестами):
  Якоби: u_i = J_i·(vx,vy,wz), J_i = [cos(θ+45σ), sin(θ+45σ), σ·sin45·R_w].
  Равновесие/виртуальная работа: суммарная сила F = Σ f_i·ê_i;
  решение симметрии: f_i = (ê_i·v̂)·F/2 (поступательное) -> мощность
  P_приводов = Σ f_i·u_i = F·v (ТОЧНО — тест согласованности).
  Отсюда момент на колесо: Tw = |f_i|·r = F_tot·r/(2√2) при |ê·v̂|=1/√2
  — геометрический коэффициент косого контакта (√2) против прямой схемы;
  реальные потери Mecanum — параметр k_mec >= 1 (НЕ факт).
Коэффициенты каталога C=1/2/2.5 из ПРЯМОУГОЛЬНОЙ компоновки: для радиальной
LLLL/RRRR симметрии 90° направления X/Y ЭКВИВАЛЕНТНЫ (тест симметрии).
r = 0.089 м (фактический радиус Ø178), R_w = 0.2489 м (из компоновки v2).
Лимиты: 7 Н·м rated; 16 Н·м (Product Guide 2026) и 27 Н·м (каталог
Rev.004.1) allowable acc/dec — ПРОТИВОРЕЧИЕ НЕ РАЗРЕШЕНО, используются оба.
Каталожная регрессия Ø240 НЕ используется как подтверждение скоростей Ø178.
"""
import itertools
import json
import math
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
G = 9.81
R_WHEEL = 0.089            # м, DOC Ø178/2
R_W = 0.2489               # м, из компоновки rvw7pg_r2_layout_v2
I_RATIO, CRR = 30, 0.05    # DOC
T_RATED = 7.0              # DOC
T_ACC_16 = 16.0            # Product Guide (PDF 10/2026, ©2021)
T_ACC_27 = 27.0            # каталог Rev.004.1 (©2020)
N_MAX_RPM = 107.3          # DOC max wheel output (60 м/мин)
MASS = [300.0, 400.0, 450.0]
SPEEDS = [0.2, 0.5, 0.7, 1.0]
ACCELS = [0.0, 0.1, 0.2, 0.3, 0.5]
GRADES = [0.0, 0.01, 0.03, 0.05, 0.08]
K_MEC = 1.0                # ПАРАМЕТР потерь Mecanum (>=1; не измерен)
CHECKS = []


def t(cid, ok, detail):
    CHECKS.append({"id": cid, "ok": bool(ok), "detail": detail})
    print(f"  [{'PASS' if ok else 'FAIL'}] {cid}: {detail}")


TH = [0.0, 90.0, 180.0, 270.0]


def jacobian(sigmas=(1, 1, 1, 1), R_w=R_W):
    J = np.zeros((4, 3))
    for i, (th, s) in enumerate(zip(TH, sigmas)):
        a = math.radians(th) + s * math.radians(45)
        J[i] = (math.cos(a), math.sin(a), s * math.sin(math.radians(45)) * R_w)
    return J


J = jacobian()

# ============ 1. Симметрия X/Y ============
print("== 1. Симметрия направлений (виртуальная работа) ==")
u_x = J @ np.array([1.0, 0.0, 0.0])
u_y = J @ np.array([0.0, 1.0, 0.0])
sym_ok = np.allclose(np.sort(np.abs(u_x)), np.sort(np.abs(u_y)), atol=1e-12)
t("T1_симметрия_XY", sym_ok,
  f"|u_i| вдоль X {np.abs(u_x).round(4).tolist()} == вдоль Y "
  f"{np.abs(u_y).round(4).tolist()} -> C(X)=C(Y)=1 в радиальной LLLL "
  f"(каталожные C=1/2/2.5 к прямоугольной схеме не применимы)")
# изотропия МОЩНОСТИ (|u_i| по отдельности направление-зависимы — это нормально)
dirs_ok = True
tw_dir = []
for ang in range(0, 360, 15):
    v = np.array([math.cos(math.radians(ang)), math.sin(math.radians(ang)), 0.0])
    f = (J @ v) / 2.0                        # f_i при F=1
    u = J @ v                                # u_i при v=1
    if abs(float(f @ u) - 1.0) > 1e-9:
        dirs_ok = False
    tw_dir.append(max(abs(f)))               # макс |f_i| на F=1
t("T2_изотропия_мощности", dirs_ok,
  "P=F·v для всех 24 поступательных направлений; НО max|f_i| зависит от "
  f"направления: X/Y = {tw_dir[0]:.4f}·F, диагональ (вдоль ê) = {max(tw_dir):.4f}·F "
  f"= F/2 — худший случай в √2 раз больше X/Y")

# ============ 2. Согласованность мощностей ============
print("\n== 2. Согласованность мощностей (виртуальная работа) ==")
p_ok = True
for ang, F in itertools.product(range(0, 360, 15), (100.0, 1000.0)):
    v_hat = np.array([math.cos(math.radians(ang)), math.sin(math.radians(ang)), 0.0])
    f = (J @ v_hat) * F / 2.0                # f_i = (ê_i·v̂)·F/2
    u = J @ v_hat                            # v = 1 м/с
    P = float(f @ u)
    if abs(P - F * 1.0) > 1e-9:
        p_ok = False
t("T3_мощности", p_ok,
  "P_приводов = Σ f_i·u_i = F·v для всех 24 направлений/сил (виртуальная работа)")

# сила для режима (формула каталога, но isotropic C=1)
def F_tot(M, a, grade):
    th = math.atan(grade)
    return M * (a + G * (math.sin(th) + CRR * math.cos(th)))


def tw_radial(M, a, grade, k_mec=K_MEC, direction="XY"):
    """Tw на колесо в радиальной LLLL, Н·м (виртуальная работа).
    direction="XY" (вдоль осей, равн. X/Y): F·r/(2√2);
    direction="worst" (вдоль ê — диагональ, максимум |f_i|): F·r/2."""
    F = F_tot(M, a, grade)
    denom = 2 * math.sqrt(2) if direction == "XY" else 2.0
    return F * R_WHEEL / denom * k_mec, F


def n_rpm(u):
    return u / R_WHEEL * 60 / (2 * math.pi)


# ============ 3. Сетка режимов ============
print("\n== 3. Сетка режимов: радиальная LLLL (C=1, k_mec параметр) ==")
grid = []
for M, a, gr in itertools.product(MASS, ACCELS, GRADES):
    tw_xy, F = tw_radial(M, a, gr, direction="XY")
    tw_worst, _ = tw_radial(M, a, gr, direction="worst")
    grid.append({"M": M, "a": a, "grade_pct": round(gr * 100, 1),
                 "Tw_XY_Нм": round(tw_xy, 2), "Tw_макс_Нм": round(tw_worst, 2),
                 "F_Н": round(F, 1),
                 "vs7": "≤7" if tw_worst <= T_RATED else
                        "7..16" if tw_worst <= T_ACC_16 else
                        "16..27" if tw_worst <= T_ACC_27 else ">27 (худшее направление)",
                 "vs7_XY": "≤7" if tw_xy <= T_RATED else
                           "7..16" if tw_xy <= T_ACC_16 else
                           "16..27" if tw_xy <= T_ACC_27 else ">27"})
from collections import Counter
c = Counter(g["vs7"] for g in grid)
c_xy = Counter(g["vs7_XY"] for g in grid)
print(f"  точек: {len(grid)}; худшее направление (диагональ): {dict(c)}")
print(f"  вдоль осей X/Y: {dict(c_xy)}")
mmax = max(grid, key=lambda g: g["Tw_макс_Нм"])
mmax_xy = max(grid, key=lambda g: g["Tw_XY_Нм"])
print(f"  максимум худший: {mmax['Tw_макс_Нм']} Н·м ({mmax['M']} кг, "
      f"a={mmax['a']}, {mmax['grade_pct']}%) -> {mmax['vs7']}; "
      f"X/Y: {mmax_xy['Tw_XY_Нм']} Н·м -> {mmax_xy['vs7_XY']}")

# крейс и удержание
print("\n== 3b. Крейс/удержание (a=0) ==")
cruise = {}
for M in MASS:
    tw_c, _ = tw_radial(M, 0.0, 0.0)
    hold = {f"{int(gr*100)}%": round(tw_radial(M, 0.0, gr)[0], 2)
            for gr in (0.01, 0.03, 0.05, 0.08)}
    cruise[M] = {"крейс": round(tw_c, 2), "удержание": hold}
    print(f"  {M:.0f} кг: крейс {tw_c:.2f} Н·м "
          f"({'≤7' if tw_c <= 7 else '>7'}) ; удержание {hold}")

# ============ 4. Скорости колёс ============
print("\n== 4. Скорости колёс (r=0.089, предел 107.3 об/мин) ==")
spd_rows = []
for v, wz in itertools.product(SPEEDS, (0.0, 0.5, 1.0)):
    u = J @ np.array([v, 0.0, wz])
    n_abs = max(abs(n_rpm(ui)) for ui in u)
    spd_rows.append({"v": v, "wz": wz, "n_max_об_мин": round(n_abs, 1),
                     "статус": "OK" if n_abs <= N_MAX_RPM else "ПРЕВЫШЕНИЕ"})
# боковое/диагональ — то же по симметрии
for v in SPEEDS:
    u = J @ np.array([0.0, v, 0.0])
    spd_rows.append({"v": v, "wz": 0.0, "режим": "боковое",
                     "n_max_об_мин": round(max(abs(n_rpm(x)) for x in u), 1),
                     "статус": "OK" if max(abs(n_rpm(x)) for x in u)
                     <= N_MAX_RPM else "ПРЕВЫШЕНИЕ"})
lim = next(r for r in spd_rows if r["v"] == 1.0 and r.get("wz", 0) == 1.0)
t("T4_скорость_комбинированная", lim["статус"] == "OK",
  f"v=1.0 м/с + wz=1 рад/с: max n = {lim['n_max_об_мин']} об/мин ≤ 107.3 (DOC)")
over = [r for r in spd_rows if r["статус"] != "OK"]
t("T5_скорость_все", not over, f"все режимы скорости в пределах 107.3 об/мин "
  f"(превышений: {len(over)})")

# ============ 5. Длительное боковое ============
print("\n== 5. Длительное боковое/диагональное движение ==")
long_rows = {}
for M in MASS:
    tw, _ = tw_radial(M, 0.0, 0.0)
    long_rows[M] = {"боковое_крейс_Нм": round(tw, 2),
                    "статус_16": "≤16 OK" if tw <= T_ACC_16 else ">16",
                    "статус_7": "≤7 (непрерывный rated)" if tw <= T_RATED
                                else ">7 — непрерывность ограничена"}
    print(f"  {M:.0f} кг: боковое/диагональное крейс {tw:.2f} Н·м "
          f"({long_rows[M]['статус_7']}, {long_rows[M]['статус_16']})")

# ============ 6. Пределы 16 vs 27 ============
print("\n== 6. Пределы 16 vs 27 Н·м (противоречие источников) ==")
over16_xy = [g for g in grid if g["Tw_XY_Нм"] > T_ACC_16]
over27_xy = [g for g in grid if g["Tw_XY_Нм"] > T_ACC_27]
over16 = [g for g in grid if g["Tw_макс_Нм"] > T_ACC_16]
over27 = [g for g in grid if g["Tw_макс_Нм"] > T_ACC_27]
print(f"  X/Y: >16 Н·м {len(over16_xy)}/{len(grid)}, >27 {len(over27_xy)}; "
      f"худшее направление: >16 {len(over16)}, >27 {len(over27)}")
t("T6_предел_27_XY", len(over27_xy) == 0,
  f"направления X/Y: максимум {mmax_xy['Tw_XY_Нм']} Н·м ≤ 27 (Rev.004.1); "
  f"при 16 (Product Guide 2026) превышений {len(over16_xy)} — "
  "27 Н·м НЕ объявляется гарантированным до выяснения ревизии")
t("T6b_предел_27_худший", len(over27) == 0,
  f"худшее направление (диагональ): максимум {mmax['Tw_макс_Нм']} Н·м; "
  f">27 Н·м: {len(over27)} точек" + (" — ПРЕВЫШЕНИЕ" if over27 else " — нет"))
t("T7_крейс_XY_непрерывный", all(cruise[m]["крейс"] <= T_RATED for m in MASS),
  "крейс вдоль осей X/Y ≤7 rated: "
  + ", ".join(f"{m}:{cruise[m]['крейс']}" for m in MASS))
tw_diag_cruise = tw_radial(450.0, 0.0, 0.0, direction="worst")[0]
print(f"  крейс по ДИАГОНАЛИ (450 кг): {tw_diag_cruise:.2f} Н·м — "
      f"{'≤7' if tw_diag_cruise <= 7 else '>7: непрерывная диагональ ограничена'}")

out = {"метод": "виртуальная работа: f_i=(ê_i·v̂)F/2, Tw=|f_i|·r, "
                "P=Σf_i·u_i=F·v; симметрия 90° -> C=1 для всех направлений",
       "параметры": {"r_колеса": R_WHEEL, "R_w": R_W, "k_mec": K_MEC,
                     "k_mec_статус": "ПАРАМЕТР (не измерен); 1 — идеальный ролик"},
       "пределы": {"rated": T_RATED, "acc_dec_ProductGuide_2026": T_ACC_16,
                   "acc_dec_каталог_Rev0041": T_ACC_27,
                   "статус": "противоречие НЕ разрешено — использованы оба"},
       "симметрия": {"u_вперёд": u_x.round(4).tolist(),
                     "u_бок": u_y.round(4).tolist(), "изотропия": dirs_ok},
       "сетка": grid, "крейс_удержание": {str(k): v for k, v in cruise.items()},
       "скорости": spd_rows, "длительное_боковое": {str(k): v for k, v in
                                                     long_rows.items()},
       "сводка": {"точек": len(grid),
                  "распределение_XY": dict(c_xy),
                  "распределение_худшее": dict(c),
                  "макс_XY_Нм": mmax_xy["Tw_XY_Нм"],
                  "макс_худший_Нм": mmax["Tw_макс_Нм"],
                  "больше_16_XY": len(over16_xy),
                  "больше_27_XY": len(over27_xy),
                  "больше_16_худшее": len(over16),
                  "больше_27_худшее": len(over27)},
       "checks": CHECKS,
       "notes": ["Коэффициент √2 — геометрия косого контакта (виртуальная работа); "
                 "реальные потери Mecanum — параметр k_mec",
                 "Регрессия Ø240 как подтверждение скоростей Ø178 НЕ используется",
                 "Вертикальные нагрузки: 450<600 кг (каталог) — НЕ доказательство "
                 "устойчивости/ударных допусков"]}
path = os.path.join(HERE, "rvw7pg_r2_forces_v2.json")
with open(path, "w", encoding="utf-8") as f:
    json.dump(out, f, ensure_ascii=False, indent=1)
print("\nJSON:", path)
print("REGRESSION:", "PASS" if all(c["ok"] for c in CHECKS) else "FAIL")