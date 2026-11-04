# -*- coding: utf-8 -*-
"""Этап 2Г — stage2g_v6_analysis.py (только Python).

Адресный анализ v6 (без детализации шасси):
  Z1 — является ли конфликт колёс схемы P с зоной лифта Ø240 наложением на одной высоте:
       диапазоны Z тел, пересечение по высоте, «свободный крест» на нижних ярусах;
  Z2 — разнесение по Z и физически возможные пути передачи сил от лифта к раме;
  K6 — инверсная кинематика и силы приводов (вперёд/вбок/по диагонали/вращение)
       для P (X-раскладка) и R (4 одинаковых колеса): ω_i, f_i, τ_i, условия сцепления.
Ранг 3 не считается доказательством пригодности — приводятся конкретные усилия.
Результат: stage2g_v6_analysis.json + таблицы.
"""
import json
import math
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
with open(os.path.join(HERE, "stage2g_v5_geometry.json"), encoding="utf-8") as f:
    GEO = json.load(f)
CASES = {(c["layout"], c["Dp"], c["wheel"], c["drive"]): c for c in GEO["cases"]}
P5 = GEO["params"]

# ============ Z1: диапазоны тел и «свободный крест» ============
Z_PANEL = (50.0, 90.0)
Z_DRIVE = (45.0, 135.0)
Z_POD = (40.0, 140.0)
Z_BAT = (50.0, 150.0)

print("== Z1: диапазоны высот тел (мм) ==")
print("  колесо-ролики: [0, D]; ступица: [0.275D, 0.725D]; панель: [50, 90];")
print("  привод: [45, 135]; блок аутригера: [40, 140]; АКБ: [50, 150];")
print("  зона лифта в v5: [0, 400] — ПОЛНАЯ высота (консервативно)\n")

z_rows = []
for layout in ("R", "P"):
    for Dp in (620.0, 640.0):
        for wkey, D in (("D152", 152.4), ("D203", 203.0), ("D230", 230.0)):
            for drive in ("belt", "hub"):
                c = CASES[(layout, Dp, wkey, drive)]
                W_eff = c["W_eff"]
                D_asm = c["D_asm"]
                if layout == "R":
                    r_free = c["well_inner"]            # макс. радиус центральной стойки на низком ярусе
                else:
                    u = c["u"]
                    r_free = min(u - D_asm / 2.0, u - W_eff / 2.0)
                z_rows.append({
                    "layout": layout, "Dp": Dp, "wheel": wkey, "drive": drive,
                    "wheel_z": [0.0, D], "hub_z": [round(0.275 * D, 1), round(0.725 * D, 1)],
                    "overlap_lift_z": [0.0, D],
                    "free_central_R_low_z": round(max(r_free, 0.0), 1),
                    "free_central_D_low_z": round(2 * max(r_free, 0.0), 1),
                    "z_separation_needed": round(D + 10.0, 1),
                    "lift_margin_v5": c["lift_margin"],
                })

print("== Z1: «свободный крест» — макс. диаметр центральной стойки ниже верха колёс ==")
print(f"{'сх':>3} {'Dпан':>5} {'колесо':>6} {'привод':>5} {'стойкаØ':>8} {'ярус Z-разнесения':>18} {'конфликт v5':>11}")
for r in z_rows:
    print(f"{r['layout']:>3} {r['Dp']:5.0f} {r['wheel']:>6} {r['drive']:>5} "
          f"{r['free_central_D_low_z']:8.1f} {r['z_separation_needed']:18.1f} "
          f"{r['lift_margin_v5']:11.1f}")

# ============ K6: кинематика и усилия ============
def mat_P(u):
    pos = [(u, u), (u, -u), (-u, u), (-u, -u)]       # FL, FR, RL, RR
    hands = (1, -1, -1, 1)                            # X-конфигурация
    rows = []
    for (x, y), h in zip(pos, hands):
        rows.append([1.0, -float(h), -(y + h * x)])
    return np.array(rows), hands, pos


def mat_R(rc):
    thetas = (0.0, 90.0, 180.0, 270.0)
    hands = (1, 1, 1, 1)                              # все одной руки (аудит v5)
    pos = [(rc * math.cos(math.radians(t)), rc * math.sin(math.radians(t))) for t in thetas]
    rows = []
    for t, h in zip(thetas, hands):
        a = math.radians(t)
        rows.append([math.cos(a) - h * math.sin(a), math.sin(a) + h * math.cos(a), h * rc])
    return np.array(rows), hands, pos


MOTIONS = {"вперёд +X": (1000.0, 0.0, 0.0),        # мм/с на 1 м/с
           "вбок +Y": (0.0, 1000.0, 0.0),
           "по диагонали": (707.1, 707.1, 0.0),
           "вращение ωz=1": (0.0, 0.0, 1.0)}        # рад/с
WRENCHES = {"F_x = 1 Н": (1.0, 0.0, 0.0),
            "F_y = 1 Н": (0.0, 1.0, 0.0),
            "M_z = 1 Н·м": (0.0, 0.0, 1000.0)}      # Н·мм

kin = {}
print("\n== K6: инверсная кинематика ω_i (рад/с на 1 м/с или 1 рад/с) и усилия ==")
for layout, Dp, wkey in (("P", 640.0, "D152"), ("P", 640.0, "D203"),
                         ("R", 620.0, "D152"), ("R", 640.0, "D152")):
    c = CASES[(layout, Dp, wkey, "belt")]
    r_roll = {"D152": 76.2, "D203": 101.5, "D230": 115.0}[wkey]
    if layout == "P":
        M, hands, pos = mat_P(c["u"])
        omega_of = lambda v: [float((M[i] @ np.array(v)) / r_roll) for i in range(4)]
        force_of = lambda w: [float(f) for f in np.linalg.pinv(M.T) @ np.array(w)]
        tau_of = lambda f: [r_roll * fi / 1000.0 for fi in f]          # Н·м
    else:
        M, hands, pos = mat_R(c["r_c"])
        omega_of = lambda v: [float(-hands[i] * (M[i] @ np.array(v)) / r_roll) for i in range(4)]
        force_of = lambda w: [float(f) for f in np.linalg.pinv(M.T) @ np.array(w)]
        tau_of = lambda f: [-hands[i] * r_roll * fi / 1000.0 for i, fi in enumerate(f)]  # Н·м

    key = f"{layout}{int(Dp)}_{wkey}"
    entry = {"u_or_rc": c.get("u", c.get("r_c")), "r_roll": r_roll,
             "positions": [[round(x, 1), round(y, 1)] for x, y in pos], "hands": hands}
    print(f"\n-- {key}: оси/центр {entry['u_or_rc']} мм, положения {entry['positions']}")
    print(f"   {'движение':>16} | " + " | ".join(f"w{i+1}" for i in range(4)) + "   (ω_i)")
    for name, v in MOTIONS.items():
        om = omega_of(v)
        entry.setdefault("omega", {})[name] = [round(o, 3) for o in om]
        print(f"   {name:>16} | " + " | ".join(f"{o:6.2f}" for o in om))
    print(f"   {'нагрузка':>16} | " + " | ".join(f"f{i+1}" for i in range(4)) + "   (f_i, Н/Н или Н/Н·м)")
    for name, w in WRENCHES.items():
        f = force_of(w)
        tau = tau_of(f)
        entry.setdefault("force", {})[name] = {"f_i": [round(x, 4) for x in f],
                                               "tau_i": [round(x, 4) for x in tau]}
        print(f"   {name:>16} | " + " | ".join(f"{x:6.3f}" for x in f) +
              "   | τ_i: " + " | ".join(f"{x:6.3f}" for x in tau))
    # условие сцепления: |f_i| <= μ*N_i; для 4 колёс N_i = G/4
    entry["slip_note"] = ("|f_i| <= μ*G/4 на колесо; макс. F_x при μ и массе G — параметры")
    kin[key] = entry

# факторы распределения усилий (единичная сила / момент) для сравнения
print("\n== K6: сравнение распределения усилий (|f_i| на единицу нагрузки) ==")
for key, e in kin.items():
    fx = e["force"]["F_x = 1 Н"]["f_i"]
    mz = e["force"]["M_z = 1 Н·м"]["f_i"]
    print(f"  {key}: F_x=1Н -> f_i {fx}  (сумма |f|={sum(abs(x) for x in fx):.3f}); "
          f"M_z=1Н·м -> f_i {mz} (сумма |f|={sum(abs(x) for x in mz):.3f})")

# ============ K6b: тяга — коэффициент ролика и условия сцепления ============
R_ROLLER = {"D152": 25.0, "D203": 32.0, "D230": 38.0}   # радиус бочки ролика — НЕ подтверждён
print("\n== K6b: тяга (параметрически) ==")
print("  Связь момента и силы контакта: |c_i| = k·|f_i|, k = √2·r/(r − R_ролика)")
print("  (сила контакта направлена по оси ролика; проскальзывание при |c_i| > μ·N_i)")
for wkey, r in (("D152", 76.2), ("D203", 101.5), ("D230", 115.0)):
    rr = R_ROLLER[wkey]
    k = math.sqrt(2.0) * r / (r - rr)
    print(f"  {wkey}: r={r}, R_ролика≈{rr} -> k≈{k:.2f}; "
          f"F_x_max = μ·G/k ≈ {1/k:.2f}·μ·G; a_x_max ≈ {1/k:.2f}·μ·g")
print("  Одинаково для P и R (ролики 45° в обеих схемах); различия — плечи вращения")
print("  (P: 2u, R: R_c) и распределение нагрузок по колёсам (без масс не определяется).")
print("  μ (линолеум/каучук), R_ролика, масса G — НЕ подтверждены; числа параметрические")

out = {"z_analysis": z_rows, "kinematics": kin,
       "motions": {k: list(v) for k, v in MOTIONS.items()},
       "wrenches": {k: list(v) for k, v in WRENCHES.items()},
       "traction": {w: {"r": r, "R_roller_est": R_ROLLER[w],
                        "k": round(math.sqrt(2.0) * r / (r - R_ROLLER[w]), 2)}
                    for w, r in (("D152", 76.2), ("D203", 101.5), ("D230", 115.0))},
       "notes": [
           "Конфликт P/лифт — наложение по высоте z ∈ [0, D]; выше верха колёс зона Ø240 свободна.",
           "Свободный крест ограничивает центральную стойку ниже верха колёс: Ø0..Ø139 (P), Ø184..Ø358 (R).",
           "Силы f_i — решение Mᵀ f = w (минимальная норма); τ_i = r·f_i (с учётом руки).",
           "Тяга: |c_i| = k·|f_i|, k ≈ 2.0–2.1; проскальзывание при |c_i| > μ·N_i.",
           "ω_i в рад/с: переводы на 1 м/с, вращение на 1 рад/с; τ_i в Н·м.",
       ]}
path = os.path.join(HERE, "stage2g_v6_analysis.json")
with open(path, "w", encoding="utf-8") as f:
    json.dump(out, f, ensure_ascii=False, indent=1)
print("\nJSON:", path)