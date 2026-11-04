# -*- coding: utf-8 -*-
"""Этап 2Г — stage2g_v6_real_drive.py (только Python).

Задача 5: реальные изделия с проверяемыми размерами (источники — SearXNG, страницы
производителей). Задачи 4/6: проверка компоновки и тяги для РЕАЛЬНОГО модуля.

Проверенные данные:
  A) RoboCT CTM1000-10R — интегрированный модуль Mecanum (колесо+мотор-редуктор+энкодер):
     Ø240 x 106.9 мм, 25 кг/колесо, передаточное 37.1, номинальный момент 74 Н·м,
     56 об/мин, 24-48 В, 220-440 Вт, IP44; нагрузка 1000 кг/комплект (4 шт).
     Источник: roboct-global.com/mobile-robot-components/mecanum-wheel/heavy-duty-mecanum-wheel.html
  B) Tongzhu TZ-MW152 — колесо Ø152.4 x 80 мм, 8 роликов, нагрузка 600 кг/комплект,
     рекоменд. 500 кг; ступица — шпоночное отверстие ИЛИ фланец (под заказ).
     Источник: tongzhutech.en.made-in-china.com (TZ-MW152).
  C) Полного комплекта «колесо D152/D203 + мотор-редуктор + ступица + крепление»
     с проверяемыми размерами в открытых источниках НЕ найдено -> статус НЕ ПОДТВЕРЖДЕНО.
"""
import json
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))

PRODUCTS = {
    "CTM1000-10R": {"vendor": "RoboCT (Hangzhou)", "D": 240.0, "W": 106.9,
                    "kg_per_wheel": 25.0, "payload_set_kg": 1000.0,
                    "gear_ratio": 37.1, "tau_rated_Nm": 74.0, "n_rated_rpm": 56.0,
                    "voltage": "24-48V", "power_W": "220-440", "IP": "IP44",
                    "integrated": "колесо+мотор-редуктор+энкодер+амортизатор",
                    "source": "roboct-global.com (heavy-duty-mecanum-wheel)"},
    "TZ-MW152": {"vendor": "Zhejiang Tongzhu (TZBOT)", "D": 152.4, "W": 80.0,
                 "rollers": 8, "payload_set_kg": 600.0, "payload_suggested_kg": 500.0,
                 "hub": "шпоночное отверстие или фланец (размеры под заказ)",
                 "motor": "НЕ ПРИВЯЗАН — размеры редуктора/ступицы не опубликованы",
                 "source": "tongzhutech.en.made-in-china.com (TZ-MW152)"},
}

P = {"lift_R": 120.0, "lift_gap": 10.0, "gap_transport": 10.0, "well_clr": 10.0,
     "S_inline": 150.0, "S_belt": 60.0}


def check_case(layout, D, W, Dp, hub_style=True):
    """hub_style: ширина W уже включает привод (интегрированный модуль)."""
    R_env = Dp / 2.0
    R_lim = R_env - P["gap_transport"]
    c = P["well_clr"]
    lift_r = P["lift_R"] + P["lift_gap"]
    if layout == "R":
        r_c = math.sqrt(max(R_lim ** 2 - (D / 2.0) ** 2, 0.0)) - W / 2.0
        well_inner = r_c - W / 2.0 - c
        lift_margin = well_inner - lift_r
        free_r = well_inner
        corner = math.hypot(r_c + W / 2.0, D / 2.0)
        supp_a = 2 * (r_c + 0.0) ** 2 / 1e6
        inrad = (r_c + 0.0) / math.sqrt(2)
    else:
        a1, b1 = D / 2.0, W / 2.0
        A, B, C = 2.0, 2.0 * (a1 + b1), a1 * a1 + b1 * b1 - R_lim * R_lim
        u = (-B + math.sqrt(max(B * B - 4 * A * C, 0.0))) / (2 * A)
        corner = math.hypot(u + a1, u + b1)
        inner = math.hypot(max(u - a1, 0.0), max(u - b1, 0.0))
        lift_margin = inner - lift_r
        free_r = min(max(u - a1, 0.0), max(u - b1, 0.0))
        r_c = u
        supp_a = 4 * u * (u + 0.0) / 1e6
        inrad = u
    return {"layout": layout, "Dp": Dp, "D": D, "W": W,
            "r_c_or_u": round(r_c, 1), "corner_R": round(corner, 1),
            "gap_to_contour": round(R_env - corner, 1),
            "lift_margin": round(lift_margin, 1),
            "free_central_D_low_z": round(2 * max(free_r, 0.0), 1),
            "supp_transport_m2": round(supp_a, 4), "inradius": round(inrad, 1),
            "pass": bool(lift_margin >= 0 and corner <= R_env + 1e-6)}


rows = []
for layout in ("R", "P"):
    for Dp in (620.0, 640.0):
        rows.append(check_case(layout, 240.0, 106.9, Dp))       # РЕАЛЬНЫЙ модуль CTM1000-10R

print("== Проверка компоновки РЕАЛЬНОГО модуля CTM1000-10R (Ø240 x 106.9) ==")
print(f"{'сх':>3} {'Dпан':>5} {'ось/центр':>9} {'зазор':>6} {'лифт±':>7} {'стойкаØ':>8} {'опора м2':>8} {'вердикт':>7}")
for r in rows:
    print(f"{r['layout']:>3} {r['Dp']:5.0f} {r['r_c_or_u']:9.1f} {r['gap_to_contour']:6.1f} "
          f"{r['lift_margin']:7.1f} {r['free_central_D_low_z']:8.1f} {r['supp_transport_m2']:8.4f} "
          f"{'OK' if r['pass'] else 'НЕТ':>7}")

# минимальные диаметры для реального модуля
def min_d(layout, D, W):
    d = 500.0
    while d <= 1000.0:
        if check_case(layout, D, W, d)["pass"]:
            return d
        d += 0.5
    return None

mins = {f"{L}_CTM240": min_d(L, 240.0, 106.9) for L in ("R", "P")}
print("\n== Минимальные диаметры для CTM1000-10R: ", mins)

# ============ Задача 6: тяга реального модуля ============
r_roll = 0.12          # 240/2 мм
R_roller = 40.0        # оценка бочки ролика — параметр
k = math.sqrt(2.0) * 120.0 / (120.0 - R_roller)
tau = PRODUCTS["CTM1000-10R"]["tau_rated_Nm"]
F_wheel = tau / r_roll                  # Н на ободе от момента
c_wheel = F_wheel / k * k               # сила контакта = k*f, f = tau/r
f_var = tau / r_roll                    # f_i (обобщённая)
c_var = k * f_var                       # сила контакта по оси ролика
n_rpm = PRODUCTS["CTM1000-10R"]["n_rated_rpm"]
v_max = n_rpm / 60.0 * 2 * math.pi * r_roll
print("\n== Задача 6: тяга реального модуля CTM1000-10R ==")
print(f"  номинальный момент {tau} Н·м -> f_i = τ/r = {f_var:.0f} Н, сила контакта c_i = k·f_i = {c_var:.0f} Н (k≈{k:.2f})")
print(f"  4 колеса: суммарная сила тяги ≈ {4*c_var:.0f} Н = {4*c_var/9.81:.0f} кгс")
print(f"  номинальная скорость {n_rpm} об/мин -> v = {v_max:.2f} м/с")
print(f"  сверка с паспортом: нагрузка 1000 кг/комплект при μ≈0.5-0.6 на линолеуме")
for G_kg in (300, 400, 450, 1000):
    N = G_kg * 9.81 / 4
    for mu in (0.3, 0.5):
        c_allow = mu * N
        f_allow = c_allow / k
        tau_allow = f_allow * r_roll
        verdict = "хватает" if tau_allow <= tau else "НЕ хватает момента"
        print(f"  G={G_kg} кг, μ={mu}: доп. момент на колесо {tau_allow:.1f} Н·м "
              f"(модуль {tau} Н·м) -> {verdict}; a_x_max = {4*c_allow/(G_kg):.2f} м/с²")

out = {"products": PRODUCTS, "ctm240_layout": rows, "min_diameter_ctm240": mins,
       "traction": {"k": round(k, 2), "f_i_rated_N": round(f_var, 0),
                    "c_i_rated_N": round(c_var, 0), "v_rated_mps": round(v_max, 2)},
       "notes": [
           "CTM1000-10R — единственный найденный модуль с ПОЛНЫМИ проверяемыми размерами и паспортом.",
           "Для D152/D203: колёса с размерами найдены (TZ-MW152), но мотор-редуктор/ступица/крепление — без размеров.",
           "Ширина модуля 106.9 мм МЕНЬШЕ условных W+60 (173 мм) — прежние допущения консервативны.",
       ]}
path = os.path.join(HERE, "stage2g_v6_real_drive.json")
with open(path, "w", encoding="utf-8") as f:
    json.dump(out, f, ensure_ascii=False, indent=1)
print("\nJSON:", path)