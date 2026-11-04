# -*- coding: utf-8 -*-
"""Этап 2Г, задача 3 — stage2g_v7_dynamics.py (только Python).

Динамика R2: Ø240, редуктор CTM1000-10R (i=37.1, τ_out=74 Н·м, 56 об/мин).
Все числа параметрические там, где источник не подтверждён (μ, Crr, η, κ).

Сцепление: сила контакта от привода направлена по оси ролика b_i (виртуальная
работа: T_i·δω_i = F_i·(b_i·δu_i), кинематика: b_i·δu_i = −δω_i·r_eff·(d_i·b_i))
=> |F_i| = √2·|T_i|/r_eff  (sin45 в знаменателе через d·b).

Нагрузка на колесо (точечная 4-опора, линейное распределение):
  N_i = M·g/4 · (1 + 2·(e·p_i)/r_c²)   (ромб вершинами на p_i).
"""
import json
import math
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))

# ============ Паспорт CTM1000-10R (две страницы производителя) ============
PASSPORT = {
    "payload_set_kg": 1000.0,      # обе страницы
    "D": 240.0, "W": 106.9,        # обе («outer space between two sides»)
    "weight_kg": 25.0,             # heavy: 25; industrial: 10 без амморт. / ~25 с ним
    "weight_no_shock_kg": 10.0,    # industrial
    "gear_ratio": 37.1,            # обе
    "tau_rated_out": 74.0,         # обе (вывод: 2 Нм мотора × 37.1 = 74.2 ✓)
    "tau_peak_motor": 4.0,         # industrial => ~148 Н·м выход (×37.1, до потерь)
    "tau_bep_motor": 2.0,          # industrial
    "n_out_rated_rpm": 56.0,       # heavy
    "n_motor_rpm": 3000.0,         # industrial (холостой: 3000/37.1=80.8 об/мин выход)
    "voltage": "24-48 (heavy) / 48 (industrial)",
    "power_W": "220-440 (heavy) / 180 (industrial)",
    "IP": "IP44 (heavy) / IP54 (industrial)",
    "encoder": "4096, магнитный (обе)",
}

# ============ Контроль согласованности паспорта ============
w_out_rated = 74.0 * 56.0 * 2 * math.pi / 60.0          # 434 Вт мех на выходе
p_needed_e = w_out_rated / 0.88                          # электрическая при η=0.88
n_noload = 3000.0 / 37.1
CONSISTENCY = {
    "P_out_at_rated_W": round(w_out_rated, 0),
    "P_elec_needed_W": round(p_needed_e, 0),
    "verdict_180W": f"180 Вт НЕСОГЛАСУЕМ: 74 Н·м×56 об/мин = {round(w_out_rated)} Вт вых.",
    "verdict_440W": f"440 Вт СОГЛАСУЕМ (≥ {round(p_needed_e)} Вт при η=0.88)",
    "n_out_noload_rpm": round(n_noload, 1),
    "v_noload_mps": round(n_noload / 60 * 2 * math.pi * 0.12, 2),
    "v_rated_mps": round(56.0 / 60 * 2 * math.pi * 0.12, 2),
    "tau_peak_out_est": "4 Нм×37.1 ≈ 148 Н·м (до потерь редуктора) — пик/непрерывный? НЕ ПОДТВЕРЖДЕНО",
}
print("== Согласованность паспорта ==")
for k, v in CONSISTENCY.items():
    print(f"  {k}: {v}")

# ============ Геометрия и параметры ============
R_EFF = 0.12            # м (эффективный; κ=0.97..1.0 параметр)
R_C = 0.2215            # м, положение колеса (радиальное)
MU = {"сухой линолеум": (0.40, 0.60), "влажный линолеум": (0.25, 0.45)}
K_MEC = (0.70, 1.00)    # понижение μ для mecanum-контакта — НЕ ИСТОЧНИК, параметр
CRR = (0.02, 0.06)      # сопротивление качению mecanum — НЕ ИСТОЧНИК, параметр
G = 9.81


def force_from_torque(tau):
    """Сила вдоль оси ролика при моменте на выходном вале."""
    return math.sqrt(2.0) * tau / R_EFF


def n_wheels(M_kg, ex=0.0, ey=0.0):
    """Нормальные реакции 4 колёс (ромб), ЦМ со смещением (ex,ey), кгс->Н."""
    pts = [np.array([R_C, 0]), np.array([0, R_C]),
           np.array([-R_C, 0]), np.array([0, -R_C])]
    e = np.array([ex, ey])
    N = []
    for p in pts:
        N.append(M_kg * G / 4.0 * (1.0 + 2.0 * (e @ p) / R_C ** 2))
    return N


M_GROSS = (300.0, 400.0, 450.0)     # масса робота — НЕ ПОДТВЕРЖДЕНА (диапазон из v3)
F_I_RATED = force_from_torque(74.0)          # 872 Н на колесо при 74 Н·м
F_I_PEAK = force_from_torque(148.0)          # 1744 Н (пик, недлительный)

print(f"\n== Сила контакта от момента: 74 Н·м -> {F_I_RATED:.0f} Н/колесо; 148 -> {F_I_PEAK:.0f} Н ==")

# ============ 1. Прямое/боковое движение: силовые пределы ============
print("\n== 1. Прямое/боковое: предел F_platform ==")
motions = {}
for name, M_kg in zip(("300", "400", "450"), M_GROSS):
    N4 = n_wheels(M_kg)
    row = {"M_kg": M_kg, "N_wheel_kg": [round(x / G, 1) for x in N4]}
    for case, (lo, hi) in MU.items():
        for mu, km in ((lo * K_MEC[0], "κ=0.7"), (hi, "κ=1.0")):
            tag = f"{case} (μ={mu:.2f}, {km})"
            f_slip = min(mu * n for n in N4)        # предел по самому нагружённому колесу
            F_slip = f_slip / 0.3536                # min-max: max|f_i|=0.3536·F_x
            F_torque = F_I_RATED / 0.3536           # от 74 Н·м
            F_min = min(F_slip, F_torque)
            row[tag] = {"F_slip_Н": round(F_slip, 0), "F_torque_Н": round(F_torque, 0),
                        "a_max_м_с2": round(F_min / M_kg, 2)}
    motions[name] = row
    k_w, k_b = f"сухой линолеум (μ={MU['сухой линолеум'][0]*K_MEC[0]:.2f}, κ=0.7)", \
               f"сухой линолеум (μ={MU['сухой линолеум'][1]:.2f}, κ=1.0)"
    k_wet = f"влажный линолеум (μ={MU['влажный линолеум'][0]*K_MEC[0]:.2f}, κ=0.7)"
    print(f"  M={M_kg} кг: N/колесо={[round(x/G,1) for x in N4]} кг; "
          f"a_max: сух.худ={row[k_w]['a_max_м_с2']}, сух.луч={row[k_b]['a_max_м_с2']}, "
          f"влажн.худ={row[k_wet]['a_max_м_с2']} м/с²; "
          f"F(привод)={row[k_b]['F_torque_Н']:.0f} Н")

# ============ 2. Вращение на месте ============
print("\n== 2. Вращение на месте (M_z) ==")
rot = {}
for M_kg in M_GROSS:
    N4 = n_wheels(M_kg)
    f_slip = 0.4 * min(N4)                          # худший при μ=0.4 (без запаса mecanum)
    Mz_slip = f_slip / 1.5961                       # max|f| = 1.5961 Н на 1 Н·м (из v7-kin)
    Mz_torque = F_I_RATED / 1.5961                  # от 74 Н·м
    Mz_peak = F_I_PEAK / 1.5961
    # сравнение с P: max|f| = 1.173
    rot[str(M_kg)] = {"Mz_slip_Нм": round(Mz_slip, 0), "Mz_torque_Нм": round(Mz_torque, 0),
                      "Mz_peak_Нм": round(Mz_peak, 0),
                      "было_бы_у_P_при_том_же_μ": round(Mz_slip * 1.5961 / 1.173, 0)}
    print(f"  M={M_kg} кг: Mz(сцепл. μ=0.4)={Mz_slip:.0f} Н·м; Mz(привод 74)={Mz_torque:.0f} Н·м; "
          f"для P сцепление дало бы {Mz_slip*1.5961/1.173:.0f} Н·м (+36% к R)")

# ============ 3. Скорость/разгон/торможение ============
print("\n== 3. Скорость и разгон ==")
V_RATED = CONSISTENCY["v_rated_mps"]
V_NOLOAD = CONSISTENCY["v_noload_mps"]
acc = {}
# прямой расчёт: min-max распределение сил => F ≤ 0.707·μ·M·g (см. раздел 1)
a_dry = (0.707 * 0.40, 0.707 * 0.60)
a_wet = (0.707 * 0.25, 0.707 * 0.45)
print(f"  a_max(сухой, κ=1.0): {a_dry[0]*G:.2f}..{a_dry[1]*G:.2f} м/с² (с коэфф. 0.707 распределения)")
print(f"  a_max(сухой, κ_mec=0.7): {a_dry[0]*0.7*G:.2f}..{a_dry[1]*0.7*G:.2f} м/с²")
print(f"  a_max(влажный): {a_wet[0]*G:.2f}..{a_wet[1]*0.7*G:.2f} м/с²")
print(f"  разгон 0..0.7 м/с (a=0.5 м/с² типичный комфорт): t = {0.7/0.5:.1f} с")
print(f"  торможение 0.7->0 за 1 м (a=0.245 м/с²) — запас до предела (сух.худ): {(0.707*0.4*G)/0.245:.0f}×")

acc = {"a_dry_g": [round(x * G, 2) for x in a_dry],
       "a_dry_kmec07": [round(x * 0.7 * G, 2) for x in a_dry],
       "a_wet_g": [round(a_wet[0] * G, 2), round(a_wet[1] * 0.7 * G, 2)],
       "v_rated": V_RATED, "v_noload": V_NOLOAD,
       "note": "предел сцепления: F≤μ·M·g (min-max по колёсам даёт тот же итог)"}

# ============ 4. Сопротивление качению и мощность ============
print("\n== 4. Мощность (качение + разгон) ==")
power = {}
for M_kg in M_GROSS:
    for crr in (CRR[0], CRR[1]):
        F_rr = crr * M_kg * G
        P_cruise = F_rr * V_RATED
        key = f"M={M_kg},Crr={crr}"
        power[key] = {"F_rr_Н": round(F_rr, 0), "P_cruise_Вт": round(P_cruise, 0),
                      "P_на_колесо_Вт": round(P_cruise / 4, 0),
                      "P_elec_на_колесо_Вт": round(P_cruise / 4 / 0.85, 0)}
        print(f"  {key}: F_сопр={F_rr:.0f} Н, P巡航={P_cruise:.0f} Вт "
              f"({P_cruise/4:.0f} Вт/колесо, электр. {P_cruise/4/0.85:.0f} Вт) "
              f"против пасп. 180/440 Вт")
# разгонная мощность
P_acc = [(M * 0.7 * V_RATED) for M in M_GROSS]
print(f"  P_разгон a=0.7: {', '.join(f'{p:.0f}' for p in P_acc)} Вт (суммарно)")

# ============ 5. Удержание при отключении питания ============
print("\n== 5. Удержание при отключении питания ==")
hold = {}
for grade in (0.0, 0.05):                     # плоский пол / 5% уклон (параметр!)
    F_hold = grade * 400 * G
    tau_hold = F_hold * 0.3536 * R_EFF / math.sqrt(2.0) / 1.0  # обратная связь через кинематику
    tau_hold = F_hold * 0.3536 * (R_EFF / math.sqrt(2.0))      # T = F_i·r/√2 (из дуальности)
    hold[f"уклон_{grade*100:.0f}%"] = {"F_Н": round(F_hold, 0),
                                       "tau_на_колесо_Нм": round(tau_hold, 1)}
    print(f"  уклон {grade*100:.0f}%: F={F_hold:.0f} Н, удерживающий τ≈{tau_hold:.1f} Н·м/колесо")
hold["verdict"] = ("Тип редуктора (планетарный/червячный) и наличие тормоза НЕ ПОДТВЕРЖДЕНЫ: "
                   "планетарный 37.1 — обратно-проводимый (нет удержания без тормоза); "
                   "удержание при отключении — ВОПРОС ПРОИЗВОДИТЕЛЮ")

# ============ 6. Распределение нагрузки и потеря контакта ============
print("\n== 6. Распределение нагрузки ==")
loads = {}
for M_kg in M_GROSS:
    for ex in (0.0, 0.05, 0.10):
        N = n_wheels(M_kg, ex=ex, ey=0.0)
        loads[f"M={M_kg},e={ex}"] = {"N_кг": [round(x / G, 1) for x in N],
                                     "макс_N_кг": round(max(N) / G, 1),
                                     "предел_250кг": "OK" if max(N) / G <= 250 else "ПРЕВЫШЕНИЕ!"}
    print(f"  M={M_kg}, e=0.10: N={[round(x/G,1) for x in n_wheels(M_kg, ex=0.10)]} кг "
          f"(предел 250 кг/колесо)")
# динамика: перенос при a_x
for a in (0.5, 1.0):
    dN = 400 * a * 0.5 / (2 * R_C)     # h_цм=0.5 м параметр
    print(f"  перенос при a={a} м/с² (h=0.5 м): ΔN={dN:.0f} Н = {dN/G:.0f} кг на колесо")
loads["динамический_перенос_400кг_h0.5"] = {
    f"a={a}": f"{400*a*0.5/(2*R_C)/G:.0f} кг" for a in (0.5, 1.0)}
loads["потеря_контакта"] = ("При ЦМ в центре потеря одного колеса => ЦМ ЛЕЖИТ НА краю "
                            "опорного треугольника (симметрия 4-точечной опоры) => запас 0. "
                            "Нужен вынос ЦМ к выжившим колёсам или оперативная реакция.")

# ============ 7. Тепловой режим ============
print("\n== 7. Тепловой режим (оценка, параметрически) ==")
th = {}
for M_kg in (400,):
    for crr in CRR:
        P = crr * M_kg * G * V_RATED
        loss = P / 4 * (1 / 0.85 - 1)
        th[f"Crr={crr}"] = {"P_потерь_на_колесо_Вт": round(loss, 1),
                            "вывод": "мало (естественное охлаждение)" if loss < 30 else "проверить"}
        print(f"  M={M_kg}, Crr={crr}: потери на колесо ≈ {loss:.0f} Вт — "
              f"{'в пределах естественного' if loss < 30 else 'нужен режим!'}")
th["пик_разгона"] = "a=0.7 м/с², 0..0.7 м/с: <2 с — кратковременный режим 148 Н·м допустим (предположение)"
th["непрерывный_северный_ход"] = "при Crr≤0.04: ≤10 Вт/колесо потерь — тривиально, НО: северный ход (вбок) μ/Crr неизвестны"

out = {"passport": PASSPORT, "consistency": CONSISTENCY,
       "F_i_rated_Н": round(F_I_RATED, 0), "F_i_peak_Н": round(F_I_PEAK, 0),
       "motions": motions, "rotation": rot, "accel": acc, "power": power,
       "holding": hold, "loads": loads, "thermal": th,
       "notes": [
           "μ линолеума: сухой 0.4-0.6 (Gerflor/DIN V 18032-2 — смежный стандарт, 🟡), влажный — параметр 🔴.",
           "κ_mec (потеря μ для mecanum) и Crr — НЕ ПОДТВЕРЖДЕНЫ, диапазоны параметрические.",
           "Паспортная грузоподъёмность 1000 кг НЕ означает тягу: тяга ограничена μ и 74 Н·м (см. разделы 1-2).",
           "180 Вт против 74 Н·м×56 об/мин — противоречие подтверждено расчётом мощности.",
       ]}
path = os.path.join(HERE, "stage2g_v7_dynamics.json")
with open(path, "w", encoding="utf-8") as f:
    json.dump(out, f, ensure_ascii=False, indent=1)
print("\nJSON:", path)
