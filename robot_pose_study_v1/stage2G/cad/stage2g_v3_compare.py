# -*- coding: utf-8 -*-
"""Этап 2Г — stage2g_v3_compare.py (только Python, без FreeCAD).
Предварительная сравнительная компоновка по НОВЫМ требованиям v3:
  - транспортный габарит: ширина <= 640, высота <= 1900; дверь 700/900; лифт 500 кг, кабина 1100x2100;
  - варианты: A (без аутригеров), B (4 автоматических аутригера);
  - ходовая: Mecanum D152/D203/D230 (2L+2R), оси колёс || Y, прямоугольная компоновка.
  - НЕ делает расчётов моментов/прочности/устойчивости — только геометрия и бюджеты масс.
Результат: stage2g_v3_compare_results.json + печать таблиц.
"""
import json
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))

# ---------------- ВХОДНЫЕ ТРЕБОВАНИЯ v3 (предварительные, проектные) ----------------
REQ = {
    "transport_width_max": 640.0,      # целевой максимум ширины
    "transport_width_pref": (600.0, 620.0),  # желательный диапазон
    "transport_height_max": 1900.0,    # целевой максимум высоты (транспортное)
    "passage_height_min": 2000.0,      # минимальная высота прохода
    "door_bathroom": 700.0,            # санузел, в свету
    "door_ward": 900.0,                # палата, в свету
    "lift_capacity_min": 500.0,        # грузоподъёмность лифта, кг
    "cabin_width": 1100.0,             # кабина (мин.), мм
    "cabin_depth": 2100.0,             # кабина (мин.), мм
    "cabin_door_width": 1100.0,        # проём двери кабины (мин.), мм
    # высота проёма кабины НЕ указана — не выдумывается (см. список исходных данных)
}

# Параметры компоновки (проектные, на согласование)
WHEELBASE = 1500.0          # расстояние между осями колёс по X (x_c = +/-750)
WELL_MARGIN = 4.0           # зазор ниши (биение/подвеска), мм
STROKE_STUDY = [200.0, 300.0, 400.0]   # обследуемые ходы аутригеров, мм (параметр!)

# Ходовые: классы промышленных колёс (офиц. MECAWHEEL/TZBOT, см. отчёт)
WHEELS = {
    "MX152": {"D": 152.4, "W": 80.0,  "load_set_kg": 300, "src": "MECAWHEEL MX152 / TZ-MW152"},
    "MX203": {"D": 203.0, "W": 103.0, "load_set_kg": 600, "src": "MECAWHEEL MX203"},
    "MX230": {"D": 230.0, "W": 113.0, "load_set_kg": 800, "src": "MECAWHEEL MX230"},
}
LOAD_SET_KG_ALT = {"MX152": 600}   # TZ-MW152: 600 кгс/компл в классе 152 (см. таблицу)

# ---------------- Массовые бюджеты (ЛИФТ 500 кг — отдельно от реакции колеса!) ----------
MASS_TIERS = [300.0, 375.0, 450.0]        # предв. диапазон массы робота (313-425 НЕ подтверждены)
K_FACTORS = {"низкий": 1.5, "расчётный": 2.0, "верхний": 2.5}   # к на согласование
G = 9.81
MASS_PERSON_PLAN = 100.0   # planning-значение массы человека (согласовать с данными здания) — НЕ норматив

def solve_alpha_max(W, L, limit):
    """Максимальный угол курса (град), при котором габарит проёма W*cos+L*sin <= limit."""
    lo, hi = 0.0, 45.0
    for _ in range(60):
        mid = (lo + hi) / 2
        a = math.radians(mid)
        if W * math.cos(a) + L * math.sin(a) <= limit:
            lo = mid
        else:
            hi = mid
    return lo

def build_option(variant, wkey, spec):
    D, W = spec["D"], spec["W"]
    y_c = REQ["transport_width_max"] / 2.0 - W / 2.0     # колёса по краям ширины
    x_c = WHEELBASE / 2.0
    L = 2.0 * (x_c + D / 2.0)                            # длина габарита по X
    T = 2.0 * y_c                                        # опорная колея (по Y)
    diag = math.hypot(L, REQ["transport_width_max"])
    well_R = math.hypot(D / 2.0, W / 2.0) + WELL_MARGIN  # R ниши по габариту D x W (методика v3)
    o = {
        "variant": variant, "wheel": wkey,
        "D": D, "W": W, "load_set_kg": spec["load_set_kg"], "src": spec["src"],
        "x_c": x_c, "y_c": round(y_c, 1),
        "length_L": round(L, 1), "width_W": REQ["transport_width_max"],
        "height_H": REQ["transport_height_max"],
        "wheelbase_B": WHEELBASE, "track_T": round(T, 1),
        "support_area_A": round(T * WHEELBASE / 1e6, 4),         # вариант A, м^2
        "diag": round(diag, 1),
        "alpha_max_door700": round(solve_alpha_max(REQ["transport_width_max"], L, REQ["door_bathroom"]), 2),
        "alpha_max_door900": round(solve_alpha_max(REQ["transport_width_max"], L, REQ["door_ward"]), 2),
        "well_R_req": round(well_R, 1),
        "cabin_slack_width": round(REQ["cabin_width"] - REQ["transport_width_max"], 1),
        "cabin_slack_depth": round(REQ["cabin_depth"] - L, 1),
        "cabin_diag_possible": round(diag, 1) < round(math.hypot(REQ["cabin_width"], REQ["cabin_depth"]), 1),
        "height_margin_vs_passage": REQ["passage_height_min"] - REQ["transport_height_max"],
        "support_B": {},   # вариант B: колея/площадь по ходам аутригеров
    }
    if variant == "B":
        for s in STROKE_STUDY:
            T_s = T + 2 * s
            o["support_B"][str(int(s))] = {
                "track": round(T_s, 1),
                "area": round(T_s * WHEELBASE / 1e6, 4),
                "pad_y": round(y_c + s, 1),
            }
    return o

options = []
for variant in ("A", "B"):
    for wkey, spec in WHEELS.items():
        options.append(build_option(variant, wkey, spec))

# ---------------- Бюджет лифта (500 кг) и отдельно — реакция колеса ----------------
lift_budget = []
for M in MASS_TIERS:
    lift_budget.append({
        "M_robot": M,
        "remaining_for_outriggers_eq_payload_people": round(REQ["lift_capacity_min"] - M, 1),
        "people_at_100kg": int((REQ["lift_capacity_min"] - M) // MASS_PERSON_PLAN),
    })

wheel_load = []
for M in MASS_TIERS:
    row = {"M_robot": M}
    for name, k in K_FACTORS.items():
        F_N = M * G / 4.0 * k
        row[f"k={k}"] = {"F_per_wheel_N": round(F_N, 0), "F_per_wheel_kgf": round(F_N / G, 0),
                         "set_kgf": round(4 * F_N / G, 0)}
    wheel_load.append(row)

# Соответствие классов колёс расчётным нагрузкам (на комплект, кгс)
requirement_tiers = {
    "низкий (300 кг, k=1.5)": 450.0,
    "расчётный (375 кг, k=2.0)": 750.0,
    "верхний (450 кг, k=2.5)": 1125.0,
}
fit_matrix = {}
for wkey, spec in WHEELS.items():
    alt = LOAD_SET_KG_ALT.get(wkey, spec["load_set_kg"])
    rating = max(spec["load_set_kg"], alt)
    fit_matrix[wkey] = {t: ("OK" if rating >= v else "НЕТ")
                        for t, v in requirement_tiers.items()}

# ---------------- Сохранение ----------------
out = {
    "requirements": {k: (list(v) if isinstance(v, tuple) else v) for k, v in REQ.items()},
    "params": {"wheelbase": WHEELBASE, "well_margin": WELL_MARGIN,
               "stroke_study_mm": STROKE_STUDY, "mass_person_plan_kg": MASS_PERSON_PLAN},
    "options": options,
    "lift_budget_500kg": lift_budget,
    "wheel_reaction_separate": wheel_load,
    "wheel_class_fit_vs_tiers": fit_matrix,
    "notes": [
        "Габарит 640x(1652..1730)x1900 — прямоугольная компоновка, колёса у бортов, оси || Y.",
        "Методика ниши v3: R >= sqrt((D/2)^2+(W/2)^2)+зазор; свиверные повороты исключены (Mecanum не рулится).",
        "Бюджет лифта (500 кг) и реакция колеса — независимые проверки.",
        "313-425 кг — НЕ подтверждённая масса; диапазон 300-450 и коэф. k=1.5..2.5 — на согласование.",
        "Высота проёма кабины лифта не задана — не выдумывалась.",
    ],
}
path = os.path.join(HERE, "stage2g_v3_compare_results.json")
with open(path, "w", encoding="utf-8") as f:
    json.dump(out, f, ensure_ascii=False, indent=1)

# ---------------- Печать таблиц ----------------
print("== ТРАНСПОРТНЫЕ ГАБАРИТЫ И ПРОХОДИМОСТЬ ==")
print(f"{'Вар':3} {'Колесо':6} {'L':>7} {'W':>5} {'H':>5} {'Диаг':>7} "
      f"{'a700':>5} {'a900':>5} {'Кабина-запас':>14} {'Опора A, м2':>11}")
for o in options:
    print(f"{o['variant']:3} {o['wheel']:6} {o['length_L']:7.1f} {o['width_W']:5.0f} "
          f"{o['height_H']:5.0f} {o['diag']:7.1f} {o['alpha_max_door700']:5.2f} "
          f"{o['alpha_max_door900']:5.2f} {o['cabin_slack_width']:6.0f}/{o['cabin_slack_depth']:6.0f} "
          f"{o['support_area_A']:11.4f}")

print("\n== ВАРИАНТ B: опорный контур по ходам аутригеров (параметр) ==")
for o in options:
    if o["variant"] != "B":
        continue
    for s, v in o["support_B"].items():
        print(f"  {o['wheel']}: ход {s} мм -> колея {v['track']} мм, площадь {v['area']} м2, pad_y=+/-{v['pad_y']}")

print("\n== БЮДЖЕТ ЛИФТА 500 кг (отдельно от реакции колеса) ==")
for r in lift_budget:
    print(f"  M_робот={r['M_robot']:.0f} кг -> остаток на аутригеры+оборуд.+груз+людей = "
          f"{r['remaining_for_outriggers_eq_payload_people']:.0f} кг "
          f"(~{r['people_at_100kg']} чел. по 100 кг, если больше ничего)")

print("\n== РЕАКЦИЯ КОЛЕСА (операционная, отдельно) ==")
for r in wheel_load:
    parts = " | ".join(f"k={k}: {v['F_per_wheel_kgf']:.0f} кгс (компл {v['set_kgf']:.0f})"
                       for k, v in r.items() if k.startswith("k="))
    print(f"  M_робот={r['M_robot']:.0f} кг -> {parts}")

print("\n== ПРИГОДНОСТЬ КЛАССОВ КОЛЁС vs УРОВНИ НАГРУЗКИ (на комплект, кгс) ==")
for wkey, row in fit_matrix.items():
    print(f"  {wkey} ({max(WHEELS[wkey]['load_set_kg'], LOAD_SET_KG_ALT.get(wkey, 0))} кгс): " +
          ", ".join(f"{t}={v}" for t, v in row.items()))

print("\nJSON:", path)
