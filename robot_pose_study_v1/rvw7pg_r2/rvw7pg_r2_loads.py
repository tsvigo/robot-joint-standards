# -*- coding: utf-8 -*-
"""Этап 2Г RVW-7PG, задача 2 — тяга, момент, мощность (rvw7pg_r2_loads.py).

Методика — ФОРМУЛЫ КАТАЛОГА Nabtesco RVW (©2020, стр.35-38) — DOC:
  разгон/торможение (плоскость): Tw = W/4 · (a + 9.8·Crr·C) · Dw/2 · 1e-3
  склон (равномерно):           Tw = W/4 · 9.8·(sinθ + Crr·C·cosθ) · Dw/2·1e-3
  склон + разгон:               Tw = W/4 · {a + 9.8·(sinθ + Crr·C·cosθ)} · Dw/2·1e-3
  мотор:                        Tm = (Tw/0.95 + T_no)/i
  C: вперёд/назад 1, вращение 1, traverse 2, diagonal 2.5
  (на склоне диагональ запрещена производителем по безопасности)
Ограничения DOC: Tw ≤ 27 Н·м (acc/dec), Tw ≤ 7 Н·м непр. (rated); правило 3/4:
  суммарная масса < 3/4 × 200 × 4 = 600 кг; нагрузка/колесо ≤ 200 кгс.
T_no (no-load torque, стр.39 — только график) — ПЕРЕМЕННАЯ, не факт.
Массы 300/400/450 кг — сценарии. Сетка: скорости/ускорения/уклоны по заданию.
"""
import itertools
import json
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))

# ---- DOC-параметры RVW-7PG (каталог Nabtesco 2020) ----
DW = 0.178            # м, DOC wheel diameter
CRR = 0.05            # DOC rolling resistance coefficient
I_RATIO = 30           # DOC speed ratio
T_RATED = 7.0          # DOC rated torque (reduction gear unit)
T_ACCMAX = 27.0        # DOC allowable acceleration/deceleration torque
V_MAX = 1.0            # м/с, DOC 60 м/мин
LOAD_MAX = 200.0       # кгс/колесо DOC (maximum radial load)
RULE_34 = 0.75 * LOAD_MAX * 4   # 600 кг — рекомендация 3/4

# ---- ПЕРЕМЕННЫЕ (не паспортные) ----
T_NO = 1.5             # Н·м — оценка по графику стр.39 (UNK, параметр)
ETA_GEAR = 0.95        # КПД редуктора из формулы каталога (0.95 — DOC множитель)
MASS = [300.0, 400.0, 450.0]
SPEEDS = [0.2, 0.5, 0.7, 1.0]
ACCELS = [0.0, 0.1, 0.2, 0.3, 0.5]
GRADES = [0.0, 0.01, 0.03, 0.05, 0.08]      # 0/1/3/5/8 %
MODES = {  # DOC correction coefficients C
    "вперёд/назад": 1.0,
    "вращение": 1.0,
    "боковое (traverse)": 2.0,
    "диагональ (flat only)": 2.5,
}
# неравномерность (ПАРАМЕТРЫ, не DOC): перекос ЦМ, потеря опоры
K_UNEVEN = [1.0, 1.25, 1.5]           # множитель на долю колеса
LOST_CONTACT = False                   # потеря контакта 1 колеса: 3 опоры

G = 9.81
CHECKS = []


def t(cid, ok, detail):
    CHECKS.append({"id": cid, "ok": bool(ok), "detail": detail})
    print(f"  [{'PASS' if ok else 'FAIL'}] {cid}: {detail}")


def tw(mode_C, grade, accel, W=None, k=1.0, m_per_wheel=None):
    """Tw на одно колесо, Н·м (формулы каталога). W — суммарная масса, кг."""
    if m_per_wheel is not None:
        m = m_per_wheel
    else:
        m = W / 4.0 * k
    th = math.atan(grade)               # угол уклона из градиента
    term = accel + G * (math.sin(th) + CRR * mode_C * math.cos(th))
    return m * term * DW / 2.0, th


# ============ 2.1 Нагрузки (вертикаль) ============
print("== 2.1 Вертикальные нагрузки на колесо (кгс) ==")
vert = {}
for M in MASS:
    base = M / 4.0
    row = {"равномерно": round(base, 1),
           "перекос k=1.25": round(base * 1.25, 1),
           "перекос k=1.5": round(base * 1.5, 1),
           "потеря 1 опоры (3 колеса)": round(M / 3.0, 1),
           "потеря опоры + k=1.25": round(M / 3.0 * 1.25, 1)}
    row["статус_200кгс"] = "FAIL" if max(row.values()) > LOAD_MAX else "PASS"
    vert[M] = row
    print(f"  {M:.0f} кг: равн {row['равномерно']}, k1.25 {row['перекос k=1.25']}, "
          f"k1.5 {row['перекос k=1.5']}, потеря опоры {row['потеря 1 опоры (3 колеса)']} "
          f"-> {row['статус_200кгс']} (лимит {LOAD_MAX} кгс, radial DOC)")
    print(f"      правило 3/4: {M} < {RULE_34} кг -> "
          f"{'PASS' if M < RULE_34 else 'FAIL'} (каталог, рекомендация)")

# ============ 2.2-2.3 Сетка режимов: Tw и сравнение с 7/27 Н·м ============
print("\n== 2.2-2.3 Сетка режимов: массы x ускорения x уклоны x режимы ==")
grid = []
for M, a, gr, (mode, C) in itertools.product(MASS, ACCELS, GRADES, MODES.items()):
    if mode.startswith("диагональ") and gr > 0:
        continue            # запрет диагонали на склоне — DOC
    Tw, th = tw(C, gr, a, W=M)
    grid.append({"M": M, "a": a, "grade_pct": round(gr * 100, 1), "mode": mode,
                 "C": C, "Tw_Нм": round(Tw, 2),
                 "сравнение": ("≤7 rated" if Tw <= T_RATED else
                               "7..27 кратк." if Tw <= T_ACCMAX else
                               ">27 ПРЕВЫШЕНИЕ")})
over7 = [g for g in grid if g["Tw_Нм"] > T_RATED]
over27 = [g for g in grid if g["Tw_Нм"] > T_ACCMAX]
print(f"  точек сетки: {len(grid)}; Tw>7 Н·м: {len(over7)}; Tw>27 Н·м: {len(over27)}")
# экстремумы по режимам
from collections import defaultdict
best = defaultdict(lambda: (0, None))
for g in grid:
    if g["Tw_Нм"] > best[g["mode"]][0]:
        best[g["mode"]] = (g["Tw_Нм"], g)
for mode, (tw_max, g) in sorted(best.items(), key=lambda kv: -kv[1][0]):
    print(f"  макс по режиму «{mode}»: {tw_max} Н·м при {g['M']} кг, "
          f"a={g['a']}, уклон {g['grade_pct']}% -> {g['сравнение']}")

# скоростная сетка: omega = v/(D/2); P_мех = Tw·omega
print("\n== 2.3b Мощность (по точкам максимального момента) ==")
power_rows = []
for M in MASS:
    for v in SPEEDS:
        w = v / (DW / 2)
        # худший Tw при этой массе без скорости (скорость не влияет на Tw по каталогу)
        tws = [g["Tw_Нм"] for g in grid if g["M"] == M]
        twc = [g["Tw_Нм"] for g in grid if g["M"] == M and g["mode"] == "вперёд/назад"
               and g["grade_pct"] == 0.0 and g["a"] == 0.0]   # крейс: a=0, плоскость
        twc_max = max(twc) if twc else 0.0
        tw_max = max(tws) if tws else 0.0
        power_rows.append({
            "M": M, "v": v,
            "n_об_мин": round(w * 60 / (2 * math.pi), 1),
            "Tw_крейс_Нм_вперёд_0%": twc_max,
            "P_мех_крейс_Вт": round(twc_max * w, 1),
            "Tw_макс_сетки_Нм": tw_max,
            "P_мех_макс_Вт": round(tw_max * w, 1),
            "статус_v": "OK" if v <= V_MAX else "> V_MAX DOC"})
for p in power_rows[:8]:
    print(f"  M={p['M']:.0f} v={p['v']}: n={p['n_об_мин']} об/мин, "
          f"Tw крейс {p['Tw_крейс_Нм_вперёд_0%']} -> P={p['P_мех_крейс_Вт']} Вт, "
          f"макс {p['Tw_макс_сетки_Нм']} Н·м -> {p['P_мех_макс_Вт']} Вт")

# ============ 2.4 Электрическая мощность и нагрев (параметрически) ============
print("\n== 2.4 Электрическая мощность и нагрев (параметрическая модель) ==")
heat = {}
for M in MASS:
    # крейс = равномерное движение по плоскости (a=0, 0%); удержание = a=0 на склоне
    tw_cruise = max(g["Tw_Нм"] for g in grid if g["M"] == M
                     and g["mode"] == "вперёд/назад" and g["grade_pct"] == 0.0
                     and g["a"] == 0.0)
    tw_grade = {p: max((g["Tw_Нм"] for g in grid if g["M"] == M
                        and g["grade_pct"] == p and g["mode"] == "вперёд/назад"
                        and g["a"] == 0.0), default=0)
                for p in (1.0, 3.0, 5.0, 8.0)}
    for v in (0.5,):
        w = v / (DW / 2)
        p_out = tw_cruise * w
        p_in = p_out / ETA_GEAR
        p_loss = p_in - p_out
        heat[M] = {"Tw_крейс_a0": round(tw_cruise, 2),
                   "Tw_удержание_а0": {f"{p}%": round(v, 2) for p, v in tw_grade.items()},
                   "P_вых_Вт": round(p_out, 1), "P_вход_Вт": round(p_in, 1),
                   "потери_редуктор_Вт": round(p_loss, 1),
                   "статус_крейс": ("крейс ≤7 rated — непрерывно допустимо"
                                    if tw_cruise <= T_RATED else
                                    "крейс > rated: непрерывность НЕ подтверждена"),
                   "статус_удержание": {f"{p}%": ("≤7" if v <= T_RATED else
                                                   "7..27 — только кратковременно/оговорить"
                                                   if v <= T_ACCMAX else ">27")
                                        for p, v in tw_grade.items()}}
    h = heat[M]
    tw_grid_max = max(g["Tw_Нм"] for g in grid if g["M"] == M)
    print(f"  {M:.0f} кг @0.5 м/с: Tw крейс(a=0) {h['Tw_крейс_a0']} -> {h['статус_крейс']}; "
          f"удержание a=0: {h['Tw_удержание_а0']} -> {h['статус_удержание']}; "
          f"Pвых крейс {h['P_вых_Вт']} Вт (max сетки {tw_grid_max} Н·м), "
          f"потери редуктора {h['потери_редуктор_Вт']} Вт")
print("  тепловая модель: НЕ подтверждена (нет паспортных тепловых данных): "
      "ограничения DOC — поверхность редуктора ≤60°C, среда -10..40°C, "
      ">40°C — досрочная смазка; параметры (θ_потерь мотора, R_th, t_нагрева) — UNK")

# длительные сценарии 15/30/60 мин — параметрически
long_run = {}
for mins in (15, 30, 60):
    long_run[mins] = {
        "допущение": "нагрев ограничен только DOC-ограничением 60°C; "
                     "время до 60°C НЕ вычислимо без R_th/потерь мотора (UNK)",
        "статус": "ТРЕБУЕТСЯ паспорт мотора (ток, КПД, тепловое сопротивление)"}
print(f"  длительность 15/30/60 мин: время до предела НЕ вычислимо (T_no, η_мотора, "
      f"R_th — UNK) -> {long_run[15]['статус']}")

# ============ 2.5 Безопасная остановка ============
brake = {
    "тормоз_в_модуле": "В КАТАЛОГЕ НЕ УПОМИНАЕТСЯ (grep 'brake' в каталоге — нет) -> UNK",
    "устойчивое_удержание": "каталог: отдельного стояночного тормоза нет; "
                            "удержание на уклоне — моментом мотора (Tm ≤ rated) или "
                            "аутригерами при работе манипулятора — решение не принято",
    "удержание_на_уклоне_моментом": {
        f"{int(gr*100)}%": round(tw(1.0, gr, 0.0, W=450.0)[0], 2)
        for gr in (0.01, 0.03, 0.05, 0.08)},
    "внешние_силы_манипулятора": "не рассчитаны (зависят от B подъёмника — не утверждён)",
    "отказ_1_привода": "расчёт не выполнен (требует динамики) — открытый вопрос",
    "медицинское_предупреждение": "каталог: при использовании в medical equipment "
                                  "требуется индивидуальная проверка и контакт с Nabtesco",
}
print("\n== 2.5 Удержание (450 кг, момент на колесо по формуле каталога) ==")
for k, v in brake["удержание_на_уклоне_моментом"].items():
    print(f"  уклон {k}: Tw={v} Н·м -> {'≤7 rated' if v <= 7 else '7..27 (кратк.)' if v <= 27 else '>27'}")

# ============ Итоговые заключения ============
t("T1_правило_3_4", all(M < RULE_34 for M in MASS),
  f"массы 300/400/450 < {RULE_34} кг (правило 3/4 каталога)")
t("T2_нагрузка_200", all(v["статус_200кгс"] == "PASS" for v in vert.values()),
  "все режимы вертикальной нагрузки ≤ 200 кгс (включая потерю опоры)")
t("T3_момент_крейс", all(h["Tw_крейс_a0"] <= T_RATED for h in heat.values()),
  "крейсерский Tw (плоскость, a=0) ≤ 7 Н·м для 300/400/450 кг: "
  + ", ".join(f"{m}:{h['Tw_крейс_a0']}" for m, h in zip(MASS, heat.values())))
n_over27 = len(over27)
t("T4_сетка_без_превышений", n_over27 == 0,
  f"точек сетки с Tw>27 Н·м: {n_over27} (см. таблицу — при "
  f"{sorted({g['M'] for g in over27})} кг и режимах "
  f"{sorted({g['mode'] for g in over27})})")

out = {"doc_params": {"DW": DW, "CRR": CRR, "i": I_RATIO, "T_rated": T_RATED,
                      "T_accmax": T_ACCMAX, "V_max": V_MAX, "load_max_kgs": LOAD_MAX,
                      "rule_34_kg": RULE_34},
       "variables": {"T_no": T_NO, "eta_gear": ETA_GEAR, "mass": MASS,
                     "k_uneven": K_UNEVEN,
                     "статус": "T_no — оценка по графику стр.39 (UNK)"},
       "vertical_loads": {str(k): v for k, v in vert.items()},
       "torque_grid": grid,
       "grid_summary": {"n": len(grid), "over_7": len(over7), "over_27": n_over27},
       "power": power_rows, "heat": {str(k): v for k, v in heat.items()},
       "long_run": long_run, "brake": brake,
       "checks": CHECKS,
       "notes": ["Формулы — каталог Nabtesco RVW (©2020), стр.35-38 (DOC).",
                 "Диагональное движение на склоне запрещено производителем.",
                 "Tw — момент НА КОЛЕСО; Tm мотора = (Tw/0.95 + T_no)/i."]}
path = os.path.join(HERE, "rvw7pg_r2_loads.json")
with open(path, "w", encoding="utf-8") as f:
    json.dump(out, f, ensure_ascii=False, indent=1)
print("\nJSON:", path)
print("REGRESSION:", "PASS" if all(c["ok"] for c in CHECKS) else "FAIL")