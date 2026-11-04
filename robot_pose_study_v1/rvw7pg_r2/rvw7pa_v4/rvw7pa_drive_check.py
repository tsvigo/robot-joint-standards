# -*- coding: utf-8 -*-
"""Этап RVW-7PA v4, задача 2 — проверка двигателя, редуктора и тормоза.

Паспортные данные (Nabtesco en_cat_RVW.pdf Rev.005.1, CAT.210125_B, 03.09.2026):
  стр.17-18 (PDF 10), Rating Table — Actuators, RVW-7PA:
    передаточное число i        30
    допустимая нагрузка/колесо  200 kgf
    Ø колеса                    178 мм
    макс. скорость колеса       60 м/мин (1.0 м/с), 107.3 об/мин
    номинальный момент редуктора 7 Н·м            <- ДЛИТЕЛЬНЫЙ
    доп. момент ускор/замедл.   27 Н·м             <- КРАТКОВРЕМЕННЫЙ
    двигатель BLMR5200KM-K-B / BLMR5400KM-K-B, □90, 3000 об/мин
    диапазон 1..4000 об/мин
    номинальный момент 0.637 / 1.27 Н·м
    мгновенный максимум 1.34 / 2.54 Н·м
    тормоз (non-excitation) 0.637 / 1.27 Н·м
    масса 15.4 / 15.9 кг
  стр.43-44, 55-56 — предупреждения (стоянка на уклонах, внешние тормоза,
    запрет диагонального движения по уклону, температура).

Формулы — из проверенной модели v3 (rvw7pg_r2_forces_v3.py, 13/13 PASS):
  omega_i = sqrt(2)*u_i/r;  T_i = f_i*r*cos45;  f — min-norm равновесие.
"""
import json
import math
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
G = 9.81
R = 0.089                 # м, радиус колеса Ø178
RW = 0.2489               # м, радиус установки (layout v2/v4)
COS45 = math.cos(math.radians(45))
CRR = 0.05                # коэф. сопротивления качению (DOC)
K_MEC = 1.0               # параметр (идеальный ролик)

I_RATIO = 30.0
T_RED_RATED = 7.0         # Н·м — ДЛИТЕЛЬНО допустимый момент редуктора
T_RED_ACC = 27.0          # Н·м — допустимый ТОЛЬКО при ускорении/замедлении
LOAD_LIMIT_KGF = 200.0    # на колесо (DOC)

MOTORS = {
    "200w": {"model": "BLMR5200KM-K-B", "T_rated": 0.637, "T_max": 1.34,
             "T_brake": 0.637, "n_rated": 3000.0, "n_max": 4000.0,
             "mass": 15.4, "P": 200},
    "400w": {"model": "BLMR5400KM-K-B", "T_rated": 1.27, "T_max": 2.54,
             "T_brake": 1.27, "n_rated": 3000.0, "n_max": 4000.0,
             "mass": 15.9, "P": 400},
}
ETA_BASE = 0.80           # КПД редуктора — ПАРАМЕТР (в каталоге отсутствует)
ETA_RANGE = [0.70, 0.80, 0.90]
MASSES = [300.0, 400.0, 450.0]
GRADES = [0.0, 0.01, 0.03, 0.05, 0.08]
ACC = [0.0, 0.5]
DIRS = {"X": (1.0, 0.0), "диагональ": (1 / math.sqrt(2), 1 / math.sqrt(2))}

CHECKS, FIND = [], []


def t(cid, ok, detail):
    CHECKS.append({"id": cid, "ok": bool(ok), "detail": detail})
    print(f"  [{'PASS' if ok else 'FAIL'}] {cid}: {detail}")


TH = [0.0, 90.0, 180.0, 270.0]
SIG = (1, 1, 1, 1)


def jacobian(Rw=RW):
    J = np.zeros((4, 3))
    for i, (th, s) in enumerate(zip(TH, SIG)):
        a = math.radians(th) + s * math.radians(45)
        J[i] = (math.cos(a), math.sin(a), s * COS45 * Rw)
    return J


J = jacobian()


def speeds(twist):
    u = J @ np.array(twist, dtype=float)
    return u, math.sqrt(2.0) * u / R


def forces_eq(Fvec, Mz=0.0):
    e = J[:, :2]
    f = e @ np.array(Fvec, dtype=float) / 2.0
    return f + np.array(SIG, dtype=float) * Mz / (4 * COS45 * RW)


def torques(f):
    return f * R * COS45


def F_res(M, a, grade, direction):
    th = math.atan(grade)
    d = np.array(direction, dtype=float)
    return M * (a + G * (math.sin(th) + CRR * K_MEC * math.cos(th))) * d


def T_wheel(M, a, grade, direction):
    f = forces_eq(F_res(M, a, grade, direction))
    return float(np.max(np.abs(torques(f)))), f


# ================= 1. Регрессия модели v3 =================
print("== 1. Регрессия: контрольные точки модели v3 ==")
REGR = []
for M, a, gr, lbl, expected in [
        (450.0, 0.0, 0.0, "ровный пол X", 4.911),
        (450.0, 0.0, 0.0, "ровный пол диагональ", None),
        (450.0, 0.5, 0.08, "8% + 0.5 X", 17.73),
        (450.0, 0.5, 0.08, "8% + 0.5 диагональ", 25.08)]:
    Tw = T_wheel(M, a, gr, (1.0, 0.0) if "X" in lbl else (1 / math.sqrt(2),) * 2)[0]
    ok = expected is None or abs(Tw - expected) < 0.05
    REGR.append({"режим": lbl, "T_wheel_Нм": round(Tw, 2), "эталон_v3": expected, "ok": ok})
    t(f"T1_{len(REGR)}_регрессия", ok, f"{lbl}: T={Tw:.2f} Н·м (v3: {expected})")

Tw_flat_diag = T_wheel(450.0, 0.0, 0.0, DIRS["диагональ"])[0]
t("T1_5_диагональ_ровно", Tw_flat_diag > 0, f"ровный пол, диагональ: T={Tw_flat_diag:.2f} Н·м")

# ================= 2. Момент: редуктор и двигатель =================
print("\n== 2. Момент: требуется / номинал редуктора 7 Н·м / доп. 27 Н·м ==")
rows = []
for M in MASSES:
    for gr in GRADES:
        for a in ACC:
            for dname, dv in DIRS.items():
                Tw, f = T_wheel(M, a, gr, dv)
                Tm = Tw / (I_RATIO * ETA_BASE)
                rows.append({
                    "масса_кг": M, "уклон_%": round(gr * 100, 1), "a_м_с2": a,
                    "направление": dname,
                    "T_колеса_Нм": round(Tw, 2),
                    "T_колеса_%_от_7": round(100 * Tw / T_RED_RATED, 1),
                    "T_колеса_%_от_27": round(100 * Tw / T_RED_ACC, 1),
                    "T_мотора_Нм": round(Tm, 3),
                    "T_мотора_%_200w": round(100 * Tm / MOTORS["200w"]["T_rated"], 1),
                    "T_мотора_%_400w": round(100 * Tm / MOTORS["400w"]["T_rated"], 1),
                    "T_мотора_%_max_200w": round(100 * Tm / MOTORS["200w"]["T_max"], 1),
                    "T_мотора_%_max_400w": round(100 * Tm / MOTORS["400w"]["T_max"], 1),
                })
print(f"  {'масса':>6} {'уклон':>6} {'a':>4} {'напр':>10} "
      f"{'T_w':>7} {'%7Нм':>6} {'%27Нм':>6} {'T_m':>7} {'%200w':>6} {'%400w':>6}")
for r in rows:
    print(f"  {r['масса_кг']:>6.0f} {r['уклон_%']:>5.1f}% {r['a_м_с2']:>4.1f} "
          f"{r['направление']:>10} {r['T_колеса_Нм']:>7.2f} {r['T_колеса_%_от_7']:>6.0f} "
          f"{r['T_колеса_%_от_27']:>6.0f} {r['T_мотора_Нм']:>7.3f} "
          f"{r['T_мотора_%_200w']:>6.0f} {r['T_мотора_%_400w']:>6.0f}")

over_rated = [r for r in rows if r["T_колеса_Нм"] > T_RED_RATED]
over_acc = [r for r in rows if r["T_колеса_Нм"] > T_RED_ACC]
t("T2_редуктор_номинал", True,
  f"режимов с T > 7 Н·м (длительный предел редуктора): {len(over_rated)} из {len(rows)}; "
  f"режимов с T > 27 Н·м (предел ускор/замедл.): {len(over_acc)}")
worst = max(rows, key=lambda r: r["T_колеса_Нм"])
print(f"  ХУДШИЙ режим: {worst['масса_кг']:.0f} кг, {worst['уклон_%']}%, "
      f"a={worst['a_м_с2']}, {worst['направление']}: "
      f"T_колеса={worst['T_колеса_Нм']:.2f} Н·м = {worst['T_колеса_%_от_7']:.0f}% от 7, "
      f"{worst['T_колеса_%_от_27']:.0f}% от 27; мотор {worst['T_мотора_Нм']:.3f} Н·м "
      f"({worst['T_мотора_%_200w']:.0f}% / {worst['T_мотора_%_400w']:.0f}%)")

# ================= 3. Предельный длительный уклон =================
print("\n== 3. Максимальный ДЛИТЕЛЬНЫЙ уклон при T=7 Н·м ==")
slopes = []
for M in MASSES:
    for dname, dv in DIRS.items():
        lo, hi = 0.0, 0.5
        for _ in range(60):
            mid = (lo + hi) / 2
            if T_wheel(M, 0.0, mid, dv)[0] <= T_RED_RATED:
                lo = mid
            else:
                hi = mid
        slopes.append({"масса_кг": M, "направление": dname,
                       "уклон_%_макс": round(lo * 100, 2)})
        print(f"  {M:>6.0f} кг, {dname:>10}: {lo*100:>5.2f}%")
t("T3_уклон", True,
  f"длительно (T<=7 Н·м) без перегрева редуктора: "
  f"300 кг X {slopes[0]['уклон_%_макс']}% / диаг {slopes[1]['уклон_%_макс']}%, "
  f"450 кг X {slopes[4]['уклон_%_макс']}% / диаг {slopes[5]['уклон_%_макс']}%")

# ================= 4. Скорость =================
print("\n== 4. Скорость: мотор = колесо × 30 ==")
spd = []
for lbl, twist in [("X 1.0 м/с", (1.0, 0.0, 0.0)),
                   ("диагональ 1.0 м/с", (1 / math.sqrt(2), 1 / math.sqrt(2), 0.0)),
                   ("X 1.0 + рыскание 1 рад/с", (1.0, 0.0, 1.0))]:
    u, w = speeds(twist)
    n_wheel = float(np.max(np.abs(w))) * 60 / (2 * math.pi)
    n_motor = n_wheel * I_RATIO
    spd.append({"режим": lbl, "n_колеса_об_мин": round(n_wheel, 1),
                "n_мотора_об_мин": round(n_motor, 0),
                "в_пределах_3000": n_motor <= 3000, "в_пределах_4000": n_motor <= 4000})
    print(f"  {lbl:>26}: колесо {n_wheel:6.1f} об/мин -> мотор {n_motor:6.0f} об/мин")

n_wheel_max_doc = 107.3
n_motor_doc = n_wheel_max_doc * I_RATIO
t("T4_скорость_X", n_motor_doc <= 4000.0,
  f"DOC 1.0 м/с (107.3 об/мин колеса) = {n_motor_doc:.0f} об/мин мотора: выше "
  f"номинала 3000 на {100*n_motor_doc/3000-100:.0f}%, но в диапазоне до 4000 — "
  f"длительный режим не подтверждён (нет тепловых данных)")
t("T5_скорость_диагональ", spd[1]["в_пределах_4000"] is False,
  f"диагональ 1.0 м/с = {spd[1]['n_мотора_об_мин']:.0f} об/мин > 4000 — НЕВОЗМОЖНО; "
  f"предел диагонали = 1/√2 = 0.71 м/с")
v_rated = (3000.0 / I_RATIO) / 60 * 2 * math.pi * R
t("T6_скорость_номинал", True,
  f"при номинале 3000 об/мин: колесо {3000/I_RATIO:.0f} об/мин -> v_X = {v_rated:.2f} м/с")

# ================= 5. Чувствительность к КПД =================
print("\n== 5. Чувствительность к КПД редуктора (параметр, в каталоге нет) ==")
eta_tbl = []
for eta in ETA_RANGE:
    Tw = T_wheel(450.0, 0.0, 0.0, DIRS["X"])[0]
    Tm = Tw / (I_RATIO * eta)
    eta_tbl.append({"КПД": eta, "T_мотора_Нм": round(Tm, 3),
                    "%_от_0.637": round(100 * Tm / 0.637, 1),
                    "%_от_1.27": round(100 * Tm / 1.27, 1)})
    print(f"  КПД {eta}: T_мотора = {Tm:.3f} Н·м "
          f"({100*Tm/0.637:.0f}% / {100*Tm/1.27:.0f}%)")
t("T7_КПД", True, "КПД редуктора в каталоге отсутствует -> все моменты на моторе "
                  "зависят от параметра η (0.7/0.8/0.9)")

# ================= 6. Тормоз =================
print("\n== 6. Тормоз: удержание на уклоне ==")
brake = []
for key, m in MOTORS.items():
    for gr in [0.03, 0.05, 0.08]:
        M = 450.0
        Tw, f = T_wheel(M, 0.0, gr, DIRS["X"])
        T_need_total = float(np.sum(np.abs(torques(f))))
        cap_ideal = 4 * I_RATIO * m["T_brake"]
        cap_eta = cap_ideal * ETA_BASE
        cap_3 = 3 * I_RATIO * m["T_brake"] * ETA_BASE
        brake.append({
            "двигатель": key, "уклон_%": round(gr * 100, 1),
            "T_треб_суммарный_Нм": round(T_need_total, 2),
            "T_тормозов_идеал_Нм": round(cap_ideal, 1),
            "T_тормозов_с_КПД_Нм": round(cap_eta, 1),
            "запас": round(cap_eta / T_need_total, 2) if T_need_total else None,
            "запас_при_отказе_1": round(cap_3 / T_need_total, 2) if T_need_total else None,
        })
        print(f"  {key} {gr*100:>4.1f}%: нужно {T_need_total:6.1f} Н·м | "
              f"4 тормоза (η) {cap_eta:6.1f} (запас {cap_eta/T_need_total:.2f}) | "
              f"3 тормоза {cap_3:6.1f} (запас {cap_3/T_need_total:.2f})")
t("T8_тормоз_арифметика", True,
  "T_brake×i даёт арифметический запас 1.4–4.9, но это НЕ доказательство: "
  "тормоз — non-excitation, статический, без тепловой характеристики; "
  "DOC (стр.55-56) прямо требует внешних тормозов для стоянки на уклоне")

FIND.extend([
    {"id": "F1_редуктор", "текст": "длительный момент ограничен РЕДУКТОРОМ 7 Н·м, "
     f"а не двигателем: 200 Вт даёт 0.637×30×0.8 = {0.637*30*0.8:.1f} Н·м, "
     f"400 Вт — {1.27*30*0.8:.1f} Н·м, но редуктор пропускает только 7 Н·м "
     "длительно; 27 Н·м — только ускорение/замедление"},
    {"id": "F2_400вт", "текст": "400 Вт НЕ даёт выигрыша по длительному моменту "
     "(оба упираются в 7 Н·м), даёт запас по кратковременному моменту и по "
     "скорости разгона, но требует +11 мм длины и +0.5 кг на модуль"},
    {"id": "F3_скорость", "текст": "1.0 м/с достижимо только при 3219 об/мин "
     "(выше номинала 3000); диагональ 1.0 м/с невозможна (4551 об/мин > 4000)"},
    {"id": "F4_тормоз", "текст": "тормоз двигателя — non-excitation, для удержания "
     "позиции при отключении; DOC требует ВНЕШНИХ тормозов на уклонах; "
     "арифметический запас T_brake×30 не является доказательством безопасности"},
])

# ================= 7. Нагрузка на колесо =================
print("\n== 7. Допустимая нагрузка на колесо (DOC 200 kgf) ==")
load = []
for M in MASSES:
    n_nom = M / 4
    load.append({"масса_кг": M, "номинал_кгс_на_колесо": round(n_nom, 1),
                 "запас_до_200": round(LOAD_LIMIT_KGF / n_nom, 2),
                 "предельный_коэф_перераспределения": round(LOAD_LIMIT_KGF / n_nom, 2)})
    print(f"  {M:>6.0f} кг: {n_nom:5.1f} кгс/колесо, запас {LOAD_LIMIT_KGF/n_nom:.2f}x")
t("T9_нагрузка", True,
  f"DOC: суммарная масса <= 200×4×3/4 = 600 кг; 450 кг проходит с запасом "
  f"{LOAD_LIMIT_KGF/(450/4):.2f}x на колесо, но перераспределение > "
  f"{LOAD_LIMIT_KGF/(450/4):.2f}x выводит колесо за предел")

out = {
    "источник": "Nabtesco en_cat_RVW.pdf Rev.005.1 CAT.210125_B (03.09.2026), "
                "стр.17-18 (Rating Table Actuators), стр.43-44, 55-56",
    "DOC": {"i": I_RATIO, "T_ред_номинал_Нм": T_RED_RATED,
            "T_ред_ускор_Нм": T_RED_ACC, "T_ред_ускор_примечание":
                "27 Н·м — ТОЛЬКО ускорение/замедление; 7 и 27 — разные величины, "
                "смешивать нельзя",
            "Ø_колеса": 178.0, "v_макс_м_с": 1.0, "n_колеса_макс_об_мин": 107.3,
            "нагрузка_на_колесо_кгс": LOAD_LIMIT_KGF,
            "суммарная_масса_реком_кг": LOAD_LIMIT_KGF * 4 * 0.75,
            "двигатели": MOTORS},
    "параметры": {"КПД_редуктора": ETA_BASE, "КПД_диапазон": ETA_RANGE,
                  "Crr": CRR, "k_mec": K_MEC, "R_м": R, "RW_м": RW},
    "регрессия_v3": REGR,
    "моменты": rows,
    "уклоны_длительные": slopes,
    "скорости": spd,
    "чувствительность_КПД": eta_tbl,
    "тормоз": brake,
    "нагрузка_на_колесо": load,
    "findings": FIND, "checks": CHECKS,
}
p = os.path.join(HERE, "rvw7pa_drive_check.json")
with open(p, "w", encoding="utf-8") as f:
    json.dump(out, f, ensure_ascii=False, indent=1)
print("\nREGRESSION:", "PASS" if all(c["ok"] for c in CHECKS) else "FAIL")
print("JSON:", p)
