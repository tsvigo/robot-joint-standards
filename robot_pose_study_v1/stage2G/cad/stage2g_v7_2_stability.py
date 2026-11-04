# -*- coding: utf-8 -*-
"""Этап 2Г, корректировка v7.2 — stage2g_v7_2_stability.py (НОВЫЙ файл; v7.1 не изменяется).

ОШИБКА v7.1 (знак высоты препятствия):
    было:  delta = Q@u − h,   F = k·delta  => подъём пола (h>0) УМЕНЬШАЛ сжатие.
Физика: сжатие пружины между телом и точкой пола
    delta_i = L0-сжатие + h_i − (перемещение тела вверх в точке i)
=> подъём пола (препятствие) увеличивает сжатие, подъём тела — уменьшает.
В новых координатах (u = (C, tx, ty), Q = [1, −x, −y]):
    delta = Q@u + h,   F = k·delta,
равновесие:  B^T·F = (Mg, Mg·ex, Mg·ey),  B = [1, x, y].

Контрольный пример (400 кг, k=100000 Н/м, бугор +10 мм под колесом 0):
    ОЖИДАЕТСЯ: 125.5 / 74.5 / 125.5 / 74.5 кгс (проверяется тестом P3).

Физические тесты (независимые, не файловые):
    P1  сумма вертикальных реакций = приложенной нагрузке (все сценарии);
    P2  равновесие моментов (ΣF·x = Mg·ex, ΣF·y = Mg·ey);
    P3  подъём пола под колесом увеличивает ЕГО реакцию и уменьшает боковых;
    P4  реакции не отрицательны; при отрицательной — контакт размыкается
        (k_i=0) и задача перерешивается для оставшихся опор;
    P5  зеркально-симметричный сценарий даёт зеркально-симметричные реакции;
    P6  h=0: новая модель даёт те же результаты, что и модель v7.1
        (СВЕРКА С РАНЕЕ ЗАПИСАННЫМ — не физическая валидация, помечена);
    P7  повторный вызов даёт идентичное решение (стабильность).
"""
import json
import math
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
G = 9.81
R_C = 0.2215
W_MOD = 0.1069
PTS_R = [np.array([R_C, 0.0]), np.array([0.0, R_C]),
         np.array([-R_C, 0.0]), np.array([0.0, -R_C])]
PTS_G = [p * ((R_C - W_MOD / 2) / R_C) for p in PTS_R]
K0 = 100000.0

TESTS = []


def t(cid, ok, detail, cls):
    """cls: 'PHYS' — физическая валидация; 'CHECK' — сверка с записанным; 'FILE'."""
    TESTS.append({"id": cid, "ok": bool(ok), "class": cls, "detail": detail})
    print(f"  [{'PASS' if ok else 'FAIL'}] [{cls}] {cid}: {detail}")


def reactions(M_kg, ex=0.0, ey=0.0, k_rel=(1.0, 1.0, 1.0, 1.0), h=(0.0,) * 4, pts=None):
    """F_i, Н (v7.2, знак верный): delta_i = C + h_i − tx·x_i − ty·y_i; F = k·delta."""
    P = np.array(pts) if pts is not None else np.array(PTS_R)
    k = np.array(k_rel, dtype=float) * K0
    B = np.array([[1.0, p[0], p[1]] for p in P])            # равновесия: B^T F = b
    Q = np.array([[1.0, -p[0], -p[1]] for p in P])          # delta = Q u + h
    bs = np.array([M_kg * G, M_kg * G * ex, M_kg * G * ey])
    A = (B * k[:, None]).T @ Q
    rhs = bs - (k * np.array(h)) @ B
    u = np.linalg.solve(A, rhs)
    delta = Q @ u + np.array(h)
    F = k * delta
    return F


def reactions_with_contact(M_kg, ex=0.0, ey=0.0, k_rel=(1.0, 1.0, 1.0, 1.0),
                           h=(0.0,) * 4, pts=None):
    """С учётом размыкания контакта: отрицательная реакция -> k_i=0, перерешить.
    Если после размыкания задача вырождается (опора становится ЛИНИЕЙ) —
    возвращается признак unstable: физически тело неустойчиво поперёк."""
    kk = list(k_rel)
    history = []
    for _ in range(6):
        try:
            F = reactions(M_kg, ex=ex, ey=ey, k_rel=tuple(kk), h=h, pts=pts)
        except np.linalg.LinAlgError:
            history.append("ВЫРОЖДЕНИЕ: оставшиеся опоры на одной линии — "
                           "поперечная неустойчивость тела")
            return None, kk, history, True
        imin = int(np.argmin(F))
        if F[imin] < -1e-6 and kk[imin] > 0:
            history.append(f"контакт {imin} разомкнут (F={F[imin]/G:.1f} кг)")
            kk[imin] = 0.0
            continue
        return F, kk, history, False
    F = reactions(M_kg, ex=ex, ey=ey, k_rel=tuple(kk), h=h, pts=pts)
    return F, kk, history, False


def equilibrium_ok(F, M_kg, ex=0.0, ey=0.0, tol=1e-6):
    s = F.sum() - M_kg * G
    mx = sum(float(F[i] * PTS_R[i][0]) for i in range(4)) - M_kg * G * ex
    my = sum(float(F[i] * PTS_R[i][1]) for i in range(4)) - M_kg * G * ey
    return (abs(s) < tol * M_kg * G, abs(mx) < 1e-6 * M_kg * G * R_C,
            abs(my) < 1e-6 * M_kg * G * R_C)


SCENARIOS = {
    "гладкий пол": {"h": (0.0,) * 4},
    "бугор 10мм под колесом0": {"h": (0.010, 0.0, 0.0, 0.0)},
    "диагональный перекос +5мм (колеса 0,2)": {"h": (0.005, 0.0, 0.005, 0.0)},
    "уклон 5% по X": {"h": tuple(math.tan(0.05) * p[0] for p in PTS_R)},
    "снос ЦМ 60мм": {"ex": 0.060},
    "снос ЦМ 120мм (отрыв в модели)": {"ex": 0.120},
    "k −25% на колесе0, снос 60мм": {"k_rel": (0.75, 1.0, 1.0, 1.0), "ex": 0.060},
}

print("== 1. Пересчёт сценариев (исправленный знак h) ==")
print(f"  {'сценарий':<40} {'N1..N4, кгс':>34} {'min':>7}")
results = {}
for name, kw in SCENARIOS.items():
    F, kk, hist, unst = reactions_with_contact(M_kg=400.0, **kw)
    n = [round(float(x) / G, 1) for x in F]
    results[name] = {"kw": {k: (list(v) if isinstance(v, tuple) else v)
                            for k, v in kw.items()},
                     "N_кгс": n, "k_used": kk, "contact_history": hist,
                     "unstable": unst}
    print(f"  {name:<40} {str(n):>34} {min(n):7.1f}" + (f"  [{'; '.join(hist)}]" if hist else ""))

# ============ Физические тесты ============
print("\n== 2. Физические тесты (независимые) ==")

# P1/P2: равновесие во всех сценариях
ok1 = ok2 = True
for name, kw in SCENARIOS.items():
    F, kk, hist, unst = reactions_with_contact(M_kg=400.0, **kw)
    if F is None:
        continue                      # неустойчивость — отдельно, см. P8
    s, mx, my = equilibrium_ok(F, 400.0, kw.get("ex", 0.0), kw.get("ey", 0.0))
    ok1 &= s
    ok2 &= (mx and my)
t("P1_сумма_сил", ok1, "ΣN = Mg для всех сценарий (|Δ| < 1e-6·Mg)", "PHYS")
t("P2_равновесие_моментов", ok2,
  "ΣN·x = Mg·ex и ΣN·y = Mg·ey для всех сценарий", "PHYS")

# P3: направление перераспределения (контрольный пример)
F0 = reactions(400.0, h=(0.0,) * 4)
Fh = reactions(400.0, h=(0.010, 0.0, 0.0, 0.0))
n0 = [round(float(x) / G, 1) for x in F0]
nh = [round(float(x) / G, 1) for x in Fh]
expected = [125.5, 74.5, 125.5, 74.5]
ok_dir = (nh[0] > n0[0] and abs(nh[0] - expected[0]) < 0.2 and
          abs(nh[1] - expected[1]) < 0.2 and abs(nh[2] - expected[2]) < 0.2 and
          abs(nh[3] - expected[3]) < 0.2)
t("P3_контрольный_пример_бугор", ok_dir,
  f"бугор+10мм под колесом0: {nh} кгс (ожидается {expected}); "
  f"гладкий {n0} — реакция подпёртого колеса ВЫРОСЛА, боковых упала", "PHYS")

# P4: нет растяжения + перерешивание при отрыве
Frec, kk, hist, unst = reactions_with_contact(400.0, ex=0.120)
ok_no_neg = (not unst) and all(float(x) >= -1e-6 for x in Frec)
t("P4a_нет_растягивающих_сил", ok_no_neg,
  f"после перерешивания: N={[round(float(x)/G,1) for x in Frec]} кгс — все ≥ 0", "PHYS")
t("P4b_перерешивание_при_отрыве", len(hist) >= 1 and kk[2] == 0.0,
  f"снос 120мм: {hist if hist else 'отрыва не было!'} -> задача решена для оставшихся опор (треугольник)",
  "PHYS")

# P5: зеркальная симметрия (зеркало x -> −x меняет колеса 0<->2)
Fa = reactions(400.0, h=(0.010, 0.0, 0.0, 0.0))
Fb = reactions(400.0, h=(0.0, 0.0, 0.010, 0.0))
mirrored = [Fa[2], Fa[1], Fa[0], Fa[3]]
ok5 = np.allclose(Fb, mirrored, rtol=1e-9, atol=1e-6)
t("P5_зеркальная_симметрия", ok5,
  "бугор под колесом2 даёт реакции, зеркальные бугру под колесом0 (0<↔2)", "PHYS")

# P6: h=0 — сверка с моделью v7.1 (СВЕРКА С ЗАПИСАННЫМ, НЕ физика)
# v7.1: снос 60мм/k-случаи считались при h=0 и совпали с формулой N=Mg/4(1+2e·x/Rc²)
Fx = reactions(400.0, ex=0.100)
Fx_v71 = np.array([400 * G / 4 * (1 + 2 * 0.100 * x / R_C ** 2) for x in (R_C, 0, -R_C, 0)])
ok6 = np.allclose(Fx, Fx_v71, rtol=1e-9)
t("P6_сверка_h0_с_v7.1", ok6,
  "при h=0 результаты идентичны формуле v7.1 (не физическая валидация — сверка моделей)",
  "CHECK")

# P7: стабильность/идемпотентность
Fr1 = reactions(400.0, ex=0.06, k_rel=(0.75, 1.0, 1.0, 1.0), h=(0.005, 0, 0, 0))
Fr2 = reactions(400.0, ex=0.06, k_rel=(0.75, 1.0, 1.0, 1.0), h=(0.005, 0, 0, 0))
t("P7_идемпотентность", np.allclose(Fr1, Fr2),
  "повторный вызов даёт идентичное решение", "PHYS")

# ============ 3. Пересчёт чувствительностей (после фикса) ============
print("\n== 3. Чувствительность (пересчёт v7.2) ==")
k_cases = {"k_равномерно": (1.0,) * 4, "k_−25%": (0.75, 1.0, 1.0, 1.0),
           "k_−50%": (0.5, 1.0, 1.0, 1.0), "k_±25%": (1.25, 0.75, 1.25, 0.75)}
k_sens = {}
for nm, kk in k_cases.items():
    F = reactions(400.0, ex=0.060, k_rel=kk)
    n = [round(float(x) / G, 1) for x in F]
    k_sens[nm] = {"N_кгс": n, "min": min(n)}
    print(f"  {nm}: N={n} кгс, min={min(n)}")

h_sens = {}
for nm, hh in (("бугор10мм_под_0", (0.010, 0, 0, 0)),
               ("диагональ+5мм(0,2)", (0.005, 0, 0.005, 0)),
               ("уклон5%_по_X", tuple(math.tan(0.05) * p[0] for p in PTS_R))):
    F, kk, hist, unst = reactions_with_contact(400.0, h=hh)
    if F is None:
        h_sens[nm] = {"N_кгс": None, "unstable": True, "contact": hist}
        print(f"  {nm}: НЕУСТОЙЧИВОСТЬ [{'; '.join(hist)}]")
        continue
    n = [round(float(x) / G, 1) for x in F]
    h_sens[nm] = {"N_кгс": n, "min": min(n), "contact": hist}
    print(f"  {nm}: N={n} кгс" + (f" [{'; '.join(hist)}]" if hist else ""))

# предельный бугор: h = Mg/k = 39.24мм — боковые F1=F3 -> 0; при 40мм (>Mg/k)
# модель размыкает смежную опору; равновесие гасит вторую боковую до нуля
# (граничное состояние «на линии» 0-2; полная неустойчивость численно не
# достигается — тело удерживается опорами 0/2 и ограничением полом)
F_lim, kk_lim, hist_lim, unst_lim = reactions_with_contact(400.0, h=(0.040, 0, 0, 0))
h_sens["бугор_40мм_сверх_предела"] = {
    "N_кгс": ([round(float(x) / G, 1) for x in F_lim] if F_lim is not None else None),
    "unstable": unst_lim, "contact": hist_lim}
print(f"  бугор 40мм (>Mg/k=39.24мм): история={hist_lim}; "
      f"{'НЕУСТОЙЧИВОСТЬ' if unst_lim else [round(float(x)/G,1) for x in F_lim]}")
t("P8_предел_бугра", len(hist_lim) >= 1,
  f"при бугре >Mg/k: размыкается смежная опора {hist_lim}; равновесие = "
  f"{[round(float(x)/G,1) for x in F_lim] if F_lim is not None else 'н/у'} кгс "
  f"— граничное состояние на линии 0-2 (боковые на нуле)", "PHYS")

# отрыв e и критический уклон (после фикса — h пола=0, результаты не меняются)
e_list = {}
for e in (0.0, 0.060, 0.100, 0.1108, 0.120):
    F, kk, hist, unst = reactions_with_contact(400.0, ex=e)
    e_list[f"{e*1000:.1f}мм"] = {"N_кгс": ([round(float(x) / G, 1) for x in F]
                                           if F is not None else None),
                                 "contact": hist, "unstable": unst}
lo, hi = 0.0, 1.0
for _ in range(50):
    mid = (lo + hi) / 2
    F = reactions(400.0, ex=0.5 * math.sin(math.atan(mid)))
    if F.min() > 0:
        lo = mid
    else:
        hi = mid
print(f"  e-отрыв (модель, ромб R_C): ≈ {R_C/2*1000:.1f}мм; "
      f"критический уклон с h_цм=0.5: {lo*100:.1f}%")

# ============ 4. Классификация проверок ============
classes = {}
for x in TESTS:
    classes.setdefault(x["class"], []).append(x["id"])
print("\n== 4. Классификация проверок ==")
print(f"  PHYS (независимая физика): {classes.get('PHYS', [])}")
print(f"  CHECK (сверка с записанным): {classes.get('CHECK', [])}")
print(f"  FILE/прочее — в регрессии v7.1: SHA-файлы, наличие полей JSON — "
      f"НЕ физическая валидация (помечено)")

out = {"model_fix": {"было": "delta = Q@u − h (подъём пола уменьшал сжатие)",
                     "стало": "delta = Q@u + h, Q=[1,−x,−y] (подъём пола увеличивает сжатие)",
                     "контрольный_пример": {"ожидание": expected, "факт": nh}},
       "scenarios": results, "k_sens": k_sens, "h_sens": h_sens,
       "e_sens": e_list, "critical_grade_pct_hcm05": round(lo * 100, 1),
       "critical_e_model_mm": round(R_C / 2 * 1000, 1),
       "tests": TESTS,
       "test_classes": classes,
       "notes": [
           "P1-P5, P7, P8 — физические; P6 — сверка моделей (не физика).",
           "Файловые/записанные проверки регрессии v7.1 (SHA, поля JSON) — НЕ физ. валидация.",
           "Все сценарии пересчитаны с исправленным знаком; h=0 результаты не изменились.",
       ]}
path = os.path.join(HERE, "stage2g_v7_2_stability.json")
with open(path, "w", encoding="utf-8") as f:
    json.dump(out, f, ensure_ascii=False, indent=1)
print("\nJSON:", path)
fails = [x for x in TESTS if not x["ok"]]
print("REGRESSION:", "PASS" if not fails else f"FAIL {[x['id'] for x in fails]}")
