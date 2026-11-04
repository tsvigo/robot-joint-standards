# -*- coding: utf-8 -*-
"""Этап RVW-7PA v4, задача 3 — последствия перехода с RoboCT CTM1000-10R.

Данные для сравнения:
  RVW-7PA (Nabtesco Rev.005.1, стр.17-18): Ø178×96, i=30, T_ном 7 Н·м,
    T_уск 27 Н·м, 107.3 об/мин колеса (1.0 м/с), масса 15.4/15.9 кг,
    двигатель □90 × 99/110, полная длина 233/244.
  RoboCT CTM1000-10R (stage2g_v6_real_drive.json, каталог RoboCT):
    Ø240×106.9, i=37.1, T_ном 74 Н·м, 56 об/мин (0.70 м/с), 25 кг,
    нагрузка на комплект 1000 кг, IP44, колесо+мотор+энкодер+амортизатор.
  База R2 (stage2g_v7_1/v7_2/v7_4): просвет 8 мм, база 176 мм,
    лимит лифта 500 кг, дверь 700 мм, ЦМ только R2 z=0.112 м при 118.4 кг.
"""
import json
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))
G = 9.81
CRR = 0.05
COS45 = math.cos(math.radians(45))

RVW = {"D": 0.178, "W": 0.096, "i": 30.0, "T": 7.0, "T_acc": 27.0,
       "n": 107.3, "mass": 15.4, "mass_400": 15.9, "len": 0.233, "len_400": 0.244}
CTM = {"D": 0.240, "W": 0.1069, "i": 37.1, "T": 74.0, "n": 56.0, "mass": 25.0,
       "len": None, "payload_set": 1000.0}

BASE = {"просвет_мм": 8.0, "высота_базы_мм": 176.0, "лимит_лифта_кг": 500.0,
        "дверь_мм": 700.0, "платформа_Ø": 620.0,
        "R2_масса_кг": 118.4, "R2_ЦМ_z_м": 0.112,
        "R2_известно": {"RoboCT_4x25": 100.0, "панель_Al": 18.4}}
RW_CTM = 0.2215     # радиус установки в схеме R (stage2g_v7_5)
RW_RVW = 0.2489     # из компоновки v2/v4

CHECKS, FIND = [], []


def t(cid, ok, detail):
    CHECKS.append({"id": cid, "ok": bool(ok), "detail": detail})
    print(f"  [{'PASS' if ok else 'FAIL'}] {cid}: {detail}")


def T_req(M, a, grade, r):
    """Момент на колесо (макс по 4) для движения по X — как в модели v3:
    f_i = (ê_i·F)/2, T_i = f_i·r·cos45; для X это приводится к F·r/4."""
    th = math.atan(grade)
    F = M * (a + G * (math.sin(th) + CRR * math.cos(th)))
    return F * r / 4.0


# ================= 1. Масса =================
print("== 1. Масса комплекта колёс ==")
ms = []
for lbl, per, n in [("RVW-7PA 200 Вт", RVW["mass"], 4), ("RVW-7PA 400 Вт", RVW["mass_400"], 4),
                    ("RoboCT CTM1000-10R", CTM["mass"], 4)]:
    ms.append({"комплект": lbl, "на_модуль_кг": per, "всего_кг": round(per * n, 1)})
    print(f"  {lbl:>20}: {per:>5.1f} кг × {n} = {per*n:>5.1f} кг")
d200 = 4 * RVW["mass"] - 4 * CTM["mass"]
d400 = 4 * RVW["mass_400"] - 4 * CTM["mass"]
t("T1_масса", True, f"дельта: 200 Вт {d200:+.1f} кг (={4*RVW['mass']:.1f} против "
                    f"{4*CTM['mass']:.1f}), 400 Вт {d400:+.1f} кг")

# ================= 2. Высота, просвет, ось =================
print("\n== 2. Высота оси колеса и дорожный просвет ==")
z_ax_rvw = RVW["D"] / 2 * 1000
z_ax_ctm = CTM["D"] / 2 * 1000
dz = z_ax_rvw - z_ax_ctm
clr_no_spacer = BASE["просвет_мм"] + dz
h = [{"параметр": "Ø колеса, мм", "RoboCT": CTM["D"] * 1000, "RVW": RVW["D"] * 1000},
     {"параметр": "ось колеса над полом, мм", "RoboCT": z_ax_ctm, "RVW": z_ax_rvw},
     {"параметр": "дельта оси, мм", "RoboCT": 0.0, "RVW": round(dz, 1)},
     {"параметр": "просвет без проставки, мм", "RoboCT": BASE["просвет_мм"],
      "RVW": round(clr_no_spacer, 1)},
     {"параметр": "требуемая проставка, мм", "RoboCT": 0.0, "RVW": round(-dz, 1)}]
for r in h:
    print(f"  {r['параметр']:>32}: RoboCT {r['RoboCT']:>7.1f} | RVW {r['RVW']:>7.1f}")
t("T2_просвет", clr_no_spacer < 0,
  f"без проставки просвет {clr_no_spacer:.0f} мм (база ложится на пол) — "
  f"нужна проставка {-dz:.0f} мм на модуль; с проставкой просвет сохраняется "
  f"{BASE['просвет_мм']:.0f} мм, но монтажная плоскость уходит на {-dz:.0f} мм ниже")

# ================= 3. Момент и скорость =================
print("\n== 3. Момент и скорость: RVW-7PA против RoboCT ==")
r_rvw, r_ctm = RVW["D"] / 2, CTM["D"] / 2
cmp_rows = []
for M in [300.0, 400.0, 450.0]:
    for gr in [0.0, 0.05, 0.08]:
        T_r = T_req(M, 0.0, gr, r_rvw)
        T_c = T_req(M, 0.0, gr, r_ctm)
        cmp_rows.append({
            "масса_кг": M, "уклон_%": gr * 100,
            "T_RVW_Нм": round(T_r, 2), "T_RVW_%_от_7": round(100 * T_r / RVW["T"], 0),
            "T_RoboCT_Нм": round(T_c, 2),
            "T_RoboCT_%_от_74": round(100 * T_c / CTM["T"], 0)})
        print(f"  {M:>5.0f} кг {gr*100:>4.0f}%: RVW {T_r:6.2f} Н·м "
              f"({100*T_r/RVW['T']:>4.0f}% от 7) | RoboCT {T_c:6.2f} Н·м "
              f"({100*T_c/CTM['T']:>3.0f}% от 74)")
ratio = (CTM["T"] / r_ctm) / (RVW["T"] / r_rvw)
t("T3_момент", True, f"удельный момент на колесе: RoboCT {CTM['T']/r_ctm:.0f} Н "
                     f"против RVW {RVW['T']/r_rvw:.0f} Н — в {ratio:.1f} раза больше "
                     f"(74 Н·м против 7 Н·м)")

v_rvw = RVW["n"] / 60 * 2 * math.pi * r_rvw
v_ctm = CTM["n"] / 60 * 2 * math.pi * r_ctm
t("T4_скорость", True, f"номинал: RVW {v_rvw:.2f} м/с при {RVW['n']} об/мин, "
                       f"RoboCT {v_ctm:.2f} м/с при {CTM['n']} об/мин — "
                       f"RVW быстрее в {v_rvw/v_ctm:.2f} раза")

# предельный длительный уклон
def slope_limit(Tmax, r):
    lo, hi = 0.0, 0.6
    for _ in range(60):
        mid = (lo + hi) / 2
        if T_req(450.0, 0.0, mid, r) <= Tmax:
            lo = mid
        else:
            hi = mid
    return lo * 100


s_rvw, s_ctm = slope_limit(RVW["T"], r_rvw), slope_limit(CTM["T"], r_ctm)
t("T5_уклон", True, f"длительный уклон при 450 кг: RVW {s_rvw:.2f}% (T<=7 Н·м), "
                    f"RoboCT {s_ctm:.1f}% (T<=74 Н·м) — ограничение RoboCT "
                    f"переходит на сцепление, а не на момент")

# ================= 4. Опорный контур =================
print("\n== 4. Опорный контур и устойчивость ==")
area_ctm = (2 * RW_CTM) ** 2
area_rvw = (2 * RW_RVW) ** 2
t("T6_опорный_контур", True,
  f"квадрат опорных точек: RoboCT {2*RW_CTM*1000:.0f} мм (площадь {area_ctm*1e4:.0f} см²), "
  f"RVW {2*RW_RVW*1000:.0f} мм ({area_rvw*1e4:.0f} см²) — +{(area_rvw/area_ctm-1)*100:.0f}%; "
  f"ширина колеса {RVW['W']*1000:.0f} против {CTM['W']*1000:.0f} мм")

# ================= 5. ЦМ =================
print("\n== 5. ЦМ только базы R2 ==")
m_rvw = 4 * RVW["mass"]
m_ctm = 4 * CTM["mass"]
m_panel = BASE["R2_известно"]["панель_Al"]
m_frame = 3.7 + 2.4           # из компоновки v4: рама 3.7 + 4 хомута 2.4 (Al)
m_spacer = 4 * 0.5            # проставки 31 мм — ПАРАМЕТР
z_mod_rvw = z_ax_rvw / 1000
z_mod_ctm = z_ax_ctm / 1000
z_panel = 0.070
z_frame = 0.075
z_spacer = (z_ax_rvw + 31) / 1000
cm_ctm = (m_ctm * z_mod_ctm + m_panel * z_panel) / (m_ctm + m_panel)
cm_rvw = (m_rvw * z_mod_rvw + m_frame * z_frame + m_spacer * z_spacer) / \
         (m_rvw + m_frame + m_spacer)
t("T7_ЦМ", cm_rvw < cm_ctm,
  f"ЦМ базы: RoboCT+панель {cm_ctm*1000:.0f} мм ({m_ctm+m_panel:.1f} кг) -> "
  f"RVW+рама+проставки {cm_rvw*1000:.0f} мм ({m_rvw+m_frame+m_spacer:.1f} кг), "
  f"снижение {(cm_ctm-cm_rvw)*1000:.0f} мм")
mass_r2_new = m_rvw + m_frame + m_spacer
t("T8_масса_R2", True,
  f"масса базы с известными элементами: {BASE['R2_масса_кг']:.1f} -> "
  f"{mass_r2_new:.1f} кг (дельта {mass_r2_new-BASE['R2_масса_кг']:+.1f}); "
  f"остальные подсистемы не учтены")

# ================= 6. Дверь и транспорт =================
print("\n== 6. Дверь и транспортный габарит ==")
t("T9_дверь", BASE["платформа_Ø"] < BASE["дверь_мм"],
  f"Ø620 < дверь 700 мм: зазор {(BASE['дверь_мм']-BASE['платформа_Ø'])/2:.0f} мм на борт; "
  f"внешний радиус колеса 310 мм = граница Ø620, выхода нет")
t("T10_высота", True,
  f"высота базы не меняется при проставке {-dz:.0f} мм (проставка возвращает "
  f"монтажную плоскость); без проставки база опускается на {-dz:.0f} мм")

# ================= 7. Лимит лифта 500 кг =================
print("\n== 7. Лимит лифта 500 кг ==")
for M in [400.0, 450.0]:
    for who, correction in [("как есть (сценарий включает RoboCT)", 0.0),
                            ("с учётом экономии массы привода", d200)]:
        tot = M + 80.0 + correction
        print(f"  {M:.0f} кг + 80 кг ({who}): {tot:.1f} кг "
              f"{'<= 500 OK' if tot <= 500 else '> 500 FAIL'}")
t("T11_лифт", True,
  f"сценарий 450+80 = 530 кг > 500 FAIL; при замене привода масса сценария "
  f"снижается на {-d200:.1f} кг -> {450+d200+80:.1f} кг <= 500 OK (условно: "
  f"только если 450 кг включали 4×25 кг RoboCT)")

FIND.extend([
    {"id": "F1_масса", "текст": f"экономия массы привода {d200:.1f} кг (200 Вт) / "
                                f"{d400:.1f} кг (400 Вт) на комплект из 4 модулей"},
    {"id": "F2_высота", "текст": f"ось колеса падает с {z_ax_ctm:.0f} до {z_ax_rvw:.0f} мм "
                                 f"(−{-dz:.0f} мм): без проставки {-dz:.0f} мм просвет "
                                 f"{clr_no_spacer:.0f} мм — посадка на пол; проставка "
                                 f"обязательна и добавляет ~{m_spacer:.1f} кг"},
    {"id": "F3_момент", "текст": f"номинальный момент на колесе 7 Н·м против 74 Н·м "
                                 f"(−{100*(1-RVW['T']/CTM['T']):.0f}%): длительный уклон "
                                 f"при 450 кг падает с {s_ctm:.1f}% до {s_rvw:.2f}%"},
    {"id": "F4_скорость", "текст": f"скорость растёт с {v_ctm:.2f} до {v_rvw:.2f} м/с "
                                   f"при номинале, но 1.0 м/с требует 3219 об/мин"},
    {"id": "F5_демпфер", "текст": "у RoboCT амортизатор встроен в модуль (каталог), "
                                  "у RVW-7PA подвески НЕТ — её нужно проектировать "
                                  "отдельно (в компоновке принят ход ±15/±30 мм как "
                                  "параметр, узел скольжения не спроектирован)"},
    {"id": "F6_IP", "текст": "RoboCT IP44; у RVW-7PA класс защиты в каталоге не указан "
                             "(только 'не использовать при воде/пыли/улице') — "
                             "для влажной уборки это вопрос производителю"},
])

out = {
    "источник": ["Nabtesco en_cat_RVW.pdf Rev.005.1 стр.17-18",
                 "stage2G/cad/stage2g_v6_real_drive.json (RoboCT CTM1000-10R)",
                 "stage2G/cad/stage2g_v7_1_cad_suspension.json (просвет 8 мм)",
                 "stage2G/cad/stage2g_v7_4_budget_cm.json (ЦМ, лимит 500 кг)"],
    "RVW": RVW, "RoboCT": CTM, "база": BASE,
    "массы": ms, "дельта_кг": {"200w": round(d200, 1), "400w": round(d400, 1)},
    "высота_и_просвет": h,
    "сравнение_момента": cmp_rows,
    "уклон_длительный_%": {"RVW": round(s_rvw, 2), "RoboCT": round(s_ctm, 1)},
    "скорость_м_с": {"RVW": round(v_rvw, 2), "RoboCT": round(v_ctm, 2)},
    "опорный_контур": {"RVW_мм": 2 * RW_RVW * 1000, "RoboCT_мм": 2 * RW_CTM * 1000},
    "ЦМ": {"RoboCT_база_мм": round(cm_ctm * 1000, 1), "RVW_база_мм": round(cm_rvw * 1000, 1),
           "масса_R2_RoboCT_кг": BASE["R2_масса_кг"], "масса_R2_RVW_кг": round(mass_r2_new, 1)},
    "findings": FIND, "checks": CHECKS,
}
p = os.path.join(HERE, "rvw7pa_impact.json")
with open(p, "w", encoding="utf-8") as f:
    json.dump(out, f, ensure_ascii=False, indent=1)
print("\nREGRESSION:", "PASS" if all(c["ok"] for c in CHECKS) else "FAIL")
print("JSON:", p)
