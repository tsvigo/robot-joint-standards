# -*- coding: utf-8 -*-
"""Этап 2Г v7.4, задача 1 — stage2g_v7_4_panel.py (FreeCAD, ТОЛЬКО ЧТЕНИЕ v6).

Восстановление объёма физического CAD-тела панели R2 (Ø620×40 с вырезами ниш)
и масс при ρ: алюминий 2700, сталь 7850 кг/м³.
Контроль: сплошной цилиндр Ø620×40 = 32.6 кг (Al) / 94.8 кг (сталь).
Происхождение прежнего диапазона 45–76 кг проверяется обратным расчётом.
Рёбра/крронштейны/усилители/крепления — ОТДЕЛЬНАЯ строка, масса не назначается.
v6 не сохраняется.
"""
import hashlib
import json
import math
import os

import FreeCAD as App

try:
    SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
except NameError:
    SCRIPT_DIR = os.getcwd()

FC = os.path.join(SCRIPT_DIR, "stage2g_v6_variants.FCStd")


def sha(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()[:16]


sha0, mt0 = sha(FC), os.path.getmtime(FC)
doc = App.openDocument(FC)
doc.recompute()

body = doc.getObject("R620_CTM_body")            # панель с вырезами (тело)
assert body is not None, "тело панели не найдено"
V_body = body.Shape.Volume / 1e9                 # м³ (мм³ -> м³)
bb = body.Shape.BoundBox
V_solid = math.pi * 0.31 ** 2 * 0.040            # сплошной цилиндр, м³

# контрольные массы
def m(rho, V):
    return rho * V

rows = [
    {"тело": "сплошной цилиндр Ø620×40 (контроль)", "V_м3": round(V_solid, 6),
     "Al_2700_кг": round(m(2700, V_solid), 1), "сталь_7850_кг": round(m(7850, V_solid), 1)},
    {"тело": "панель R2 с вырезами (R620_CTM_body)", "V_м3": round(V_body, 6),
     "Al_2700_кг": round(m(2700, V_body), 1), "сталь_7850_кг": round(m(7850, V_body), 1)},
]
print("== Масса панели по CAD-объёму ==")
for r in rows:
    print(f"  {r['тело']}: V={r['V_м3']} м³ -> Al {r['Al_2700_кг']} кг, "
          f"сталь {r['сталь_7850_кг']} кг")
print(f"  вычет под вырезы: {(1 - V_body / V_solid) * 100:.1f}% объёма")
print(f"  BoundBox тела: {bb.XLength:.1f} × {bb.YLength:.1f} × {bb.ZLength:.1f} мм "
      f"(ожидание 620 × 620 × 40)")

# ============ обратный расчёт прежнего диапазона 45–76 кг ============
print("\n== Обратный расчёт прежнего диапазона 45–76 кг (v7/v7.1) ==")
old = {"45кг_Al_объём_м3": round(45.0 / 2700, 6),
       "76кг_сталь_объём_м3": round(76.0 / 7850, 6),
       "сравнение_Al": f"45/2700 = {45/2700:.6f} м³ против сплошного {V_solid:.6f} м³ "
                       f"= {45/2700/V_solid*100:.0f}% — БОЛЬШЕ сплошного цилиндра "
                       f"даже без вырезов => НЕВОЗМОЖНО как масса этой панели",
       "сравнение_сталь": f"76/7850 = {76/7850:.6f} м³ = {76/7850/V_solid*100:.0f}% "
                          f"сплошного — соответствует панели с вырезами "
                          f"(факт {V_body/V_solid*100:.0f}%) с запасом",
       "вывод": ("Диапазон 45–76 НЕ воспроизводится как (Al, сталь) одного CAD-тела. "
                 "Кандидаты происхождения (не доказаны): (а) 76 = сталь с вырезами × "
                 "запас; (б) 45 = Al-панель + усилители. Прежняя оценка признаётся "
                 "неконтролируемой; заменяется расчётом по CAD-объёму выше.")}
for k, v in old.items():
    print(f"  {k}: {v}")

# отдельные компоненты, НЕ входящие в тело панели
extra = {
    "рёбра_усилители_кронштейны_крепления": {
        "масса": "НЕ ОПРЕДЕЛЕНА", "статус": "не входят в тело панели (CAD их не содержит); "
                 "не утверждаются до согласования конструкции"},
    "фланец лифта Ø240×10 (отдельное тело в v6)": {
        "V_м3": round(math.pi * 0.12 ** 2 * 0.010, 6),
        "Al_кг": round(m(2700, math.pi * 0.12 ** 2 * 0.010), 2),
        "сталь_кг": round(m(7850, math.pi * 0.12 ** 2 * 0.010), 2),
        "статус": "отдельное тело, показано отдельно (тоже требует материала)"},
}
print("\n== Отдельно (не входят в тело панели) ==")
for k, v in extra.items():
    print(f"  {k}: {v}")

out = {"rows": rows, "old_range_45_76": old, "extra_components": extra,
       "panel_bounding_box_mm": [round(bb.XLength, 1), round(bb.YLength, 1),
                                 round(bb.ZLength, 1)],
       "V_solid_m3": round(V_solid, 6), "V_body_m3": round(V_body, 6),
       "notes": ["v6 FCStd не изменён (save не вызывался).",
                 "Материал панели не выбран — обе плотности показаны параллельно."]}
path = os.path.join(SCRIPT_DIR, "stage2g_v7_4_panel.json")
with open(path, "w", encoding="utf-8") as f:
    json.dump(out, f, ensure_ascii=False, indent=1)

App.closeDocument(doc.Name)
print(f"\nv6 целостность: {'OK' if (sha(FC) == sha0 and os.path.getmtime(FC) == mt0) else 'ИЗМЕНЁН!'}")
print("JSON:", path)
