# -*- coding: utf-8 -*-
"""Задача 3 — check_stage2g_v6_3d.py: реальные 3D-пересечения и расстояния между телами
(не проекции на XY). OCC distToShape / common по фактическим телам CAD.

Проверяются: stage2g_v5_compare.FCStd (базовый: лифт на полную высоту) и
stage2g_v6_variants.FCStd (Z-разнесённые варианты, если файл существует).
Пары: колесо-лифт, ступица-лифт, колесо-панель, колесо-привод, колесо-колесо,
блок аутригера-панель, блок-АКБ. Оценка доступа монтажа/обслуживания — по зазорам.
"""
import json
import math
import os
import FreeCAD as App

try:
    SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
except NameError:
    SCRIPT_DIR = os.getcwd()


def get(doc, name):
    o = doc.getObject(name)
    return o if (o is not None and hasattr(o, "Shape") and not o.Shape.isNull()) else None


def pair(a, b, name):
    if a is None or b is None:
        return None
    d, _pts, _info = a.Shape.distToShape(b.Shape)
    common = a.Shape.common(b.Shape)
    vol = common.Volume
    return {"pair": name, "dist_mm": round(float(d), 2),
            "common_cm3": round(vol / 1000.0, 3),
            "verdict": ("ПЕРЕСЕЧЕНИЕ" if vol > 1.0 else ("КАСАНИЕ/ЗАЗОР<0.01" if d < 0.01 else "ЗАЗОР"))}


def check_group(doc, gname, has_drv=True, has_bat=True, wheel_tags=("0", "90")):
    res = []
    lifts = [o for o in doc.Objects if o.Name.startswith(gname + "_")
             and "_lift" in o.Name and hasattr(o, "Shape") and not o.Shape.isNull()]
    body = get(doc, f"{gname}_body")
    for tag in wheel_tags:
        whl = get(doc, f"{gname}_WHL{tag}")
        hub = get(doc, f"{gname}_hub{tag}")
        if whl is None:
            whl = get(doc, f"{gname}_WHL_{ {'0': 'FL_L', '90': 'FR_R'}.get(tag, tag) }")
        if hub is None:
            hub = get(doc, f"{gname}_hub_{ {'0': 'FL_L', '90': 'FR_R'}.get(tag, tag) }")
        for lp in lifts:
            for p in (pair(whl, lp, f"{gname}: колесо{tag}-{lp.Name.split('_')[-1]}"),
                      pair(hub, lp, f"{gname}: ступица{tag}-{lp.Name.split('_')[-1]}")):
                if p:
                    res.append(p)
        for p in (pair(whl, body, f"{gname}: колесо{tag}-панель"),
                  pair(hub, body, f"{gname}: ступица{tag}-панель")):
            if p:
                res.append(p)
        if has_drv:
            drv = get(doc, f"{gname}_drv{tag}")
            if drv is None:   # схема P: имена drv1_1 и т.п.
                for o in doc.Objects:
                    if o.Name.startswith(f"{gname}_drv"):
                        drv = o
                        break
            p = pair(whl, drv, f"{gname}: колесо{tag}-привод")
            if p:
                res.append(p)
    # соседние колёса
    w0 = get(doc, f"{gname}_WHL0") or get(doc, f"{gname}_WHL_FL_L")
    w90 = get(doc, f"{gname}_WHL90") or get(doc, f"{gname}_WHL_FR_R")
    p = pair(w0, w90, f"{gname}: колесо-колесо (соседи)")
    if p:
        res.append(p)
    # аутригеры
    pod = get(doc, f"{gname}_pod45") or get(doc, f"{gname}_pod0")
    if pod is not None and body is not None:
        p = pair(pod, body, f"{gname}: блок аутригера-панель")
        if p:
            res.append(p)
    if has_bat and pod is not None:
        bat = get(doc, f"{gname}_bat45") or get(doc, f"{gname}_bat0")
        p = pair(pod, bat, f"{gname}: блок аутригера-АКБ")
        if p:
            res.append(p)
    return res


all_res = {}

# ---------- v5: базовый (лифт на полную высоту z 0..400) ----------
f5 = os.path.join(SCRIPT_DIR, "stage2g_v5_compare.FCStd")
doc5 = App.openDocument(f5)
doc5.recompute()
rows = []
for g in ("R620_D152", "R640_D152", "R620_D203", "R640_D203",
          "P620_D152", "P640_D152", "P620_D203", "P640_D203",
          "R640_D230", "P640_D230"):
    rows += check_group(doc5, g)
all_res["v5_baseline"] = rows

# ---------- v6: Z-разнесённые варианты (если есть) ----------
f6 = os.path.join(SCRIPT_DIR, "stage2g_v6_variants.FCStd")
if os.path.exists(f6):
    doc6 = App.openDocument(f6)
    doc6.recompute()
    rows6 = []
    for g in ("P640_D152_Z", "R620_CTM"):
        rows6 += check_group(doc6, g, has_drv=(g == "P640_D152_Z"), has_bat=True,
                             wheel_tags=("0", "90"))
    all_res["v6_variants"] = rows6

# ---------- печать ----------
for src, rows in all_res.items():
    print(f"\n== {src} ==")
    print(f"{'пара':<42} {'расст., мм':>10} {'общ., см3':>10} {'статус':>17}")
    for r in rows:
        print(f"{r['pair']:<42} {r['dist_mm']:>10.2f} {r['common_cm3']:>10.3f} {r['verdict']:>17}")

out = {"results": all_res, "notes": [
    "distToShape: минимальное 3D-расстояние между телами; common.Volume > 0 = реальное пересечение.",
    "Пересечение 'лифт-панель' — конвенция модели (зона лифта проходит сквозь панель; в изделии — отверстие).",
    "Доступ монтажа/обслуживания — по зазорам: колёса вставляются сверху в вертикальные ниши,",
    "ступицы крепятся к щекам ниши; приводы монтируются в полосе; блоки аутригеров — снаружи панели.",
]}
path = os.path.join(SCRIPT_DIR, "stage2g_v6_3d_check.json")
with open(path, "w", encoding="utf-8") as f:
    json.dump(out, f, ensure_ascii=False, indent=1)
print("\nJSON:", path)