# -*- coding: utf-8 -*-
"""Этап 2Г — build_stage2g_v3_layout.py
Сравнительная CAD-компоновка v3 (новый файл, v1/v2 НЕ изменяются):
  варианты A (без аутригеров) и B (с 4 аутригерами) x колёса MX152/MX203/MX230.
Геометрия — только транспортные габариты, колёса, опорные плоскости и объёмы аутригеров
(параметрические оценки, НЕ утверждённые). Расчётов моментов/прочности/устойчивости нет.
"""
import json
import math
import os
import FreeCAD as App
import Part

try:
    SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
except NameError:
    SCRIPT_DIR = os.getcwd()

with open(os.path.join(SCRIPT_DIR, "stage2g_v3_compare_results.json"), encoding="utf-8") as f:
    CMP = json.load(f)

REQ = CMP["requirements"]
WMAX = REQ["transport_width_max"]      # 640
HMAX = REQ["transport_height_max"]     # 1900
POD = {"LX": 200.0, "CY": 120.0, "CZ": 120.0}   # параметрический объём углового блока аутригера
PAD = {"LX": 150.0, "CZ": 40.0}                  # параметрическая площадка опоры (след на полу)

doc = App.newDocument("stage2g_v3_layout")


def make_box(name, lx, ly, lz, cx, cy, cz, group):
    o = doc.addObject("Part::Box", name)
    o.Length, o.Width, o.Height = lx, ly, lz
    o.Placement.Base = App.Vector(cx - lx / 2, cy - ly / 2, cz)
    group.addObject(o)
    return o


def make_wheel(name, D, W, xc, yc, group):
    o = doc.addObject("Part::Cylinder", name)
    o.Radius, o.Height = D / 2.0, W
    # диск с осью || Y: центр в (xc, yc, D/2); привязка по Y со смещением -W/2
    o.Placement.Base = App.Vector(xc, yc - W / 2.0, D / 2.0)
    o.Placement.Rotation = App.Rotation(App.Vector(1, 0, 0), -90.0)
    group.addObject(o)
    return o


def make_plane(name, tx, bx, z_h, group):
    o = doc.addObject("Part::Box", name)
    o.Length, o.Width, o.Height = bx, tx, z_h
    o.Placement.Base = App.Vector(-bx / 2, -tx / 2, -z_h)   # плоскость опор под полом (силуэт)
    group.addObject(o)
    return o


for opt in CMP["options"]:
    v, wk = opt["variant"], opt["wheel"]
    grp = doc.addObject("App::Part", f"{v}_{wk}")
    grp.Label = f"Вариант {v} / {wk}"
    D, W = opt["D"], opt["W"]
    L, T = opt["length_L"], opt["track_T"]
    xc, yc = opt["x_c"], opt["y_c"]

    # транспортный габарит (лайт-бокс)
    make_box(f"{v}_{wk}_env", L, WMAX, HMAX, 0, 0, 0, grp)
    # опорная плоскость (силуэт варианта A)
    make_plane(f"{v}_{wk}_suppA", T, opt["wheelbase_B"], 3.0, grp)

    # колёса 2L+2R (X-конфигурация: FL+BR одна рука, FR+BL другая — см. отчёт)
    hands = {"FL": "L", "FR": "R", "BL": "R", "BR": "L"}  # X-раскладка по диагоналям
    for key, sx, sy in (("FL", +1, +1), ("FR", +1, -1), ("BL", -1, +1), ("BR", -1, -1)):
        wh = make_wheel(f"{v}_{wk}_WHL_{key}", D, W, sx * xc, sy * yc, grp)
        # маркер руки: кубик 10 мм сбоку от оси (L/R диагонали)
        m = doc.addObject("Part::Box", f"{v}_{wk}_hand_{key}{hands[key]}")
        m.Length = m.Width = m.Height = 10.0
        m.Placement.Base = App.Vector(sx * xc - 5, sy * yc + (5 if hands[key] == "L" else -5) - 5,
                                      (120.0 if D > 200 else 60.0))
        grp.addObject(m)

    if v == "B":
        s = CMP["params"]["stroke_study_mm"][1]   # показательный ход 300 мм (параметр)
        # угловые блоки в транспортном положении (внутри габарита 640 — без выступов)
        for sx, sy in ((+1, +1), (+1, -1), (-1, +1), (-1, -1)):
            make_box(f"{v}_{wk}_pod_{'f' if sx > 0 else 'b'}{'l' if sy > 0 else 'r'}",
                     POD["LX"], POD["CY"], POD["CZ"],
                     sx * xc, sy * (WMAX / 2 - POD["CY"] / 2 - 5), 40.0, grp)
            # выдвинутая площадка (показательный ход) + опорный след
            make_box(f"{v}_{wk}_arm_{'f' if sx > 0 else 'b'}{'l' if sy > 0 else 'r'}",
                     80.0, s, 50.0, sx * xc, sy * (yc + s / 2), 35.0, grp)
            make_box(f"{v}_{wk}_pad_{'f' if sx > 0 else 'b'}{'l' if sy > 0 else 'r'}",
                     PAD["LX"], PAD["LX"], PAD["CZ"],
                     sx * xc, sy * (yc + s), 0.0, grp)
        # опорная плоскость варианта B (ход 300)
        make_plane(f"{v}_{wk}_suppB", T + 2 * s, opt["wheelbase_B"], 3.0, grp)

    doc.recompute()

fcstd = os.path.join(SCRIPT_DIR, "stage2g_v3_layout.FCStd")
doc.saveAs(fcstd)
print(f"[v3] saved: {fcstd}  objects={len(doc.Objects)}")

# --- контроль габаритов групп ---
print("== CHECKS ==")
fail = 0
for opt in CMP["options"]:
    v, wk = opt["variant"], opt["wheel"]
    grp = doc.getObject(f"{v}_{wk}")
    objs = [o for o in doc.Objects if o.Name.startswith(f"{v}_{wk}_") and hasattr(o, "Shape")
            and not o.Shape.isNull()]
    # транспортный габарит по боксам-оболочкам: env задан явно WMAX/HMAX
    env = doc.getObject(f"{v}_{wk}_env")
    ok_w = abs(env.Shape.BoundBox.YLength - WMAX) < 1e-6
    ok_h = abs(env.Shape.BoundBox.ZLength - HMAX) < 1e-6
    ok_l = abs(env.Shape.BoundBox.XLength - opt["length_L"]) < 1e-3
    # колёса не шире габарита
    wheels_ok = all(doc.getObject(f"{v}_{wk}_WHL_{k}").Shape.BoundBox.YLength <= WMAX + 1e-6
                    for k in ("FL", "FR", "BL", "BR"))
    status = "PASS" if (ok_w and ok_h and ok_l and wheels_ok) else "FAIL"
    if status == "FAIL":
        fail += 1
    print(f"  {status} {v}_{wk}: L={opt['length_L']} W={WMAX} H={HMAX}; wheels_within={wheels_ok}")
print("CHECK_STATUS:", "PASS" if fail == 0 else f"FAIL {fail}")
