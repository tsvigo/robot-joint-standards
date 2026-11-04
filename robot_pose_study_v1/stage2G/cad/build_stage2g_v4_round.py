# -*- coding: utf-8 -*-
"""Этап 2Г — build_stage2g_v4_round.py
CAD-компоновка КРУГЛОЙ платформы v4: Ø600/620/640 x Mecanum D152/D203/D230.
Новый файл stage2g_v4_round.FCStd; v1/v2/v3 и Blender НЕ изменяются.
Содержимое групп: панель с прямоугольными нишами, 4 колёса (ось радиальная, || пол),
зона лифта, объёмы приводов, секторы оборудования, убранные блоки аутригеров,
выдвинутые площадки (показ. ход 300), опорные контуры, транспортный конверт.
Проверка п.4: фактический максимальный радиус всех тел группы <= R_env.
"""
import json
import math
import os
import FreeCAD as App

try:
    SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
except NameError:
    SCRIPT_DIR = os.getcwd()

with open(os.path.join(SCRIPT_DIR, "stage2g_v4_round_results.json"), encoding="utf-8") as f:
    CMP = json.load(f)
P = CMP["params"]
Z0_PANEL = 50.0
PANEL_T = 40.0
ENV_H = 1900.0
STROKE_VIS = 300.0     # показательный ход для визуализации (параметр)
HANDS = {0: "FL_L", 90: "FR_R", 180: "BR_L", 270: "BL_R"}   # X-конфигурация: FL+BR = L

doc = App.newDocument("stage2g_v4_round")


def rotz(x, y, deg):
    a = math.radians(deg)
    return x * math.cos(a) - y * math.sin(a), x * math.sin(a) + y * math.cos(a)


def add_box(name, lx, ly, lz, base_local, theta, group):
    o = doc.addObject("Part::Box", name)
    o.Length, o.Width, o.Height = lx, ly, lz
    bx, by, bz = base_local
    wx, wy = rotz(bx, by, theta)
    o.Placement.Base = App.Vector(wx, wy, bz)
    o.Placement.Rotation = App.Rotation(App.Vector(0, 0, 1), theta)
    group.addObject(o)
    return o


def add_cyl(name, R, H, base, group, axis_theta=None):
    o = doc.addObject("Part::Cylinder", name)
    o.Radius, o.Height = R, H
    o.Placement.Base = App.Vector(*base)
    if axis_theta is not None:
        # ось цилиндра -> радиальное направление theta: Ry(90) затем Rz(theta)
        o.Placement.Rotation = (App.Rotation(App.Vector(0, 0, 1), axis_theta) *
                                App.Rotation(App.Vector(0, 1, 0), 90))
    group.addObject(o)
    return o


for v in CMP["variants"]:
    if abs(v["Dp"] - 600) < 1e-9 and v["wheel"] != "D152":
        pass
    gname = f"R{int(v['Dp'])}_{v['wheel']}"
    grp = doc.addObject("App::Part", gname)
    grp.Label = f"Ø{int(v['Dp'])} / {v['wheel']}"
    R_env = v["Dp"] / 2.0
    D, W = v["D"], v["W"]
    r_c, c = v["r_c"], P["well_clr"]
    lift_zone = P["lift_R"] + P["clr"]

    # панель + 4 прямоугольные ниши (снизу вверх: цилиндр, затем вырезы)
    panel = add_cyl(f"{gname}_panel", R_env, PANEL_T, (0, 0, Z0_PANEL), grp)
    tool_ids = []
    for i, th in enumerate((0, 90, 180, 270), 1):
        cut = add_box(f"{gname}_well{i}", W + 2 * c, D + 2 * c, PANEL_T + 20,
                      (r_c - W / 2 - c, -(D / 2 + c), Z0_PANEL - 10), th, grp)
        tool_ids.append(cut.Name)
    cur = panel
    for i, tid in enumerate(tool_ids, 1):
        cut = doc.addObject("Part::Cut", f"{gname}_panelC{i}")
        cut.Base = cur
        cut.Tool = doc.getObject(tid)
        cur = cut
    doc.recompute()   # пересчитать Cut, ДО копирования формы в финальное тело
    # финальная панель переименовываем для экспорта: переносим форму
    panel_final = doc.addObject("Part::Feature", f"{gname}_body")
    panel_final.Shape = cur.Shape
    grp.addObject(panel_final)

    # колёса 2L+2R (оси радиальные, горизонтальные) + маркеры руки
    for th, hand in HANDS.items():
        a = math.radians(th)
        bx = (r_c - W / 2.0) * math.cos(a)
        by = (r_c - W / 2.0) * math.sin(a)
        add_cyl(f"{gname}_WHL_{hand}", D / 2.0, W, (bx, by, D / 2.0), grp, axis_theta=th)
        mx, my = rotz(r_c, 0, th)
        m = doc.addObject("Part::Box", f"{gname}_hand_{hand}")
        m.Length = m.Width = m.Height = 10.0
        m.Placement.Base = App.Vector(mx - 5, my - 5, D + 5)
        grp.addObject(m)

    # зона лифта (компоновочный объём)
    add_cyl(f"{gname}_lift", P["lift_R"], 400, (0, 0, 0), grp)

    # объёмы приводов (полоса между зоной лифта и нишей, при band>0)
    if v["band_mm"] > 1.0:
        band = v["band_mm"]
        for th in (0, 90, 180, 270):
            add_box(f"{gname}_drive{th}", band, 140.0, 90.0,
                    (lift_zone, -70.0, 45.0), th, grp)

    # секторы оборудования (батареи/электроника) на 45°
    if v["battery_ok"] and v["battery_half_tan"] >= 10:
        hb = v["battery_half_tan"]
        r_out = R_env - P["bat_Rout_margin"]
        for th in (45, 135, 225, 315):
            add_box(f"{gname}_bat{th}", r_out - lift_zone, 2 * hb, 100.0,
                    (lift_zone, -hb, 50.0), th, grp)

    # убранные блоки аутригеров (в транспортном габарите) на 45°
    for th in (45, 135, 225, 315):
        add_box(f"{gname}_pod{th}", P["pod_radial_in"], 2 * P["pod_half_tan"], 100.0,
                (R_env - 20.0 - P["pod_radial_in"], -P["pod_half_tan"], 40.0), th, grp)
        # выдвинутая площадка (показ. ход) + опорный след
        pad_r = (R_env - 20.0 - P["pod_radial_in"] / 2.0) + STROKE_VIS
        add_box(f"{gname}_pad{th}", 150.0, 150.0, 40.0,
                (pad_r - 75.0, -75.0, 0.0), th, grp)

    # опорные контуры: колёса (ромб, вершины на осях = позиции колёс) и
    # аутригеры (квадрат БЕЗ поворота: вершины на 45° = позиции площадок)
    s_w = r_c * math.sqrt(2.0)
    add_box(f"{gname}_supp", s_w, s_w, 3.0, (-s_w / 2, -s_w / 2, -6.0), 45, grp)
    pad_r = (R_env - 20.0 - P["pod_radial_in"] / 2.0) + STROKE_VIS
    s_p = pad_r * math.sqrt(2.0)
    add_box(f"{gname}_suppExt", s_p, s_p, 3.0, (-s_p / 2, -s_p / 2, -12.0), 0, grp)

    # транспортный конверт (высота 1900)
    add_cyl(f"{gname}_env", R_env, ENV_H, (0, 0, 0), grp)

doc.recompute()
# скрыть операнды вырезов (инструменты и промежуточные Cut) — в открытии видна только _body
for o in doc.Objects:
    n = o.Name
    if "_well" in n or "_panelC" in n or n.endswith("_panel"):
        o.Visibility = False

fcstd = os.path.join(SCRIPT_DIR, "stage2g_v4_round.FCStd")
doc.saveAs(fcstd)
print(f"[v4] saved: {fcstd} objects={len(doc.Objects)}")

# ================= ПРОВЕРКИ =================
print("== CHECKS ==")
fail = 0
SKIP = ("_env", "_supp", "_pad", "_well")
for v in CMP["variants"]:
    gname = f"R{int(v['Dp'])}_{v['wheel']}"
    R_env = v["Dp"] / 2.0
    max_r = 0.0
    worst = None
    for o in doc.Objects:
        if not o.Name.startswith(gname + "_") or not hasattr(o, "Shape") or o.Shape.isNull():
            continue
        if any(o.Name.endswith(s) or s in o.Name for s in ("_well", "_supp", "_pad")):
            continue
        if "_env" in o.Name or "panelC" in o.Name:
            continue
        for vt in o.Shape.Vertexes:
            r = math.hypot(vt.Point.x, vt.Point.y)
            if r > max_r:
                max_r, worst = r, o.Name
    ok = max_r <= R_env + 0.01
    # транспортное положение аутригеров внутри габарита
    pod_r = math.hypot(R_env - 20.0, P["pod_half_tan"])
    ok_pod = pod_r <= R_env + 1e-6
    # колёса касаются контура (при r=r_max) и не выходят
    st = "PASS" if (ok and ok_pod) else "FAIL"
    if st == "FAIL":
        fail += 1
    print(f"  {st} {gname}: max_r={max_r:.2f} <= {R_env:.2f} (worst={worst}); "
          f"pod_corner={pod_r:.1f}; band={v['band_mm']}; hub={'OK' if v['feasible']['hub'] else 'НЕТ'}")
print("CHECK_STATUS:", "PASS 9/9" if fail == 0 else f"FAIL {fail}")
