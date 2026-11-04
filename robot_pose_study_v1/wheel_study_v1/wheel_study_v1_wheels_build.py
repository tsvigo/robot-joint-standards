# -*- coding: utf-8 -*-
"""Принципиальные CAD-модели собственных Mecanum-колёс Ø180/200/220/240
(этап 1, задача 8). Создаёт НОВЫЙ файл wheel_study_v1_wheels.FCStd.

Принципиальный уровень: ступица-кольцо + приводной фланец + N свободных
роликов (цилиндры) на осях под 45° + оси роликов. Без деталировки
(канавки, резьбы, дорожки подшипников, крепёж) — она не требуется на этапе 1.
Геометрия — из wheel_study_v1_parametric.json (переменные, не принятые
параметры). v6 и Blender не изменяются.
"""
import json
import math
import os

import FreeCAD as App
import Part

HERE = os.path.dirname(os.path.abspath(__file__))
with open(os.path.join(HERE, "wheel_study_v1_parametric.json"), encoding="utf-8") as f:
    P = json.load(f)

ALPHA = math.radians(45.0)
ROLLER_D = P["variables"]["ROLLER_D"] * 1000      # мм
ROLLER_L = P["variables"]["ROLLER_L"] * 1000
AXIS_D = P["variables"]["AXIS_D"] * 1000
FLANGE_D = P["variables"]["HUB_FLANGE_D"] * 1000
FLANGE_T = 8.0

doc = App.newDocument("wheel_study_v1")
made = []
for row in P["geometry"]:
    g = row["geometry"]
    D = g["D_мм"]
    N = g["N_роликов"]
    R_axis = g["R_осей_мм"]                      # радиус установки осей роликов
    W = g["W_мм"]                                # ширина колеса
    R = g["R_мм"]

    # --- ступица: кольцо (наружный диаметр ~1.1*R_axis, внутренний под полость) ---
    hub_out_d = min(2 * R_axis * 0.98, D - 20)
    hub_in_d = FLANGE_D + 10
    hub_out = Part.makeCylinder(hub_out_d / 2, W)
    hub_in = Part.makeCylinder(hub_in_d / 2, W + 2, App.Vector(0, 0, -1))
    hub = hub_out.cut(hub_in)

    # --- приводной фланец (по оси колеса z) ---
    flange = Part.makeCylinder(FLANGE_D / 2, FLANGE_T + 6,
                               App.Vector(0, 0, -W / 2 - 6))

    shapes = [hub, flange]
    # --- N роликов на осях под 45° (радиальная плоскость) ---
    for i in range(N):
        phi = 2 * math.pi * i / N
        er = App.Vector(math.cos(phi), math.sin(phi), 0)
        # направление оси ролика: 45° к оси колеса в радиальной плоскости
        n = App.Vector(er.x * math.cos(ALPHA), er.y * math.cos(ALPHA),
                       math.sin(ALPHA))
        n.normalize()
        # центр оси ролика — точка обода (R_axis, z=0)
        c = App.Vector(er.x * R_axis, er.y * R_axis, 0)
        start = c - n.multiply(ROLLER_L / 2)
        roller = Part.makeCylinder(ROLLER_D / 2, ROLLER_L, start, n)
        shapes.append(roller)
        # ось ролика (d=10) — чуть длиннее
        a_start = c - n.multiply(ROLLER_L / 2 + 4)
        axis = Part.makeCylinder(AXIS_D / 2, ROLLER_L + 8, a_start, n)
        shapes.append(axis)

    compound = Part.makeCompound(shapes)
    obj = doc.addObject("Part::Feature", f"wheel_D{int(D)}")
    obj.Shape = compound
    obj.Label = f"Mecanum_D{int(D)}_N{N}"
    made.append({"label": obj.Label, "D": D, "N": N, "R_axis_mm": R_axis,
                 "W_mm": W, "rollers": N})

doc.recompute()
out_path = os.path.join(HERE, "wheel_study_v1_wheels.FCStd")
if os.path.exists(out_path):
    os.remove(out_path)                            # только что созданный нами файл
doc.saveAs(out_path)
print("== Принципиальные CAD-модели колёс ==")
for m in made:
    print(f"  {m['label']}: Ø{m['D']:.0f}, N={m['N']}, оси R={m['R_axis_mm']:.1f}, "
          f"W={m['W_mm']:.1f} мм")
print("Сохранено:", out_path)
# проверка чтением обратно
import hashlib
h = hashlib.sha256()
with open(out_path, "rb") as f:
    for chunk in iter(lambda: f.read(1 << 20), b""):
        h.update(chunk)
print("SHA16:", h.hexdigest()[:16])
App.closeDocument(doc.Name)