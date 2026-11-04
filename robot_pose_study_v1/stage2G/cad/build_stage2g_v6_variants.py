# -*- coding: utf-8 -*-
"""Этап 2Г — build_stage2g_v6_variants.py

Два компактных сравнительных варианта (задача 9), НОВЫЙ файл stage2g_v6_variants.FCStd:
  P640_D152_Z — схема P (параллельные оси, X-раскладка), D152, Ø640, Z-разнесённый лифт:
      колёса/приводы/нижняя рама — низко, механизм лифта — выше верха колёс;
      путь передачи сил: лифт -> переходной конус -> стойка Ø120 -> центр панели ->
      радиальные рёбра -> узлы крепления колёс.
  R620_CTM   — схема R (радиальные оси, 4 одинаковых модуля), РЕАЛЬНЫЙ модуль
      RoboCT CTM1000-10R (Ø240 x 106.9), Ø620: лифт во всю высоту (запас +28 мм по 2D).
Пересечения v5 устранены: блоки аутригеров и приводы вынесены под панель (z 12..48),
батареи — на панель (z 92..192). v1–v5 и Blender не изменяются.
"""
import json
import math
import os
import FreeCAD as App

try:
    SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
except NameError:
    SCRIPT_DIR = os.getcwd()

with open(os.path.join(SCRIPT_DIR, "stage2g_v5_geometry.json"), encoding="utf-8") as f:
    G = json.load(f)
CASES = {(c["layout"], c["Dp"], c["wheel"], c["drive"]): c for c in G["cases"]}
P5 = G["params"]
LIFT_CLEAR_R = G["lift_clear_R"]
STROKE = 300.0
Z_PANEL = (50.0, 90.0)

doc = App.newDocument("stage2g_v6_variants")


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


def add_cyl(name, R, H, base, group):
    o = doc.addObject("Part::Cylinder", name)
    o.Radius, o.Height = R, H
    o.Placement.Base = App.Vector(*base)
    group.addObject(o)
    return o


def add_cyl_radial(name, R, H, base, theta, group):
    o = doc.addObject("Part::Cylinder", name)
    o.Radius, o.Height = R, H
    o.Placement.Base = App.Vector(*base)
    o.Placement.Rotation = (App.Rotation(App.Vector(0, 0, 1), theta) *
                            App.Rotation(App.Vector(0, 1, 0), 90))
    group.addObject(o)
    return o


def add_cyl_y(name, R, H, base, group):
    o = doc.addObject("Part::Cylinder", name)
    o.Radius, o.Height = R, H
    o.Placement.Base = App.Vector(*base)
    o.Placement.Rotation = App.Rotation(App.Vector(1, 0, 0), -90)
    group.addObject(o)
    return o


def cut_wells(panel, tools, gname, grp):
    cur = panel
    for i, t in enumerate(tools, 1):
        cut = doc.addObject("Part::Cut", f"{gname}_panelC{i}")
        cut.Base = cur
        cut.Tool = t
        cur = cut
    doc.recompute()
    body = doc.addObject("Part::Feature", f"{gname}_body")
    body.Shape = cur.Shape
    grp.addObject(body)
    return body


# ============================================================
# Вариант 1: P640_D152_Z — схема P с Z-разнесённым лифтом
# ============================================================
gname = "P640_D152_Z"
grp = doc.addObject("App::Part", gname)
grp.Label = "P1: схема P, D152, Ø640, лифт Z-разнесён"
c = CASES[("P", 640.0, "D152", "belt")]
u = c["u"]
D, W = 152.4, 80.0                     # номинальные размеры колеса D152
W_eff, D_asm = c["W_eff"], c["D_asm"]
cc = P5["well_clr"]
R_env = 640.0 / 2.0

panel = add_cyl(f"{gname}_panel", R_env, 40.0, (0, 0, Z_PANEL[0]), grp)
tools = []
for i, (sx, sy) in enumerate(((1, 1), (1, -1), (-1, 1), (-1, -1)), 1):
    tools.append(add_box(f"{gname}_well{i}", D_asm + 2 * cc, W_eff + 2 * cc, 60.0,
                         (sx * u - D_asm / 2 - cc, sy * u - W_eff / 2 - cc, Z_PANEL[0] - 10), 0, grp))
cut_wells(panel, tools, gname, grp)

for tag, sx, sy in (("FL_L", 1, 1), ("FR_R", 1, -1), ("RL_R", -1, 1), ("RR_L", -1, -1)):
    x_i, y_i = sx * u, sy * u
    add_cyl_y(f"{gname}_WHL_{tag}", D / 2.0, W, (x_i, y_i - W / 2.0, D / 2.0), grp)
    add_cyl_y(f"{gname}_hub_{tag}", D * 0.225, W_eff, (x_i, y_i - W_eff / 2.0, D / 2.0), grp)
    m = doc.addObject("Part::Box", f"{gname}_hand_{tag}")
    m.Length = m.Width = m.Height = 10.0
    m.Placement.Base = App.Vector(x_i - 5, y_i - 5, D + 5)
    grp.addObject(m)

# приводы — под панелью (z 12..48), в полосе за колесами
band = c["band_mm"]
for sx, sy in ((1, 1), (1, -1), (-1, 1), (-1, -1)):
    y_face = u - W_eff / 2 - cc
    base_y = (y_face - band) if sy > 0 else -y_face
    add_box(f"{gname}_drv{sx}_{sy}", 60.0, band, 32.0, (sx * u - 30.0, base_y, 8.0), 0, grp)

# путь передачи сил лифта: стойка Ø120 -> конус -> верхняя зона Ø240
add_cyl(f"{gname}_lift_col", 60.0, 72.0, (0, 0, 90.0), grp)                    # стойка в «кресте»
cone = doc.addObject("Part::Cone", f"{gname}_lift_cone")
cone.Radius1, cone.Radius2, cone.Height = 60.0, 120.0, 20.0
cone.Placement.Base = App.Vector(0, 0, 162.0)
grp.addObject(cone)
add_cyl(f"{gname}_lift_top", 120.0, 218.0, (0, 0, 182.0), grp)                 # механизм лифта
# радиальные рёбра жёсткости в свободных лучах 0/90/180/270 (путь силы к раме)
for th in (0, 90, 180, 270):
    add_box(f"{gname}_rib{th}", 125.0, 80.0, 20.0, (55.0, -40.0, 30.0), th, grp)

# батареи — на панель (z 92..192)
hb = c["battery_half_tan"]
r_out = R_env - P5["bat_Rout_margin"]
for th in (0, 90, 180, 270):
    add_box(f"{gname}_bat{th}", r_out - LIFT_CLEAR_R, 2 * hb, 100.0,
            (LIFT_CLEAR_R, -hb, 92.0), th, grp)

# аутригеры: блоки под панелью + телескоп + площадки
pod_r_out = R_env - 30.0
pod_center = pod_r_out - 50.0
pad_r = pod_center + STROKE
for th in (0, 90, 180, 270):
    add_box(f"{gname}_pod{th}", 100.0, 140.0, 34.0, (pod_r_out - 100.0, -70.0, 8.0), th, grp)
    add_box(f"{gname}_armO{th}", 170.0, 60.0, 26.0, (pod_center, -30.0, 12.0), th, grp)
    add_box(f"{gname}_armI{th}", pad_r - (pod_center + 170.0), 40.0, 18.0,
            (pod_center + 170.0, -20.0, 14.0), th, grp)
    add_box(f"{gname}_pad{th}", 150.0, 150.0, 30.0, (pad_r - 75.0, -75.0, 0.0), th, grp)
    mk = doc.addObject("Part::Box", f"{gname}_pt{th}")
    mk.Length = mk.Width = mk.Height = 30.0
    mx, my = rotz(pad_r, 0, th)
    mk.Placement.Base = App.Vector(mx - 15, my - 15, -3)
    grp.addObject(mk)

# опорные контуры
add_box(f"{gname}_supp", 2 * u, 2 * (u + W / 2.0), 3.0, (-u, -(u + W / 2.0), -6.0), 0, grp)
s2 = pad_r * math.sqrt(2.0)
add_box(f"{gname}_suppExt", s2, s2, 3.0, (-s2 / 2, -s2 / 2, -12.0), 45, grp)
add_cyl(f"{gname}_env", R_env, 1900.0, (0, 0, 0), grp)

# ============================================================
# Вариант 2: R620_CTM — схема R с реальным модулем CTM1000-10R
# ============================================================
gname = "R620_CTM"
grp = doc.addObject("App::Part", gname)
grp.Label = "R2: схема R, модуль CTM1000-10R (Ø240x106.9), Ø620"
D_m, W_m = 240.0, 106.9
R_env = 620.0 / 2.0
r_c = math.sqrt((R_env - 10.0) ** 2 - (D_m / 2.0) ** 2) - W_m / 2.0   # = 221.5

panel = add_cyl(f"{gname}_panel", R_env, 40.0, (0, 0, Z_PANEL[0]), grp)
tools = []
for i, th in enumerate((0, 90, 180, 270), 1):
    tools.append(add_box(f"{gname}_well{i}", W_m + 2 * 10.0, D_m + 2 * 10.0, 60.0,
                         (r_c - W_m / 2 - 10.0, -(D_m / 2 + 10.0), Z_PANEL[0] - 10), th, grp))
cut_wells(panel, tools, gname, grp)

for th in (0, 90, 180, 270):
    a = math.radians(th)
    bx, by = (r_c - W_m / 2.0) * math.cos(a), (r_c - W_m / 2.0) * math.sin(a)
    add_cyl_radial(f"{gname}_WHL{th}", D_m / 2.0, W_m, (bx, by, D_m / 2.0), th, grp)
    mx, my = rotz(r_c, 0, th)
    m = doc.addObject("Part::Box", f"{gname}_hand_A{th}_L")
    m.Length = m.Width = m.Height = 10.0
    m.Placement.Base = App.Vector(mx - 5, my - 5, D_m + 5)
    grp.addObject(m)

# лифт во всю высоту (2D-запас +28 мм): стойка на панели
add_cyl(f"{gname}_lift", 120.0, 310.0, (0, 0, 90.0), grp)
add_cyl(f"{gname}_lift_flange", 120.0, 10.0, (0, 0, 90.0), grp)

hb = 30.0
r_out = R_env - P5["bat_Rout_margin"]
for th in (45, 135, 225, 315):
    add_box(f"{gname}_bat{th}", r_out - LIFT_CLEAR_R, 2 * hb, 100.0,
            (LIFT_CLEAR_R, -hb, 92.0), th, grp)

pod_r_out = R_env - 30.0
pod_center = pod_r_out - 50.0
pad_r = pod_center + STROKE
for th in (45, 135, 225, 315):
    # узкий блок (±30): широкие модули D240 занимают ±35° на внутренней кромке
    add_box(f"{gname}_pod{th}", 100.0, 60.0, 34.0, (pod_r_out - 100.0, -30.0, 8.0), th, grp)
    add_box(f"{gname}_armO{th}", 170.0, 40.0, 26.0, (pod_center, -20.0, 12.0), th, grp)
    add_box(f"{gname}_armI{th}", pad_r - (pod_center + 170.0), 30.0, 18.0,
            (pod_center + 170.0, -15.0, 14.0), th, grp)
    add_box(f"{gname}_pad{th}", 150.0, 150.0, 30.0, (pad_r - 75.0, -75.0, 0.0), th, grp)
    mk = doc.addObject("Part::Box", f"{gname}_pt{th}")
    mk.Length = mk.Width = mk.Height = 30.0
    mx, my = rotz(pad_r, 0, th)
    mk.Placement.Base = App.Vector(mx - 15, my - 15, -3)
    grp.addObject(mk)

rr = r_c + W_m / 2.0
s1 = rr * math.sqrt(2.0)
add_box(f"{gname}_supp", s1, s1, 3.0, (-s1 / 2, -s1 / 2, -6.0), 45, grp)
s2 = pad_r * math.sqrt(2.0)
add_box(f"{gname}_suppExt", s2, s2, 3.0, (-s2 / 2, -s2 / 2, -12.0), 0, grp)
add_cyl(f"{gname}_env", R_env, 1900.0, (0, 0, 0), grp)

doc.recompute()
for o in doc.Objects:
    n = o.Name
    if "_well" in n or "_panelC" in n or n.endswith("_panel"):
        o.Visibility = False

path = os.path.join(SCRIPT_DIR, "stage2g_v6_variants.FCStd")
doc.saveAs(path)
print(f"[v6] saved: {path} objects={len(doc.Objects)}")

# ================= ПРОВЕРКИ: пересечений быть не должно =================
print("== CHECKS v6: парные 3D-проверки (пересечений > 1 см3 быть не должно) ==")
import itertools
GROUPS = {"P640_D152_Z": ["_body", "_WHL_", "_hub_", "_lift", "_drv", "_bat", "_pod", "_rib", "_arm"],
          "R620_CTM": ["_body", "_WHL", "_lift", "_bat", "_pod", "_arm"]}
fails = 0
def assembly_pair(a, b):
    """Пары, являющиеся одной сборкой (не дефекты): ступица в колесе, рычаг в блоке."""
    n1, n2 = a.Name, b.Name
    if ("_WHL" in n1 and "_hub" in n2) or ("_WHL" in n2 and "_hub" in n1):
        return True
    if ("_pod" in n1 and "_arm" in n2) or ("_pod" in n2 and "_arm" in n1):
        return True
    if "_lift" in n1 and "_lift" in n2:
        return True   # детали лифта — одна сборка
    return False

for gname, pats in GROUPS.items():
    objs = [o for o in doc.Objects if o.Name.startswith(gname + "_")
            and any(p in o.Name for p in pats) and hasattr(o, "Shape") and not o.Shape.isNull()]
    for a, b in itertools.combinations(objs, 2):
        if assembly_pair(a, b):
            continue
        v = a.Shape.common(b.Shape).Volume
        if v > 1000.0:
            fails += 1
            print(f"  FAIL {a.Name} x {b.Name}: общ.объём {v/1000:.1f} см3")
print("CHECK_STATUS:", "PASS — пересечений нет" if fails == 0 else f"FAIL {fails}")
