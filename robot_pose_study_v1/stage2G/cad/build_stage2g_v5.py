# -*- coding: utf-8 -*-
"""Этап 2Г — build_stage2g_v5.py

CAD-сравнение двух схем круглого основания (НОВЫЙ файл stage2g_v5_compare.FCStd):
  R — радиальные оси (колёса на 0/90/180/270°), руки ВСЕ ОДНОЙ руки (после аудита:
      X-раскладка LRLR сингулярна, ранг 2);
  P — параллельные оси || Y на диагоналях FL/FR/RL/RR, стандартная X-раскладка (FL+RR=L).
Геометрия из stage2g_v5_geometry.json (случай belt как визуальный базис, зазоры ненулевые).
18 групп: 2 схемы x Ø600/620/640 x D152/D203/D230.
Состояния разведены ПО КЛАССАМ: transport (кузов/колёса/зоны/блоки) и
working (руки выдвижения + площадки + опорный контур работы).
v1–v4 и Blender не изменяются.
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
P = G["params"]
LIFT_CLEAR_R = G["lift_clear_R"]
STROKE_VIS = 300.0
Z0_PANEL, PANEL_T = 50.0, 40.0
ENV_H = 1900.0

# индексация случаев: (layout, Dp, wheel, drive)
CASES = {}
for c in G["cases"]:
    CASES[(c["layout"], c["Dp"], c["wheel"], c["drive"])] = c

doc = App.newDocument("stage2g_v5_compare")


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


def add_cyl(name, R, H, base, group, axis=None):
    o = doc.addObject("Part::Cylinder", name)
    o.Radius, o.Height = R, H
    o.Placement.Base = App.Vector(*base)
    if axis == "radial":
        raise ValueError("use axis_radial")
    group.addObject(o)
    return o


def add_cyl_radial(name, R, H, base, theta, group):
    """Ось цилиндра -> радиальное направление theta (горизонтально)."""
    o = doc.addObject("Part::Cylinder", name)
    o.Radius, o.Height = R, H
    o.Placement.Base = App.Vector(*base)
    o.Placement.Rotation = (App.Rotation(App.Vector(0, 0, 1), theta) *
                            App.Rotation(App.Vector(0, 1, 0), 90))
    group.addObject(o)
    return o


def add_cyl_y(name, R, H, base, group):
    """Ось цилиндра -> +Y (параллельная схема)."""
    o = doc.addObject("Part::Cylinder", name)
    o.Radius, o.Height = R, H
    o.Placement.Base = App.Vector(*base)
    o.Placement.Rotation = App.Rotation(App.Vector(1, 0, 0), -90)
    group.addObject(o)
    return o


for layout in ("R", "P"):
    for Dp in (600.0, 620.0, 640.0):
        for wkey, D_w in (("D152", 152.4), ("D203", 203.0), ("D230", 230.0)):
            case = CASES[(layout, Dp, wkey, "belt")]     # визуальный базис
            gname = f"{layout}{int(Dp)}_{wkey}"
            grp = doc.addObject("App::Part", gname)
            grp.Label = f"{layout} Ø{int(Dp)} / {wkey}"
            R_env = Dp / 2.0
            D, W = D_w, {"D152": 80.0, "D203": 103.0, "D230": 113.0}[wkey]
            D_asm = D + P["dia_extra"]
            W_eff = W + P["asm_extra"]
            c = P["well_clr"]

            # ---------- панель с нишами ----------
            panel = add_cyl(f"{gname}_panel", R_env, PANEL_T, (0, 0, Z0_PANEL), grp)
            tools = []
            if layout == "R":
                r_c = case["r_c"]
                for i, th in enumerate((0, 90, 180, 270), 1):
                    tools.append(add_box(f"{gname}_well{i}", W_eff + 2 * c, D_asm + 2 * c,
                                         PANEL_T + 20,
                                         (r_c - W_eff / 2 - c, -(D_asm / 2 + c), Z0_PANEL - 10),
                                         th, grp))
            else:
                u = case["u"]
                for i, (sx, sy) in enumerate(((1, 1), (1, -1), (-1, 1), (-1, -1)), 1):
                    tools.append(add_box(f"{gname}_well{i}", D_asm + 2 * c, W_eff + 2 * c,
                                         PANEL_T + 20,
                                         (sx * u - D_asm / 2 - c, sy * u - W_eff / 2 - c,
                                          Z0_PANEL - 10), 0, grp))
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

            # ---------- колёса (ролики + ступица) и маркеры рук ----------
            if layout == "R":
                r_c = case["r_c"]
                wheels = [(0, (r_c, 0.0)), (90, (0.0, r_c)), (180, (-r_c, 0.0)), (270, (0.0, -r_c))]
                hands = {"0": "L", "90": "L", "180": "L", "270": "L"}   # аудит: все одной руки
                for th, (x_i, y_i) in wheels:
                    a = math.radians(th)
                    bx, by = (r_c - W / 2.0) * math.cos(a), (r_c - W / 2.0) * math.sin(a)
                    add_cyl_radial(f"{gname}_WHL{th}", D / 2.0, W, (bx, by, D / 2.0), th, grp)
                    hx, hy = (r_c - W_eff / 2.0) * math.cos(a), (r_c - W_eff / 2.0) * math.sin(a)
                    add_cyl_radial(f"{gname}_hub{th}", D * 0.225, W_eff,
                                   (hx, hy, D / 2.0), th, grp)
                    m = doc.addObject("Part::Box", f"{gname}_hand_A{th}_{hands[str(th)]}")
                    m.Length = m.Width = m.Height = 10.0
                    m.Placement.Base = App.Vector(x_i - 5, y_i - 5, D + 5)
                    grp.addObject(m)
            else:
                u = case["u"]
                wheels = [("FL_L", 1, 1), ("FR_R", 1, -1), ("RL_R", -1, 1), ("RR_L", -1, -1)]
                for tag, sx, sy in wheels:
                    x_i, y_i = sx * u, sy * u
                    add_cyl_y(f"{gname}_WHL_{tag}", D / 2.0, W,
                              (x_i, y_i - W / 2.0, D / 2.0), grp)
                    add_cyl_y(f"{gname}_hub_{tag}", D * 0.225, W_eff,
                              (x_i, y_i - W_eff / 2.0, D / 2.0), grp)
                    m = doc.addObject("Part::Box", f"{gname}_hand_{tag}")
                    m.Length = m.Width = m.Height = 10.0
                    m.Placement.Base = App.Vector(x_i - 5, y_i - 5, D + 5)
                    grp.addObject(m)

            # ---------- зоны: лифт, приводы, оборудование ----------
            add_cyl(f"{gname}_lift", P["lift_R"], 400, (0, 0, 0), grp)
            if case["band_mm"] > 1.0:
                if layout == "R":
                    for th in (0, 90, 180, 270):
                        add_box(f"{gname}_drv{th}", case["band_mm"], 140.0, 90.0,
                                (LIFT_CLEAR_R, -70.0, 45.0), th, grp)
                else:
                    u = case["u"]
                    for sx, sy in ((1, 1), (1, -1), (-1, 1), (-1, -1)):
                        y_face = u - W_eff / 2 - c                 # внутренняя грань колеса
                        base_y = (y_face - case["band_mm"]) if sy > 0 else -y_face
                        add_box(f"{gname}_drv{sx}_{sy}", 60.0, case["band_mm"], 90.0,
                                (sx * u - 30.0, base_y, 45.0), 0, grp)
            hb = case["battery_half_tan"]
            if hb >= 10:
                r_out = R_env - P["bat_Rout_margin"]
                sect = (45, 135, 225, 315) if layout == "R" else (0, 90, 180, 270)
                for th in sect:
                    add_box(f"{gname}_bat{th}", r_out - LIFT_CLEAR_R, 2 * hb, 100.0,
                            (LIFT_CLEAR_R, -hb, 50.0), th, grp)

            # ---------- аутригеры: блоки (транспорт) + рычаги/площадки (работа) ----------
            pod_r_out = R_env - P["pod_r_out_off"]
            pod_center = pod_r_out - P["pod_depth"] / 2.0
            pad_dirs = case["pad_dir_deg"]
            pad_r = pod_center + STROKE_VIS
            for th in pad_dirs:
                add_box(f"{gname}_pod{th}", P["pod_depth"], 2 * P["pod_half_tan"], 100.0,
                        (pod_r_out - P["pod_depth"], -P["pod_half_tan"], 40.0), th, grp)
                # телескопический рычаг (предполагаемый механизм выдвижения)
                add_box(f"{gname}_armO{th}", 170.0, 60.0, 60.0,
                        (pod_center, -30.0, 60.0), th, grp)
                add_box(f"{gname}_armI{th}", pad_r - (pod_center + 170.0), 40.0, 40.0,
                        (pod_center + 170.0, -20.0, 70.0), th, grp)
                add_box(f"{gname}_pad{th}", 150.0, 150.0, 40.0,
                        (pad_r - 75.0, -75.0, 0.0), th, grp)
                mk = doc.addObject("Part::Box", f"{gname}_pt{th}")
                mk.Length = mk.Width = mk.Height = 30.0
                mx, my = rotz(pad_r, 0, th)
                mk.Placement.Base = App.Vector(mx - 15, my - 15, -3)
                grp.addObject(mk)

            # ---------- силуэты опорных контуров ----------
            W_n = W
            if layout == "R":
                rr = case["r_c"] + W_n / 2.0            # вершины на осях = пятна колёс
                s = rr * math.sqrt(2.0)
                add_box(f"{gname}_supp", s, s, 3.0, (-s / 2, -s / 2, -6.0), 45, grp)
                s2 = pad_r * math.sqrt(2.0)             # вершины на 45° = площадки
                add_box(f"{gname}_suppExt", s2, s2, 3.0, (-s2 / 2, -s2 / 2, -12.0), 0, grp)
            else:
                u = case["u"]
                add_box(f"{gname}_supp", 2 * u, 2 * (u + W_n / 2.0), 3.0,
                        (-u, -(u + W_n / 2.0), -6.0), 0, grp)
                s2 = pad_r * math.sqrt(2.0)             # вершины на осях = площадки
                add_box(f"{gname}_suppExt", s2, s2, 3.0, (-s2 / 2, -s2 / 2, -12.0), 45, grp)

            add_cyl(f"{gname}_env", R_env, ENV_H, (0, 0, 0), grp)

doc.recompute()
for o in doc.Objects:
    n = o.Name
    if "_well" in n or "_panelC" in n or n.endswith("_panel"):
        o.Visibility = False

fcstd = os.path.join(SCRIPT_DIR, "stage2g_v5_compare.FCStd")
doc.saveAs(fcstd)
print(f"[v5] saved: {fcstd} objects={len(doc.Objects)}")

# ================= ПРОВЕРКИ (задачи 5 и 9) =================
print("== CHECKS: внешний габарит всей сборки (Ø = панель; реальные элементы <= R − зазор) ==")
fail = 0
for layout in ("R", "P"):
    for Dp in (600.0, 620.0, 640.0):
        for wkey in ("D152", "D203", "D230"):
            gname = f"{layout}{int(Dp)}_{wkey}"
            R_env = Dp / 2.0
            gap = P["gap_transport"]
            max_real, worst = 0.0, None
            max_panel = 0.0
            for o in doc.Objects:
                if not o.Name.startswith(gname + "_") or not hasattr(o, "Shape") or o.Shape.isNull():
                    continue
                n = o.Name
                if "_well" in n or "_panelC" in n or "supp" in n or "_pad" in n \
                        or "_pt" in n or "_arm" in n:
                    continue
                if "_env" in n:
                    continue
                is_panel = n.endswith("_body")
                for vt in o.Shape.Vertexes:
                    r = math.hypot(vt.Point.x, vt.Point.y)
                    if is_panel:
                        max_panel = max(max_panel, r)
                    else:
                        if r > max_real:
                            max_real, worst = r, o.Name
            ok = (max_panel <= R_env + 0.01) and (max_real <= R_env - gap + 0.01)
            st = "PASS" if ok else "FAIL"
            if not ok:
                fail += 1
            print(f"  {st} {gname}: сборка_max_r={max_panel:.1f} (=Ø{int(Dp)}), "
                  f"реальные элементы={max_real:.1f} ({worst}), "
                  f"зазор={R_env - max_real:.1f} мм (треб >= {gap:.0f})")
print("CHECK_STATUS:", "PASS 18/18" if fail == 0 else f"FAIL {fail}")
