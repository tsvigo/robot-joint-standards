# -*- coding: utf-8 -*-
"""
Этап 2Г — build_stage2g_base.py
Параметрическая модель мобильной базы робота-санитара (первая версия).

Запуск (после установки FreeCAD):
    freecadcmd build_stage2g_base.py        # headless
    либо в GUI: Macro > Execute macro

Координаты: база-локальные, мм. Начало = центр base_link,
+X вперёд (к кровати), +Y влево, +Z вверх.

Принципы (ограничения этапа 2Г):
  * Подтверждённые размеры (Blender) отделены от параметрических (provisional)
    в таблице params (столбец status). Параметрические значения — рабочие
    гипотезы компоновки, НЕ утверждённые размеры.
  * Массы не назначаются: физические детали помечены bom_status=
    'part_mass_pending', компоновочные объёмы — 'envelope_no_mass'.
  * Габарит 670x670x176 — ограничение (GUIDE_envelope), не деталь.
  * Сплошной блок 213 кг не создаётся и не считается деталью.
  * Геометрия строится примитивами Part (редактируемое дерево), НЕ mesh-импорт.
"""

import os
import re
import FreeCAD as App
import Part

# ---------------------------------------------------------------- исходные
# Подтверждено (model_key_facts.json / STAGE2G_SOURCE_DATA.md), мм:
CONF = dict(
    env_W=670.0, env_H=670.0, env_T=176.0,
    floor_z=-134.55,
    wheel_d=150.8, wheel_w=106.8,
    wbl_x=-258.6, wbl_y=258.2,
    wbr_x=-259.6, wbr_y=-257.7,
    wfl_x=257.8,  wfl_y=256.9,
    wfr_x=258.6,  wfr_y=-257.3,
    lift_x=0.003, lift_y=-4.41,
)
# Параметрические рабочие гипотезы (НЕ утверждены; диапазоны из stage2g_base_params.json):
PROV = dict(
    w_frame=40.0,      # ширина профиля рамы [30..60]
    clr_wheel=20.0,    # зазор ниши: нужен R>=sqrt((d/2)^2+(w/2)^2)=92.4 для диска при любом повороте steering [>=17]
    R_lift=120.0,      # радиус зоны колонны лифта (узлы неизвестны -> параметр)
    plate_D=300.0,     # плита посадки лифта
    plate_t=8.0,
    cross_w=40.0,      # поперечины под плиту
    cross_h=40.0,
    t_panel=3.0,       # толщина верхней панели [2..6]
    wlm_s=60.0,        # кронштейн колеса (площадка над осью)
    wlm_t=25.0,
    zone_bat_L=220.0, zone_bat_W=90.0, zone_bat_H=100.0, zone_bat_x=-185.0, zone_bat_y=135.0,
    zone_ele_L=220.0, zone_ele_W=90.0, zone_ele_H=100.0, zone_ele_x=185.0,  zone_ele_y=135.0,
    zone_pwr_L=205.0, zone_pwr_W=90.0, zone_pwr_H=100.0, zone_pwr_x=-192.5, zone_pwr_y=-135.0,
    zone_cbl_L=180.0, zone_cbl_W=40.0, zone_cbl_H=40.0,  zone_cbl_x=185.0,  zone_cbl_y=-160.0,
)

# производные (те же формулы, что в ячейках таблицы params)
env_bottom = -CONF["env_T"] / 2.0          # -88
env_top = CONF["env_T"] / 2.0              # +88
wheel_z = CONF["floor_z"] + CONF["wheel_d"] / 2.0   # -59.15: ОСЬ диска на радиусе над полом (v2)
wheel_top = wheel_z + CONF["wheel_w"] / 2.0         # -27.75
well_r = CONF["wheel_d"] / 2.0 + PROV["clr_wheel"]  # 85.4
cross_top = env_top - PROV["t_panel"] - PROV["plate_t"]   # 77 (верх поперечины; плита 77..85, панель 85..88)
cross_x_off = PROV["R_lift"] + PROV["cross_w"] / 2.0       # 140 (поперечина касается зоны колонны)
panel_size = CONF["env_W"] - 4.0 * PROV["w_frame"]         # 590 (проём между профилями)
zone_z = env_bottom  # дно зон на внутреннем дне базы

# ---------------------------------------------------------------- документ
doc = App.newDocument("stage2g_base_v2")

# --- таблица конструктивных параметров (Spreadsheet) --------------------
ps = doc.addObject("Spreadsheet::Sheet", "params")
ps.set("A1", "parameter")
ps.set("B1", "value")
ps.set("C1", "status")

plain = []    # (alias, число, статус)
formulas = [] # (alias, формула, статус)

for k in ("env_W", "env_H", "env_T"):
    plain.append((k, CONF[k], "confirmed (Blender gabarit limit)"))
plain.append(("floor_z", CONF["floor_z"], "confirmed (Blender)"))
plain.append(("wheel_d", CONF["wheel_d"], "confirmed (Blender)"))
plain.append(("wheel_roll_axis", "Y", "parametric: ось качения диска (перпендикулярна подъезду +X); v2"))
plain.append(("wheel_w", CONF["wheel_w"], "confirmed (Blender)"))
for k in ("wbl_x", "wbl_y", "wbr_x", "wbr_y", "wfl_x", "wfl_y", "wfr_x", "wfr_y"):
    plain.append((k, CONF[k], "confirmed (Blender wheel center)"))
plain.append(("lift_x", CONF["lift_x"], "confirmed (Blender lift axis)"))
plain.append(("lift_y", CONF["lift_y"], "confirmed (Blender lift axis)"))

plain.append(("w_frame", PROV["w_frame"], "parametric (provisional, range 30..60)"))
plain.append(("clr_wheel", PROV["clr_wheel"], "parametric (provisional, range 5..15)"))
plain.append(("R_lift", PROV["R_lift"], "parametric (real lift nodes unknown)"))
plain.append(("plate_D", PROV["plate_D"], "parametric (provisional)"))
plain.append(("plate_t", PROV["plate_t"], "parametric (provisional)"))
plain.append(("cross_w", PROV["cross_w"], "parametric (provisional)"))
plain.append(("cross_h", PROV["cross_h"], "parametric (provisional)"))
plain.append(("t_panel", PROV["t_panel"], "parametric (provisional, range 2..6)"))
plain.append(("wlm_s", PROV["wlm_s"], "parametric (provisional bracket)"))
plain.append(("wlm_t", PROV["wlm_t"], "parametric (provisional bracket)"))
for z in ("bat", "ele", "pwr", "cbl"):
    plain.append((f"zone_{z}_L", PROV[f"zone_{z}_L"], "parametric zone (envelope, no mass)"))
    plain.append((f"zone_{z}_W", PROV[f"zone_{z}_W"], "parametric zone (envelope, no mass)"))
    plain.append((f"zone_{z}_H", PROV[f"zone_{z}_H"], "parametric zone (envelope, no mass)"))
    plain.append((f"zone_{z}_x", PROV[f"zone_{z}_x"], "parametric zone placement (provisional)"))
    plain.append((f"zone_{z}_y", PROV[f"zone_{z}_y"], "parametric zone placement (provisional)"))
plain.append(("zone_z", env_bottom, "derived (дно зон = внутреннее дно базы)"))

# формулы — вторым проходом, когда все алиасы-числа объявлены
formulas.append(("base_bottom", "=-env_T/2", "confirmed (derived)"))
formulas.append(("base_top", "=env_T/2", "confirmed (derived)"))
formulas.append(("wheel_z", "=floor_z+wheel_d/2", "confirmed (derived: ось диска = радиус над полом; v2)"))
formulas.append(("wheel_top", "=floor_z+wheel_d", "confirmed (derived: верх диска; v2)"))
formulas.append(("well_r", "=wheel_d/2+clr_wheel", "derived"))
formulas.append(("lift_hole_r", "=R_lift", "derived (проём = зона колонны)"))
formulas.append(("cross_top", "=base_top-t_panel-plate_t", "derived (плита под панелью)"))
formulas.append(("cross_x_off", "=R_lift+cross_w/2", "derived (поперечина касается зоны колонны)"))
formulas.append(("panel_size", "=env_W-2*w_frame", "derived (проём между профилями = 670-2*40 = 590)"))

_row = 2
for alias, value, status in plain + formulas:
    ps.set(f"A{_row}", alias)
    ps.set(f"B{_row}", str(value))
    ps.set(f"C{_row}", status)
    ps.setAlias(f"B{_row}", alias)
    _row += 1

# --- группы: физические детали / компоновочные объёмы / инструменты ----
g_parts = doc.addObject("App::Part", "PARTS")
g_parts.Label = "PARTS (physical, mass pending)"
g_comps = doc.addObject("App::Part", "COMPONENTS")
g_comps.Label = "COMPONENTS (envelopes, no mass)"
g_guide = doc.addObject("App::Part", "GUIDES")
g_guide.Label = "GUIDES (tools + envelope, not exported)"

_IDENT = re.compile(r"(?<![\w.])([A-Za-z_][A-Za-z0-9_]*)")  # захват группы для \\1 в sub


def expr(obj, prop, e):
    """Привязка свойства к ячейкам таблицы: каждый идентификатор -> params.X."""
    full = e if e.startswith("params.") else _IDENT.sub(r"params.\1", e)
    obj.setExpression(prop, full)


def place(obj, x, y, z):
    expr(obj, "Placement.Base.x", x)
    expr(obj, "Placement.Base.y", y)
    expr(obj, "Placement.Base.z", z)


def mark(obj, bom):
    obj.addProperty("App::PropertyString", "bom_status", "Stage2G", "mass accounting status")
    obj.bom_status = bom


def hide(obj):
    # в консоли FreeCADCmd ViewObject отсутствует — визуализация не влияет на геометрию
    try:
        if obj.ViewObject:
            obj.ViewObject.Visibility = False
    except Exception:
        pass


def make_box(name, size, pos, bom="part_mass_pending"):
    b = doc.addObject("Part::Box", name)
    expr(b, "Length", size[0])
    expr(b, "Width", size[1])
    expr(b, "Height", size[2])
    place(b, pos[0], pos[1], pos[2])
    mark(b, bom)
    return b


def make_cyl(name, r, h, pos, bom="part_mass_pending"):
    c = doc.addObject("Part::Cylinder", name)
    expr(c, "Radius", r)
    expr(c, "Height", h)
    place(c, pos[0], pos[1], pos[2])
    mark(c, bom)
    return c


def cut(name, base, tool):
    c = doc.addObject("Part::Cut", name)
    c.Base = base
    c.Tool = tool
    mark(c, base.bom_status)
    return c


WHEEL_POS = (("FL", "wfl_x", "wfl_y"), ("FR", "wfr_x", "wfr_y"),
             ("BL", "wbl_x", "wbl_y"), ("BR", "wbr_x", "wbr_y"))

# =========================================================== 1. РАМА (FRM)
# ниши колёс (инструменты вычитания — в GUIDES)
wells = {}
for key, ax, ay in WHEEL_POS:
    w = make_cyl(f"TOOL_well_{key}", "well_r", "env_T+40",
                 (ax, ay, "base_bottom-20"), bom="tool_not_a_part")
    wells[key] = w
    g_guide.addObject(w)
    hide(w)

# периметровые профили (промежуточные боксы — скрыты, в STEP не идут)
fr_f = make_box("FRM_front_box", ("env_W", "w_frame", "env_T"),
                ("-env_W/2", "env_H/2-w_frame", "base_bottom"))
fr_b = make_box("FRM_back_box", ("env_W", "w_frame", "env_T"),
                ("-env_W/2", "-env_H/2", "base_bottom"))
fr_l = make_box("FRM_left_box", ("w_frame", "env_H-2*w_frame", "env_T"),
                ("-env_W/2", "w_frame-env_H/2", "base_bottom"))
fr_r = make_box("FRM_right_box", ("w_frame", "env_H-2*w_frame", "env_T"),
                ("env_W/2-w_frame", "w_frame-env_H/2", "base_bottom"))
for o in (fr_f, fr_b, fr_l, fr_r):
    hide(o)

# вычет ниш: перед(FL,BL), зад(FR,BR), лево(BL,BR), право(FL,FR)
f1 = cut("FRM_front_c1", fr_f, wells["FL"])
f2 = cut("FRM_front", f1, wells["BL"])
b1 = cut("FRM_back_c1", fr_b, wells["FR"])
b2 = cut("FRM_back", b1, wells["BR"])
l1 = cut("FRM_left_c1", fr_l, wells["BL"])
l2 = cut("FRM_left", l1, wells["BR"])
r1 = cut("FRM_right_c1", fr_r, wells["FL"])
r2 = cut("FRM_right", r1, wells["FR"])
for o in (f1, b1, l1, r1):
    hide(o)
for o in (f2, b2, l2, r2):
    g_parts.addObject(o)

# поперечины под плиту лифта (торцами на переднем/заднем профиле, зона колонны не пересекается)
cr1 = make_box("FRM_cross_1", ("cross_w", "env_H-2*w_frame", "cross_h"),
               ("lift_x+cross_x_off-cross_w/2", "w_frame-env_H/2", "cross_top-cross_h"))
cr2 = make_box("FRM_cross_2", ("cross_w", "env_H-2*w_frame", "cross_h"),
               ("lift_x-cross_x_off-cross_w/2", "w_frame-env_H/2", "cross_top-cross_h"))
g_parts.addObject(cr1)
g_parts.addObject(cr2)

# плита посадки лифта: цилиндр - проём колонны
pl_c = make_cyl("FRM_plate_body", "plate_D/2", "plate_t", ("lift_x", "lift_y", "cross_top"))
pl_h = make_cyl("FRM_plate_hole", "lift_hole_r", "plate_t+20",
                ("lift_x", "lift_y", "cross_top-10"), bom="tool_not_a_part")
hide(pl_c)
g_guide.addObject(pl_h)
hide(pl_h)
pl = cut("FRM_lift_plate", pl_c, pl_h)
g_parts.addObject(pl)

# верхняя панель доступа: плита - проём колонны лифта
pn_c = make_box("PNL_top_body", ("panel_size", "panel_size", "t_panel"),
                ("-panel_size/2", "-panel_size/2", "base_top-t_panel"))
pn_h = make_cyl("PNL_top_hole", "lift_hole_r", "t_panel+20",
                ("lift_x", "lift_y", "base_top-t_panel-10"), bom="tool_not_a_part")
hide(pn_c)
g_guide.addObject(pn_h)
hide(pn_h)
pn = cut("PNL_top", pn_c, pn_h)
g_parts.addObject(pn)

# кронштейны колёс (площадки над осью, нижняя грань = верх колеса)
for key, ax, ay in WHEEL_POS:
    wlm = make_box(f"WLM_{key}", ("wlm_s", "wlm_s", "wlm_t"),
                   (f"{ax}-wlm_s/2", f"{ay}-wlm_s/2", "wheel_top"))
    g_parts.addObject(wlm)

# ================================================= 2. КОМПОНЕНТЫ (envelopes)
# колёса: подтверждённые габариты, вертикальная ось, дно на уровне пола
for key, ax, ay in WHEEL_POS:
    # v2: диск с горизонтальной осью качения (∥Y): Rx(-90) направляет лок. ось цилиндра в +Y;
    # привязка (x_c, y_c - w/2, z_ax): низ диска = пол, центр = подтверждённый центр wheel joint
    wh = make_cyl(f"WHL_{key}", "wheel_d/2", "wheel_w",
                  (ax, f"{ay}-wheel_w/2", "wheel_z"),
                  bom="envelope_no_mass (dims confirmed; roll axis //Y v2; mass from vendor spec)")
    wh.Placement.Rotation = App.Rotation(App.Vector(1, 0, 0), -90.0)
    g_comps.addObject(wh)

zl = make_cyl("ZN_lift_column", "R_lift", "env_T", ("lift_x", "lift_y", "base_bottom"),
              bom="envelope_no_mass (component zone)")
g_comps.addObject(zl)
for z in ("bat", "ele", "pwr", "cbl"):
    zb = make_box(f"ZN_{z}",
                  (f"zone_{z}_L", f"zone_{z}_W", f"zone_{z}_H"),
                  (f"zone_{z}_x-zone_{z}_L/2", f"zone_{z}_y-zone_{z}_W/2", "zone_z"),
                  bom="envelope_no_mass (component zone)")
    g_comps.addObject(zb)

# ======================================================== 3. GUIDES
env = make_box("GUIDE_envelope", ("env_W", "env_H", "env_T"),
               ("-env_W/2", "-env_H/2", "base_bottom"), bom="guide_not_a_part")
g_guide.addObject(env)
hide(env)

# ----------------------------------------------------------- recompute/save
doc.recompute()

# App::Part в 1.1.4 auto-тянет зависимости (промежуточные боксы/цилиндры/инструменты),
# поэтому экспорт и контроль габарита — только по финальным объектам (фильтр по именам)
_INTER = re.compile(r"(_box|_body|_hole|_c1)$")


def _finals(group):
    return [o for o in group
            if hasattr(o, "Shape") and not o.Shape.isNull()
            and not o.Name.startswith(("TOOL_", "GUIDE_"))
            and not _INTER.search(o.Name)]


part_objs = _finals(g_parts.Group)
comp_objs = _finals(g_comps.Group)

# контрольный габарит физических деталей (должен укладываться в 670x670x176)
shapes = [o.Shape for o in part_objs]
if shapes:
    bb = shapes[0].BoundBox
    for s in shapes[1:]:
        bb.add(s.BoundBox)
    print(f"[stage2G] PARTS bbox: X {bb.XMin:.2f}..{bb.XMax:.2f}  "
          f"Y {bb.YMin:.2f}..{bb.YMax:.2f}  Z {bb.ZMin:.2f}..{bb.ZMax:.2f}")
    print(f"[stage2G] envelope limit: X/Y +-{CONF['env_W']/2:.2f}, "
          f"Z {env_bottom:.2f}..{env_top:.2f}")

try:
    script_dir = os.path.dirname(os.path.abspath(__file__))
except NameError:
    script_dir = os.getcwd()

fcstd = os.path.join(script_dir, "stage2g_base_v2.FCStd")
doc.saveAs(fcstd)
print(f"[stage2G v2] saved: {fcstd}")

# нейтральный экспорт: детали отдельно; раскладка = детали + компоновочные объёмы
# (part_objs/comp_objs уже отфильтрованы выше)
step_parts = os.path.join(script_dir, "stage2g_base_v2_parts.step")
step_layout = os.path.join(script_dir, "stage2g_base_v2_layout.step")
Part.export(part_objs, step_parts)
Part.export(part_objs + comp_objs, step_layout)
print(f"[stage2G] STEP: {step_parts}")
print(f"[stage2G] STEP: {step_layout}")
print(f"[stage2G] counts: parts={len(part_objs)} components={len(comp_objs)} "
      f"guides={len(g_guide.Group)} params={len(plain) + len(formulas)}")
print("[stage2G] done — masses NOT computed (no approved material/section).")
