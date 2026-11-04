# -*- coding: utf-8 -*-
"""Этап 2Г RVW-7PG, задача 1 — компоновка 4×RVW-7PG в платформе Ø620
(rvw7pg_r2_layout.py, FreeCAD, создаёт НОВЫЙ файл rvw7pg_r2_layout.FCStd).

DOC-габариты RVW-7PG (каталог Nabtesco 2020, чертежи стр.11-12):
  колесо Ø178 (a), ширина 96 (b), main unit 113±4 + выступ вала 11±4
  (сумма 124 = c), фланец мотора Ø140, центровка Ø115H7, масса 11.8 кг
  БЕЗ фланца мотора (NOTE на чертеже), инерция 8.76e-5 кг·м².
Переменные (НЕ паспортные): мотор (D_m, L_m), ход подвески (±15/±30 —
сценарии), z-диапазон подъёмника (полный габарит Ø240 задан заказчиком,
вертикальный ход — параметр), R_w (радиус оси колеса).
Проверки: коллизии OpenCascade (common volume), зазоры (distToShape),
габарит, крайние положения подвески. Вердикт по правилу заказчика:
FIT только при проверенном полном габарите ВКЛЮЧАЯ мотор/тормоз/крепления.
"""
import hashlib
import json
import math
import os

import FreeCAD as App
import Part

HERE = os.path.dirname(os.path.abspath(__file__))

# ---------------- DOC (каталог Nabtesco) ----------------
WHEEL_D, WHEEL_B = 178.0, 96.0        # a, b, мм
MAIN_L, VALVE_L = 113.0, 11.0         # 113±4 + 11±4 = 124 = c (DOC)
FLANGE_D, FLANGE_BOSS = 140.0, 11.0   # Ø140 фланец мотора (DOC)
PLATFORM_D, PANEL_Z0, PANEL_Z1 = 620.0, 50.0, 90.0   # Ø620 панель R2 (CAD)
LIFT_D = 240.0                        # DOC-требование заказчика: подъёмник Ø240

# ---------------- ПЕРЕМЕННЫЕ ----------------
RW = 262.0              # радиус оси колеса: наружный край = RW+48 = 310 = R платформы
D_MOTOR, L_MOTOR = 80.0, 100.0        # параметрический мотор (UNK — НЕ паспортный!)
LIFT_Z0, LIFT_Z1 = 90.0, 400.0        # z-диапазон подъёмника (ПАРАМЕТР)
SUSP = [-30.0, -15.0, 0.0, 15.0, 30.0]  # сценарии подвески (переменные)
GAP_PANEL = 15.0                      # зазор колесо-ниша панели (переменная)

CHECKS = []


def t(cid, ok, detail):
    CHECKS.append({"id": cid, "ok": bool(ok), "detail": detail})
    print(f"  [{'PASS' if ok else 'FAIL'}] {cid}: {detail}")


def sha(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()[:16]


doc = App.newDocument("rvw7pg_r2_layout")

# ================= геометрия =================
# панель-диск Ø620 с 4 радиальными нишами под ВЕСЬ модуль
# (колесо + корпус + вал + мотор) — новые ниши, не ниша RoboCT
Z_AX = WHEEL_D / 2
x_wheel0 = RW - WHEEL_B / 2            # 214: внутренний торец колеса
x_body0 = x_wheel0 - (MAIN_L - WHEEL_B)          # 197
x_valve0 = x_body0 - VALVE_L                      # 186
panel = Part.makeCylinder(PLATFORM_D / 2, PANEL_Z1 - PANEL_Z0,
                          App.Vector(0, 0, PANEL_Z0))
half_w = WHEEL_D / 2 + GAP_PANEL   # полупаз по РАДИУСУ колеса Ø178 (+зазор)
x_notch0 = x_valve0 - L_MOTOR - 10     # ниша под мотор с зазором 10 мм
for ang in (0, 90, 180, 270):
    box = Part.makeBox(RW + WHEEL_D / 2 + 10 - x_notch0, 2 * half_w,
                       PANEL_Z1 - PANEL_Z0 + 20,
                       App.Vector(x_notch0, -half_w, PANEL_Z0 - 10))
    panel = panel.cut(box.rotate(App.Vector(0, 0, 0), App.Vector(0, 0, 1), ang))
doc.addObject("Part::Feature", "platform_panel").Shape = panel

# подъёмник Ø240 (полный конструктивный габарит, z-диапазон — параметр)
lift = Part.makeCylinder(LIFT_D / 2, LIFT_Z1 - LIFT_Z0,
                         App.Vector(0, 0, LIFT_Z0))
doc.addObject("Part::Feature", "lift_zone").Shape = lift

# модуль RVW-7PG на +X: колесо (214..310) + корпус 17 (197..214) +
# выступ вала 11 (186..197) + параметрический мотор (внутрь от 186).
# Ось модуля (и центры цилиндров) — на z = WHEEL_D/2 = 89 мм (колесо на полу).
Z_AX = WHEEL_D / 2
x_wheel0 = RW - WHEEL_B / 2            # 214: внутренний торец колеса
x_body0 = x_wheel0 - (MAIN_L - WHEEL_B)          # 197
x_valve0 = x_body0 - VALVE_L                      # 186
wheel = Part.makeCylinder(WHEEL_D / 2, WHEEL_B,
                          App.Vector(x_wheel0, 0, Z_AX), App.Vector(1, 0, 0))
body = Part.makeCylinder(FLANGE_D / 2, MAIN_L - WHEEL_B,
                         App.Vector(x_body0, 0, Z_AX), App.Vector(1, 0, 0))
valve = Part.makeCylinder(20.0, VALVE_L,
                          App.Vector(x_valve0, 0, Z_AX), App.Vector(1, 0, 0))
motor = Part.makeCylinder(D_MOTOR / 2, L_MOTOR,
                          App.Vector(x_valve0 - L_MOTOR, 0, Z_AX),
                          App.Vector(1, 0, 0))
module0 = Part.makeCompound([wheel, body, valve, motor])
mod_obj = doc.addObject("Part::Feature", "module_D0")
mod_obj.Shape = module0

# колесо должно стоять на полу: цилиндр с осью по X в плоскости z=0..178 —
# ось на z=D/2, сам цилиндр уже на z 0..178 после dz? Ось цилиндра шла по
# z=0 -> после dz ось на 89, колесо 0..178. Проверка:
print("== Проверка позиции колеса ==")
bb = module0.BoundBox
t("T1_колесо_на_полу", abs(bb.ZMin) < 1.0,
  f"нижняя точка модуля z={bb.ZMin:.1f} мм (колесо на полу)")

# повороты 4 модулей + подъём подвески
def module_at(ang, dz_susp):
    c = module0.copy()
    c = c.rotate(App.Vector(0, 0, 0), App.Vector(0, 0, 1), ang)
    c = c.translate(App.Vector(0, 0, dz_susp))
    return c

modules = {ang: module0 if ang == 0 else module_at(ang, 0.0) for ang in (0, 90, 180, 270)}
for ang, sh in modules.items():
    o = doc.addObject("Part::Feature", f"module_D{ang}")
    o.Shape = sh

# ================= проверки =================
print("\n== 1.2а Коллизии OpenCascade (модули/панель/подъёмник, подвеска=0) ==")
bodies = {"панель": panel, "подъёмник": lift}
coll_rows = []
for ang, m in modules.items():
    for name, b in bodies.items():
        common = m.common(b)
        vol = common.Volume if common else 0.0
        dist = m.distToShape(b)[0]
        coll_rows.append({"модуль": f"D{ang}", "тело": name,
                          "общий_объём_мм3": round(vol, 1),
                          "зазор_мм": round(dist, 1)})
        print(f"  модуль D{ang} <-> {name}: общий объём {vol:.1f} мм3, "
              f"зазор {dist:.1f} мм")
for a1, a2 in ((0, 90), (90, 180), (180, 270), (270, 0)):
    m1, m2 = modules[a1], modules[a2]
    common = m1.common(m2)
    vol = common.Volume if common else 0.0
    dist = m1.distToShape(m2)[0]
    coll_rows.append({"модуль": f"D{a1}-D{a2}", "тело": "сосед",
                      "общий_объём_мм3": round(vol, 1), "зазор_мм": round(dist, 1)})
    print(f"  модули D{a1} <-> D{a2}: общий объём {vol:.1f} мм3, зазор {dist:.1f} мм")

# критический зазор: мотор ↔ подъёмник
m0 = modules[0]
d_motor_lift = motor.distToShape(lift)[0] / 1.0
t("T2_мотор_не_касается_подъёмника",
  not (motor.common(lift) and motor.common(lift).Volume > 1.0),
  f"мотор (параметр {D_MOTOR}×{L_MOTOR}) ↔ подъёмник Ø240: "
  f"зазор {m0.distToShape(lift)[0]:.1f} мм; критерий: L_m ≤ RW-196 = {RW-196:.0f} мм")

print("\n== 1.2б Крайние положения подвески (±15/±30 мм) ==")
susp_rows = []
for s in SUSP:
    mm = module_at(0, s)
    v_p = mm.common(panel)
    v_l = mm.common(lift)
    susp_rows.append({"подвеска_мм": s, "общий_объём_с_панелью_мм3":
                      round(v_p.Volume if v_p else 0.0, 1),
                      "общий_объём_с_подъёмником_мм3":
                      round(v_l.Volume if v_l else 0.0, 1),
                      "зазор_до_панели_мм": round(mm.distToShape(panel)[0], 1),
                      "зазор_до_подъёмника_мм": round(mm.distToShape(lift)[0], 1)})
    print(f"  подвеска {s:+.0f} мм: зазор панель {susp_rows[-1]['зазор_до_панели_мм']}, "
          f"подъёмник {susp_rows[-1]['зазор_до_подъёмника_мм']} мм")

# ================= габарит =================
all_shapes = [panel, lift] + list(modules.values())
xmin = min(s.BoundBox.XMin for s in all_shapes)
xmax = max(s.BoundBox.XMax for s in all_shapes)
ymin = min(s.BoundBox.YMin for s in all_shapes)
ymax = max(s.BoundBox.YMax for s in all_shapes)
zmax = max(s.BoundBox.ZMax for s in all_shapes)
R_max = max(math.hypot(x, y) for s in all_shapes
            for x in (s.BoundBox.XMin, s.BoundBox.XMax)
            for y in (s.BoundBox.YMin, s.BoundBox.YMax))
# точнее: углы боксов — для панели ровно 310
R_max = max(R_max, PLATFORM_D / 2)
t("T3_габарит_620", 2 * max(abs(xmin), abs(xmax), abs(ymin), abs(ymax))
  <= PLATFORM_D + 1.0 or R_max <= PLATFORM_D / 2 + 1.0,
  f"габарит сборки: X[{xmin:.0f},{xmax:.0f}] Y[{ymin:.0f},{ymax:.0f}] "
  f"Zmax={zmax:.0f} мм (полный диаметр с бокс-обёртками см. JSON; колёса Ø178 "
  f"на RW={RW}: наружный край {RW+WHEEL_D/2:.0f} ≤ {PLATFORM_D/2:.0f})")

# ================= вердикт =================
any_collision = any(r["общий_объём_мм3"] > 1.0 for r in coll_rows
                    if "панель" not in r["тело"] and "подъёмник" not in r["тело"])
lift_coll = any(r["общий_объём_мм3"] > 1.0 for r in coll_rows
                if r["тело"] == "подъёмник")
panel_coll = any(r["общий_объём_мм3"] > 1.0 for r in coll_rows
                 if r["тело"] == "панель")
if lift_coll or panel_coll or any_collision:
    verdict = "NO FIT (при текущих параметрах — см. коллизии)"
elif L_MOTOR <= RW - 196:
    verdict = ("CONDITIONAL: мотор умещается при L_m ≤ "
               f"{RW-196:.0f} мм; но полный габарит (мотор/тормоз/крепления/кабели) "
               "НЕ проверен -> FIT невозможен по правилу заказчика")
else:
    verdict = "NO FIT: мотор не умещается"
print(f"\nВЕРДИКТ: {verdict}")

doc.recompute()
out_path = os.path.join(HERE, "rvw7pg_r2_layout.FCStd")
if os.path.exists(out_path):
    os.remove(out_path)
doc.saveAs(out_path)
App.closeDocument(doc.Name)

out = {"doc_dims": {"wheel_d": WHEEL_D, "wheel_b": WHEEL_B, "main_l": MAIN_L,
                    "valve_l": VALVE_L, "flange_d": FLANGE_D,
                    "mass_note": "11.8 кг БЕЗ фланца мотора (NOTE чертежа)"},
       "variables": {"RW": RW, "D_motor": D_MOTOR, "L_motor": L_MOTOR,
                     "lift_z": [LIFT_Z0, LIFT_Z1], "susp": SUSP,
                     "статус": "мотор/подъёмник-z/подвеска — параметры, не DOC"},
       "collisions": coll_rows, "suspension": susp_rows,
       "verdict": verdict,
       "geom_checks": CHECKS,
       "unknowns": ["полную длину с мотором (каталог: c зависит от мотора)",
                    "габарит/наличие тормоза (в каталоге не упомянут)",
                    "крепёжные кронштейны и кабели",
                    "z-диапазон подъёмника Ø240 (задан только диаметр)",
                    "монтажные ориентации (чертежи flange — за логином)"],
       "fcstd_sha16": sha(out_path)}
path = os.path.join(HERE, "rvw7pg_r2_layout.json")
with open(path, "w", encoding="utf-8") as f:
    json.dump(out, f, ensure_ascii=False, indent=1)
print("FCStd:", out_path, "SHA16:", out["fcstd_sha16"])
print("JSON:", path)
print("REGRESSION:", "PASS" if all(c["ok"] for c in CHECKS) else "FAIL")