# -*- coding: utf-8 -*-
"""Этап 2Г RVW-7PG, задача 2 (корректировка) — компоновка v2
(rvw7pg_r2_layout_v2.py, FreeCAD, НОВЫЙ файл rvw7pg_r2_layout_v2.FCStd).

Исправления по замечаниям заказчика:
  1) Радиус установки колеса пересчитан: наружный торец Ø178×96 должен
     лежать ВНУТРИ окружности R=310: sqrt((RW+48)^2 + 89^2) <= 310
     -> RW <= sqrt(310^2-89^2) - 48 = 248.92 мм (принято RW=248.9).
  2) Выход за Ø620 проверяется РЕАЛЬНЫМ объёмом (shape.cut(цилиндр R310)),
     а не BoundBox.
  3) Связность панели: len(Solids)==1 — иначе FAIL.
  4) Доступная радиальная длина моторной зоны у подъёмника Ø240 пересчитана:
     L_m_max = RW - 76 - 120 = 52.9 мм без зазора.
  5) Исследуется вертикальное разнесение мотора и подъёмника (параметр
     z_lift0) как способ освободить моторную зону.
Старый rvw7pg_r2_layout.FCStd НЕ изменяется.
"""
import hashlib
import json
import math
import os

import FreeCAD as App
import Part

HERE = os.path.dirname(os.path.abspath(__file__))

# ---------------- DOC (каталог Nabtesco Rev.004.1 / Product Guide 2026) ----------------
WHEEL_D, WHEEL_B = 178.0, 96.0
MAIN_L, VALVE_L = 113.0, 11.0          # чертёж: 113±4 + 11±4 (Rev.004.1 c=124;
                                       # Product Guide 2026: c=118 — расхождение фиксируется)
FLANGE_D = 140.0
PLATFORM_R = 310.0                     # Ø620
PANEL_Z0, PANEL_Z1 = 50.0, 90.0
LIFT_D = 240.0

# ---------------- ПЕРЕМЕННЫЕ ----------------
RW = 248.9                             # радиус установки (пересчёт, см. шапку)
D_MOTOR, L_MOTOR = 76.0, 50.0          # параметрический мотор (UNK) — в пределах 52.9
LIFT_Z0, LIFT_Z1 = 90.0, 400.0         # z подъёмника (параметр)
GAP = 10.0                             # зазоры (переменные)
SUSP = [-30.0, -15.0, 0.0, 15.0, 30.0]

L_MOTOR_MAX = RW - 76.0 - LIFT_D / 2   # 52.9 мм — чистая геометрия
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


doc = App.newDocument("rvw7pg_r2_layout_v2")

# контроль пересчёта RW
rw_max = math.sqrt(PLATFORM_R ** 2 - (WHEEL_D / 2) ** 2) - WHEEL_B / 2
print(f"RW_max по критерию торца: sqrt(310^2-89^2)-48 = {rw_max:.2f} мм")
t("T1_RW_пересчёт", abs(RW - rw_max) < 0.2,
  f"RW={RW} мм <= {rw_max:.2f} мм (наружный торец целиком в R=310)")

# ---------------- панель с радиальными нишами под модуль ----------------
x_wheel0 = RW - WHEEL_B / 2            # внутренний торец колеса
x_body0 = x_wheel0 - (MAIN_L - WHEEL_B)
x_valve0 = x_body0 - VALVE_L
x_notch0 = x_valve0 - L_MOTOR - GAP    # внутренний край ниши

panel = Part.makeCylinder(PLATFORM_R, PANEL_Z1 - PANEL_Z0,
                          App.Vector(0, 0, PANEL_Z0))
half_w = WHEEL_D / 2 + GAP
x1 = PLATFORM_R + 20.0
boxes = [
    Part.makeBox(x1 - x_notch0, 2 * half_w, PANEL_Z1 - PANEL_Z0 + 20,
                 App.Vector(x_notch0, -half_w, PANEL_Z0 - 10)),
    Part.makeBox(2 * half_w, x1 - x_notch0, PANEL_Z1 - PANEL_Z0 + 20,
                 App.Vector(-half_w, x_notch0, PANEL_Z0 - 10)),
    Part.makeBox(x1 - x_notch0, 2 * half_w, PANEL_Z1 - PANEL_Z0 + 20,
                 App.Vector(-x1, -half_w, PANEL_Z0 - 10)),
    Part.makeBox(2 * half_w, x1 - x_notch0, PANEL_Z1 - PANEL_Z0 + 20,
                 App.Vector(-half_w, -x1, PANEL_Z0 - 10)),
]
for b in boxes:
    panel = panel.cut(b)
panel_obj = doc.addObject("Part::Feature", "platform_panel")
panel_obj.Shape = panel

n_solids = len(panel.Solids)
print(f"панель: Solids={n_solids}, объём={panel.Volume/1e6:.2f} млн мм3")
t("T2_панель_связна", n_solids == 1,
  f"панель после ниш: {n_solids} отдельных тел (нужно 1) — "
  + ("несвязная конструкция = FAIL" if n_solids > 1 else "связна"))

# подъёмник
lift = Part.makeCylinder(LIFT_D / 2, LIFT_Z1 - LIFT_Z0,
                         App.Vector(0, 0, LIFT_Z0))
doc.addObject("Part::Feature", "lift_zone").Shape = lift

# ---------------- модуль ----------------
Z_AX = WHEEL_D / 2
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


def module_at(ang, dz=0.0):
    c = module0.copy()
    if ang:
        c = c.rotate(App.Vector(0, 0, 0), App.Vector(0, 0, 1), ang)
    if dz:
        c = c.translate(App.Vector(0, 0, dz))
    return c


modules = {a: module_at(a) for a in (0, 90, 180, 270)}
for a, m in modules.items():
    o = doc.addObject("Part::Feature", f"module_D{a}")
    o.Shape = m

# ---------------- проверки ----------------
cyl_outer = Part.makeCylinder(PLATFORM_R, 600.0, App.Vector(0, 0, -50.0))

print("\n== Реальный выход за Ø620 (объём вне цилиндра R310) ==")
out_rows = []
for name, sh in [("панель", panel), ("подъёмник", lift)] + \
                [(f"модуль D{a}", m) for a, m in modules.items()]:
    outside = sh.cut(cyl_outer)
    v_out = outside.Volume if outside else 0.0
    out_rows.append({"тело": name, "объём_вне_Ø620_мм3": round(v_out, 1)})
    print(f"  {name}: вне Ø620 = {v_out:.1f} мм3")
t("T3_габарит_реальный", all(r["объём_вне_Ø620_мм3"] <= 1.0 for r in out_rows),
  "весь объём сборки внутри цилиндра Ø620 (критерий по объёму, не BoundBox)")

print("\n== Коллизии OpenCascade (подвеска=0) ==")
coll_rows = []
for a, m in modules.items():
    for nm, b in (("панель", panel), ("подъёмник", lift)):
        c = m.common(b)
        v = c.Volume if c else 0.0
        d = m.distToShape(b)[0]
        coll_rows.append({"модуль": f"D{a}", "тело": nm,
                          "общий_объём_мм3": round(v, 1), "зазор_мм": round(d, 1)})
        print(f"  D{a} <-> {nm}: объём {v:.0f} мм3, зазор {d:.1f} мм")
for a1, a2 in ((0, 90), (90, 180), (180, 270), (270, 0)):
    c = modules[a1].common(modules[a2])
    v = c.Volume if c else 0.0
    d = modules[a1].distToShape(modules[a2])[0]
    coll_rows.append({"модуль": f"D{a1}-D{a2}", "тело": "сосед",
                      "общий_объём_мм3": round(v, 1), "зазор_мм": round(d, 1)})
    print(f"  D{a1} <-> D{a2}: объём {v:.0f} мм3, зазор {d:.1f} мм")

lift_hit = any(r["тело"] == "подъёмник" and r["общий_объём_мм3"] > 1.0
               for r in coll_rows)
t("T4_мотор_без_подъёмника", not lift_hit,
  f"мотор ({D_MOTOR}×{L_MOTOR}) vs подъёмник Ø240: "
  + ("пересечение есть" if lift_hit else "нет пересечения") +
  f"; предел L_m ≤ {L_MOTOR_MAX:.1f} мм (без зазора)")

print("\n== Подвеска ±15/±30 мм ==")
susp_rows = []
for s in SUSP:
    m = module_at(0, s)
    vp = m.common(panel).Volume if m.common(panel) else 0.0
    vl = m.common(lift).Volume if m.common(lift) else 0.0
    susp_rows.append({"подвеска_мм": s, "с_панелью_мм3": round(vp, 1),
                      "с_подъёмником_мм3": round(vl, 1),
                      "зазор_панель_мм": round(m.distToShape(panel)[0], 1),
                      "зазор_подъёмник_мм": round(m.distToShape(lift)[0], 1)})
    print(f"  {s:+.0f} мм: панель зазор {susp_rows[-1]['зазор_панель_мм']}, "
          f"подъёмник {susp_rows[-1]['зазор_подъёмник_мм']} мм, "
          f"объёмы {vp:.0f}/{vl:.0f}")

# ---------------- вертикальное разнесение ----------------
# Если подъёмник начинается ВЫШЕ моторной зоны, XY-пересечение устраняется:
z_motor_top = Z_AX + D_MOTOR / 2
z_lift0_min = z_motor_top + GAP
print(f"\n== Вертикальное разнесение ==")
print(f"  верх моторной зоны: {z_motor_top:.0f} мм -> подъёмник должен начинаться "
      f"выше {z_lift0_min:.0f} мм (параметр конструкции)")
sep_ok = LIFT_Z0 >= z_lift0_min
t("T5_разнесение", sep_ok,
  f"при z_lift0={LIFT_Z0:.0f} мм: " + ("подъёмник отделён от мотора по Z"
   if sep_ok else f"моторная зона перекрыта — нужен z_lift0 >= {z_lift0_min:.0f} мм "
                  f"ИЛИ L_m <= {L_MOTOR_MAX:.1f} мм на одном уровне"))

# ---------------- вердикт ----------------
if n_solids > 1:
    verdict = "FAIL: панель разрезана на несвязанные части"
elif any(r["объём_вне_Ø620_мм3"] > 1.0 for r in out_rows):
    verdict = "NO FIT: часть сборки вне Ø620"
elif lift_hit and not sep_ok:
    verdict = ("NO FIT при параметрах: мотор пересекает подъёмник "
               f"(нужно L_m≤{L_MOTOR_MAX:.1f} мм при z_lift0={LIFT_Z0:.0f} "
               f"или вертикальное разнесение z_lift0≥{z_lift0_min:.0f} мм)")
elif lift_hit and sep_ok:
    verdict = ("CONDITIONAL: мотор отделён от подъёмника по Z "
               "(требует согласования хода подъёмника)")
else:
    verdict = ("CONDITIONAL: коллизий нет при параметрах "
               f"(мотор {D_MOTOR}×{L_MOTOR}); полный габарит с тормозом/"
               "креплениями/кабелями не проверен — FIT невозможен")
print(f"\nВЕРДИКТ: {verdict}")

doc.recompute()
out_path = os.path.join(HERE, "rvw7pg_r2_layout_v2.FCStd")
if os.path.exists(out_path):
    os.remove(out_path)
doc.saveAs(out_path)
App.closeDocument(doc.Name)

out = {"пересчёт_RW": {"формула": "sqrt(310^2-89^2)-48", "RW_max": round(rw_max, 2),
                       "RW_принят": RW},
       "моторная_зона": {"x_valve0": x_valve0, "L_m": L_MOTOR,
                          "L_m_max_без_зазора": round(L_MOTOR_MAX, 1),
                          "зазор_до_подъёмника_мм": round(x_valve0 - L_MOTOR
                                                          - LIFT_D / 2, 1)},
       "панель": {"Solids": n_solids, "объём_мм3": round(panel.Volume, 0)},
       "габарит_вне_Ø620": out_rows, "коллизии": coll_rows,
       "подвеска": susp_rows,
       "вертикальное_разнесение": {"z_motor_top": z_motor_top,
                                    "z_lift0_min": z_lift0_min,
                                    "z_lift0_текущий": LIFT_Z0,
                                    "статус": "разрешает конфликт"
                                    if sep_ok else "не применено"},
       "вердикт": verdict, "checks": CHECKS,
       "variables": {"D_m": D_MOTOR, "L_m": L_MOTOR, "z_lift": [LIFT_Z0, LIFT_Z1],
                     "статус": "мотор/подъёмник-z — параметры, не DOC"},
       "fcstd_sha16": sha(out_path)}
path = os.path.join(HERE, "rvw7pg_r2_layout_v2.json")
with open(path, "w", encoding="utf-8") as f:
    json.dump(out, f, ensure_ascii=False, indent=1)
print("FCStd:", out_path, "SHA16:", out["fcstd_sha16"])
print("JSON:", path)
print("REGRESSION:", "PASS" if all(c["ok"] for c in CHECKS) else "FAIL")