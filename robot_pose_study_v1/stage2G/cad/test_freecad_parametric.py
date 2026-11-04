# -*- coding: utf-8 -*-
"""
Этап 2Г — test_freecad_parametric.py
Контрольный тест среды FreeCAD перед сборкой основания (п.8 плана установки):
  1) Spreadsheet с алиасами + выражения на свойствах (Part::Box, Part::Cylinder)
  2) выражение на Placement.Base.z
  3) булев вычет Part::Cut
  4) группа App::Part + пользовательское свойство (bom_status)
  5) пересчёт: изменение ячейки -> изменение геометрии
  6) сохранение FCStd -> закрытие -> повторное открытие -> проверка
  7) экспорт STEP, проверка файла
Вывод завершается маркером TEST_RESULT: PASS или TEST_RESULT: FAIL.
"""

import os
import sys
import FreeCAD as App
import Part

try:
    script_dir = os.path.dirname(os.path.abspath(__file__))
except NameError:
    script_dir = os.getcwd()

RESULTS = []
FAILED = []


LOGF = open(os.path.join(os.path.dirname(os.path.abspath(__file__)) if "__file__" in globals() else ".", "zz_selftest_results.txt"), "a", buffering=1)


def check(name, ok, detail=""):
    RESULTS.append((name, bool(ok), detail))
    if not ok:
        FAILED.append(name)
    line = (f"{'PASS' if ok else 'FAIL'} | {name}" + (f" | {detail}" if detail else ""))
    print(line)
    LOGF.write(line + "\n")
    LOGF.flush()


# --- 1. документ, таблица параметров, выражения -------------------------
doc = App.newDocument("fc_selftest")
ps = doc.addObject("Spreadsheet::Sheet", "params")
ps.set("A1", "p"); ps.set("B1", "v")
ps.set("A2", "L"); ps.set("B2", "50"); ps.setAlias("B2", "pL")
ps.set("A3", "W"); ps.set("B3", "30"); ps.setAlias("B3", "pW")
ps.set("A4", "Z"); ps.set("B4", "=pL/2"); ps.setAlias("B4", "pZ")   # формула-ячейка

box = doc.addObject("Part::Box", "T_box")
box.setExpression("Length", "params.pL")
box.setExpression("Width", "params.pW")
box.setExpression("Height", "params.pL")
box.setExpression("Placement.Base.z", "params.pZ")   # выражение на позиции
box.addProperty("App::PropertyString", "bom_status", "t", "test")
box.bom_status = "part_mass_pending"

cyl = doc.addObject("Part::Cylinder", "T_cyl")
cyl.setExpression("Radius", "params.pW/2")
cyl.setExpression("Height", "params.pL+10")

tool = doc.addObject("Part::Cylinder", "T_tool")
tool.Radius = 5.0
tool.Height = 200.0
tool.Placement = App.Placement(App.Vector(0, 0, -50), App.Rotation())

cutf = doc.addObject("Part::Cut", "T_cut")
cutf.Base = box
cutf.Tool = tool
cutf.addProperty("App::PropertyString", "bom_status", "t", "test")
cutf.bom_status = box.bom_status

grp = doc.addObject("App::Part", "T_GRP")
grp.addObject(cutf)
grp.addObject(cyl)

doc.recompute()

# --- 2. проверка геометрии после первого пересчёта ----------------------
bb = cutf.Shape.BoundBox
check("box Length = L(50)", abs(bb.XLength - 50) < 1e-6, f"{bb.XLength}")
check("box по Z от Z=L/2=25", abs(bb.ZMin - 25) < 1e-6, f"ZMin={bb.ZMin}")
check("cylinder R=W/2=15", abs(cyl.Shape.BoundBox.XLength - 30) < 1e-6,
      f"{cyl.Shape.BoundBox.XLength}")
_v_exp = 50 * 30 * 50 - 3.14159265 * 25 * 50 / 4.0  # ось цилиндра в углу бокса -> четверть выреза
check("boolean cut: четверть выреза цилиндра", abs(cutf.Shape.Volume - _v_exp) < 5.0,
      f"V={cutf.Shape.Volume:.1f}, ожидалось {_v_exp:.1f}")
check("App::Part содержит cutf и cyl (auto-тянет зависимости: 4)",
      cutf in grp.Group and cyl in grp.Group, f"{len(grp.Group)} объектов")
check("пользовательское свойство bom_status", box.bom_status == "part_mass_pending")

# --- 3. пересчёт параметра: меняем ячейку -> геометрия должна вырасти ---
ps.set("B2", "80")
doc.recompute()
bb2 = cutf.Shape.BoundBox
check("пересчёт: L=50 -> 80 меняет геометрию", abs(bb2.XLength - 80) < 1e-6,
      f"XLength={bb2.XLength}")
check("пересчёт: Z=L/2=40 (формула-ячейка)", abs(bb2.ZMin - 40) < 1e-6, f"ZMin={bb2.ZMin}")

# --- 4. сохранение / закрытие / повторное открытие ---------------------
fcstd = os.path.join(script_dir, "zz_selftest.FCStd")
doc.saveAs(fcstd)
docname = doc.Name
App.closeDocument(docname)
check("FCStd сохранён", os.path.isfile(fcstd) and os.path.getsize(fcstd) > 0,
      f"{os.path.getsize(fcstd) if os.path.isfile(fcstd) else 0} байт")

doc2 = App.openDocument(fcstd)
box2 = doc2.getObject("T_box")
grp2 = doc2.getObject("T_GRP")
ps2 = doc2.getObject("params")
ok_reopen = box2 is not None and grp2 is not None and ps2 is not None
check("повторное открытие: объекты на месте", ok_reopen)
if ok_reopen:
    doc2.recompute()
    check("после открытия: выражения живы (Length=80)",
          abs(box2.Shape.BoundBox.XLength - 80) < 1e-6,
          f"{box2.Shape.BoundBox.XLength}")
    check("после открытия: bom_status сохранён",
          box2.bom_status == "part_mass_pending", box2.bom_status)

# --- 5. STEP ------------------------------------------------------------
step_path = os.path.join(script_dir, "zz_selftest.step")
# ВАЖНО: cutf/cyl — объекты ЗАКРЫТОГО документа (closeDocument) — не экспортировать!
cutf2 = doc2.getObject("T_cut")
cyl2 = doc2.getObject("T_cyl")
assert cutf2 is not None and cyl2 is not None, "объекты doc2 не найдены"
LOGF.write("STEP: начато (Import.export, объекты doc2)\n"); LOGF.flush()
import Import
Import.export([cutf2, cyl2], step_path)
LOGF.write("STEP: Import.export выполнен\n"); LOGF.flush()
size = os.path.getsize(step_path) if os.path.isfile(step_path) else 0
check("STEP экспорт", size > 1000, f"{size} байт")
with open(step_path, "r", errors="ignore") as f:
    head = f.read(200)
check("STEP имеет заголовок ISO-10303", head.startswith("ISO-10303"), head[:40])

# --- итог ---------------------------------------------------------------
ver = ".".join(str(x) for x in App.Version()[:3])
print(f"[selftest] FreeCAD {ver}")
print(f"[selftest] шагов: {len(RESULTS)}, провалов: {len(FAILED)}")
_final = "TEST_RESULT: " + ("PASS" if not FAILED else "FAIL " + ",".join(FAILED))
print(_final)
LOGF.write(_final + "\n"); LOGF.flush()
sys.exit(0 if not FAILED else 1)
