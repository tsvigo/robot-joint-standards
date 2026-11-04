# -*- coding: utf-8 -*-
"""Адресная проверка ориентации осей колёс stage2g_base.FCStd (только чтение)."""
import os
import math
import FreeCAD as App

try:
    d = os.path.dirname(os.path.abspath(__file__))
except NameError:
    d = os.getcwd()

doc = App.openDocument(os.path.join(d, "stage2g_base.FCStd"))
doc.recompute()

FLOOR = -134.55
BASE_BOTTOM = -88.0
LOG = open(os.path.join(d, "wheel_axis_report.txt"), "w", encoding="utf-8", buffering=1)


def w(s=""):
    LOG.write(s + "\n")
    print(s)


w("== ОРИЕНТАЦИЯ ОСЕЙ WHL_* И WLM_* (мировая СК FreeCAD, мм) ==")
w(f"пол={FLOOR}, низ базы={BASE_BOTTOM}, просвет={BASE_BOTTOM - FLOOR:.2f}")
w()

for name in ["WHL_FL", "WHL_FR", "WHL_BL", "WHL_BR",
             "WLM_FL", "WLM_FR", "WLM_BL", "WLM_BR"]:
    o = doc.getObject(name)
    q = o.Placement.Rotation
    b = o.Placement.Base
    # локальная ось цилиндра (+Z) -> мировой вектор
    ax = q.multVec(App.Vector(0, 0, 1))
    bb = o.Shape.BoundBox
    w(f"{name}:")
    w(f"  Placement.Base = ({b.x:.3f}, {b.y:.3f}, {b.z:.3f})")
    w(f"  Rotation: axis=({q.Axis.x:.3f},{q.Axis.y:.3f},{q.Axis.z:.3f}) angle={math.degrees(q.Angle):.1f} deg")
    w(f"  ОСЬ ЦИЛИНДРА в мировой СК = ({ax.x:.6f}, {ax.y:.6f}, {ax.z:.6f}) "
      f"-> {'ВЕРТИКАЛЬНАЯ (+Z)' if abs(abs(ax.z) - 1) < 1e-9 else 'ГОРИЗОНТАЛЬНАЯ/произвольная'}")
    w(f"  bbox: X {bb.XMin:.2f}..{bb.XMax:.2f}  Y {bb.YMin:.2f}..{bb.YMax:.2f}  Z {bb.ZMin:.2f}..{bb.ZMax:.2f}")
    if name.startswith("WHL"):
        w(f"  нижняя точка = {bb.ZMin:.3f} (пол {FLOOR}) -> дельта {bb.ZMin - FLOOR:+.3f}")
        w(f"  касание пола: {'ДА' if abs(bb.ZMin - FLOOR) < 1e-6 else 'НЕТ'}")
    w()

# --- пересечение кронштейнов с габаритами колёс (не покрывается C3/C4)
w("== WLM_* vs WHL_* (пересечение тел) ==")
for k in ("FL", "FR", "BL", "BR"):
    a = doc.getObject(f"WLM_{k}").Shape
    b = doc.getObject(f"WHL_{k}").Shape
    com = a.common(b)
    v = com.Volume if not com.isNull() else 0.0
    w(f"WLM_{k} vs WHL_{k}: volume_common = {v:.3f} мм^3 "
      f"({'ПЕРЕСЕЧЕНИЕ' if v > 1e-6 else 'нет (касание/разрыв)'}")

# --- ниша рамы: фактический радиус выреза по имени детали (параметр clr)
w()
w("== Радиус ниши в раме (габарит выреза по XY) ==")
ps = doc.getObject("params")
for cell in ("well_r", "clr_wheel", "wheel_d", "wheel_w"):
    try:
        w(f"  params.{cell} = {ps.get(cell)}")
    except Exception as e:
        w(f"  params.{cell}: {e}")

LOG.close()
