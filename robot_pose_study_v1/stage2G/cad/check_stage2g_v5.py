# -*- coding: utf-8 -*-
"""Контроль внешнего габарита stage2g_v5_compare.FCStd (задачи 5 и 9).

Правило: Ø600/620/640 — внешний транспортный габарит ВСЕЙ сборки.
Панель (body) = ровно ØDp; все реальные элементы (колёса, ступицы, зоны,
приводы, блоки аутригеров) — не ближе gap_transport к контуру.
Рабочие элементы (рычаги, площадки, опорные контуры) исключены: они выдвинуты.
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
    GAP = json.load(f)["params"]["gap_transport"]

doc = App.openDocument(os.path.join(SCRIPT_DIR, "stage2g_v5_compare.FCStd"))
doc.recompute()

SKIP = ("_well", "_panelC", "supp", "_pad", "_pt", "_arm", "_env")
PANEL = ("_body", "_panel")

print("== CHECKS: внешний габарит сборки (панель = ØDp; реальные элементы <= R − 10) ==")
fail = 0
summary = []
for layout in ("R", "P"):
    for Dp in (600.0, 620.0, 640.0):
        for wkey in ("D152", "D203", "D230"):
            gname = f"{layout}{int(Dp)}_{wkey}"
            R_env = Dp / 2.0
            max_real, worst = 0.0, None
            max_panel = 0.0
            for o in doc.Objects:
                if not o.Name.startswith(gname + "_") or not hasattr(o, "Shape") or o.Shape.isNull():
                    continue
                n = o.Name
                if any(s in n for s in SKIP):
                    continue
                is_panel = n.endswith(PANEL)
                verts, _facets = o.Shape.tessellate(0.5)   # точки на поверхности, не только шов
                for vt in verts:
                    r = math.hypot(vt.x, vt.y)
                    if is_panel:
                        max_panel = max(max_panel, r)
                    elif r > max_real:
                        max_real, worst = r, o.Name
            ok = (max_panel <= R_env + 0.01) and (max_real <= R_env - GAP + 0.01)
            gap = R_env - max_real
            st = "PASS" if ok else "FAIL"
            if not ok:
                fail += 1
            summary.append({"group": gname, "status": st,
                            "panel_r": round(max_panel, 1),
                            "real_max_r": round(max_real, 1), "worst": worst,
                            "gap_mm": round(gap, 1)})
            print(f"  {st} {gname}: панель={max_panel:.1f} (=Ø{int(Dp)}), "
                  f"реальные={max_real:.1f} ({worst}), зазор={gap:.1f} мм (треб >= {GAP:.0f})")

status = "PASS 18/18" if fail == 0 else f"FAIL {fail}"
print("CHECK_STATUS:", status)
with open(os.path.join(SCRIPT_DIR, "stage2g_v5_contour_check.json"), "w", encoding="utf-8") as f:
    json.dump({"gap_required": GAP, "status": status, "groups": summary}, f,
              ensure_ascii=False, indent=1)
