# -*- coding: utf-8 -*-
"""Корректировка v7.5 (задача 7) — геометрия шасси R2 из v6, ТОЛЬКО ЧТЕНИЕ.

Выгрузка z-границ и XY-зоны панели R620_CTM и колёсных модулей для включения
в полный транспортный габарит (внешний габарит сборки не может быть меньше
Ø620 мм по X/Y). v6 FCStd не сохраняется.
"""
import hashlib
import json
import os

import FreeCAD as App

try:
    SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
except NameError:
    SCRIPT_DIR = os.getcwd()

FC = os.path.join(SCRIPT_DIR, "stage2g_v6_variants.FCStd")
# Документ v6 уже хранится в СК R2 (мм, пол z=0, центр базы 0,0) —
# вычеты OX/OY/FLOOR НЕ применяются (они дали бы ложный сдвиг ±3.6/1.9/5.6 мм).
R2Z = lambda z: z  # noqa: E731


def sha(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()[:16]


sha0, mt0 = sha(FC), os.path.getmtime(FC)
doc = App.openDocument(FC)
doc.recompute()

rows = []
for obj in doc.Objects:
    n = obj.Name
    if not n.startswith("R620_CTM"):
        continue
    try:
        bb = obj.Shape.BoundBox
    except Exception:
        continue
    rows.append({
        "name": n, "label": getattr(obj, "Label", n),
        "x": [round(bb.XMin, 4), round(bb.XMax, 4)],
        "y": [round(bb.YMin, 4), round(bb.YMax, 4)],
        "z_r2": [round(R2Z(bb.ZMin), 4), round(R2Z(bb.ZMax), 4)],
        "max_radius_xy": round(max(abs(bb.XMin), abs(bb.XMax),
                                   abs(bb.YMin), abs(bb.YMax)), 4),
    })
print("== Объекты R620_CTM в v6 (СК R2, z от пола) ==")
for r in sorted(rows, key=lambda r: r["z_r2"][0]):
    print(f"  {r['name']:32s} X{r['x']} Y{r['y']} Z{r['z_r2']} "
          f"maxR={r['max_radius_xy']}")

# итоговые зоны шасси
panel = [r for r in rows if r["name"].endswith("_body")]
mods = [r for r in rows if "panelC" not in r["name"]
        and not r["name"].endswith("_body") and r["name"] != "R620_CTM_lift_flange"]
z_all = [r["z_r2"] for r in rows]
out = {
    "objects": sorted(rows, key=lambda r: r["name"]),
    "panel_body": panel[0] if panel else None,
    "notes": ["z_r2 — высота от пола R2; X/Y — в СК R2 (центр базы 0,0).",
              "v6 не изменён (save не вызывался)."],
}
path = os.path.join(SCRIPT_DIR, "stage2g_v7_5_chassis_geom.json")
with open(path, "w", encoding="utf-8") as f:
    json.dump(out, f, ensure_ascii=False, indent=1)

App.closeDocument(doc.Name)
print("\nv6 целостность:", "OK" if (sha(FC) == sha0 and os.path.getmtime(FC) == mt0)
      else "ИЗМЕНЁН!")
print("JSON:", path)