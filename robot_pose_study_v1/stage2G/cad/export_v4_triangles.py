# -*- coding: utf-8 -*-
"""Экспорт треугольников stage2g_v4_round с группировкой по gname -> class."""
import os, json
import FreeCAD as App

try:
    d = os.path.dirname(os.path.abspath(__file__))
except NameError:
    d = os.getcwd()

doc = App.openDocument(os.path.join(d, "stage2g_v4_round.FCStd"))
doc.recompute()

def classify(n):
    if "_well" in n or "_panelC" in n or n.endswith("_panel"):
        return None                      # операнды вырезов
    if "_WHL_" in n or "_hand_" in n:
        return "wheels"
    if "_env" in n:
        return "env"
    if "_pad" in n:
        return "pads"
    if "_supp" in n:
        return "supp"
    if "_lift" in n or "_drive" in n or "_bat" in n or "_pod" in n:
        return "zones"
    if "_body" in n:
        return "frame"
    return None

groups = {}   # gname -> class -> [tris]
for o in doc.Objects:
    n = o.Name
    if "_" not in n:
        continue
    cls = classify(n)
    if cls is None or not hasattr(o, "Shape") or o.Shape.isNull():
        continue
    # gname = часть до последнего "_<class-суффикса>": проще выделить префикс R<dp>_<wheel>
    parts = n.split("_")
    gname = f"{parts[0]}_{parts[1]}"        # R600_D152
    gd = groups.setdefault(gname, {"wheels": [], "env": [], "pads": [],
                                   "supp": [], "zones": [], "frame": []})
    verts, facets = o.Shape.tessellate(1.0)
    for f in facets:
        gd[cls].append([[round(verts[i].x, 1), round(verts[i].y, 1), round(verts[i].z, 1)]
                        for i in f])

out = {"groups": groups}
path = os.path.join(d, "stage2g_v4_round_tris.json")
with open(path, "w") as f:
    json.dump(out, f)
tot = sum(len(v) for g in groups.values() for v in g.values())
print(f"групп: {len(groups)}, треугольников: {tot}")
for g, gd in sorted(groups.items()):
    print(f"  {g}: " + ", ".join(f"{k}={len(v)}" for k, v in gd.items() if v))
print("JSON:", path, os.path.getsize(path), "байт")
