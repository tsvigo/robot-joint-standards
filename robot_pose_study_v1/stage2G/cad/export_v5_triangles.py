# -*- coding: utf-8 -*-
"""Экспорт треугольников stage2g_v5_compare.FCStd: gname -> class -> tris.
Классы разделяют состояния: transport (body/wheels/zones/pods/supp/env) и
working (arms/pads/suppext)."""
import os
import json
import FreeCAD as App

try:
    d = os.path.dirname(os.path.abspath(__file__))
except NameError:
    d = os.getcwd()

doc = App.openDocument(os.path.join(d, "stage2g_v5_compare.FCStd"))
doc.recompute()

CLASSES = ("body", "wheels", "zones", "pods", "arms", "pads", "supp", "suppext", "env")


def classify(n):
    if "_well" in n or "_panelC" in n or n.endswith("_panel"):
        return None
    if "suppExt" in n:
        return "suppext"
    if "_supp" in n:
        return "supp"
    if "_WHL" in n or "_hub" in n or "_hand" in n:
        return "wheels"
    if "_lift" in n or "_drv" in n or "_bat" in n:
        return "zones"
    if "_pod" in n:
        return "pods"
    if "_arm" in n:
        return "arms"
    if "_pad" in n or "_pt" in n:
        return "pads"
    if "_env" in n:
        return "env"
    if "_body" in n:
        return "body"
    return None


groups = {}
for o in doc.Objects:
    n = o.Name
    if "_" not in n or not hasattr(o, "Shape") or o.Shape.isNull():
        continue
    cls = classify(n)
    if cls is None:
        continue
    parts = n.split("_")
    gname = f"{parts[0]}_{parts[1]}"
    gd = groups.setdefault(gname, {c: [] for c in CLASSES})
    verts, facets = o.Shape.tessellate(1.0)
    for f in facets:
        gd[cls].append([[round(verts[i].x, 1), round(verts[i].y, 1), round(verts[i].z, 1)]
                        for i in f])

path = os.path.join(d, "stage2g_v5_tris.json")
with open(path, "w") as f:
    json.dump({"groups": groups}, f)
tot = sum(len(v) for g in groups.values() for v in g.values())
print(f"групп: {len(groups)}, треугольников: {tot}")
for g in sorted(groups):
    print(f"  {g}: " + ", ".join(f"{k}={len(v)}" for k, v in groups[g].items() if v))
print("JSON:", path, os.path.getsize(path), "байт")
