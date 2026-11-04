# -*- coding: utf-8 -*-
"""Экспорт треугольников stage2g_base_v2 для построения видов (изо/сверху/сбоку/снизу)."""
import os, json, re
import FreeCAD as App

try:
    d = os.path.dirname(os.path.abspath(__file__))
except NameError:
    d = os.getcwd()

doc = App.openDocument(os.path.join(d, "stage2g_base_v2.FCStd"))
doc.recompute()

_INTER = re.compile(r"(_box|_body|_hole|_c1)$")
groups = {"parts": [], "wheels": [], "zones": []}
for o in doc.Objects:
    if not hasattr(o, "Shape") or o.Shape.isNull():
        continue
    if o.Name.startswith(("FRM_", "PNL_", "WLM_")) and not _INTER.search(o.Name):
        groups["parts"].append(o)
    elif o.Name.startswith("WHL_"):
        groups["wheels"].append(o)
    elif o.Name.startswith("ZN_"):
        groups["zones"].append(o)

out = {"triangles": {}}
for gname, objs in groups.items():
    tris = []
    for o in objs:
        verts, facets = o.Shape.tessellate(1.0)
        for f in facets:
            tris.append([[round(verts[i].x, 2), round(verts[i].y, 2), round(verts[i].z, 2)] for i in f])
    out["triangles"][gname] = tris
    print(f"{gname}: {len(objs)} объектов, {len(tris)} треугольников")

path = os.path.join(d, "stage2g_base_v2_tris.json")
with open(path, "w") as f:
    json.dump(out, f)
print("JSON:", path, os.path.getsize(path), "байт")
