# -*- coding: utf-8 -*-
"""Экспорт данных v6 для видов: треугольники вариантов + линии разрезов + отметки
реальных пересечений (красным). Разрезы: P1 — плоскость x=y (через колёса на диагонали),
R2 — плоскость y=0 (через колёса 0/180°), базис v5 P640_D203 — плоскость x=y.
"""
import os
import json
import FreeCAD as App
import Part

try:
    d = os.path.dirname(os.path.abspath(__file__))
except NameError:
    d = os.getcwd()

OUT = {"tris": {}, "sections": {}, "intersections": {}}


def classify(n):
    if "_well" in n or "_panelC" in n or n.endswith("_panel"):
        return None
    if "suppExt" in n:
        return "suppext"
    if "_supp" in n:
        return "supp"
    if "_WHL" in n or "_hub" in n or "_hand" in n:
        return "wheels"
    if "_lift" in n:
        return "lift"
    if "_drv" in n:
        return "zones"
    if "_bat" in n:
        return "zones"
    if "_pod" in n:
        return "pods"
    if "_arm" in n:
        return "arms"
    if "_pad" in n or "_pt" in n:
        return "pads"
    if "_env" in n:
        return "env"
    if "_body" in n or "_rib" in n:
        return "body"
    return None


def dump_tris(doc, gname):
    groups = {}
    for o in doc.Objects:
        n = o.Name
        if not n.startswith(gname + "_") or not hasattr(o, "Shape") or o.Shape.isNull():
            continue
        cls = classify(n)
        if cls is None:
            continue
        verts, facets = o.Shape.tessellate(1.0)
        groups.setdefault(cls, [])
        for f in facets:
            groups[cls].append([[round(verts[i].x, 1), round(verts[i].y, 1), round(verts[i].z, 1)]
                                for i in f])
    OUT["tris"][gname] = groups


def plane_face(kind):
    if kind == "diag":     # плоскость x = y
        pts = [App.Vector(0, 0, -60), App.Vector(700, 700, -60),
               App.Vector(700, 700, 2000), App.Vector(0, 0, 2000)]
    else:                  # плоскость y = 0
        pts = [App.Vector(-700, 0, -60), App.Vector(700, 0, -60),
               App.Vector(700, 0, 2000), App.Vector(-700, 0, 2000)]
    return Part.Face(Part.makePolygon(pts + [pts[0]]))


def dump_section(doc, gname, kind, key):
    face = plane_face(kind)
    lines = {}
    for o in doc.Objects:
        n = o.Name
        if not n.startswith(gname + "_") or not hasattr(o, "Shape") or o.Shape.isNull():
            continue
        cls = classify(n)
        if cls is None or cls in ("env", "supp", "suppext", "pads"):
            continue
        try:
            sect = o.Shape.section(face)
        except Exception:
            continue
        for e in sect.Edges:
            try:
                pts = e.discretize(16)
            except Exception:
                continue
            lines.setdefault(cls, []).append(
                [[round(p.x, 1), round(p.y, 1), round(p.z, 1)] for p in pts])
    OUT["sections"][key] = {"plane": kind, "lines": lines}


def dump_intersections(doc, gname, kind, key, pairs):
    face = plane_face(kind)
    reds = []
    for a_name, b_name in pairs:
        a, b = doc.getObject(a_name), doc.getObject(b_name)
        if a is None or b is None:
            continue
        common = a.Shape.common(b.Shape)
        if common.Volume <= 1.0:
            continue
        for e in common.section(face).Edges:
            try:
                pts = e.discretize(12)
            except Exception:
                continue
            reds.append([[round(p.x, 1), round(p.y, 1), round(p.z, 1)] for p in pts])
    OUT["intersections"][key] = reds


# ---------- варианты v6 ----------
doc6 = App.openDocument(os.path.join(d, "stage2g_v6_variants.FCStd"))
doc6.recompute()
dump_tris(doc6, "P640_D152_Z")
dump_tris(doc6, "R620_CTM")
dump_section(doc6, "P640_D152_Z", "diag", "P1")
dump_section(doc6, "R620_CTM", "y0", "R2")
dump_intersections(doc6, "P640_D152_Z", "diag", "P1", [])
dump_intersections(doc6, "R620_CTM", "y0", "R2", [])

# ---------- базис v5: P640_D203 с лифтом во всю высоту (пересечения красным) ----------
doc5 = App.openDocument(os.path.join(d, "stage2g_v5_compare.FCStd"))
doc5.recompute()
dump_section(doc5, "P640_D203", "diag", "BASE")
dump_intersections(doc5, "P640_D203", "diag", "BASE", [
    ("P640_D203_WHL_FL_L", "P640_D203_lift"),
    ("P640_D203_hub_FL_L", "P640_D203_lift"),
])

path = os.path.join(d, "stage2g_v6_views_data.json")
with open(path, "w") as f:
    json.dump(OUT, f)
tot = sum(len(v) for g in OUT["tris"].values() for v in g.values())
print(f"треугольников: {tot}; разрезы: " +
      ", ".join(f"{k}={sum(len(v) for v in s['lines'].values())}" for k, s in OUT["sections"].items()) +
      "; красных: " + ", ".join(f"{k}={len(v)}" for k, v in OUT["intersections"].items()))
print("JSON:", path, os.path.getsize(path), "байт")
