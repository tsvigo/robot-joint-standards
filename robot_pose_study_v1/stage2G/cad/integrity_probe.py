import os, FreeCAD as App
d = os.path.dirname(os.path.abspath(__file__))
for f, tag in (("stage2g_base.FCStd","v1"), ("stage2g_base_v2.FCStd","v2")):
    doc = App.openDocument(os.path.join(d, f))
    doc.recompute()
    names = sorted(o.Name for o in doc.Objects)
    parts = [n for n in names if n.startswith(("FRM_","PNL_","WLM_")) and not n.endswith(("_box","_body","_hole","_c1"))]
    wheels = [n for n in names if n.startswith("WHL_")]
    zones = [n for n in names if n.startswith("ZN_")]
    # оси колёс
    axes = []
    for k in ("FL","FR","BL","BR"):
        o = doc.getObject(f"WHL_{k}")
        if o: axes.append(tuple(round(v,3) for v in o.Placement.Rotation.multVec(App.Vector(0,0,1))))
    # габарит деталей
    xs, ys, zs = [], [], []
    for n in parts:
        b = doc.getObject(n).Shape.BoundBox
        xs += [b.XMin, b.XMax]; ys += [b.YMin, b.YMax]; zs += [b.ZMin, b.ZMax]
    print(f"{tag}: objects={len(doc.Objects)} parts={len(parts)} wheels={len(wheels)} zones={len(zones)} "
          f"bbox=X{min(xs):.1f}..{max(xs):.1f} Y{min(ys):.1f}..{max(ys):.1f} Z{min(zs):.1f}..{max(zs):.1f}")
    print(f"    axes={axes}")
    App.closeDocument(doc.Name)
