import os, FreeCAD as App
d = os.path.dirname(os.path.abspath(__file__))
doc = App.openDocument(os.path.join(d, "stage2g_v4_round.FCStd"))
doc.recompute()
for o in doc.Objects:
    n = o.Name
    if not n.startswith("R600_D152"):
        continue
    extra = ""
    if hasattr(o, "Shape") and not o.Shape.isNull():
        b = o.Shape.BoundBox
        extra = (f" | V={o.Shape.Volume/1000:.1f} см3 | bbox X{b.XMin:.0f}..{b.XMax:.0f} "
                 f"Y{b.YMin:.0f}..{b.YMax:.0f} Z{b.ZMin:.0f}..{b.ZMax:.0f}")
    vis = "hidden" if not o.Visibility else "visible"
    print(f"{n:32s} {o.TypeId:20s} {vis:7s} {extra}")
