import os, FreeCAD as App
d = os.path.dirname(os.path.abspath(__file__))
doc = App.openDocument(os.path.join(d, "stage2g_v4_round.FCStd"))
for n in ("R600_D152_body", "R600_D152_panelC4", "R600_D152_WHL_FL_L"):
    o = doc.getObject(n)
    print(n, "->", "None" if o is None else f"type={o.TypeId} null={o.Shape.isNull()} vol={o.Shape.Volume if not o.Shape.isNull() else 0:.0f}")
