import os, FreeCAD as App
d = os.path.dirname(os.path.abspath(__file__)) if "__file__" in globals() else "."
doc = App.openDocument(os.path.join(d, "stage2g_base_v2.FCStd"))
doc.recompute()
for k in ("FL", "FR", "BL", "BR"):
    o = doc.getObject(f"WHL_{k}")
    b = o.Shape.BoundBox
    ax = o.Placement.Rotation.multVec(App.Vector(0, 0, 1))
    print(f"WHL_{k}: X {b.XMin:.1f}..{b.XMax:.1f} (dX={b.XLength:.1f})  "
          f"Y {b.YMin:.1f}..{b.YMax:.1f} (dY={b.YLength:.1f})  "
          f"Z {b.ZMin:.1f}..{b.ZMax:.1f} (dZ={b.ZLength:.1f})  ось=({ax.x:.2f},{ax.y:.2f},{ax.z:.2f})")
