import sys, os
import FreeCAD as App

HERE = os.path.dirname(os.path.abspath(__file__))
BASE = os.path.dirname(HERE)

for name in ("rvw7pa_layout_200w.FCStd", "rvw7pa_layout_400w.FCStd"):
    p = os.path.join(BASE, name)
    doc = App.openDocument(p)
    print("=" * 70)
    print("FILE:", name)
    for o in doc.Objects:
        sh = getattr(o, "Shape", None)
        if sh is None:
            print(f"  {o.Name:<28} (no Shape)")
            continue
        bb = sh.BoundBox
        n = len(sh.Solids)
        print(f"  {o.Name:<28} solids={n:<3} vol={sh.Volume:12.1f} "
              f"X[{bb.XMin:8.2f},{bb.XMax:8.2f}] Y[{bb.YMin:8.2f},{bb.YMax:8.2f}] "
              f"Z[{bb.ZMin:8.2f},{bb.ZMax:8.2f}]")
    App.closeDocument(doc.Name)
