import os, FreeCAD as App
d = os.path.dirname(os.path.abspath(__file__))
doc = App.openDocument(os.path.join(d, "stage2g_base_v2.FCStd"))
doc.recompute()

print("=== ВСЕ ОБЪЕКТЫ (имя, тип, видимость, родитель) ===")
for o in doc.Objects:
    vis = "hidden" if not o.Visibility else "visible"
    parent = o.InList[0].Name if o.InList else "root"
    print(f"  {o.Name:30s}  {o.TypeId:25s}  {vis:8s}  parent={parent}")

print("\n=== TOOL_* ===")
for o in doc.Objects:
    if o.Name.startswith("TOOL_"):
        print(f"  {o.Name}: Type={o.TypeId}, Vis={o.Visibility}, parent={[p.Name for p in o.InList]}")

print("\n=== WHL_* visibility ===")
for k in ("FL","FR","BL","BR"):
    o = doc.getObject(f"WHL_{k}")
    print(f"  WHL_{k}: Vis={o.Visibility}, InList={[p.Name for p in o.InList]}")

print("\n=== WLM_* visibility ===")
for k in ("FL","FR","BL","BR"):
    o = doc.getObject(f"WLM_{k}")
    print(f"  WLM_{k}: Vis={o.Visibility}, InList={[p.Name for p in o.InList]}")

print("\n=== Группы ===")
for o in doc.Objects:
    if o.TypeId == "App::Part" or o.TypeId == "App::DocumentObjectGroup":
        print(f"  {o.Name}: {o.TypeId}, children={len(o.Group) if hasattr(o,'Group') else 'N/A'}")
