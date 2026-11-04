import os, FreeCAD as App
d = os.path.dirname(os.path.abspath(__file__))
doc = App.openDocument(os.path.join(d, "stage2g_base_v2.FCStd"))
doc.recompute()

print("=== WLM_* vs WHL_* геометрия (v2) ===")
for k in ("FL","FR","BL","BR"):
    wlm = doc.getObject(f"WLM_{k}")
    whl = doc.getObject(f"WHL_{k}")
    wb = wlm.Shape.BoundBox
    hb = whl.Shape.BoundBox
    # ось колеса
    ax = whl.Placement.Rotation.multVec(App.Vector(0,0,1))
    # центр колеса
    cw = hb.Center
    # центр кронштейна
    cm = wb.Center
    # дистанция от оси к центру кронштейна
    d = ((cm.x-cw.x)**2 + (cm.y-cw.y)**2)**0.5
    # проекция на ось колеса
    v = App.Vector(cm.x-cw.x, cm.y-cw.y, cm.z-cw.z)
    proj = v.x*ax.x + v.y*ax.y + v.z*ax.z
    print(f"\n{k}:")
    print(f"  WHL: center=({cw.x:.1f},{cw.y:.1f},{cw.z:.1f}) axis=({ax.x:.2f},{ax.y:.2f},{ax.z:.2f})")
    print(f"  WLM: center=({cm.x:.1f},{cm.y:.1f},{cm.z:.1f})")
    print(f"  d_xy от оси = {d:.1f} мм, проекция на ось = {proj:.1f} мм")
    print(f"  WLM ZMin={wb.ZMin:.1f}, ZMax={wb.ZMax:.1f} (верх диска={hb.ZMax:.1f})")
    # пересечение
    com = wlm.Shape.common(whl.Shape)
    v = com.Volume if not com.isNull() else 0.0
    print(f"  пересечение объём = {v:.1f} мм^3")

# Параметры колеса
ps = doc.getObject("params")
for p in ("wheel_d","wheel_w","wheel_roll_axis","clr_wheel","wheel_top"):
    try:
        print(f"  {p} = {ps.get(p)}")
    except: pass
