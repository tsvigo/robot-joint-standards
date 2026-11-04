# -*- coding: utf-8 -*-
"""Read-only проба .blend: фактические оси колёс и wheel joints. Файл НЕ сохраняется."""
import math
import bpy
import mathutils

print("=== BLENDER WHEEL PROBE (read-only, без сохранения) ===")
print("файл:", bpy.data.filepath)

# 1) объекты с wheel/whl в имени
for o in sorted(bpy.data.objects, key=lambda x: x.name):
    ln = o.name.lower()
    if "wheel" not in ln and "whl" not in ln:
        continue
    mw = o.matrix_world
    ax_z = (mw.to_3x3() @ mathutils.Vector((0, 0, 1))).normalized()
    ax_x = (mw.to_3x3() @ mathutils.Vector((1, 0, 0))).normalized()
    dims = tuple(round(v * 1000, 1) for v in o.dimensions)  # мм
    loc = tuple(round(v, 4) for v in mw.translation)
    rot = tuple(round(math.degrees(a), 1) for a in o.rotation_euler)
    print(f"\nОБЪЕКТ {o.name!r} type={o.type} parent={o.parent.name if o.parent else None}")
    print(f"  loc_world_m={loc}  rotation_euler_deg={rot}")
    print(f"  dimensions_мм={dims}")
    print(f"  локальная Z -> мир = ({ax_z.x:.6f}, {ax_z.y:.6f}, {ax_z.z:.6f}) "
          f"{'ВЕРТИКАЛЬНАЯ' if abs(abs(ax_z.z)-1) < 1e-4 else 'ГОРИЗОНТАЛЬНАЯ/наклонная'}")
    print(f"  локальная X -> мир = ({ax_x.x:.6f}, {ax_x.y:.6f}, {ax_x.z:.6f})")
    if o.type == "MESH":
        # мировые ограничения вершин
        ws = [o.matrix_world @ v.co for v in o.data.vertices]
        zs = [v.z for v in ws]
        print(f"  world Z вершин: {min(zs):.4f}..{max(zs):.4f} (пол -5.6086) -> низ-пол={min(zs)+5.6086:+.5f} м")
        # признак цилиндра: ось с минимальным разбросом? простая эвристика по AABB
        xs = [v.x for v in ws]; ys = [v.y for v in ws]
        print(f"  world AABB м: X {min(xs):.4f}..{max(xs):.4f}, Y {min(ys):.4f}..{max(ys):.4f}")

# 2) кости/суставы wheel в armatures
print("\n=== WHEEL JOINTS (bones / pose bones) ===")
found = 0
for arm in bpy.data.armatures:
    for pb in arm.pose.bones:
        if "wheel" not in pb.name.lower():
            continue
        found += 1
        b = pb.bone
        mw = arm.matrix_world @ pb.matrix
        ax = (mw.to_3x3() @ mathutils.Vector((0, 0, 1))).normalized()
        vec = (arm.matrix_world.to_3x3() @ b.vector).normalized()
        print(f"  {pb.name}: bone_vec_world=({vec.x:.4f},{vec.y:.4f},{vec.z:.4f}) "
              f"roll={math.degrees(b.roll):.1f}deg localZ_world=({ax.x:.4f},{ax.y:.4f},{ax.z:.4f}) "
              f"rot={tuple(round(math.degrees(a),1) for a in pb.rotation_euler)}")
if not found:
    print("  костей с wheel в имени нет (суставы — объекты или empty)")

# 3) объекты-суставы *_joint
print("\n=== *_wheel_*_joint objects ===")
for o in sorted(bpy.data.objects, key=lambda x: x.name):
    if "joint" in o.name.lower() and "wheel" in o.name.lower():
        mw = o.matrix_world
        axz = (mw.to_3x3() @ mathutils.Vector((0, 0, 1))).normalized()
        axx = (mw.to_3x3() @ mathutils.Vector((1, 0, 0))).normalized()
        print(f"  {o.name}: type={o.type} loc={tuple(round(v,4) for v in mw.translation)} "
              f"Z_world=({axz.x:.4f},{axz.y:.4f},{axz.z:.4f}) X_world=({axx.x:.4f},{axx.y:.4f},{axx.z:.4f})")
print("=== PROBE END (без bpy.ops.wm.save — файл не изменён) ===")
