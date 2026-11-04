import bpy, math
o = bpy.data.objects.get("wheel_FL")
print("=== ФОРМА wheel_FL ===")
zs = sorted({round((o.matrix_world @ v.co).z, 5) for v in o.data.vertices})
print("вершин:", len(o.data.vertices), "уникальных мировых Z уровней:", len(zs), zs[:6])
mw = o.matrix_world
cx = sum((mw @ v.co).x for v in o.data.vertices)/len(o.data.vertices)
cy = sum((mw @ v.co).y for v in o.data.vertices)/len(o.data.vertices)
rs = [math.hypot((mw @ v.co).x-cx, (mw @ v.co).y-cy) for v in o.data.vertices]
print(f"XY-радиусы от центра: min={min(rs)*1000:.1f} max={max(rs)*1000:.1f} мм")
zs_local = sorted({round(v.co.z, 5) for v in o.data.vertices})
print("локальных Z уровней вершин:", len(zs_local), zs_local[:6])
# полигоны: признак цилиндра — боковые четырёхугольники по вертикали
print("полигонов:", len(o.data.polygons))
