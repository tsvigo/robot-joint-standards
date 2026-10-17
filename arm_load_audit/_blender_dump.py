"""READ-ONLY дамп геометрии робота (Blender side).

Запуск:
    blender --background <file.blend> --python-exit-code 13 --python _blender_dump.py -- <out.json>
Ничего не сохраняет. Пишет JSON со всеми объектами: матрицы, потомки, свойства,
габариты, объёмы, материалы, ограничения.
"""
import bpy
import json
import sys
from mathutils import Vector

args = sys.argv
out_path = args[args.index('--') + 1]


def m4(m):
    return [[float(x) for x in row] for row in m]


def jval(v):
    if isinstance(v, (list, tuple)):
        return [jval(x) for x in v]
    if hasattr(v, 'to_list'):
        try:
            return v.to_list()
        except Exception:
            pass
    if isinstance(v, (bool, int, float, str)) or v is None:
        return v
    try:
        return float(v)
    except Exception:
        return str(v)


data = {'blender_version': bpy.app.version_string, 'filepath': bpy.data.filepath}
sc = bpy.context.scene
data['scene'] = {'name': sc.name, 'frame': sc.frame_current,
                 'unit_system': sc.unit_settings.system,
                 'scale_length': sc.unit_settings.scale_length}
try:
    data['scene']['gravity'] = [float(x) for x in sc.gravity]
except Exception:
    data['scene']['gravity'] = None

objs = {}
for o in bpy.data.objects:
    rec = {'type': o.type,
           'parent': o.parent.name if o.parent else None,
           'children': [c.name for c in o.children],
           'mw': m4(o.matrix_world),
           'loc': [float(x) for x in o.location],
           'rot_euler': [float(x) for x in o.rotation_euler],
           'rot_mode': o.rotation_mode,
           'scale': [float(x) for x in o.scale],
           'dims': [float(x) for x in o.dimensions],
           'constraints': [c.type for c in o.constraints],
           'props': {k: jval(o[k]) for k in o.keys() if k != '_RNA_UI'},
           'parent_type': o.parent_type,
           'modifiers': [m.type for m in o.modifiers],
           'data_name': o.data.name if o.data else None}
    if o.type == 'MESH':
        me = o.data
        rec['verts'] = len(me.vertices)
        rec['polys'] = len(me.polygons)
        try:
            import bmesh
            bm = bmesh.new()
            bm.from_mesh(me)
            rec['vol_local'] = float(bm.calc_volume(signed=False))
            bm.free()
        except Exception:
            rec['vol_local'] = None
        corners = [o.matrix_world @ Vector(c) for c in o.bound_box]
        rec['bbox_world_min'] = [min(float(v[i]) for v in corners) for i in range(3)]
        rec['bbox_world_max'] = [max(float(v[i]) for v in corners) for i in range(3)]
        rec['mat_slots'] = [m.name if m else None for m in me.materials]
    if o.type == 'ARMATURE':
        rec['bones'] = {}
        for b in o.data.bones:
            rec['bones'][b.name] = {
                'head_world': [float(x) for x in (o.matrix_world @ b.head_local)],
                'tail_world': [float(x) for x in (o.matrix_world @ b.tail_local)],
                'parent': b.parent.name if b.parent else None}
    objs[o.name] = rec

data['objects'] = objs
mats = []
for m in bpy.data.materials:
    mats.append({'name': m.name, 'use_nodes': m.use_nodes,
                 'props': {k: str(m[k])[:80] for k in m.keys() if k != '_RNA_UI'}})
data['materials'] = mats

with open(out_path, 'w') as f:
    json.dump(data, f, ensure_ascii=False, indent=1)
print('DUMP OK', len(objs), 'objects ->', out_path)
