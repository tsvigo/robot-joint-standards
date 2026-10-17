"""READ-ONLY тест позы «руки вперёд» (Blender side).

Запускается ТОЛЬКО на временной КОПИИ .blend; никогда не сохраняет файл.
Поворачивает плечевой сустав shoulder_joint_empty_2 (ось Z) на 30/60/85°,
добавляет комбинированный тест «обе руки вперёд 85°», проверяет пересечения
по BVH реальных мешей (collision-оболочки phobostype=4 исключены) и пишет JSON.

Запуск:
    blender --background <copy.blend> --python-exit-code 13 --python _blender_pose_test.py -- <out.json>
"""
import bpy
import json
import math
import sys
from mathutils import Vector
from mathutils.bvhtree import BVHTree

args = sys.argv
out_path = args[args.index('--') + 1] if '--' in args else '/dev/null'

objs = bpy.data.objects


def descendants(root_name):
    seen = [root_name]
    i = 0
    while i < len(seen):
        o = objs.get(seen[i])
        for c in o.children:
            seen.append(c.name)
        i += 1
    return seen


def is_shell(o):
    try:
        return int(o.get('phobostype', -1)) == 4
    except Exception:
        return False


def make_bvh(o):
    me = o.data
    mw = o.matrix_world
    verts = [mw @ v.co for v in me.vertices]
    polys = [tuple(p.vertices) for p in me.polygons]
    return BVHTree.FromPolygons(verts, polys)


def corners_world(o):
    mw = o.matrix_world
    return [mw @ Vector(c) for c in o.bound_box]


def aabb(corners):
    mn = [min(c[i] for c in corners) for i in range(3)]
    mx = [max(c[i] for c in corners) for i in range(3)]
    return mn, mx


def aabb_gap(a, b):
    g = 0.0
    for i in range(3):
        if a[1][i] < b[0][i]:
            g = max(g, b[0][i] - a[1][i])
        elif b[1][i] < a[0][i]:
            g = max(g, a[0][i] - b[1][i])
    return g


drivers_total = 0
for o in objs:
    ad = o.animation_data
    if ad and ad.drivers:
        drivers_total += len(ad.drivers)

res = {
    'file': bpy.data.filepath,
    'blender': bpy.app.version_string,
    'drivers_total': drivers_total,
    'excluded_shells': sorted(o.name for o in objs if o.type == 'MESH' and is_shell(o)),
    'tests': [],
    'baseline_bounds': {},
}

mesh_objs = [o for o in objs if o.type == 'MESH' and not is_shell(o)]

for o in mesh_objs:
    mn, mx = aabb(corners_world(o))
    res['baseline_bounds'][o.name] = [mn, mx]

CHAIN = ['{s}_shoulder_joint_empty_3', '{s}_shoulder_3_link', '{s}_shoulder_joint_empty_2',
         '{s}_shoulder_2_link', '{s}_shoulder_joint_empty_1', '{s}_shoulder_link',
         '{s}_elbow_joint_empty', '{s}_forearm_link', '{s}_proximal_radioulnar_joint_empty',
         '{s}_radioulnar_link', '{s}_wrist_joint_1_empty', '{s}_wrist_pitch_link',
         '{s}_wrist_joint_2_empty', '{s}_palm_link']

for side, sign in (('left', -1.0), ('right', 1.0)):
    j2_name = '%s_shoulder_joint_empty_2' % side
    j2 = objs[j2_name]
    j2.rotation_mode = 'XYZ'
    moved = set(descendants(j2_name))
    moved_meshes = [o for o in mesh_objs if o.name in moved]
    static_meshes = [o for o in mesh_objs if o.name not in moved]
    static_trees = []
    for o in static_meshes:
        static_trees.append((o.name, make_bvh(o), aabb(corners_world(o))))

    palm_name = '%s_palm_link' % side
    rest_palm_x = objs[palm_name].matrix_world[0][3]

    for angle in (0.0, 30.0, 60.0, 85.0):
        j2.rotation_euler[2] = math.radians(sign * angle)
        bpy.context.view_layer.update()
        test = {'side': side, 'angle_deg': angle, 'sign': sign, 'overlaps': [], 'min_gaps': []}
        test['palm_dx_m'] = objs[palm_name].matrix_world[0][3] - rest_palm_x
        mv = []
        for o in moved_meshes:
            mv.append((o.name, make_bvh(o), aabb(corners_world(o))))
        for mname, mtree, ma in mv:
            for sname, stree, sa in static_trees:
                g = aabb_gap(ma, sa)
                if g == 0.0:
                    pairs = mtree.overlap(stree)
                    if pairs:
                        test['overlaps'].append({'moved': mname, 'static': sname,
                                                 'poly_pairs': len(pairs)})
                        continue
                if g < 0.5:
                    best = None
                    me = objs[mname].data
                    mw = objs[mname].matrix_world
                    for v in me.vertices:
                        w = mw @ v.co
                        loc, nor, idx, dist = stree.find_nearest(w, 1.0)
                        if loc is not None:
                            d = (loc - w).length
                            if best is None or d < best:
                                best = d
                    if best is not None:
                        test['min_gaps'].append({'moved': mname, 'static': sname,
                                                 'gap_m': round(best, 5)})
        jm = {}
        for tpl in CHAIN:
            n = tpl.format(s=side)
            o = objs[n]
            jm[n] = {'T': [o.matrix_world[i][3] for i in range(3)],
                     'R': [[o.matrix_world[i][j] for j in range(3)] for i in range(3)]}
        test['joints'] = jm
        test['moved_mesh_count'] = len(moved_meshes)
        res['tests'].append(test)

    j2.rotation_euler[2] = 0.0
    bpy.context.view_layer.update()

# Комбинированный тест: обе руки вперёд 85° одновременно
for side, sign in (('left', -1.0), ('right', 1.0)):
    objs['%s_shoulder_joint_empty_2' % side].rotation_euler[2] = math.radians(sign * 85.0)
bpy.context.view_layer.update()
combined = {'angle_deg': 85.0, 'overlaps': [], 'cross_overlaps': []}
left_moved = set(descendants('left_shoulder_joint_empty_2'))
right_moved = set(descendants('right_shoulder_joint_empty_2'))
static_now = [o for o in mesh_objs if o.name not in left_moved and o.name not in right_moved]
static_trees2 = [(o.name, make_bvh(o), aabb(corners_world(o))) for o in static_now]
left_trees = [(o.name, make_bvh(o), aabb(corners_world(o)))
              for o in mesh_objs if o.name in left_moved]
right_trees = [(o.name, make_bvh(o), aabb(corners_world(o)))
               for o in mesh_objs if o.name in right_moved]
for name, tree, a in left_trees + right_trees:
    for sname, stree, sa in static_trees2:
        if aabb_gap(a, sa) > 0.0:
            continue
        pairs = tree.overlap(stree)
        if pairs:
            combined['overlaps'].append({'moved': name, 'static': sname,
                                         'poly_pairs': len(pairs)})
for lname, ltree, la in left_trees:
    for rname, rtree, ra in right_trees:
        if aabb_gap(la, ra) > 0.0:
            continue
        pairs = ltree.overlap(rtree)
        if pairs:
            combined['cross_overlaps'].append({'left': lname, 'right': rname,
                                               'poly_pairs': len(pairs)})
combined['hands'] = {}
for side in ('left', 'right'):
    palm = objs['%s_palm_link' % side]
    combined['hands'][side] = {'T': [palm.matrix_world[i][3] for i in range(3)]}
for side, sign in (('left', -1.0), ('right', 1.0)):
    objs['%s_shoulder_joint_empty_2' % side].rotation_euler[2] = 0.0
bpy.context.view_layer.update()
res['combined_85'] = combined

with open(out_path, 'w') as f:
    json.dump(res, f, ensure_ascii=False, indent=1)
print('POSE TEST OK ->', out_path)
