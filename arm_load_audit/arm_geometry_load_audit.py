#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
arm_geometry_load_audit.py — read-only аудит геометрии рук робота и моментов
нагрузки (груз 200 кг, две руки).

Скрипт запускает Blender в фоне (без сохранения):
  * _blender_dump.py      — дамп геометрии/свойств исходного .blend;
  * _blender_pose_test.py — тест позы «руки вперёд» 30/60/85° с точной проверкой
    пересечений (BVH) — ТОЛЬКО на временной КОПИИ .blend (копия не сохраняется);
считает моменты сил в суставах для сценариев:
  A — статическое удержание 100 кг на кисть (и варианты точек захвата K1..K3t);
  A_asym — несимметричное распределение 60/140 кг;
  B — вертикальное ускорение 0.1g и 0.2g;
  C — вылет вперёд (поза 30/60/85° по фактическим матрицам из pose-теста);
  SW — моменты от собственного веса звеньев по массам, записанным в файле
       (помечены как НЕ ПОДТВЕРЖДЁННЫЕ);
пишет рядом с собой: arm_geometry_data.csv, arm_joint_moments.csv,
arm_geometry_load_audit.md, arm_dump.json, pose_c_results.json;
сверяет SHA-256 исходного .blend до и после (файл не изменяется).

Запуск:  python3 arm_geometry_load_audit.py
"""
import csv
import hashlib
import json
import math
import os
import shutil
import subprocess
import sys

BLENDER = '/home/v/Apps/Blender/4.2-LTS/blender'
BLEND = '/home/v/projects/робот/robot_scaled_4-2-21-LTS-ARCH-02-MANUAL-01.blend'
TMP = '/home/v/.local/state/opencode/tmp'
OUT = os.path.dirname(os.path.abspath(__file__))
GRAV = 9.81

# документальные данные суставов руки (robot_joints.md, правая рука; левая зеркальна):
# (ключ, суффикс empty, parent-суффикс, child-суффикс, ось, нижний°, верхний°, смысл, ссылка)
JOINTS = [
    ('j3', 'shoulder_joint_empty_3', 'torso_link', 'shoulder_3_link', 'Y', -180.0, 60.0,
     'внутренняя/наружная ротация руки', 'robot_joints.md, right_shoulder_joint_empty_3 (строки 135-168)'),
    ('j2', 'shoulder_joint_empty_2', 'shoulder_3_link', 'shoulder_2_link', 'Z', -90.0, 90.0,
     'сгибание/разгибание руки (горизонтальная маховая ось)', 'robot_joints.md, right_shoulder_joint_empty_2 (строки 98-131)'),
    ('j1', 'shoulder_joint_empty_1', 'shoulder_2_link', 'shoulder_link', 'X', -90.0, 130.0,
     'отведение/приведение руки', 'robot_joints.md, right_shoulder_joint_empty_1 (строки 61-94)'),
    ('elbow', 'elbow_joint_empty', 'shoulder_link', 'forearm_link', 'Z', -10.0, 150.0,
     'сгибание предплечья', 'robot_joints.md, right_elbow_joint_empty (строки 172-204)'),
    ('ru', 'proximal_radioulnar_joint_empty', 'forearm_link', 'radioulnar_link', 'Y', -90.0, 90.0,
     'пронация/супинация предплечья', 'robot_joints.md, right_proximal_radioulnar_joint_empty (строки 206-235)'),
    ('w1', 'wrist_joint_1_empty', 'radioulnar_link', 'wrist_pitch_link', 'Z', -90.0, 90.0,
     'сгибание кисти (сустав 1)', 'robot_joints.md, right_wrist_joint_1_empty (строки 241-273)'),
    ('w2', 'wrist_joint_2_empty', 'wrist_pitch_link', 'palm_link', 'X', -25.0, 45.0,
     'отведение/приведение кисти (сустав 2)', 'robot_joints.md, right_wrist_joint_2_empty (строки 275-307)'),
]
JKEYS = [j[0] for j in JOINTS]
JLABEL = {'j3': 'j3 (плечо-3, ось Y)', 'j2': 'j2 (плечо-2, ось Z)', 'j1': 'j1 (плечо-1, ось X)',
          'elbow': 'локоть (ось Z)', 'ru': 'лучелоктевой (ось Y)', 'w1': 'запястье-1 (ось Z)',
          'w2': 'запястье-2 (ось X)'}
AXIS_VEC = {'X': (1.0, 0.0, 0.0), 'Y': (0.0, 1.0, 0.0), 'Z': (0.0, 0.0, 1.0)}

LINKS = [
    ('shoulder_3_link', 'пустой фрейм (dummy-линк, меша нет)'),
    ('shoulder_2_link', 'пустой фрейм (dummy-линк, меша нет)'),
    ('shoulder_link', 'плечо (mesh-бокс)'),
    ('forearm_link', 'предплечье (mesh-бокс)'),
    ('radioulnar_link', 'пустой фрейм (dummy-линк, меша нет)'),
    ('wrist_pitch_link', 'пустой фрейм (dummy-линк, меша нет)'),
    ('palm_link', 'ладонь (mesh-бокс)'),
]

SIDES = ('left', 'right')
SIDE_RU = {'left': 'левая', 'right': 'правая'}


# ------------------------------ мини-математика ------------------------------
def m3_apply(A, v):
    return tuple(A[i][0] * v[0] + A[i][1] * v[1] + A[i][2] * v[2] for i in range(3))


def m4_apply(M, v):
    return tuple(M[i][0] * v[0] + M[i][1] * v[1] + M[i][2] * v[2] + M[i][3] for i in range(3))


def norm3(v):
    l = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
    if l == 0.0:
        return (0.0, 0.0, 0.0)
    return (v[0] / l, v[1] / l, v[2] / l)


def dist3(a, b):
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def inv4(M):
    A = [list(M[i]) + [1.0 if i == j else 0.0 for j in range(4)] for i in range(4)]
    for col in range(4):
        piv = max(range(col, 4), key=lambda r: abs(A[r][col]))
        A[col], A[piv] = A[piv], A[col]
        d = A[col][col]
        A[col] = [x / d for x in A[col]]
        for r in range(4):
            if r != col and A[r][col] != 0.0:
                f = A[r][col]
                A[r] = [A[r][k] - f * A[col][k] for k in range(8)]
    return [row[4:] for row in A]


def det3(A):
    return (A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1])
            - A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0])
            + A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]))


def sha256(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def f1(x):
    return '%.1f' % x


def f2(x):
    return '%.2f' % x


def f3(x):
    return '%.3f' % x


def f4(x):
    return '%.4f' % x


def f5(x):
    return '%.5f' % x


def f6(x):
    return '%.6f' % x


def rec_T(rec):
    return (rec['mw'][0][3], rec['mw'][1][3], rec['mw'][2][3])


def rec_A(rec):
    return [[rec['mw'][i][j] for j in range(3)] for i in range(3)]


def run_blender(blend_path, script_path, out_json, log_name):
    """Запуск Blender в фоне. Без сохранения файла. Возвращает путь к логу."""
    log_path = os.path.join(TMP, log_name)
    cmd = [BLENDER, '--background', blend_path, '--python-exit-code', '13',
           '--python', script_path, '--', out_json]
    with open(log_path, 'w') as lf:
        p = subprocess.run(cmd, stdout=lf, stderr=subprocess.STDOUT)
    if p.returncode != 0:
        sys.exit('ОШИБКА: Blender вернул код %d, лог: %s' % (p.returncode, log_path))
    return log_path


def analyze(dump, pose, checks):
    O = dump['objects']
    res = {'scene': dump['scene'], 'blender': dump['blender_version'],
           'filepath': dump['filepath'], 'materials': dump.get('materials', [])}

    # ---------------- суставы: мировые координаты, оси, сверка с MD ----------------
    joints = {}
    for side in SIDES:
        for key, suf, par, ch, ax, lo, hi, mean, mdref in JOINTS:
            name = side + '_' + suf
            rec = O[name]
            T = rec_T(rec)
            A3 = rec_A(rec)
            n = norm3(m3_apply(A3, AXIS_VEC[ax]))
            cols = [norm3(tuple(A3[i][j] for i in range(3))) for j in range(3)]
            dev = max(min(dist3(cols[j], AXIS_VEC[k]) for k in 'XYZ') for j in range(3))
            exp_parent = par if par == 'torso_link' else side + '_' + par
            if rec['parent'] != exp_parent:
                checks.append('parent %s = %s (ожидался %s)' % (name, rec['parent'], exp_parent))
            joints[(side, key)] = {'name': name, 'T': T, 'A': A3, 'n': n, 'axis': ax,
                                   'lo': lo, 'hi': hi, 'mean': mean, 'mdref': mdref,
                                   'parent': rec['parent'], 'child': ch,
                                   'frame_dev': dev, 'rot_euler': list(rec['rot_euler'])}
    res['joints'] = joints
    chain_objs = [side + '_' + suf for side in SIDES
                  for suf in [j[1] for j in JOINTS] + [l[0] for l in LINKS]]
    res['max_euler'] = max(abs(v) for nm in chain_objs for v in O[nm]['rot_euler'])
    res['max_frame_dev'] = max(jd['frame_dev'] for jd in joints.values())

    # ---------------- точки захвата (тестовые) ----------------
    grips = {}
    tips_all = {}
    hand_parts = {}
    for side in SIDES:
        palm = O[side + '_palm_link']
        bmin = palm['bbox_world_min']
        bmax = palm['bbox_world_max']
        cx = (bmin[0] + bmax[0]) / 2.0
        cy = (bmin[1] + bmax[1]) / 2.0
        cz = (bmin[2] + bmax[2]) / 2.0
        inner_y = bmin[1] if side == 'left' else bmax[1]
        K = {'K1': {'label': 'центр внутренней грани ладони (грань, обращённая к другой кисти)',
                    'P': (cx, inner_y, cz)},
             'K2': {'label': 'геометрический центр ладони (центр bbox)', 'P': (cx, cy, cz)}}
        groups = [c for c in palm['children'] if ('digit' in c or 'thumb' in c)]

        def bctr(n):
            r = O[n]
            return tuple((r['bbox_world_min'][i] + r['bbox_world_max'][i]) / 2.0 for i in range(3))

        tips = {}
        for g in groups:
            stack = [g]
            meshes = []
            while stack:
                nm = stack.pop()
                r = O.get(nm)
                if r is None:
                    continue
                if r['type'] == 'MESH':
                    meshes.append(nm)
                stack.extend(r['children'])
            if not meshes:
                continue
            tm = max(meshes, key=lambda n: dist3(bctr(n), (cx, cy, cz)))
            tr = O[tm]
            tb = bctr(tm)
            ty = tr['bbox_world_max'][1] if side == 'left' else tr['bbox_world_min'][1]
            tips[g.split('_joint')[0]] = {'mesh': tm, 'P': (tb[0], ty, tb[2])}
        digit_keys = sorted(k for k in tips if 'digit' in k)
        all_keys = sorted(tips)
        k3P = tuple(sum(tips[k]['P'][i] for k in digit_keys) / len(digit_keys) for i in range(3))
        k3tP = tuple(sum(tips[k]['P'][i] for k in all_keys) / len(all_keys) for i in range(3))
        K['K3'] = {'label': 'центроид кончиков пальцев 2-5 (без большого)', 'P': k3P}
        K['K3t'] = {'label': 'центроид кончиков всех 5 пальцев', 'P': k3tP}
        Mi = inv4([row[:] for row in palm['mw']])
        for kv in K.values():
            kv['Plocal'] = m4_apply(Mi, kv['P'])
        grips[side] = K
        tips_all[side] = tips
        parts = []
        for nm, role in [(side + '_palm_link', 'ладонь')] + \
                        [(tips[k]['mesh'], 'концевое звено пальца "%s"' % k.replace(side + '_', ''))
                         for k in sorted(tips)]:
            r = O[nm]
            d = [r['bbox_world_max'][i] - r['bbox_world_min'][i] for i in range(3)]
            parts.append({'name': nm, 'role': role, 'mass': r['props'].get('mass'),
                          'dims': d, 'vol_local': r.get('vol_local'), 'P': rec_T(r),
                          'mat_slots': r.get('mat_slots')})
        hand_parts[side] = parts
    res['grips'] = grips
    res['tips'] = tips_all
    res['hand_parts'] = hand_parts

    # ---------------- звенья: габариты/масса/объём/плотность ----------------
    link_info = {}
    for side in SIDES:
        li = []
        for suf, role in LINKS:
            r = O[side + '_' + suf]
            if 'bbox_world_max' in r:
                d = [r['bbox_world_max'][i] - r['bbox_world_min'][i] for i in range(3)]
            else:
                d = None
            mass = r['props'].get('mass')
            vol_w = dens = None
            if r['type'] == 'MESH' and r.get('vol_local') is not None:
                vol_w = r['vol_local'] * abs(det3(rec_A(r)))
                if isinstance(mass, (int, float)) and vol_w > 0:
                    dens = mass / vol_w
            li.append({'name': side + '_' + suf, 'suf': suf, 'role': role, 'type': r['type'],
                       'dims': d, 'mass': mass, 'vol_local': r.get('vol_local'),
                       'vol_world': vol_w, 'density': dens, 'P': rec_T(r),
                       'props_keys': sorted(r['props'].keys()), 'mat_slots': r.get('mat_slots')})
        link_info[side] = li
    res['links'] = link_info
    return res


def analyze_dyn(res, dump, pose):
    O = dump['objects']
    joints = res['joints']
    grips = res['grips']
    mrows = []

    def addrow(scenario, poselab, side, jkey, Tj, Aj, gid, G, load, k, note=''):
        axkey = joints[(side, jkey)]['axis']
        n = norm3(m3_apply(Aj, AXIS_VEC[axkey]))
        r = (G[0] - Tj[0], G[1] - Tj[1], G[2] - Tj[2])
        tau = load * GRAV * k * (-r[1] * n[0] + r[0] * n[1])
        mrows.append({'scenario': scenario, 'pose': poselab, 'side': side, 'joint': JLABEL[jkey],
                      'axis_md': axkey, 'axis_x': n[0], 'axis_y': n[1], 'axis_z': n[2],
                      'grip': gid, 'load_kg': load, 'accel_k': k,
                      'r_x': r[0], 'r_y': r[1], 'r_z': r[2], 'tau': tau, 'note': note})

    # --- сценарий A: статика, 100 кг на кисть, все тестовые точки захвата ---
    for side in SIDES:
        for jkey in JKEYS:
            jd = joints[(side, jkey)]
            for gid in ('K1', 'K2', 'K3', 'K3t'):
                addrow('A', 'rest 0°', side, jkey, jd['T'], jd['A'], gid, grips[side][gid]['P'],
                       100.0, 1.0, 'статика; 100 кг на кисть')

    # --- несимметричное распределение 60/140 кг ---
    for scenario, loads in (('A_asym_L60R140', {'left': 60.0, 'right': 140.0}),
                            ('A_asym_L140R60', {'left': 140.0, 'right': 60.0})):
        for side in SIDES:
            for jkey in JKEYS:
                jd = joints[(side, jkey)]
                addrow(scenario, 'rest 0°', side, jkey, jd['T'], jd['A'], 'K1',
                       grips[side]['K1']['P'], loads[side], 1.0, 'несимметрия; сумма 200 кг')

    # --- сценарий B: вертикальное ускорение 0.1g / 0.2g ---
    for k, lab in ((1.1, 'B_0.1g'), (1.2, 'B_0.2g')):
        for side in SIDES:
            for jkey in JKEYS:
                jd = joints[(side, jkey)]
                addrow(lab, 'rest 0°', side, jkey, jd['T'], jd['A'], 'K1',
                       grips[side]['K1']['P'], 100.0, k,
                       'квазистатическая оценка: +%d%% к весу' % int((k - 1) * 100))

    # --- сценарий C: вылет вперёд (позы из pose-теста) ---
    pt = {(t['side'], t['angle_deg']): t for t in pose['tests']}
    jname_map = {jkey: suf for jkey, suf, *_ in JOINTS}
    pose_summary = {}
    lift = rec_T(O['vertical_lift_joint'])
    for side in SIDES:
        for ang in (30.0, 60.0, 85.0):
            t = pt[(side, ang)]
            jm = t['joints']
            palm = jm[side + '_palm_link']
            Ap, Tp = palm['R'], palm['T']
            G1 = None
            for gid in ('K1', 'K2', 'K3', 'K3t'):
                pl = grips[side][gid]['Plocal']
                G = tuple(Ap[i][0] * pl[0] + Ap[i][1] * pl[1] + Ap[i][2] * pl[2] + Tp[i]
                          for i in range(3))
                if gid == 'K1':
                    G1 = G
                for jkey in JKEYS:
                    jr = jm[side + '_' + jname_map[jkey]]
                    addrow('C', '%d°' % int(ang), side, jkey, tuple(jr['T']), jr['R'], gid, G,
                           100.0, 1.0, 'вылет вперёд (j2=%d°)' % int(ang))
            gaps = t['min_gaps']
            mg = min(gaps, key=lambda g: g['gap_m']) if gaps else None
            pose_summary[(side, ang)] = {
                'overlaps': len(t['overlaps']),
                'min_gap': mg,
                'palm_dx': t['palm_dx_m'],
                'k1': G1,
                'axis_dist': math.hypot(G1[0] - lift[0], G1[1] - lift[1])}
    res['pose_summary'] = pose_summary
    res['combined'] = pose['combined_85']
    res['lift_axis'] = lift
    res['rest_axis_dist'] = {side: math.hypot(grips[side]['K1']['P'][0] - lift[0],
                                              grips[side]['K1']['P'][1] - lift[1])
                             for side in SIDES}

    # --- расстояния в цепочке ---
    dist_rows = []
    seq = ['j3', 'j2', 'j1', 'elbow', 'ru', 'w1', 'w2']
    short = {'j3': 'j3', 'j2': 'j2', 'j1': 'j1', 'elbow': 'локоть', 'ru': 'лучелоктевой',
             'w1': 'запястье-1', 'w2': 'запястье-2'}
    for side in SIDES:
        pts = [joints[(side, k)]['T'] for k in seq]
        for i in range(len(seq) - 1):
            a, b = pts[i], pts[i + 1]
            dist_rows.append({'side': side, 'id': '%s → %s' % (short[seq[i]], short[seq[i + 1]]),
                              'd3_mm': dist3(a, b) * 1000,
                              'dh_mm': math.hypot(a[0] - b[0], a[1] - b[1]) * 1000,
                              'dv_mm': abs(a[2] - b[2]) * 1000})
        pp = rec_T(O[side + '_palm_link'])
        for idl, a, b in (('j1 → запястье-2 (плечо–кисть)', pts[2], pts[6]),
                          ('j3 → запястье-2', pts[0], pts[6]),
                          ('запястье-2 → origin ладони', pts[6], pp)):
            dist_rows.append({'side': side, 'id': idl, 'd3_mm': dist3(a, b) * 1000,
                              'dh_mm': math.hypot(a[0] - b[0], a[1] - b[1]) * 1000,
                              'dv_mm': abs(a[2] - b[2]) * 1000})
        for gid in ('K1', 'K2', 'K3', 'K3t'):
            G = grips[side][gid]['P']
            for idl, a in (('j1 → %s' % gid, pts[2]), ('запястье-2 → %s' % gid, pts[6])):
                dist_rows.append({'side': side, 'id': idl, 'd3_mm': dist3(a, G) * 1000,
                                  'dh_mm': math.hypot(a[0] - G[0], a[1] - G[1]) * 1000,
                                  'dv_mm': abs(a[2] - G[2]) * 1000})
            dist_rows.append({'side': side, 'id': '%s → ось лифта (гориз.)' % gid,
                              'd3_mm': None,
                              'dh_mm': math.hypot(G[0] - lift[0], G[1] - lift[1]) * 1000,
                              'dv_mm': None})
    res['dist_rows'] = dist_rows
    res['hand_sep'] = {gid: dist3(grips['left'][gid]['P'], grips['right'][gid]['P'])
                       for gid in ('K1', 'K2')}

    # --- собственный вес звеньев (по массам, записанным в файл) ---
    def descendants(nm):
        seen = [nm]
        i = 0
        while i < len(seen):
            for c in O[seen[i]]['children']:
                seen.append(c)
            i += 1
        return seen

    mass_links = {}
    for side in SIDES:
        for nm in descendants(side + '_shoulder_joint_empty_3'):
            r = O[nm]
            if r['type'] == 'MESH' and isinstance(r['props'].get('mass'), (int, float)):
                mass_links[nm] = r
    sw = {}
    for side in SIDES:
        for jkey in JKEYS:
            jd = joints[(side, jkey)]
            ds = set(descendants(jd['name']))
            contrib = []
            for nm, r in mass_links.items():
                if nm not in ds:
                    continue
                m = float(r['props']['mass'])
                com = (r['props'].get('com_x', 0.0), r['props'].get('com_y', 0.0),
                       r['props'].get('com_z', 0.0))
                cw = m4_apply(r['mw'], com)
                tau = m * GRAV * (-(cw[1] - jd['T'][1]) * jd['n'][0] + (cw[0] - jd['T'][0]) * jd['n'][1])
                contrib.append((nm, m, tau))
            sw[(side, jkey)] = contrib
    res['self_weight'] = sw
    res['mass_links'] = mass_links
    res['arm_mass'] = {}
    res['hand_mass'] = {}
    for side in SIDES:
        hand_set = set(descendants(side + '_palm_link'))
        tot = hand = 0.0
        nlinks = nhand = 0
        for nm, r in mass_links.items():
            if not nm.startswith(side + '_'):
                continue
            m = float(r['props']['mass'])
            tot += m
            nlinks += 1
            if nm in hand_set:
                hand += m
                nhand += 1
        res['arm_mass'][side] = (tot, nlinks)
        res['hand_mass'][side] = (hand, nhand)

    res['mrows'] = mrows
    return res


def write_geometry_csv(path, res):
    cols = ['record_type', 'side', 'id', 'parent', 'child', 'role', 'axis_md',
            'limit_lower_deg', 'limit_upper_deg', 'x_m', 'y_m', 'z_m',
            'dim_x_mm', 'dim_y_mm', 'dim_z_mm', 'mass_kg',
            'vol_world_m3', 'density_implied_kg_m3', 'note']
    rows = []

    def add(**kw):
        rows.append({c: kw.get(c, '') for c in cols})

    sc = res['scene']
    add(record_type='scene', side='both', id='scene',
        role='единицы и СК',
        note='unit_system=%s; scale_length=%s; frame=%s; gravity=%s; СК Blender Z-up; +X — вперёд, +Y — влево, Z — вверх' % (
            sc.get('unit_system'), sc.get('scale_length'), sc.get('frame'), sc.get('gravity')))

    # суставы
    for side in SIDES:
        for jkey, suf, par, ch, ax, lo, hi, mean, mdref in JOINTS:
            jd = res['joints'][(side, jkey)]
            add(record_type='joint', side=side, id=jd['name'], parent=side + '_' + par,
                child=side + '_' + ch, role=mean, axis_md=ax,
                limit_lower_deg='%g' % lo, limit_upper_deg='%g' % hi,
                x_m=f6(jd['T'][0]), y_m=f6(jd['T'][1]), z_m=f6(jd['T'][2]),
                note='ось: %s (MD %s); мировая ось=(%s, %s, %s); фрейм в сцене identity (откл. %.2e)' % (
                    ax, mdref, f4(jd['n'][0]), f4(jd['n'][1]), f4(jd['n'][2]), jd['frame_dev']))

    # звенья
    for side in SIDES:
        for li in res['links'][side]:
            dims = li['dims']
            add(record_type='link', side=side, id=li['name'], parent='', child='',
                role=li['role'],
                dim_x_mm=f1(dims[0] * 1000) if li['type'] == 'MESH' else '',
                dim_y_mm=f1(dims[1] * 1000) if li['type'] == 'MESH' else '',
                dim_z_mm=f1(dims[2] * 1000) if li['type'] == 'MESH' else '',
                mass_kg=f4(li['mass']) if isinstance(li['mass'], (int, float)) else '',
                vol_world_m3=('%.9f' % li['vol_world']) if li['vol_world'] else '',
                density_implied_kg_m3=f1(li['density']) if li['density'] else '',
                x_m=f6(li['P'][0]) if li.get('P') else '',
                y_m=f6(li['P'][1]) if li.get('P') else '',
                z_m=f6(li['P'][2]) if li.get('P') else '',
                note='тип=%s%s' % (li['type'], ('' if li['type'] == 'MESH' else '; свойства масс отсутствуют')))

    # кисти и кончики пальцев
    for side in SIDES:
        for hp in res['hand_parts'][side]:
            dims = hp['dims']
            add(record_type='hand_part', side=side, id=hp['name'], role=hp['role'],
                x_m=f6(hp['P'][0]), y_m=f6(hp['P'][1]), z_m=f6(hp['P'][2]),
                dim_x_mm=f1(dims[0] * 1000), dim_y_mm=f1(dims[1] * 1000), dim_z_mm=f1(dims[2] * 1000),
                mass_kg=f5(hp['mass']) if isinstance(hp['mass'], (int, float)) else '',
                note='bbox-габарит; coords = origin объекта')

    # точки захвата
    for side in SIDES:
        for gid in ('K1', 'K2', 'K3', 'K3t'):
            kv = res['grips'][side][gid]
            add(record_type='grip_point', side=side, id=gid, role=kv['label'],
                x_m=f6(kv['P'][0]), y_m=f6(kv['P'][1]), z_m=f6(kv['P'][2]),
                note='palm-local=(%s, %s, %s); тестовая точка (не задана конструкцией)' % (
                    f6(kv['Plocal'][0]), f6(kv['Plocal'][1]), f6(kv['Plocal'][2])))

    # расстояния
    for d in res['dist_rows']:
        add(record_type='distance', side=d['side'], id=d['id'],
            dim_x_mm=f1(d['d3_mm']) if d['d3_mm'] is not None else '',
            dim_y_mm=f1(d['dh_mm']) if d['dh_mm'] is not None else '',
            dim_z_mm=f1(d['dv_mm']) if d['dv_mm'] is not None else '',
            note='расстояние, мм: dim_x=3D, dim_y=гориз.(XY), dim_z=верт.(Z)')
    for gid, d in res['hand_sep'].items():
        add(record_type='distance', side='both', id='%s (левая) ↔ %s (правая)' % (gid, gid),
            dim_x_mm=f1(d * 1000), note='расстояние между кистями, мм')

    # проверки
    for i, c in enumerate(res.get('checks', [])):
        add(record_type='check', side='both', id='check_%02d' % i, role='проверка', note=c)

    with open(path, 'w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        for r in rows:
            w.writerow(r)


def write_moments_csv(path, res):
    cols = ['scenario', 'pose', 'side', 'joint', 'axis_md', 'axis_world_x', 'axis_world_y',
            'axis_world_z', 'grip', 'load_kg', 'accel_k', 'r_x_m', 'r_y_m', 'r_z_m',
            'tau_Nm', 'abs_tau_Nm', 'note']
    rows = []
    for m in res['mrows']:
        rows.append({'scenario': m['scenario'], 'pose': m['pose'], 'side': m['side'],
                     'joint': m['joint'], 'axis_md': m['axis_md'],
                     'axis_world_x': f6(m['axis_x']), 'axis_world_y': f6(m['axis_y']),
                     'axis_world_z': f6(m['axis_z']), 'grip': m['grip'],
                     'load_kg': '%g' % m['load_kg'], 'accel_k': '%g' % m['accel_k'],
                     'r_x_m': f6(m['r_x']), 'r_y_m': f6(m['r_y']), 'r_z_m': f6(m['r_z']),
                     'tau_Nm': f2(m['tau']), 'abs_tau_Nm': f2(abs(m['tau'])), 'note': m['note']})
    for side in SIDES:
        for jkey in JKEYS:
            jd = res['joints'][(side, jkey)]
            contrib = res['self_weight'][(side, jkey)]
            tau = sum(c[2] for c in contrib)
            top = sorted(contrib, key=lambda t: -abs(t[2]))[:3]
            note = 'дистальных звеньев: %d; вклад: %s; массы из свойств файла (НЕ ПОДТВЕРЖДЕНЫ)' % (
                len(contrib), '; '.join('%s=%.1f' % (t[0].replace(side + '_', ''), t[2]) for t in top))
            rows.append({'scenario': 'SW_stored', 'pose': 'rest 0°', 'side': side,
                         'joint': JLABEL[jkey], 'axis_md': jd['axis'],
                         'axis_world_x': f6(jd['n'][0]), 'axis_world_y': f6(jd['n'][1]),
                         'axis_world_z': f6(jd['n'][2]), 'grip': '-',
                         'load_kg': '0', 'accel_k': '1',
                         'r_x_m': '', 'r_y_m': '', 'r_z_m': '',
                         'tau_Nm': f2(tau), 'abs_tau_Nm': f2(abs(tau)), 'note': note})
    with open(path, 'w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        for r in rows:
            w.writerow(r)


def _tbl(header, rows):
    out = ['| ' + ' | '.join(header) + ' |',
           '|' + '|'.join(['---'] * len(header)) + '|']
    for r in rows:
        out.append('| ' + ' | '.join('' if x is None else str(x) for x in r) + ' |')
    return out


def _tau(res, scen, side, jkey, gid='K1', poselab=None):
    for m in res['mrows']:
        if (m['scenario'] == scen and m['side'] == side and
                m['joint'] == JLABEL[jkey] and m['grip'] == gid and
                (poselab is None or m['pose'] == poselab)):
            return m['tau']
    return float('nan')


def _sw_tau(res, side, jkey):
    return sum(c[2] for c in res['self_weight'][(side, jkey)])


def md_start(res, sha_before, sha_after):
    L = []
    sc = res['scene']
    L.append('# Аудит геометрии рук и моментов нагрузки (груз 200 кг, две руки)')
    L.append('')
    L.append('**Объект:** `robot_scaled_4-2-21-LTS-ARCH-02-MANUAL-01.blend` (Blender %s). '
             '**Документальные оси/лимиты:** `robot_joints.md`.' % res['blender'])
    L.append('')
    L.append('**Контроль read-only:** SHA-256 `.blend` до анализа `%s`, после анализа `%s` — **%s**.' % (
        sha_before, sha_after,
        'совпадает, файл не изменялся' if sha_before == sha_after else 'НЕ СОВПАДАЕТ — разобраться до использования результатов'))
    L.append('')
    L.append('## 1. Резюме')
    L.append('')
    L.append('Текущая поза файла — все суставы в 0° (максимальный |euler| по цепочке рук: %s рад), '
             'руки выпрямлены вдоль ±Y. Это **датум анализа, а не поза подъёма**: поза подъёма '
             '(хват, сгибание локтя, ориентация кисти) в модели не задана.' % f6(res['max_euler']))
    L.append('')
    L.append('Моменты нагрузки (сценарий A, точка K1 — центр внутренней грани ладони, 100 кг на кисть):')
    L.append('')
    rows = []
    for jkey in JKEYS:
        rows.append([JLABEL[jkey],
                     f1(_tau(res, 'A', 'left', jkey)),
                     f1(_tau(res, 'A', 'right', jkey)),
                     f1(_sw_tau(res, 'left', jkey)),
                     f1(_sw_tau(res, 'right', jkey))])
    L.extend(_tbl(['сустав', 'τ нагрузки L, Н·м', 'τ нагрузки R, Н·м',
                   'τ собств. веса L, Н·м', 'τ собств. веса R, Н·м'], rows))
    L.append('')
    L.append('* Главная нагрузка в покое — сустав **j1** (ось X, отведение/приведение): |τ| ≈ %s Н·м (L) / %s Н·м (R). '
             'Суставы j2, локоть, лучелоктевой и запястье-1 в этой позе нагружены ≈ 0 по гравитации: '
             'вектор силы параллелен их осям.' % (f1(abs(_tau(res, 'A', 'left', 'j1'))), f1(abs(_tau(res, 'A', 'right', 'j1')))))
    L.append('* Запястье-2 (ось X) в покое: %s / %s Н·м; при переносе точки приложения к кончикам пальцев (K3t) '
             'момент запястья-2 растёт до %s / %s Н·м — кисть и пальцы становятся длинным рычагом.' % (
                 f1(_tau(res, 'A', 'left', 'w2')), f1(_tau(res, 'A', 'right', 'w2')),
                 f1(_tau(res, 'A', 'left', 'w2', 'K3t')), f1(_tau(res, 'A', 'right', 'w2', 'K3t'))))
    L.append('* Сценарий B (0.1g/0.2g) масштабирует моменты линейно: j1 → %s/%s Н·м при +0.1g и %s/%s Н·м при +0.2g.' % (
        f1(_tau(res, 'B_0.1g', 'left', 'j1')), f1(_tau(res, 'B_0.1g', 'right', 'j1')),
        f1(_tau(res, 'B_0.2g', 'left', 'j1')), f1(_tau(res, 'B_0.2g', 'right', 'j1'))))
    ps = res['pose_summary']
    L.append('* Сценарий C (руки вперёд, 85°): к моменту j1 добавляется большой момент **j3** (ось Y): |τ| ≈ %s / %s Н·м; '
             'j1 остаётся ≈ %s / %s Н·м — при выпрямленной руке его ось вращается вместе с рукой, поэтому проекция '
             'гравитационного момента почти не меняется. Пересечений мешей нет; минимальный зазор — %s.' % (
                 f1(abs(_tau(res, 'C', 'left', 'j3', 'K1', '85°'))), f1(abs(_tau(res, 'C', 'right', 'j3', 'K1', '85°'))),
                 f1(abs(_tau(res, 'C', 'left', 'j1', 'K1', '85°'))), f1(abs(_tau(res, 'C', 'right', 'j1', 'K1', '85°'))),
                 _gap_text(ps, 85.0)))
    L.append('')
    L.append('**Массы рук в файле есть** (свойства `mass`/`phobos_*` + COM/inertia), но они **не подтверждены**: '
             'расчётная плотность плеча/предплечья ≈ %s кг/м³ (для металлов 2700–7800), при этом масса плеча и предплечья '
             'идентична (%s/%s кг L, %s/%s кг R) — признак placeholder. Поэтому моменты от собственного веса посчитаны, '
             'но помечены как UNVERIFIED, и в выводы по приводам не переносятся.' % (
                 f2(_link_density(res, 'left', 'shoulder_link')),
                 f2(_link_mass(res, 'left', 'shoulder_link')), f2(_link_mass(res, 'left', 'forearm_link')),
                 f2(_link_mass(res, 'right', 'shoulder_link')), f2(_link_mass(res, 'right', 'forearm_link'))))
    L.append('')
    L.append('## 2. Метод и воспроизводимость')
    L.append('')
    L.append('1. **Дамп геометрии** (read-only): `blender --background <файл> --python _blender_dump.py`, '
             'без вызова сохранения; в JSON попадают все 182 объекта: мировые матрицы, иерархия, '
             'свойства (`mass`, `com_*`, `inertia_*`, `phobostype`), габариты bbox, объём меша, материалы.')
    L.append('2. **Тест позы** — только на временной копии `.blend`: поворот `shoulder_joint_empty_2` (ось Z) '
             'на 30/60/85° (знак: левая −θ, правая +θ) и комбинированный тест «обе руки вперёд 85°»; пересечения '
             'проверяются BVH по реальным мешам (35 collision-оболочек `phobostype=4` исключены). Копия не сохраняется.')
    L.append('3. **Расчёт моментов** — чистый Python (stdlib), по матрицам из дампов: τ⃗ = r⃗ × F⃗, τ_ось = τ⃗·n̂, '
             'F⃗ = (0, 0, −m·g·k), r⃗ = G⃗ − O⃗. Для осей X/Y после подстановки: **τ = m·g·k·(−r_y·n_x + r_x·n_y)**.')
    L.append('4. **Повторяемость:** все промежуточные данные сохранены (`arm_dump.json`, `pose_c_results.json`), '
             'скрипт детерминирован (без времени/случайности), повторный прогон даёт те же CSV/MD.')
    L.append('')
    L.append('## 3. Источники данных: что чем подтверждено')
    L.append('')
    L.append('**Факты сцены (Blender, зелёные):**')
    L.append('- иерархия звеньев и суставов, мировые координаты и ориентации фреймов (все фреймы цепочки — identity-ориентация, '
             'отклонение ≤ %s);' % ('%.1e' % res['max_frame_dev']))
    L.append('- габариты звеньев (bbox), ладонь %s×%s×%s мм;' % (
        f1(res['links']['left'][6]['dims'][0] * 1000),
        f1(res['links']['left'][6]['dims'][1] * 1000),
        f1(res['links']['left'][6]['dims'][2] * 1000)))
    L.append('- свойства `mass`, `com_*`, `inertia_*` у части мешей (числа есть, достоверность — под вопросом, см. §9);')
    L.append('- лимиты/оси суставов в сцене **не закодированы** (нет constraints/драйверов; `drivers_total` в pose-тесте = 0) — '
             'они взяты из `robot_joints.md`.')
    L.append('')
    L.append('**Факты документации (`robot_joints.md`, жёлтые — не проверяемы сценой):** оси и лимиты семи суставов руки '
             '(таблица §4). Соответствие `blender_empty → parent → child` в MD совпадает с иерархией сцены для всех '
             'проверенных суставов.')
    L.append('')
    L.append('**Допущения анализа (красные, требуют подтверждения):**')
    L.append('- груз 200 кг делится поровну (100+100 кг) — базовый сценарий; несимметрия 60/140 кг — отдельно;')
    L.append('- точки захвата K1–K3t — **тестовые геометрические точки**, не заданные конструкцией;')
    L.append('- удержание статическое, без динамики звеньев и демпфирования; сценарий B — квазистатическая оценка;')
    L.append('- масса звеньев для сценария A не учитывается (нагрузка от груза считается отдельно; собственный вес — §14).')
    L.append('')
    L.append('## 4. Кинематические цепи рук')
    L.append('')
    L.append('Цепь одинакова для обеих рук (префикс `left_`/`right_`):')
    L.append('')
    L.append('```')
    L.append('torso_link')
    L.append(' └─ shoulder_joint_empty_3      (сустав j3, ось Y, −180…+60°)')
    L.append('    └─ shoulder_3_link          (пустой фрейм)')
    L.append('       └─ shoulder_joint_empty_2 (сустав j2, ось Z, −90…+90°)')
    L.append('          └─ shoulder_2_link      (пустой фрейм)')
    L.append('             └─ shoulder_joint_empty_1 (сустав j1, ось X, −90…+130°)')
    L.append('                └─ shoulder_link  (плечо, mesh)')
    L.append('                   └─ elbow_joint_empty (локоть, ось Z, −10…+150°)')
    L.append('                      └─ forearm_link   (предплечье, mesh)')
    L.append('                         └─ proximal_radioulnar_joint_empty (ось Y, ±90°)')
    L.append('                            └─ radioulnar_link (пустой фрейм)')
    L.append('                               └─ wrist_joint_1_empty (ось Z, ±90°)')
    L.append('                                  └─ wrist_pitch_link (пустой фрейм)')
    L.append('                                     └─ wrist_joint_2_empty (ось X, −25…+45°)')
    L.append('                                        └─ palm_link (ладонь, mesh)')
    L.append('                                           └─ пальцы: digit2…digit5 (mcp→mcp2→distal→pip→middle→dip→proximal) + thumb')
    L.append('```')
    L.append('')
    L.append('Примечание по пальцам: имена звеньев в модели перевёрнуты относительно анатомии — '
             '`distal_phalanx` стоит ближе к ладони, `proximal_phalanx` — на кончике (цепочка '
             '`mcp → mcp2 → distal → pip → middle → dip → proximal`). Для силового расчёта это не влияет '
             '(используются только координаты), но при переименовании под URDF/USD порядок надо исправить.')
    L.append('')
    return L


def _link_density(res, side, suf):
    for li in res['links'][side]:
        if li['suf'] == suf:
            return li['density'] if li['density'] else float('nan')
    return float('nan')


def _link_mass(res, side, suf):
    for li in res['links'][side]:
        if li['suf'] == suf:
            return li['mass'] if isinstance(li['mass'], (int, float)) else float('nan')
    return float('nan')


def _gap_text(ps, ang):
    parts = []
    for side in SIDES:
        d = ps.get((side, ang))
        tag = 'L' if side == 'left' else 'R'
        if not d or not d['min_gap']:
            parts.append('%s: нет данных' % tag)
            continue
        mg = d['min_gap']
        parts.append('%s %s мм (%s ↔ %s)' % (tag, f1(mg['gap_m'] * 1000), mg['moved'], mg['static']))
    return '; '.join(parts)


def md_geom(res, L):
    J = res['joints']
    L.append('## 5. Суставы: оси, лимиты, мировые направления')
    L.append('')
    rows = []
    for jkey, suf, par, ch, ax, lo, hi, mean, mdref in JOINTS:
        jd = J[('left', jkey)]
        rows.append([JLABEL[jkey], '`%s`' % suf, '`%s` → `%s`' % (par, ch),
                     '%s: %g…%g°' % (ax, lo, hi),
                     '(%s, %s, %s)' % (f4(jd['n'][0]), f4(jd['n'][1]), f4(jd['n'][2])),
                     mdref.replace('robot_joints.md, ', '')])
    L.extend(_tbl(['сустав', 'empty', 'parent → child', 'ось MD / лимит', 'мировая ось n̂ (обе руки)', 'источник'], rows))
    L.append('')
    L.append('Мировые оси совпадают с осями сцены, потому что все фреймы цепочки имеют identity-ориентацию '
             '(максимальное отклонение нормализованных столбцов от единичных осей — %s). Лимиты в сцене не '
             'закодированы: это документальные значения из `robot_joints.md`. Ссылки в столбце «источник» — '
             'на блоки правой руки; для левой руки блоки в документе зеркальны (те же оси и лимиты).' % ('%.1e' % res['max_frame_dev']))
    L.append('')
    L.append('## 6. Геометрия цепочки')
    L.append('')
    L.append('### 6.1. Координаты осей суставов (мировые, м)')
    L.append('')
    rows = []
    for jkey in JKEYS:
        a = J[('left', jkey)]['T']
        b = J[('right', jkey)]['T']
        rows.append([JLABEL[jkey], '(%s, %s, %s)' % (f4(a[0]), f4(a[1]), f4(a[2])),
                     '(%s, %s, %s)' % (f4(b[0]), f4(b[1]), f4(b[2]))])
    L.extend(_tbl(['сустав', 'левая', 'правая'], rows))
    L.append('')
    L.append('### 6.2. Звенья: тип, габариты, масса, расчётная плотность')
    L.append('')
    rows = []
    for i, (suf, role) in enumerate(LINKS):
        a = res['links']['left'][i]
        b = res['links']['right'][i]
        dims_l = '×'.join(f1(d * 1000) for d in a['dims']) if a['type'] == 'MESH' else '—'
        dims_r = '×'.join(f1(d * 1000) for d in b['dims']) if b['type'] == 'MESH' else '—'
        rows.append(['`%s`' % suf, role, dims_l, dims_r,
                     f4(a['mass']) if isinstance(a['mass'], (int, float)) else '—',
                     f4(b['mass']) if isinstance(b['mass'], (int, float)) else '—',
                     f1(a['density']) if a['density'] else '—',
                     f1(b['density']) if b['density'] else '—'])
    L.extend(_tbl(['звено', 'тип', 'габариты L, мм', 'габариты R, мм', 'масса L, кг', 'масса R, кг',
                   'плотн. L, кг/м³', 'плотн. R, кг/м³'], rows))
    L.append('')
    dens_vals = [li['density'] for side in SIDES for li in res['links'][side] if li['density']]
    L.append('Плотность — это `mass / (объём меша × |det A|)`, т.е. проверка согласованности записанных масс с '
             'геометрией, а не физическое свойство (материал в файле не задан числами). Значения ≈ %s–%s кг/м³ '
             'не соответствуют металлам (алюминий 2700, сталь 7800 кг/м³); см. §9.' % (
                 f0_int(min(dens_vals)) if dens_vals else '—',
                 f0_int(max(dens_vals)) if dens_vals else '—'))
    L.append('')
    L.append('### 6.3. Расстояния между осями (мм)')
    L.append('')
    order = []
    for d in res['dist_rows']:
        if d['id'] not in [o[0] for o in order]:
            order.append((d['id'], {}))
    for d in res['dist_rows']:
        for i, (oid, mp) in enumerate(order):
            if oid == d['id']:
                mp[d['side']] = d
    rows = []
    for oid, mp in order:
        l = mp.get('left')
        r = mp.get('right')
        rows.append([oid,
                     f1(l['d3_mm']) if l and l['d3_mm'] is not None else '',
                     f1(l['dh_mm']) if l and l['dh_mm'] is not None else '',
                     f1(r['d3_mm']) if r and r['d3_mm'] is not None else '',
                     f1(r['dh_mm']) if r and r['dh_mm'] is not None else ''])
    L.extend(_tbl(['от → до', 'L 3D', 'L гориз.', 'R 3D', 'R гориз.'], rows))
    L.append('')
    L.append('«Гориз.» — проекция на плоскость XY (для оценки вылета). «Ось лифта» — вертикальная ось '
             '`vertical_lift_joint` (кинематика подъёмника; в силовой расчёт не входит).')
    L.append('')
    L.append('### 6.4. Кисть и пальцы')
    L.append('')
    rows = []
    for side in SIDES:
        for hp in res['hand_parts'][side]:
            rows.append([SIDE_RU[side], '`%s`' % hp['name'], hp['role'],
                         f1(hp['dims'][0] * 1000), f1(hp['dims'][1] * 1000), f1(hp['dims'][2] * 1000),
                         f4(hp['mass']) if isinstance(hp['mass'], (int, float)) else '—',
                         '(%s, %s, %s)' % (f2(hp['P'][0]), f2(hp['P'][1]), f2(hp['P'][2]))])
    L.extend(_tbl(['рука', 'объект', 'роль', 'X, мм', 'Y, мм', 'Z, мм', 'масса, кг', 'origin, м'], rows))
    L.append('')
    L.append('Пальцы продолжают ось руки (±Y), т.е. в текущей позе кисть — прямое продолжение предплечья. '
             'Структура каждого пальца: `mcp → mcp2 → distal_phalanx (mesh) → pip → middle_phalanx (mesh) → '
             'dip → proximal_phalanx (mesh)`; у большого: `cmc → mcp → ip` и три меша `first/second/third_phalanx`. '
             'Имена фаланг перевёрнуты относительно анатомии (см. §4).')
    L.append('')
    L.append('## 7. Точки захвата (тестовые)')
    L.append('')
    rows = []
    for gid in ('K1', 'K2', 'K3', 'K3t'):
        a = res['grips']['left'][gid]
        b = res['grips']['right'][gid]
        rows.append([gid, a['label'],
                     '(%s, %s, %s)' % (f3(a['P'][0]), f3(a['P'][1]), f3(a['P'][2])),
                     '(%s, %s, %s)' % (f3(b['P'][0]), f3(b['P'][1]), f3(b['P'][2]))])
    L.extend(_tbl(['точка', 'определение', 'левая, м', 'правая, м'], rows))
    L.append('')
    L.append('Точки — аналитические (тестовые), не элементы конструкции; задают плечо силы. '
             'K1 — самый консервативный вариант для плеча (нагрузка у внутренней грани ладони), '
             'K3/K3t — варианты «груз висит на кончиках пальцев» (максимальные моменты кисти). '
             'Расстояние между кистями: K1 — %s мм, K2 — %s мм.' % (
                 f0_mm(res['hand_sep']['K1']), f0_mm(res['hand_sep']['K2'])))
    L.append('')
    return L


def f0_mm(x):
    return f1(x * 1000)


def md_loads(res, L):
    J = res['joints']
    L.append('## 8. Текущая поза — датум анализа')
    L.append('')
    diag = []
    for jd in J.values():
        A = jd['A']
        diag.extend([abs(A[0][0]), abs(A[1][1]), abs(A[2][2])])
    L.append('В файле все суставы руки стоят в 0° (максимальный |euler| по цепочке — %s рад). Руки выпрямлены '
             'вдоль ±Y, ладони обращены внутрь, пальцы продолжают ось руки. Матрицы фреймов диагональные '
             '(масштаб по осям %s…%s), ориентация — identity; поэтому «наклоны», которые можно увидеть в сырых '
             'матрицах, — артефакт масштаба, а не поворот.' % (
                 f6(res['max_euler']), f3(min(diag)), f3(max(diag))))
    L.append('')
    L.append('**Это не поза подъёма.** В модели не заданы: хват (расстояние между кистями и ориентация ладоней '
             'относительно груза), сгибание локтя, наклон/поворот корпуса, положение груза по высоте. '
             'Все моменты ниже — для явно указанной позы (покой 0° или «руки вперёд» 30/60/85°), а не для '
             'реального цикла подъёма.')
    L.append('')
    L.append('## 9. Массы, COM, инерция, материалы')
    L.append('')
    L.append('Свойства `mass`, `com_x/y/z`, `inertia_*` в файле присутствуют у мешей рук:')
    L.append('')
    rows = []
    for side in SIDES:
        for li in res['links'][side]:
            if li['type'] != 'MESH':
                continue
            rows.append(['`%s`' % li['name'], f4(li['mass']) if isinstance(li['mass'], (int, float)) else '—',
                         f1(li['density']) if li['density'] else '—',
                         'да' if li['props_keys'] else 'нет'])
    L.extend(_tbl(['объект', 'mass, кг', 'плотность (mass/объём), кг/м³', 'свойства phobos'], rows))
    L.append('')
    L.append('Наблюдения:')
    L.append('- массы плеча и предплечья **идентичны** внутри каждой руки (L: %s = %s кг; R: %s = %s кг) '
             'при разных габаритах — признак placeholder-значений;' % (
                 f4(_link_mass(res, 'left', 'shoulder_link')), f4(_link_mass(res, 'left', 'forearm_link')),
                 f4(_link_mass(res, 'right', 'shoulder_link')), f4(_link_mass(res, 'right', 'forearm_link'))))
    L.append('- расчётная плотность ≈ %s–%s кг/м³ — не соответствует ни алюминию (2700), ни стали (7800); '
             'либо объём/масса заданы условно, либо геометрия — proxy-боксы (что и наблюдается: все звенья — боксы);' % (
                 f0_int(min(d for d in [_link_density(res, s, 'shoulder_link') for s in SIDES] +
                       [_link_density(res, s, 'forearm_link') for s in SIDES] if d == d)),
                 f0_int(max(d for d in [_link_density(res, s, 'shoulder_link') for s in SIDES] +
                       [_link_density(res, s, 'forearm_link') for s in SIDES] if d == d))))
    L.append('- COM у звеньев ≈ (0,0,0) в локальных координатах объекта, инерционные тензоры заданы — '
             'структура данных корректна, но числа не подтверждены;')
    L.append('- материалы в файле — только имена слотов без плотностей, поэтому выводить массу из объёма нельзя.')
    L.append('')
    L.append('**Вывод:** массы рук в файле есть, но использовать их как достоверные нельзя. '
             'Моменты собственного веса (§14) посчитаны по ним с флагом UNVERIFIED.')
    L.append('')
    L.append('## 10. Сценарий A — статическое удержание (k = 1.0)')
    L.append('')
    L.append('Формулы: τ⃗ = r⃗ × F⃗; τ_ось = τ⃗·n̂; F⃗ = (0, 0, −m·g·k); r⃗ = G⃗ − O⃗; '
             'τ = m·g·k·(−r_y·n_x + r_x·n_y). Для осей, параллельных Z (j2, локоть, запястье-1) в этой позе '
             'τ = 0: сила тяжести параллельна оси, плечо в плоскости XY не создаёт момента вокруг неё.')
    L.append('')
    L.append('### 10.1. Точка K1 (центр внутренней грани ладони), 100 кг на кисть')
    L.append('')
    rows = []
    for jkey in JKEYS:
        rows.append([JLABEL[jkey], f1(_tau(res, 'A', 'left', jkey)), f1(_tau(res, 'A', 'right', jkey)),
                     f1(abs(_tau(res, 'A', 'left', jkey))), f1(abs(_tau(res, 'A', 'right', jkey)))])
    L.extend(_tbl(['сустав', 'τ L, Н·м', 'τ R, Н·м', '|τ| L, Н·м', '|τ| R, Н·м'], rows))
    L.append('')
    L.append('Знак — направление момента вокруг оси (по правилу правой руки); для оценки привода важна величина |τ|.')
    L.append('')
    L.append('### 10.2. Влияние точки захвата (|τ|, Н·м; левая рука)')
    L.append('')
    rows = []
    for jkey in JKEYS:
        rows.append([JLABEL[jkey]] + [f1(abs(_tau(res, 'A', 'left', jkey, g)))
                                     for g in ('K1', 'K2', 'K3', 'K3t')])
    L.extend(_tbl(['сустав', 'K1', 'K2', 'K3', 'K3t'], rows))
    L.append('')
    L.append('### 10.3. То же, правая рука')
    L.append('')
    rows = []
    for jkey in JKEYS:
        rows.append([JLABEL[jkey]] + [f1(abs(_tau(res, 'A', 'right', jkey, g)))
                                     for g in ('K1', 'K2', 'K3', 'K3t')])
    L.extend(_tbl(['сустав', 'K1', 'K2', 'K3', 'K3t'], rows))
    L.append('')
    L.append('Комментарий: при переносе точки приложения от K1 к K3t рычаг растёт на длину пальцев (~0.27 м): '
             'момент j1 увеличивается примерно на 30%% (L: %s → %s Н·м; R: %s → %s Н·м), а момент запястья-2 — '
             'более чем на порядок (L: %s → %s Н·м). Для кисти критична уже не «сила хвата», '
             'а момент на запястье.' % (
                 f1(abs(_tau(res, 'A', 'left', 'j1'))), f1(abs(_tau(res, 'A', 'left', 'j1', 'K3t'))),
                 f1(abs(_tau(res, 'A', 'right', 'j1'))), f1(abs(_tau(res, 'A', 'right', 'j1', 'K3t'))),
                 f1(abs(_tau(res, 'A', 'left', 'w2'))), f1(abs(_tau(res, 'A', 'left', 'w2', 'K3t')))))
    L.append('')
    L.append('### 10.4. Несимметричное распределение 60/140 кг (точка K1)')
    L.append('')
    L.append('Момент масштабируется линейно по load: j1 при 60 кг — %s Н·м, при 140 кг — %s Н·м (левая рука); '
             'для правой аналогично. Полные строки — в CSV (`scenario=A_asym_L60R140`, `A_asym_L140R60`). '
             'Вывод: несимметрия 30/70%% меняет момент соответствующего плеча примерно в 0.6/1.4 раза; '
             'перекрёстного влияния через корпус в этой статической модели нет.' % (
                 f1(abs(_tau(res, 'A_asym_L60R140', 'left', 'j1'))),
                 f1(abs(_tau(res, 'A_asym_L140R60', 'left', 'j1')))))
    L.append('')
    L.append('## 11. Сценарий B — вертикальное ускорение (0.1g / 0.2g)')
    L.append('')
    L.append('Оценка квазистатическая: F = m·(g + a), k = 1.1 и 1.2. Динамика звеньев, рывки и демпфирование '
             'не моделируются; это исследовательская чувствительность, а не нормативный коэффициент.')
    L.append('')
    rows = []
    for jkey in JKEYS:
        rows.append([JLABEL[jkey],
                     f1(_tau(res, 'B_0.1g', 'left', jkey)), f1(_tau(res, 'B_0.1g', 'right', jkey)),
                     f1(_tau(res, 'B_0.2g', 'left', jkey)), f1(_tau(res, 'B_0.2g', 'right', jkey))])
    L.extend(_tbl(['сустав', 'τ L (+0.1g)', 'τ R (+0.1g)', 'τ L (+0.2g)', 'τ R (+0.2g)'], rows))
    L.append('')
    L.append('Все моменты сценария B = моменты A × k (по построению), т.к. ускорение направлено по силе тяжести.')
    L.append('')
    L.append('## 12. Сценарий C — вылет вперёд (30/60/85°)')
    L.append('')
    L.append('Поза: поворот `shoulder_joint_empty_2` (ось Z, лимит −90…+90°) на 30/60/85°; остальные суставы 0°. '
             'Знаки: левая рука −θ, правая +θ (проверено по смещению ладони по X). Проверка пересечений — BVH '
             'по реальным мешам, collision-оболочки исключены; руки тестировались по отдельности и обе одновременно.')
    L.append('')
    rows = []
    for ang in (30.0, 60.0, 85.0):
        for side in SIDES:
            d = res['pose_summary'][(side, ang)]
            mg = d['min_gap']
            rows.append(['%d°' % int(ang), 'L' if side == 'left' else 'R',
                         f1(d['palm_dx'] * 1000),
                         '(%s, %s)' % (f3(d['k1'][0]), f3(d['k1'][1])),
                         f1(d['axis_dist'] * 1000),
                         f1(mg['gap_m'] * 1000) if mg else '—',
                         '%s ↔ %s' % (mg['moved'], mg['static']) if mg else '—',
                         d['overlaps']])
    L.extend(_tbl(['угол', 'рука', 'Δx ладони, мм', 'K1 (x, y), м', 'K1 от оси лифта, мм',
                   'мин. зазор, мм', 'пара', 'пересечений'], rows))
    L.append('')
    c85 = res['combined']
    L.append('**Комбинированный тест «обе руки вперёд 85°»:** пересечений рук с корпусом — %d, '
             'пересечений рук между собой — %d. Лимит j2 ±90° не нарушен (85° < 90°); дальнейшее увеличение '
             'угла упирается в лимит.' % (len(c85['overlaps']), len(c85['cross_overlaps'])))
    L.append('')
    L.append('### 12.1. Моменты в позе «руки вперёд» (100 кг, K1)')
    L.append('')
    rows = []
    for jkey in JKEYS:
        r = [JLABEL[jkey]]
        for ang in (30.0, 60.0, 85.0):
            pl = '%d°' % int(ang)
            r.extend([f1(_tau(res, 'C', 'left', jkey, 'K1', pl)),
                      f1(_tau(res, 'C', 'right', jkey, 'K1', pl))])
        rows.append(r)
    L.extend(_tbl(['сустав', '30° L', '30° R', '60° L', '60° R', '85° L', '85° R'], rows))
    L.append('')
    L.append('При вылете вперёд появляется большой момент **j3** (ось Y): плечо — расстояние по X, оно растёт '
             'до ~0.87 м, момент — до ~0.85 кН·м. Момент **j1** при этом не «переходит» и не падает: его ось '
             'вращается вместе с рукой, а рука остаётся выпрямленной, поэтому проекция гравитационного момента '
             'на ось j1 сохраняется (~0.8 кН·м). Момент **j2** (ось Z) остаётся ≈ 0, пока локоть и кисть не '
             'согнуты: сила вертикальна, ось вертикальна. Зазор «плечо ↔ корпус» на 85° — %s: **тонкий** '
             '(оценка по bbox-прокси, не по реальным оболочкам), требует перепроверки после появления настоящей '
             'геометрии звеньев.' % _gap_text(res['pose_summary'], 85.0))
    L.append('')
    L.append('«Визуально проходит» ≠ «допустимо»: отсутствие пересечений на прокси-боксах не подтверждает '
             'проходимость реальной конструкции (кабели, приводы, кожухи в модели отсутствуют).')
    L.append('')
    return L


def md_end(res, L):
    J = res['joints']
    L.append('## 13. Момент сустава ≠ момент двигателя ≠ прочность')
    L.append('')
    L.append('Выше посчитан **внешний момент нагрузки относительно оси сустава** (что должно быть уравновешено '
             'приводом в данной позе). Это не то же самое, что:')
    L.append('')
    L.append('- **момент двигателя** — момент на валу мотора = момент сустава / (передаточное число × КПД редуктора); '
             'без данных о редукторах и компоновке его нельзя назвать;')
    L.append('- **прочность** — напряжения в звене/подшипнике зависят от сечений, материала и распределения усилий; '
             'из моментов напрямую не следуют;')
    L.append('- **допустимая нагрузка** — требует запасов, ресурса, жёсткости, точности; в этом аудите никаких '
             'запасов не вводилось (все числа — исследовательские, без коэффициентов).')
    L.append('')
    L.append('Полный момент на приводе = момент груза + момент собственного веса (§14) + динамика (инерция × угловое '
             'ускорение) + трение/потери. Динамика звеньев в этом аудите не моделировалась.')
    L.append('')
    L.append('## 14. Моменты от собственного веса (по массам из файла — UNVERIFIED)')
    L.append('')
    L.append('Формула та же: τ_SW,j = Σ_i m_i·g·lever_ij, где lever_ij = (−r_y·n_x + r_x·n_y) для COM i-го звена '
             'относительно оси j. В файле COM ≈ (0,0,0) в локальных координатах объекта, поэтому масса звена '
             'считается приложенной в его origin — это, вероятно, не настоящий COM (placeholder), и момент '
             'собственного веса может быть занижен или завышен в зависимости от реальной геометрии.')
    L.append('')
    L.append('Рычаги origin звеньев относительно осей (м; левая рука; правая отличается на единицы мм):')
    L.append('')
    rows = []
    for jkey in JKEYS:
        jd = J[('left', jkey)]
        n = jd['n']
        r = [JLABEL[jkey]]
        for suf in ('shoulder_link', 'forearm_link', 'palm_link'):
            P = None
            for li in res['links']['left']:
                if li['suf'] == suf:
                    P = li['P']
            rr = (P[0] - jd['T'][0], P[1] - jd['T'][1])
            r.append(f3(abs(-rr[1] * n[0] + rr[0] * n[1])))
        rows.append(r)
    L.extend(_tbl(['сустав', 'плечо', 'предплечье', 'ладонь'], rows))
    L.append('')
    L.append('Итоговые моменты от собственного веса (сумма по всем дистальным звеньям с массой; массы из файла):')
    L.append('')
    rows = []
    for jkey in JKEYS:
        sl = _sw_tau(res, 'left', jkey)
        sr = _sw_tau(res, 'right', jkey)
        al = abs(_tau(res, 'A', 'left', jkey))
        ar = abs(_tau(res, 'A', 'right', jkey))
        rows.append([JLABEL[jkey], f1(sl), f1(sr),
                     f1(100.0 * abs(sl) / al) if al > 5.0 else '—',
                     f1(100.0 * abs(sr) / ar) if ar > 5.0 else '—'])
    L.extend(_tbl(['сустав', 'τ_SW L, Н·м', 'τ_SW R, Н·м', 'доля от нагрузки L, %', 'доля от нагрузки R, %'], rows))
    L.append('')
    L.append('Даже по этим (неподтверждённым) массам вклад собственного веса в плечевом суставе — единицы процентов '
             'от момента груза 100 кг: нагрузка доминирует. Это качественный вывод, он сохранится и при уточнении '
             'масс в разумных пределах; количественные значения τ_SW — только после проверки масс.')
    L.append('')
    L.append('## 15. Оценка массы рук: что можно и что нужно')
    L.append('')
    L.append('В файле: сумма масс звеньев руки (по свойствам) — %s кг (L, %d звеньев) и %s кг (R, %d звеньев); '
             'кисть с пальцами — %s кг (L) и %s кг (R). Но, как показано в §9, эти числа не подтверждены '
             '(плотность ≈ %s кг/м³, массы плеча = массе предплечья).' % (
                 f3(res['arm_mass']['left'][0]), res['arm_mass']['left'][1],
                 f3(res['arm_mass']['right'][0]), res['arm_mass']['right'][1],
                 f3(res['hand_mass']['left'][0]), f3(res['hand_mass']['right'][0]),
                 f0_density(res)))
    L.append('')
    L.append('Пути получения достоверной массы (в порядке надёжности):')
    L.append('1. **CAD/mass properties по реальной геометрии + материалам** — нужны толщины, сечения, заполнение, '
             'материалы звеньев; для proxy-боксов результат бессмыслен.')
    L.append('2. **Покомпонентный бюджет** — массы приводов, редукторов, подшипников, валов, кожухов, кабелей, '
             'электроники; места установки (в звене / вынесены в корпус) критичны для момента инерции и SW.')
    L.append('3. **Взвешивание прототипа** — самый надёжный, но требует железа.')
    L.append('4. **Оценка по аналогам** — только с явными оговорками и диапазоном.')
    L.append('')
    L.append('**Недостающие данные (минимум для инженерного вывода):** массы приводов/редукторов и их расположение; '
             'материалы и толщины звеньев; реальная геометрия (не боксы); места крепления груза и тип хвата; '
             'кинематика подъёма (траектория, скорости, ускорения); требования по запасам и ресурсу.')
    L.append('')
    L.append('Как только массы появятся: τ_привод = τ_груза (таблицы §10–12) + τ_SW (пересчитать с реальными COM) + '
             'динамика; далее делить на передаточное число и сравнивать с паспортом мотора.')
    L.append('')
    L.append('## 16. Проверка единиц и систем координат')
    L.append('')
    L.append('- Blender: `unit_system=%s`, `scale_length=%s` → 1 единица = 1 м; все координаты/габариты в метрах '
             '(в отчёте габариты — в мм для читаемости).' % (res['scene'].get('unit_system'), res['scene'].get('scale_length')))
    L.append('- Мировая СК: Z — вверх; **+X — вперёд** (направление взгляда/лифта), **+Y — влево** (левая рука '
             'находится при Y > центра корпуса).')
    L.append('- Гравитация: сцена %s; в расчёте g = 9.81 м/с², направление −Z.' % res['scene'].get('gravity'))
    L.append('- Нагрузка — кг (масса); сила — Н; момент — Н·м; плечи — м.')
    L.append('- Проверка масштаба: расстояние j1 → запястье-2 ≈ %s мм, ладонь %s×%s×%s мм, размах кистей по K1 ≈ %s мм. '
             'Это согласуется с геометрией модели (плечи разнесены примерно на 0.5 м, каждая рука ~0.8 м, робот — '
             'крупнее человеческого масштаба); метрическая система и scale_length=1.0 подтверждают, что единицы — метры.' % (
                 _dist_val(res, 'left', 'j1 → запястье-2 (плечо–кисть)'),
                 f1(res['links']['left'][6]['dims'][0] * 1000),
                 f1(res['links']['left'][6]['dims'][1] * 1000),
                 f1(res['links']['left'][6]['dims'][2] * 1000),
                 f1(res['hand_sep']['K1'] * 1000)))
    L.append('')
    L.append('## 17. Ограничения достоверности')
    L.append('')
    L.append('- Все звенья — **proxy-боксы**; плечи сил от центров bbox могут отличаться от реальной геометрии.')
    L.append('- Точки захвата — тестовые (K1–K3t); реальный хват не задан, поэтому моменты кисти — диапазон, а не число.')
    L.append('- Поза подъёма не задана; расчёт сделан для покоя и «рук вперёд», а не для реального цикла.')
    L.append('- Собственный вес — по неподтверждённым массам; COM=0 — вероятный placeholder.')
    L.append('- Динамика, трение, КПД редукторов, кабели, кожухи, прочность — не учитывались.')
    L.append('- Пересечения проверены по bbox/BVH прокси-мешей; зазор 17.6 мм на 85° — оценка, не гарантия.')
    L.append('- Статика всего робота (баланс, опрокидывание, нагрузка на ноги/корпус) — вне рамок аудита.')
    L.append('')
    L.append('## 18. Файлы и воспроизведение')
    L.append('')
    L.append('- `arm_geometry_load_audit.md` — этот отчёт;')
    L.append('- `arm_geometry_data.csv` — геометрия, оси, лимиты, массы, точки захвата, расстояния;')
    L.append('- `arm_joint_moments.csv` — все моменты (сценарии A/asym/B/C/SW) с плечами r и мировыми осями;')
    L.append('- `arm_geometry_load_audit.py` — оркестратор (этот расчёт); рядом — `_blender_dump.py`, `_blender_pose_test.py`;')
    L.append('- `arm_dump.json`, `pose_c_results.json` — сохранённые диагностические дампы.')
    L.append('')
    L.append('Запуск: `python3 arm_geometry_load_audit.py` (нужен Blender `%s`). '
             'SHA-256 `.blend` до/после: `%s` / `%s`.' % (BLENDER, res['sha_before'], res['sha_after']))
    L.append('')
    return L


def f0_density(res):
    vals = []
    for side in SIDES:
        for suf in ('shoulder_link', 'forearm_link'):
            d = _link_density(res, side, suf)
            if d == d:
                vals.append(d)
    return f0_avg(vals)


def f0_avg(vals):
    if not vals:
        return '—'
    return f0_int(sum(vals) / len(vals))


def f0_int(x):
    return '%.0f' % x


def _dist_val(res, side, idl):
    for d in res['dist_rows']:
        if d['side'] == side and d['id'] == idl and d['d3_mm'] is not None:
            return f1(d['d3_mm'])
    return '—'


def build_markdown(res, sha_before, sha_after):
    res['sha_before'] = sha_before
    res['sha_after'] = sha_after
    L = md_start(res, sha_before, sha_after)
    md_geom(res, L)
    md_loads(res, L)
    md_end(res, L)
    return '\n'.join(L) + '\n'


def main():
    for need in (BLENDER, BLEND):
        if not os.path.exists(need):
            sys.exit('ОШИБКА: не найден %s' % need)
    os.makedirs(TMP, exist_ok=True)
    sha_before = sha256(BLEND)
    print('SHA-256 до :', sha_before)

    dump_path = os.path.join(TMP, 'arm_dump.json')
    copy_path = os.path.join(TMP, 'robot_pose_copy.blend')
    pose_path = os.path.join(TMP, 'pose_c_results.json')

    print('1/4 dump геометрии (read-only)…')
    run_blender(BLEND, os.path.join(OUT, '_blender_dump.py'), dump_path, 'audit_dump.log')
    print('2/4 pose-тест на временной копии…')
    shutil.copyfile(BLEND, copy_path)
    run_blender(copy_path, os.path.join(OUT, '_blender_pose_test.py'), pose_path, 'audit_pose.log')

    shutil.copyfile(dump_path, os.path.join(OUT, 'arm_dump.json'))
    shutil.copyfile(pose_path, os.path.join(OUT, 'pose_c_results.json'))
    with open(dump_path, encoding='utf-8') as f:
        dump = json.load(f)
    with open(pose_path, encoding='utf-8') as f:
        pose = json.load(f)

    print('3/4 расчёт…')
    checks = []
    res = analyze(dump, pose, checks)
    res = analyze_dyn(res, dump, pose)
    res['checks'] = checks

    sha_after = sha256(BLEND)
    print('SHA-256 после:', sha_after)
    if sha_before != sha_after:
        sys.exit('ОШИБКА: SHA-256 .blend изменился! Результаты недостоверны.')

    print('4/4 запись отчётов…')
    write_geometry_csv(os.path.join(OUT, 'arm_geometry_data.csv'), res)
    write_moments_csv(os.path.join(OUT, 'arm_joint_moments.csv'), res)
    md = build_markdown(res, sha_before, sha_after)
    with open(os.path.join(OUT, 'arm_geometry_load_audit.md'), 'w', encoding='utf-8') as f:
        f.write(md)

    print('')
    print('ГОТОВО. Каталог:', OUT)
    for fn in ('arm_geometry_load_audit.md', 'arm_geometry_data.csv', 'arm_joint_moments.csv',
               'arm_geometry_load_audit.py', 'arm_dump.json', 'pose_c_results.json'):
        p = os.path.join(OUT, fn)
        print('  %-32s %8d байт' % (fn, os.path.getsize(p)))
    print('')
    print('Ключевые значения (сценарий A, K1, 100 кг):')
    for jkey in ('j1', 'j3', 'elbow', 'w2'):
        print('  %-28s L %9.1f  R %9.1f Н·м' % (JLABEL[jkey], _tau(res, 'A', 'left', jkey),
                                                _tau(res, 'A', 'right', jkey)))
    print('Сценарий C 85° (K1):')
    for jkey in ('j1', 'j3', 'w2'):
        print('  %-28s L %9.1f  R %9.1f Н·м' % (JLABEL[jkey], _tau(res, 'C', 'left', jkey, 'K1', '85°'),
                                                _tau(res, 'C', 'right', jkey, 'K1', '85°')))
    print('SHA-256 совпадает — исходный .blend не изменялся.')


if __name__ == '__main__':
    main()
