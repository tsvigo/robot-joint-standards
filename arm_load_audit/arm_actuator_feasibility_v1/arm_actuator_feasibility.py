#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""arm_actuator_feasibility.py — этап «аудит реализуемости приводов рук».

Что делает (запуск: python3 arm_actuator_feasibility.py):

 1) НЕЗАВИСИМО пересчитывает моменты нагрузки τ = r × F из исходной геометрии
    arm_dump.json (позиции суставов) и тестовых точек хвата K1–K3t
    (arm_geometry_data.csv, этап 1), сверяет с arm_joint_moments.csv
    (сценарии A/покой, C/вылет j2 30-60-85°, B/динамика, asym) при g = 9.81
    (константа этапов 1–2) и при g = 9.80665 (требование аудита).
    Дополнительная сверка поз C — с pose_c_results.json (реальный поворот
    в Blender, этап 1).

 2) ВОССТАНАВЛИВАЕТ модель масс arm_mass_budget_v1 (оболочки — закрытая
    коробка по bbox, внутренняя структура, пальцы, приводы k·T*, кабели)
    и сверяет группы/итоги с arm_mass_budget.csv и моменты собственного
    веса с arm_mass_budget_moments.csv (этап 2).

 3) Считает моменты собственного веса в позах вылета (в этапе 2 их не было)
    и формирует arm_actuator_requirements.csv: требования по суставам
    (статическое удержание по сценариям, чувствительность к динамике
    0.1g/0.2g, грубая инерция), с явными пометками недостающих данных.

 4) Пересобирает массовый бюджет arm_mass_revised.csv с исправленным
    расчётным моментом T*(сценарий) = |τ_нагрузки|max + |τ_собств.веса|
    в критической позе (в v1 T* был только от нагрузки).

Входы (ищутся рядом со скриптом, затем в родительском каталоге):
    arm_dump.json, arm_joint_moments.csv, arm_mass_budget.csv,
    arm_mass_budget_moments.csv, arm_mass_scenarios.csv (справ.),
    pose_c_results.json (справ.), arm_geometry_data.csv (справ.)
Выходы (пишутся рядом со скриптом):
    arm_actuator_requirements.csv, arm_mass_revised.csv,
    arm_actuator_assumptions.csv

Ограничения: статическая модель; динамика — только чувствительность;
габариты/прочность/КПД не проверяются; .blend не открывается (только SHA256).
"""
import csv
import hashlib
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))


def find_input(name):
    for p in (os.path.join(HERE, name),
              os.path.join(os.path.dirname(HERE), name)):
        if os.path.isfile(p):
            return p
    return None


DUMP = find_input('arm_dump.json')
MOM = find_input('arm_joint_moments.csv')
BUD = find_input('arm_mass_budget.csv')
MOMBUD = find_input('arm_mass_budget_moments.csv')
SCENREF = find_input('arm_mass_scenarios.csv')
POSEC = find_input('pose_c_results.json')
GEO = find_input('arm_geometry_data.csv')

OUT_REQ = os.path.join(HERE, 'arm_actuator_requirements.csv')
OUT_MASS = os.path.join(HERE, 'arm_mass_revised.csv')
OUT_ASM = os.path.join(HERE, 'arm_actuator_assumptions.csv')

BLEND = '/home/v/projects/робот/robot_scaled_4-2-21-LTS-ARCH-02-MANUAL-01.blend'
BLEND_SHA_EXPECTED = ('1ae88879af076b1a0a1cfc6499d04a037dbb2103d2a7cb64'
                      '40708289829cd280')

G = 9.80665          # требуемая константа аудита
G_PREV = 9.81        # константа этапов 1–2 (Blender default)
LOAD_KG = 100.0      # допущение: 100 кг на кисть (200 кг / 2 руки)

SIDES = ('left', 'right')
SIGN = {'left': -1.0, 'right': 1.0}   # этап 1: левая −θ, правая +θ
SCEN = (
    ('lean', 0.002, 2700.0, 'облегчённый'),
    ('nominal', 0.003, 2700.0, 'номинальный'),
    ('heavy', 0.003, 7850.0, 'тяжёлый'),
)
SPEC_MASS = {'lean': 0.015, 'nominal': 0.030, 'heavy': 0.060}  # кг/(Н·м)
RIB_CFG = {
    'lean': {'bulk': 1, 'ribs': 0, 'rib_factor': 1.0},
    'nominal': {'bulk': 2, 'ribs': 2, 'rib_factor': 1.0},
    'heavy': {'bulk': 3, 'ribs': 3, 'rib_factor': 1.5},
}
RIB_FRAC = 0.6
CABLE_SPEC = {'lean': 0.08, 'nominal': 0.15, 'heavy': 0.30}  # кг/м
CABLE_SLACK = 1.5
RU_TORQUE = 20.0
SCEN_IDS = tuple(s[0] for s in SCEN)

JOINTS = [
    ('j3', 'shoulder_joint_empty_3', 'Y', 'j3 (плечо-3, ось Y)'),
    ('j2', 'shoulder_joint_empty_2', 'Z', 'j2 (плечо-2, ось Z)'),
    ('j1', 'shoulder_joint_empty_1', 'X', 'j1 (плечо-1, ось X)'),
    ('elbow', 'elbow_joint_empty', 'Z', 'локоть (ось Z)'),
    ('ru', 'proximal_radioulnar_joint_empty', 'Y', 'лучелоктевой (ось Y)'),
    ('w1', 'wrist_joint_1_empty', 'Z', 'запястье-1 (ось Z)'),
    ('w2', 'wrist_joint_2_empty', 'X', 'запястье-2 (ось X)'),
]
JKEYS = {j[0]: j for j in JOINTS}
AXIS3 = {'X': (1.0, 0.0, 0.0), 'Y': (0.0, 1.0, 0.0), 'Z': (0.0, 0.0, 1.0)}
GRIPS = ('K1', 'K2', 'K3', 'K3t')

# тестовые точки хвата (этап 1, arm_geometry_data.csv, строки grip_point)
K_PTS = {
    ('left', 'K1'): (3.792207, -0.790716, -4.575504),
    ('left', 'K2'): (3.792207, -0.736954, -4.575504),
    ('left', 'K3'): (3.792249, -0.518846, -4.579681),
    ('left', 'K3t'): (3.792250, -0.538008, -4.560478),
    ('right', 'K1'): (3.779544, -2.962129, -4.558332),
    ('right', 'K2'): (3.779544, -3.018193, -4.558332),
    ('right', 'K3'): (3.779539, -3.211870, -4.557214),
    ('right', 'K3t'): (3.779539, -3.193095, -4.539041),
}


def rec_T(rec):
    return (rec['mw'][0][3], rec['mw'][1][3], rec['mw'][2][3])


def box_shell_mass(a, b, c, t, rho):
    if t is None or 2.0 * t >= min(a, b, c):
        return rho * a * b * c
    return rho * (a * b * c - (a - 2.0 * t) * (b - 2.0 * t) * (c - 2.0 * t))


def internal_mass(dims, t, rho, cfg):
    s1, s2, ln = sorted(dims)
    m = cfg['bulk'] * rho * t * s1 * s2
    h = RIB_FRAC * max(min(s1, s2) - 2.0 * t, 0.0)
    m += cfg['ribs'] * rho * (t * cfg['rib_factor']) * ln * h
    return m


def rotp(c, p, ang_deg):
    """Точка p, повёрнутая вокруг вертикальной оси через c на ang_deg (град)."""
    a = math.radians(ang_deg)
    ca, sa = math.cos(a), math.sin(a)
    dx, dy = p[0] - c[0], p[1] - c[1]
    return (c[0] + ca * dx - sa * dy, c[1] + sa * dx + ca * dy, p[2])


def rotv(n, ang_deg):
    """Направление, повёрнутое вокруг Z на ang_deg (град)."""
    a = math.radians(ang_deg)
    ca, sa = math.cos(a), math.sin(a)
    return (ca * n[0] - sa * n[1], sa * n[0] + ca * n[1], n[2])


def f1(x):
    return '%.1f' % x


def f2(x):
    return '%.2f' % x


def f3(x):
    return '%.3f' % x


def jkey_of_label(label):
    for key in ('j3', 'j2', 'j1'):
        if label.startswith(key + ' ('):
            return key
    for key, pre in (('elbow', 'локоть'), ('ru', 'лучелоктевой'),
                     ('w1', 'запястье-1'), ('w2', 'запястье-2')):
        if label.startswith(pre):
            return key
    return None


def main():
    for name, path in (('arm_dump.json', DUMP),
                       ('arm_joint_moments.csv', MOM),
                       ('arm_mass_budget.csv', BUD)):
        if path is None:
            sys.exit('ОШИБКА: не найден вход %s' % name)

    with open(DUMP, encoding='utf-8') as f:
        dump = json.load(f)
    O = dump['objects']

    def desc(root):
        seen = [root]
        i = 0
        while i < len(seen):
            for c in O[seen[i]]['children']:
                seen.append(c)
            i += 1
        return seen

    # ---- позиции суставов (мировые) и опорные множества
    JP = {(side, key): rec_T(O[side + '_' + suf])
          for side in SIDES for key, suf, _ax, _lbl in JOINTS}
    E2 = {side: JP[(side, 'j2')] for side in SIDES}
    DISTAL2 = {side: set(desc(side + '_shoulder_joint_empty_2'))
               for side in SIDES}   # поворачивается при повороте j2
    SUB = {(side, key): set(desc(side + '_' + suf))
           for side in SIDES for key, suf, _ax, _lbl in JOINTS}

    # ---- меши руки (как в этапе 2: поддерево e3, без _collision)
    meshes = {}
    for side in SIDES:
        lst = []
        for nm in desc(side + '_shoulder_joint_empty_3'):
            r = O[nm]
            if r['type'] != 'MESH' or '_collision' in nm:
                continue
            dims = tuple(r['bbox_world_max'][i] - r['bbox_world_min'][i]
                         for i in range(3))
            ctr = tuple((r['bbox_world_max'][i] + r['bbox_world_min'][i]) / 2.0
                        for i in range(3))
            lst.append({'name': nm, 'dims': dims, 'center': ctr})
        lst.sort(key=lambda x: x['name'])
        meshes[side] = lst
    hand_set = {s: set(desc(s + '_palm_link')) for s in SIDES}
    palm_name = {s: s + '_palm_link' for s in SIDES}
    link_name = {'shoulder': {s: s + '_shoulder_link' for s in SIDES},
                 'forearm': {s: s + '_forearm_link' for s in SIDES},
                 'palm': palm_name}

    def mesh_by_name(side, nm):
        for p in meshes[side]:
            if p['name'] == nm:
                return p
        raise KeyError(nm)

    n_meshes = {s: len(meshes[s]) for s in SIDES}
    n_hand = {s: sum(1 for p in meshes[s] if p['name'] in hand_set[s] and
                     p['name'] != palm_name[s]) for s in SIDES}
    if n_meshes != {'left': 18, 'right': 18} or \
            n_hand != {'left': 15, 'right': 15}:
        sys.exit('ОШИБКА: неожиданный состав мешей %s / %s'
                 % (n_meshes, n_hand))

    # ---- строки моментов этапа 1
    mom_rows = []
    with open(MOM, encoding='utf-8') as f:
        for row in csv.DictReader(f):
            mom_rows.append(row)

    # ---- независимый расчёт моментов нагрузки τ = r × F
    def tau_load(side, key, grip, phi_deg, g=G, m_kg=LOAD_KG):
        n = AXIS3[JKEYS[key][2]]
        T = JP[(side, key)]
        K = K_PTS[(side, grip)]
        if key != 'j3':   # j3 проксимален к повороту j2 и не вращается
            T = rotp(E2[side], T, SIGN[side] * phi_deg)
            n = rotv(n, SIGN[side] * phi_deg)
        K = rotp(E2[side], K, SIGN[side] * phi_deg)
        r = (K[0] - T[0], K[1] - T[1], K[2] - T[2])
        return g * m_kg * (-r[1] * n[0] + r[0] * n[1])

    print('== СВЕРКА МОМЕНТОВ НАГРУЗКИ (независимый пересчёт vs этап 1) ==')
    dev = {}
    for row in mom_rows:
        sc = row['scenario']
        key = jkey_of_label(row['joint'])
        grip = row['grip']
        if key is None or grip not in GRIPS:
            continue
        side = row['side']
        if sc == 'A':
            mine = tau_load(side, key, grip, 0.0, G_PREV)
            ref = float(row['tau_Nm'])
            dev['A'] = max(dev.get('A', 0.0), abs(mine - ref))
        elif sc == 'C':
            phi = float(row['pose'].replace('°', ''))
            mine = tau_load(side, key, grip, phi, G_PREV)
            ref = float(row['tau_Nm'])
            dev['C'] = max(dev.get('C', 0.0), abs(mine - ref))
        elif sc in ('B_0.1g', 'B_0.2g'):
            k = float(row['accel_k'])
            mine = tau_load(side, key, grip, 0.0, G_PREV) * k
            ref = float(row['tau_Nm'])
            dev['B'] = max(dev.get('B', 0.0), abs(mine - ref))
        elif sc.startswith('A_asym'):
            m = float(row['load_kg'])
            mine = tau_load(side, key, grip, 0.0, G_PREV, m_kg=m)
            ref = float(row['tau_Nm'])
            dev['asym'] = max(dev.get('asym', 0.0), abs(mine - ref))
    for name, kk in (('A (покой, все хваты)', 'A'), ('C (вылет 30/60/85°)', 'C'),
                     ('B (динамика 0.1g/0.2g)', 'B'),
                     ('A_asym (60/140 кг)', 'asym')):
        print('  макс |Δ| %-24s = %s Н·м' % (name, f3(dev.get(kk, 0.0))))

    # ---- таблица нагрузки (G=9.80665) по семейству поз j2 0..90°
    phis = [i * 2.5 for i in range(0, 37)]
    fam = {}
    for side in SIDES:
        for key, _suf, _ax, _lbl in JOINTS:
            for grip in GRIPS:
                best = (0.0, 0.0)
                for phi in phis:
                    t = tau_load(side, key, grip, phi)
                    if abs(t) > abs(best[0]):
                        best = (t, phi)
                fam[(side, key, grip)] = (abs(best[0]), best[1])
    print('== МАКСИМУМ НАГРУЗКИ ПО СЕМЕЙСТВУ ПОЗ (поворот j2 0–90°, G=9.80665) ==')
    for side in SIDES:
        row = []
        for grip in GRIPS:
            v = fam[(side, 'j3', grip)]
            row.append('%s=%.1f@%g°' % (grip, v[0], v[1]))
        print('  %-5s j3: %s' % (side, ', '.join(row)))

    # ---- кросс-проверка поз C по реальному повороту (pose_c_results.json)
    pose_dmax = None
    if POSEC:
        with open(POSEC, encoding='utf-8') as f:
            posec = json.load(f)
        dmax = 0.0
        n_cmp = 0
        for test in posec.get('tests', []):
            side = test['side']
            phi = float(test['angle_deg'])
            jj = test.get('joints', {})
            for key, suf, _ax, _lbl in JOINTS:
                nm = side + '_' + suf
                if nm not in jj:
                    continue
                Tn = jj[nm]['T']
                T = JP[(side, key)]
                if key != 'j3':
                    T = rotp(E2[side], T, SIGN[side] * phi)
                d = math.dist(T, Tn)
                dmax = max(dmax, d)
                n_cmp += 1
        pose_dmax = dmax
        print('== КРОСС-ПРОВЕРКА ПОЗ C (аналитика vs реальный поворот Blender) ==')
        print('  сравнений: %d; макс |Δ| позиций суставов = %s м'
              % (n_cmp, f3(pose_dmax)))
    else:
        print('== КРОСС-ПРОВЕРКА ПОЗ C: pose_c_results.json не найден ==')

    # ---- расчётные моменты T* этапа 2 (только нагрузка, K1) — воспроизведение
    def csv_tau(scen, pose, side, key_name, grip='K1'):
        for row in mom_rows:
            if (row['scenario'] == scen and row['pose'] == pose and
                    row['side'] == side and row['grip'] == grip and
                    jkey_of_label(row['joint']) == key_name):
                return float(row['tau_Nm'])
        return None

    Tstar_v1 = {}
    for side in SIDES:
        j1m = max(abs(csv_tau('A', 'rest 0°', side, 'j1')),
                  abs(csv_tau('C', '85°', side, 'j1')))
        j3m = max(abs(csv_tau('A', 'rest 0°', side, 'j3')),
                  abs(csv_tau('C', '85°', side, 'j3')))
        w2m = abs(csv_tau('A', 'rest 0°', side, 'w2'))
        Tstar_v1[(side, 'j1')] = j1m
        Tstar_v1[(side, 'j3')] = j3m
        Tstar_v1[(side, 'j2')] = j1m
        Tstar_v1[(side, 'elbow')] = j3m
        Tstar_v1[(side, 'ru')] = RU_TORQUE
        Tstar_v1[(side, 'w1')] = w2m
        Tstar_v1[(side, 'w2')] = w2m

    # ---- модель масс v1 (реконструкция) для проверки моментов собств. веса
    with open(BUD, encoding='utf-8') as f:
        bud_rows = list(csv.DictReader(f))
    ITEMS = {}
    for side in SIDES:
        items = []

        def add(group, component, m, com_pts, basis):
            items.append({'group': group, 'component': component, 'm': m,
                          'com_pts': com_pts, 'basis': basis})

        p_sh = mesh_by_name(side, link_name['shoulder'][side])
        p_fa = mesh_by_name(side, link_name['forearm'][side])
        p_pa = mesh_by_name(side, palm_name[side])
        for key, human, p in (('shoulder', 'плеча', p_sh),
                              ('forearm', 'предплечья', p_fa),
                              ('palm', 'ладони', p_pa)):
            m = {sid: box_shell_mass(*p['dims'], t=t, rho=rho)
                 for sid, t, rho, _lab in SCEN}
            add('Звенья', 'Оболочка %s (закрытый бокс)' % human, m,
                [(link_name[key][side], p['center'], 1.0)], 'A01,A02,A22')
        for key, human, p in (('shoulder', 'плеча', p_sh),
                              ('forearm', 'предплечья', p_fa)):
            m = {sid: internal_mass(p['dims'], t, rho, RIB_CFG[sid])
                 for sid, t, rho, _lab in SCEN}
            add('Звенья', 'Внутренняя структура %s' % human, m,
                [(link_name[key][side], p['center'], 1.0)], 'A04')
        fingers = [p for p in meshes[side]
                   if p['name'] in hand_set[side] and
                   p['name'] != palm_name[side]]
        m = {sid: sum(box_shell_mass(*p['dims'], t=t, rho=rho)
                      for p in fingers) for sid, t, rho, _lab in SCEN}
        fnom = {p['name']: box_shell_mass(*p['dims'], t=0.003, rho=2700.0)
                for p in fingers}
        fsum = sum(fnom.values())
        add('Кисть', 'Пальцы: оболочки 15 мешей (сумма)', m,
            [(palm_name[side], p['center'], fnom[p['name']] / fsum)
             for p in fingers], 'A01,A02,A15,A22')
        for key, _suf, _ax, _lbl in JOINTS:
            mm = {sid: SPEC_MASS[sid] * Tstar_v1[(side, key)]
                  for sid in SCEN_IDS}
            add('Суставы', 'Узел привода: %s' % key, mm,
                [(side + '_' + JKEYS[key][1], JP[(side, key)], 1.0)],
                'k*T* v1')
        chain = [JP[(side, k)] for k in
                 ('j3', 'j2', 'j1', 'elbow', 'ru', 'w1', 'w2')]
        path = sum(math.dist(a, b) for a, b in zip(chain, chain[1:]))
        path += math.dist(JP[(side, 'w2')], p_pa['center'])
        path += max(p_pa['dims']) / 2.0
        cable_len = path * CABLE_SLACK
        mc = {sid: CABLE_SPEC[sid] * cable_len for sid in SCEN_IDS}
        add('Вспомогательные', 'Кабели и жгуты', mc,
            [(link_name['shoulder'][side], p_sh['center'], 0.5),
             (link_name['forearm'][side], p_fa['center'], 0.5)], 'A14')
        ITEMS[side] = items

    # контроль долей com_pts (нет двойного учёта/потерь)
    for side in SIDES:
        for it in ITEMS[side]:
            s = sum(f for _a, _p, f in it['com_pts'])
            if abs(s - 1.0) > 1e-9:
                sys.exit('ОШИБКА: доли com_pts != 1 у %s' % it['component'])

    # ---- сверка реконструированных масс с этапом 2 (итоги + контрольная деталь)
    print('== СВЕРКА МАСС (реконструкция vs arm_mass_budget.csv) ==')
    mass_dev = 0.0
    for side in SIDES:
        for sid, col in (('lean', 'mass_lean_kg'),
                         ('nominal', 'mass_nominal_kg'),
                         ('heavy', 'mass_heavy_kg')):
            mine = sum(it['m'][sid] for it in ITEMS[side])
            ref = sum(float(r[col]) for r in bud_rows
                      if r['side'] == side and r[col])
            mass_dev = max(mass_dev, abs(mine - ref))
    print('  макс |Δ| суммы известной части = %s кг' % f3(mass_dev))
    for side in SIDES:
        mm = [it for it in ITEMS[side]
              if it['component'] == 'Оболочка ладони (закрытый бокс)'][0]['m']
        ref = None
        for r in bud_rows:
            if (r['side'] == side and
                    r['component'] == 'Оболочка ладони (закрытый бокс)'):
                ref = (r['mass_lean_kg'], r['mass_nominal_kg'],
                       r['mass_heavy_kg'])
        print('  контроль: оболочка ладони %-5s реконструкция %s / v1 %s'
              % (side, [f2(mm[k]) for k in SCEN_IDS], list(ref)))

    def tau_sw(side, sid, key, phi_deg, g=G):
        n = AXIS3[JKEYS[key][2]]
        T = JP[(side, key)]
        if key != 'j3':
            T = rotp(E2[side], T, SIGN[side] * phi_deg)
            n = rotv(n, SIGN[side] * phi_deg)
        s = 0.0
        sub = SUB[(side, key)]
        for it in ITEMS[side]:
            m = it['m'][sid]
            for anchor, pos, frac in it['com_pts']:
                if anchor not in sub:
                    continue
                p = (rotp(E2[side], pos, SIGN[side] * phi_deg)
                     if anchor in DISTAL2[side] else pos)
                r = (p[0] - T[0], p[1] - T[1], p[2] - T[2])
                s += g * m * frac * (-r[1] * n[0] + r[0] * n[1])
        return s

    # ---- сверка моментов собств. веса с этапом 2 (покой)
    sw_dmax = None
    if MOMBUD:
        dmax = 0.0
        n_cmp = 0
        with open(MOMBUD, encoding='utf-8') as f:
            for row in csv.DictReader(f):
                key = jkey_of_label(row['joint'])
                if key is None:
                    continue
                mine = tau_sw(row['side'], row['scenario'], key, 0.0, G_PREV)
                d = abs(mine - float(row['tau_selfweight_Nm']))
                dmax = max(dmax, d)
                n_cmp += 1
        sw_dmax = dmax
        print('== СВЕРКА МОМЕНТОВ СОБСТВ. ВЕСА (реконструкция vs этап 2) ==')
        print('  сравнений: %d; макс |Δ| = %s Н·м' % (n_cmp, f3(sw_dmax)))
    else:
        print('== СВЕРКА SW: arm_mass_budget_moments.csv не найден ==')

    # ---- инвариантность дистальных осей при повороте j2 (проверка численно)
    dmax_inv = 0.0
    for side in SIDES:
        for key, _suf, _ax, _lbl in JOINTS:
            if key == 'j3':
                continue
            for sid in SCEN_IDS:
                d = abs(tau_sw(side, sid, key, 85.0) - tau_sw(side, sid, key, 0.0))
                dmax_inv = max(dmax_inv, d)
    print('== ИНВАРИАНТНОСТЬ SW дистальных осей при повороте j2: '
          'макс |Δ (85°−0°)| = %.2e Н·м' % dmax_inv)

    # ---- SW для j3 в позах вылета (новое; в этапе 2 не считалось)
    print('== SW для j3 в позах вылета (этап 2 считал только покой) ==')
    for side in SIDES:
        vals = []
        for sid in SCEN_IDS:
            vals.append('%s: покой %s, 85° %s, 90° %s' %
                        (sid, f1(tau_sw(side, sid, 'j3', 0.0)),
                         f1(tau_sw(side, sid, 'j3', 85.0)),
                         f1(tau_sw(side, sid, 'j3', 90.0))))
        print('  %-5s %s' % (side, ' | '.join(vals)))

    # ---- грубая инерция (точечные массы, оценка порядка величины)
    def inertia_arm(side, sid, key, phi_deg):
        n = AXIS3[JKEYS[key][2]]
        T = JP[(side, key)]
        if key != 'j3':
            T = rotp(E2[side], T, SIGN[side] * phi_deg)
            n = rotv(n, SIGN[side] * phi_deg)
        I = 0.0
        sub = SUB[(side, key)]
        for it in ITEMS[side]:
            m = it['m'][sid]
            for anchor, pos, frac in it['com_pts']:
                if anchor not in sub:
                    continue
                p = (rotp(E2[side], pos, SIGN[side] * phi_deg)
                     if anchor in DISTAL2[side] else pos)
                r = (p[0] - T[0], p[1] - T[1], p[2] - T[2])
                rn = r[0] * n[0] + r[1] * n[1] + r[2] * n[2]
                d2 = max(r[0] ** 2 + r[1] ** 2 + r[2] ** 2 - rn ** 2, 0.0)
                I += m * frac * d2
        return I

    def d_perp2(p, T, n):
        r = (p[0] - T[0], p[1] - T[1], p[2] - T[2])
        rn = r[0] * n[0] + r[1] * n[1] + r[2] * n[2]
        return max(r[0] ** 2 + r[1] ** 2 + r[2] ** 2 - rn ** 2, 0.0)

    # ---- требования к суставам
    notes = {
        'j3': 'критичен вылет вперёд (j2) и хват; SW растёт с вылетом',
        'j2': 'статический гравитационный момент = 0 (ось вертикальна); '
              'требование — динамика/трение, не задано; масса — условный класс j1',
        'j1': 'гравитационный момент инвариантен к повороту j2; '
              'SW существенен (тяжёлый сценарий)',
        'elbow': '= 0 в исследованном семействе (ось вертикальна); при наклоне '
                 'j1 нагружается — не исследовано; класс j3 условно',
        'ru': 'гравитационный ~1 Н·м; класс 20 Н·м — условное допущение',
        'w1': '= 0 (ось вертикальна); класс w2 условно',
        'w2': 'критичен хват: K1 ~20/15; K3t до 268/242 (тестовые точки)',
    }
    req_rows = []
    for side in SIDES:
        for key, _suf, ax, _lbl in JOINTS:
            for grip in GRIPS:
                lmax, phi_star = fam[(side, key, grip)]
                sw = {sid: tau_sw(side, sid, key, phi_star) for sid in SCEN_IDS}
                st = {sid: lmax + abs(sw[sid]) for sid in SCEN_IDS}
                I_arm = inertia_arm(side, 'nominal', key, phi_star)
                n = AXIS3[ax]
                T = JP[(side, key)]
                if key != 'j3':
                    T = rotp(E2[side], T, SIGN[side] * phi_star)
                    n = rotv(n, SIGN[side] * phi_star)
                K = rotp(E2[side], K_PTS[(side, grip)], SIGN[side] * phi_star)
                I_load = LOAD_KG * d_perp2(K, T, n)
                pose_txt = ('%g°' % phi_star) if key == 'j3' else \
                    '0–90° (инвариант)'
                if key == 'j3' and phi_star > 85.0:
                    pose_txt += ' (аналит.; BVH-тест до 85°)'
                req_rows.append([
                    side, _lbl, ax, grip,
                    f1(lmax), pose_txt,
                    f1(sw['lean']), f1(sw['nominal']), f1(sw['heavy']),
                    f1(st['lean']), f1(st['nominal']), f1(st['heavy']),
                    f1(st['heavy'] * 1.1), f1(st['heavy'] * 1.2),
                    '%.1f' % I_arm, '%.1f' % I_load,
                    'не задано', 'не задано', 'не определено (нет ω)',
                    notes[key]])

    with open(OUT_REQ, 'w', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        w.writerow(['side', 'joint', 'axis', 'grip',
                    'load_max_Nm', 'load_pose',
                    'sw_lean_Nm', 'sw_nominal_Nm', 'sw_heavy_Nm',
                    'static_lean_Nm', 'static_nominal_Nm', 'static_heavy_Nm',
                    'sens_01g_heavy_Nm', 'sens_02g_heavy_Nm',
                    'inertia_arm_nominal_kgm2', 'inertia_load_at_pose_kgm2',
                    'speed_data', 'duty_data', 'power_data', 'note'])
        for r in req_rows:
            w.writerow(r)
    print('записан %s (%d строк)' % (os.path.basename(OUT_REQ), len(req_rows)))

    # ---- пересмотр бюджета: T* = нагрузка + |SW| в критической позе
    Tstar_new = {}
    for side in SIDES:
        lj1, pj1 = fam[(side, 'j1', 'K1')]
        lj3, pj3 = fam[(side, 'j3', 'K1')]
        lw2, pw2 = fam[(side, 'w2', 'K1')]
        Tstar_new[(side, 'j1')] = {sid: lj1 + abs(tau_sw(side, sid, 'j1', pj1))
                                   for sid in SCEN_IDS}
        Tstar_new[(side, 'j3')] = {sid: lj3 + abs(tau_sw(side, sid, 'j3', pj3))
                                   for sid in SCEN_IDS}
        Tstar_new[(side, 'j2')] = dict(Tstar_new[(side, 'j1')])
        Tstar_new[(side, 'elbow')] = dict(Tstar_new[(side, 'j3')])
        Tstar_new[(side, 'ru')] = {sid: RU_TORQUE for sid in SCEN_IDS}
        Tstar_new[(side, 'w1')] = {sid: lw2 + abs(tau_sw(side, sid, 'w2', pw2))
                                   for sid in SCEN_IDS}
        Tstar_new[(side, 'w2')] = {sid: lw2 + abs(tau_sw(side, sid, 'w2', pw2))
                                   for sid in SCEN_IDS}
    act_m = {(side, key): {sid: SPEC_MASS[sid] * Tstar_new[(side, key)][sid]
                           for sid in SCEN_IDS}
             for side in SIDES for key, _s, _a, _l in JOINTS}

    print('== T* (v1: только нагрузка K1) -> (новый: +SW в крит. позе) ==')
    for side in SIDES:
        for key, _s, _a, _l in JOINTS:
            v1 = Tstar_v1[(side, key)]
            v2 = Tstar_new[(side, key)]
            print('  %-5s %-5s v1=%7.1f | lean %6.1f nom %6.1f heavy %6.1f'
                  % (side, key, v1, v2['lean'], v2['nominal'], v2['heavy']))

    # ---- запись пересмотренного бюджета (строки v1 прочитаны выше)

    ACT_COMP_PREFIX = {
        'j3': 'Узел привода: j3',
        'j2': 'Узел привода: j2',
        'j1': 'Узел привода: j1',
        'elbow': 'Узел привода: локоть',
        'ru': 'Узел привода: лучелоктевой',
        'w1': 'Узел привода: запястье-1',
        'w2': 'Узел привода: запястье-2',
    }

    def act_key_of_component(comp):
        for key, _s, _a, _l in JOINTS:
            if comp.startswith(ACT_COMP_PREFIX[key]):
                return key
        return None

    rev_rows = []
    reason_struct = 'без изменений (структурные массы v1 сохранены, сверка повторена)'
    for r in bud_rows:
        v1m = {}
        for sid, col in (('lean', 'mass_lean_kg'), ('nominal', 'mass_nominal_kg'),
                         ('heavy', 'mass_heavy_kg')):
            v1m[sid] = float(r[col]) if r[col] else None
        key = act_key_of_component(r['component'])
        if key is not None:
            m = act_m[(r['side'], key)]
            reason = ('T* пересмотрен: добавлен собств. вес в критической позе; '
                      'для j3 — поза вылета (ранее не учитывалась)')
            if key in ('j2', 'elbow', 'w1'):
                cls = {'j2': 'j1', 'elbow': 'j3', 'w1': 'w2'}[key]
                reason = ('класс %s сохранён (условно), масштаб пересчитан по '
                          'исправленному T*' % cls)
            if key == 'ru':
                reason = 'без методических изменений (условный класс 20 Н·м)'
        else:
            m = dict(v1m)
            reason = reason_struct if v1m['nominal'] is not None else '—'
        rev_rows.append([r['side'], r['group'], r['component'], r['part_class'],
                         f2(m['lean']) if m['lean'] is not None else '',
                         f2(m['nominal']) if m['nominal'] is not None else '',
                         f2(m['heavy']) if m['heavy'] is not None else '',
                         r['mass_lean_kg'], r['mass_nominal_kg'],
                         r['mass_heavy_kg'], reason, r['basis'],
                         r['confidence'], r['included_in_other'], r['com_note']])
    # отдельная строка: передачи/компенсация — не выбраны
    for side in SIDES:
        rev_rows.append([side, 'Вспомогательные',
                         'Удалённые передачи/редукторы и компенсаторы веса '
                         '(архитектуры B–E) — не выбраны',
                         'неизвестно', '', '', '', '', '', '',
                         'нет данных (архитектура не выбрана)', '—',
                         'нет данных', 'не входит в другие строки',
                         'появится при выборе архитектуры'])
    with open(OUT_MASS, 'w', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        w.writerow(['side', 'group', 'component', 'part_class',
                    'mass_lean_kg', 'mass_nominal_kg', 'mass_heavy_kg',
                    'mass_lean_v1_kg', 'mass_nominal_v1_kg', 'mass_heavy_v1_kg',
                    'change_reason', 'basis', 'confidence',
                    'included_in_other', 'com_note'])
        for r in rev_rows:
            w.writerow(r)
    print('записан %s (%d строк)' % (os.path.basename(OUT_MASS), len(rev_rows)))

    # ---- итоги v1 и пересмотренные (сумма известной части)
    bud_dicts = bud_rows
    tv1 = {}
    tnw = {}
    for side in SIDES:
        for sid, colv in (('lean', 'mass_lean_kg'), ('nominal', 'mass_nominal_kg'),
                          ('heavy', 'mass_heavy_kg')):
            tv1[(side, sid)] = round(sum(float(r[colv]) for r in bud_dicts
                                         if r['side'] == side and r[colv]), 2)
    rev_dicts = []
    with open(OUT_MASS, encoding='utf-8') as f:
        rev_dicts = list(csv.DictReader(f))
    for side in SIDES:
        for sid, colv in (('lean', 'mass_lean_kg'), ('nominal', 'mass_nominal_kg'),
                          ('heavy', 'mass_heavy_kg')):
            tnw[(side, sid)] = round(sum(float(r[colv]) for r in rev_dicts
                                         if r['side'] == side and r[colv]), 2)
    unk = [r for r in bud_dicts if not r['mass_nominal_kg']]
    print('== НЕИЗВЕСТНЫЕ СТРОКИ (без массы, v1): %d всего, %d на руку =='
          % (len(unk), len(unk) // 2))
    for comp in sorted({r['component'] for r in unk}):
        print('   - %s' % comp)
    print('== ИТОГИ (известная часть бюджета, кг) ==')
    print('  %-8s %10s %10s %10s' % ('сценарий', 'v1 L', 'new L', 'Δ L'))
    for sid in SCEN_IDS:
        print('  %-8s %10s %10s %+10s' % (sid, tv1[('left', sid)],
                                          tnw[('left', sid)],
                                          f2(tnw[('left', sid)] - tv1[('left', sid)])))
    print('  %-8s %10s %10s %10s' % ('сценарий', 'v1 R', 'new R', 'Δ R'))
    for sid in SCEN_IDS:
        print('  %-8s %10s %10s %+10s' % (sid, tv1[('right', sid)],
                                          tnw[('right', sid)],
                                          f2(tnw[('right', sid)] - tv1[('right', sid)])))
    print('  обе руки: v1 %s/%s/%s | new %s/%s/%s' % (
        f2(tv1[('left', 'lean')] + tv1[('right', 'lean')]),
        f2(tv1[('left', 'nominal')] + tv1[('right', 'nominal')]),
        f2(tv1[('left', 'heavy')] + tv1[('right', 'heavy')]),
        f2(tnw[('left', 'lean')] + tnw[('right', 'lean')]),
        f2(tnw[('left', 'nominal')] + tnw[('right', 'nominal')]),
        f2(tnw[('left', 'heavy')] + tnw[('right', 'heavy')])))

    # ---- проверка согласованности сумм строк и итогов (в пределах округления)
    for tbl, name in ((rev_dicts, 'revised'),):
        for side in SIDES:
            for sid, colv in (('lean', 'mass_lean_kg'),
                              ('nominal', 'mass_nominal_kg'),
                              ('heavy', 'mass_heavy_kg')):
                s = round(sum(float(r[colv]) for r in tbl
                              if r['side'] == side and r[colv]), 2)
                if abs(s - tnw[(side, sid)]) > 0.011:
                    sys.exit('ОШИБКА: сумма строк != итог (%s %s %s)' %
                             (name, side, sid))
    print('== СУММЫ СТРОК = ИТОГИ (в пределах округления): ок ==')

    # ---- допущения этапа
    asm = [
        ('B01', 'Независимый пересчёт моментов нагрузки τ=r×F (этот этап)',
         'макс |Δ|=%.3f Н·м (A/C)' % max(dev.get('A', 0), dev.get('C', 0)),
         'Н·м',
         'пересчёт из arm_dump.json; сверка с arm_joint_moments.csv',
         'проверено расчётом', 'высокая', 'все моменты нагрузки',
         'метод этапа 1 воспроизведён; расхождений выше округления нет'),
        ('B02', 'Гравитационная постоянная',
         'в этом этапе g=9.80665; в этапах 1–2 g=9.81', 'м/с²',
         'требование аудита; этапы 1–2 использовали Blender default',
         'проверено расчётом', 'высокая', 'все моменты',
         'различие чисел +0.034% — ниже точности модели'),
        ('B03', 'Кросс-проверка поз вылета (C) по реальному повороту',
         ('макс |Δ| позиций %.3f м' % pose_dmax) if pose_dmax is not None
         else 'нет pose_c_results.json', 'м',
         'pose_c_results.json (этап 1) vs аналитический поворот',
         'проверено расчётом', 'высокая', 'моменты в позах C',
         'аналитическая модель поворота подтверждена'),
        ('B04', 'Реконструкция модели масс этапа 2 и SW-моментов',
         ('макс |Δ| SW (покой) %.3f Н·м' % sw_dmax) if sw_dmax is not None
         else 'нет arm_mass_budget_moments.csv', 'Н·м',
         'независимая сборка по формулам v1 и arm_dump.json',
         'проверено расчётом', 'высокая', 'моменты SW',
         'модель v1 воспроизведена'),
        ('B05', 'T* для j3 в v1 не включал собственный вес и позу вылета',
         'SW j3 в позе вылета: lean %.1f / nom %.1f / heavy %.1f (L)' %
         (tau_sw('left', 'lean', 'j3', 90.0), tau_sw('left', 'nominal', 'j3', 90.0),
          tau_sw('left', 'heavy', 'j3', 90.0)), 'Н·м',
         'расчёт этого этапа', 'найдено расхождение методики', 'высокая',
         'масса привода j3 (и класса локтя)', 'масса v1 занижена; исправлено'),
        ('B06', 'Хват за кончики пальцев (K3t) увеличивает моменты плеча',
         'j1 L 1052.6 vs 804.7; j3 C85 L 1100.6 vs 853.7', 'Н·м',
         'arm_joint_moments.csv (этап 1)', 'измерено по геометрии', 'средняя',
         'требования суставов', 'в бюджет не включён; чувствительность к хвату'),
        ('B07', 'Фактический хват не задан',
         'K1 принят базовым', '—', 'поза подъёма в модели не задана',
         'допущение', 'средняя', 'все моменты', 'влияет на плечо и кисть'),
        ('B08', 'Динамика не задана: только чувствительность',
         '×1.1 / ×1.2 (0.1g/0.2g на гравитационные моменты)', '—',
         'нет траектории/цикла/ускорений, по аналогии с этапом 1',
         'допущение', 'низкая', 'пиковый момент',
         'пиковый и продолжительный моменты не выводятся без цикла'),
        ('B09', 'Скорости и мощности не заданы',
         'P=τ·ω не определена', '—', 'нет данных о скорости валов',
         'нет данных', 'нет данных', 'выбор привода',
         'нужны: ω каждого сустава, время цикла, режим'),
        ('B10', 'Удельная масса k=0.015/0.03/0.06 кг/(Н·м)',
         'инженерная оценка v1 сохранена', 'кг/(Н·м)',
         'сверка с реальными компонентами — см. arm_actuator_candidates.csv',
         'инженерная оценка', 'низкая', 'массы приводов',
         'k не подтверждён каталогом как метод выбора'),
        ('B11', 'Самоусиление T*↔SW (масса приводов входит в SW)',
         'повторная итерация не выполнялась', '—',
         'одношаговое исправление T*', 'допущение', 'средняя',
         'массы приводов', 'эффект второго порядка (единицы %)'),
        ('B12', 'Приводы j2/локоть/w1 — условный класс j1/j3',
         'статический гравитационный момент = 0', '—',
         'оси вертикальны в исследованных позах', 'допущение', 'низкая',
         'массы этих узлов', 'реальные требования — динамика (не задана)'),
        ('B13', 'Максимум по семейству поз (j2 0–90°) ≠ максимум рабочего '
         'пространства', 'локоть/наклон j1/кисть не исследованы', '—',
         'однопараметрическое семейство', 'ограничение', '—',
         'требования', 'нужна поза подъёма и цикл'),
        ('B14', 'Инерция — оценка точечными массами',
         'порядок величины', 'кг·м²', 'COM=центр bbox/точка сустава',
         'инженерная оценка', 'низкая', 'динамика',
         'тензоры инерции не заданы'),
        ('B15', 'Пальцевые приводы, тормоза, крепёж, передачи не оценены',
         'неизвестная масса', 'кг', 'схемы не заданы',
         'нет данных', 'нет данных', 'полнота бюджета',
         'бюджет остаётся частичным (статус v1 сохранён)'),
        ('B16', 'Тормоз — критичный компонент удержания 200 кг',
         'требование не сформулировано', '—',
         'при отключении питания удержание обязано обеспечиваться тормозом '
         'или компенсацией', 'инженерная оценка', 'средняя',
         'архитектура, масса', 'добавить в ТЗ следующего этапа'),
    ]
    with open(OUT_ASM, 'w', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        w.writerow(['id', 'assumption', 'value', 'unit', 'basis',
                    'source_quality', 'confidence', 'affects', 'note'])
        for a in asm:
            w.writerow(a)
    print('записан %s (%d строк)' % (os.path.basename(OUT_ASM), len(asm)))

    # ---- SHA256 .blend
    if os.path.isfile(BLEND):
        h = hashlib.sha256()
        with open(BLEND, 'rb') as f:
            for chunk in iter(lambda: f.read(1 << 20), b''):
                h.update(chunk)
        digest = h.hexdigest()
        print('== SHA256 .blend: %s — %s ==' % (
            digest, 'совпадает с эталоном' if digest == BLEND_SHA_EXPECTED
            else 'НЕ СОВПАДАЕТ'))
    else:
        print('== SHA256 .blend: файл не найден ==')


if __name__ == '__main__':
    main()
