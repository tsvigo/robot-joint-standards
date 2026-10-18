#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""arm_mass_budget.py — предварительный массовый бюджет обеих рук и моменты
собственного веса (этап аудита; продолжение arm_geometry_load_audit.md и
arm_mass_scenarios.md).

Blender не открывается: геометрия и позиции суставов берутся из arm_dump.json,
моменты нагрузки — из arm_joint_moments.csv (сценарий A, покой 0 град., K1;
справочно — C 85 град.).

Группы бюджета:
  * оболочки звеньев — закрытая коробка на габаритах bbox (как этап 2);
  * внутренняя структура (перегородки/рёбра) — параметрическое допущение;
  * кисть — оболочки ладони и фаланг; механизмы/приводы пальцев НЕ оценены;
  * узлы суставов — параметрическая оценка по расчётному моменту и удельной
    массе (НЕ выбор оборудования);
  * кабели — длина трассы x удельная масса;
  * крепёж, тормоза, разъёмы/датчики/кожухи — не оценены (нет данных).

Сценарии: lean (Al 2 мм), nominal (Al 3 мм), heavy (сталь 3 мм);
удельная масса приводов 0.015/0.030/0.060 кг на 1 Н*м статического момента.

Читает:  arm_dump.json, arm_joint_moments.csv, arm_mass_scenarios.csv
         (ищет рядом со скриптом, затем в родительском каталоге)
Пишет:   arm_mass_budget.csv, arm_mass_budget_moments.csv,
         arm_mass_budget_assumptions.csv, arm_mass_budget.md

Запуск:  python3 arm_mass_budget.py
"""
import csv
import hashlib
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))


def find_input(name):
    for p in (os.path.join(HERE, name), os.path.join(os.path.dirname(HERE), name)):
        if os.path.isfile(p):
            return p
    return None


DUMP = find_input('arm_dump.json')
MOM = find_input('arm_joint_moments.csv')
REF = find_input('arm_mass_scenarios.csv')

OUT_CSV = os.path.join(HERE, 'arm_mass_budget.csv')
OUT_MOM = os.path.join(HERE, 'arm_mass_budget_moments.csv')
OUT_ASM = os.path.join(HERE, 'arm_mass_budget_assumptions.csv')
OUT_MD = os.path.join(HERE, 'arm_mass_budget.md')

BLEND = '/home/v/projects/робот/robot_scaled_4-2-21-LTS-ARCH-02-MANUAL-01.blend'
BLEND_SHA_EXPECTED = ('1ae88879af076b1a0a1cfc6499d04a037dbb2103d2a7cb64'
                      '40708289829cd280')

G = 9.81
SIDES = ('left', 'right')
SIDE_RU = {'left': 'левая', 'right': 'правая'}
SCEN = (
    ('lean', 'облегчённый', 0.002, 2700.0, 'Al 2 мм'),
    ('nominal', 'номинальный', 0.003, 2700.0, 'Al 3 мм'),
    ('heavy', 'тяжёлый', 0.003, 7850.0, 'сталь 3 мм'),
)
SCEN_LABEL = {s[0]: s[1] for s in SCEN}
SPEC_MASS = {'lean': 0.015, 'nominal': 0.030, 'heavy': 0.060}  # кг/(Н*м)
RIB_CFG = {
    'lean': {'bulk': 1, 'ribs': 0, 'rib_factor': 1.0},
    'nominal': {'bulk': 2, 'ribs': 2, 'rib_factor': 1.0},
    'heavy': {'bulk': 3, 'ribs': 3, 'rib_factor': 1.5},
}
RIB_FRAC = 0.6          # высота продольного ребра, доля внутреннего габарита
CABLE_SPEC = {'lean': 0.08, 'nominal': 0.15, 'heavy': 0.30}  # кг/м
CABLE_SLACK = 1.5       # запас длины трассы
RU_TORQUE = 20.0        # условный расчётный момент лучелоктевого сустава, Н*м

JOINTS = [
    ('j3', 'shoulder_joint_empty_3', 'Y', 'j3 (плечо-3, ось Y)'),
    ('j2', 'shoulder_joint_empty_2', 'Z', 'j2 (плечо-2, ось Z)'),
    ('j1', 'shoulder_joint_empty_1', 'X', 'j1 (плечо-1, ось X)'),
    ('elbow', 'elbow_joint_empty', 'Z', 'локоть (ось Z)'),
    ('ru', 'proximal_radioulnar_joint_empty', 'Y', 'лучелоктевой (ось Y)'),
    ('w1', 'wrist_joint_1_empty', 'Z', 'запястье-1 (ось Z)'),
    ('w2', 'wrist_joint_2_empty', 'X', 'запястье-2 (ось X)'),
]
JKEY = {j[0]: j for j in JOINTS}
AXIS = {'X': (1.0, 0.0, 0.0), 'Y': (0.0, 1.0, 0.0), 'Z': (0.0, 0.0, 1.0)}
GROUPS = ('Звенья', 'Кисть', 'Суставы', 'Вспомогательные')


def hash_file(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def rec_T(rec):
    return (rec['mw'][0][3], rec['mw'][1][3], rec['mw'][2][3])


def f1(x):
    if abs(x) < 0.05:
        x = 0.0
    return '%.1f' % x


def f2(x):
    return '%.2f' % x


def f3(x):
    return '%.3f' % x


def tbl(headers, rows):
    out = ['| ' + ' | '.join(headers) + ' |',
           '|' + '|'.join(['---'] * len(headers)) + '|']
    for r in rows:
        out.append('| ' + ' | '.join(str(c) for c in r) + ' |')
    return out


def box_shell_mass(a, b, c, t, rho):
    """Масса закрытой коробчатой оболочки; при 2t >= min — сплошное тело."""
    if t is None or 2.0 * t >= min(a, b, c):
        return rho * a * b * c
    return rho * (a * b * c - (a - 2.0 * t) * (b - 2.0 * t) * (c - 2.0 * t))


def main():
    for name, path in (('arm_dump.json', DUMP),
                       ('arm_joint_moments.csv', MOM)):
        if path is None:
            sys.exit('ОШИБКА: не найден %s' % name)

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

    # ---- меши руки (как этап 2: поддерево shoulder_joint_empty_3, без collision)
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
            lst.append({'name': nm, 'dims': dims, 'center': ctr,
                        'stored': r['props'].get('mass')})
        lst.sort(key=lambda x: x['name'])
        meshes[side] = lst

    hand_set = {s: set(desc(s + '_palm_link')) for s in SIDES}
    palm_name = {s: s + '_palm_link' for s in SIDES}
    link_name = {
        'shoulder': {s: s + '_shoulder_link' for s in SIDES},
        'forearm': {s: s + '_forearm_link' for s in SIDES},
        'palm': palm_name,
    }
    joint_pos = {}
    for side in SIDES:
        for key, suf, _ax, _lbl in JOINTS:
            joint_pos[(side, key)] = rec_T(O[side + '_' + suf])
    # поддеревья суставов: масса учитывается только у СВОЕГО сустава и proximal
    subtree = {}
    for side in SIDES:
        for key, suf, _ax, _lbl in JOINTS:
            subtree[(side, key)] = set(desc(side + '_' + suf))

    def mesh_by_name(side, nm):
        for p in meshes[side]:
            if p['name'] == nm:
                return p
        raise KeyError(nm)

    n_meshes = {s: len(meshes[s]) for s in SIDES}
    n_hand = {s: sum(1 for p in meshes[s] if p['name'] in hand_set[s] and
                     p['name'] != palm_name[s]) for s in SIDES}
    if n_meshes != {'left': 18, 'right': 18} or n_hand != {'left': 15, 'right': 15}:
        sys.exit('ОШИБКА: неожиданный состав мешей: %s / кисть %s'
                 % (n_meshes, n_hand))

    # ---- моменты нагрузки из аудита
    loadA, loadC = {}, {}
    with open(MOM, encoding='utf-8') as f:
        for row in csv.DictReader(f):
            if row['grip'] != 'K1':
                continue
            key = (row['side'], row['joint'])
            if row['scenario'] == 'A' and row['pose'] == 'rest 0°':
                loadA[key] = float(row['tau_Nm'])
            elif row['scenario'] == 'C' and row['pose'] == '85°':
                loadC[key] = float(row['tau_Nm'])
    if len(loadA) != 14 or len(loadC) != 14:
        sys.exit('ОШИБКА: моменты нагрузки прочитаны не полностью (%d/%d)'
                 % (len(loadA), len(loadC)))

    lbl = {j[0]: j[3] for j in JOINTS}
    design_T = {}
    for side in SIDES:
        j1m = max(abs(loadA[(side, lbl['j1'])]), abs(loadC[(side, lbl['j1'])]))
        j3m = max(abs(loadA[(side, lbl['j3'])]), abs(loadC[(side, lbl['j3'])]))
        w2m = abs(loadA[(side, lbl['w2'])])
        design_T[(side, 'j1')] = j1m
        design_T[(side, 'j3')] = j3m
        design_T[(side, 'j2')] = j1m      # класс плечевого узла — условно
        design_T[(side, 'elbow')] = j3m   # класс плечевого узла — условно
        design_T[(side, 'ru')] = RU_TORQUE
        design_T[(side, 'w1')] = w2m      # класс запястья — условно
        design_T[(side, 'w2')] = w2m

    # ---- оболочки всех 18 мешей по сценариям + сверка с этапом 2
    shell = {}
    for side in SIDES:
        for sid, _ru, t, rho, _lab in SCEN:
            shell[(side, sid)] = sum(
                box_shell_mass(*p['dims'], t=t, rho=rho) for p in meshes[side])
    ref = {}
    ref_msg = 'arm_mass_scenarios.csv не найден — сверка с этапом 2 пропущена'
    if REF:
        cols = {'lean': 'Al 2_kg', 'nominal': 'Al 3_kg', 'heavy': 'Ст 3_kg'}
        sums = {}
        with open(REF, encoding='utf-8') as f:
            for row in csv.DictReader(f):
                for sid, col in cols.items():
                    sums[(row['side'], sid)] = sums.get((row['side'], sid), 0.0) + \
                        float(row[col])
        dmax = max(abs(sums[(s, sid)] - shell[(s, sid)])
                   for s in SIDES for sid in cols)
        ref_msg = ('сверка оболочек с arm_mass_scenarios.csv: макс |Δ| = %s кг'
                   ' (допуск 0.01)' % f3(dmax))
        if dmax > 0.01:
            ref_msg = 'ПРОВАЛ СВЕРКИ этапа 2: ' + ref_msg

    # ---- внутренняя структура звена (перегородки + рёбра), параметрически
    def internal_mass(dims, t, rho, cfg):
        s1, s2, ln = sorted(dims)
        m = cfg['bulk'] * rho * t * s1 * s2
        h = RIB_FRAC * max(min(s1, s2) - 2.0 * t, 0.0)
        m += cfg['ribs'] * rho * (t * cfg['rib_factor']) * ln * h
        return m

    # ---- строка бюджета
    rows = []

    def add(side, group, component, part, masses, basis, confidence,
            included, com_note, com_pts):
        rows.append({'side': side, 'group': group, 'component': component,
                     'part': part, 'mass': masses, 'basis': basis,
                     'confidence': confidence, 'included': included,
                     'com_note': com_note, 'com_pts': com_pts})

    def m3(fn):
        return {sid: fn(sid) for sid, _ru, _t, _rho, _lab in SCEN}

    for side in SIDES:
        # оболочки трёх крупных звеньев
        for key, human in (('shoulder', 'плеча'), ('forearm', 'предплечья'),
                           ('palm', 'ладони')):
            p = mesh_by_name(side, link_name[key][side])
            masses = m3(lambda sid, p=p: box_shell_mass(
                *p['dims'], t=SCEN[[x[0] for x in SCEN].index(sid)][2],
                rho=SCEN[[x[0] for x in SCEN].index(sid)][3]))
            add(side, 'Звенья', 'Оболочка %s (закрытый бокс)' % human, 'оболочка',
                masses, 'A01, A02, A22', 'средняя (в рамках допущений)',
                'нет; торцы входят в оболочку (6 стенок)',
                'центр bbox звена (допущение A03)', [(p['name'], p['center'], 1.0)])

        # внутренняя структура плеча и предплечья
        for key, human in (('shoulder', 'плеча'), ('forearm', 'предплечья')):
            p = mesh_by_name(side, link_name[key][side])
            masses = {}
            for sid, _ru2, t, rho, _lab in SCEN:
                masses[sid] = internal_mass(p['dims'], t, rho, RIB_CFG[sid])
            add(side, 'Звенья', 'Внутренняя структура %s: перегородки + рёбра'
                % human, 'рёбра/перегородки', masses, 'A04', 'низкая',
                'нет (самостоятельная строка; оболочка — отдельно)',
                'центр bbox звена (допущение A03)', [(p['name'], p['center'], 1.0)])

        # кисть: оболочки пальцев (15 мешей), ладонь учтена выше
        p_palm = mesh_by_name(side, palm_name[side])
        masses = {}
        for sid, _ru2, t, rho, _lab in SCEN:
            masses[sid] = sum(box_shell_mass(*p['dims'], t=t, rho=rho)
                              for p in meshes[side]
                              if p['name'] in hand_set[side] and
                              p['name'] != palm_name[side])
        finger_nom = {p['name']: box_shell_mass(*p['dims'], t=0.003, rho=2700.0)
                      for p in meshes[side]
                      if p['name'] in hand_set[side] and
                      p['name'] != palm_name[side]}
        fsum = sum(finger_nom.values())
        fingers_pts = [(mesh_by_name(side, nm)['center'], m / fsum)
                       for nm, m in finger_nom.items()]
        add(side, 'Кисть', 'Пальцы: оболочки 15 мешей (сумма)', 'оболочка',
            masses, 'A01, A02, A15, A22', 'средняя (в рамках допущений)',
            'нет; механизмы/приводы пальцев — отдельные неизвестные строки',
            'центры bbox каждого из 15 мешей; вклад по массе (допущение A03)',
            [(palm_name[side], c, f) for (c, f) in fingers_pts])

        # кисть: неизвестные составляющие
        for comp, why in (
                ('Механизм пальцев (шарниры, тяги/передачи, возврат)',
                 'схема захвата не задана'),
                ('Приводы и редукторы пальцев',
                 'тип и размещение приводов не заданы'),
                ('Захватные поверхности, датчики, проводка кисти',
                 'конструкция кисти не определена')):
            add(side, 'Кисть', comp, 'неизвестно', None,
                'A15', 'нет данных', 'не входит в другие строки',
                'неизвестно', None)

        # узлы суставов — параметрическая оценка
        for key, _suf, _ax, label in JOINTS:
            t_star = design_T[(side, key)]
            masses = {sid: SPEC_MASS[sid] * t_star
                      for sid, _ru2, _t, _rho, _lab in SCEN}
            if key == 'j1':
                src = 'A06'
            elif key == 'j3':
                src = 'A07'
            elif key == 'w2':
                src = 'A12'
            elif key == 'j2':
                src = 'A08'
            elif key == 'elbow':
                src = 'A09'
            elif key == 'w1':
                src = 'A10'
            else:
                src = 'A11'
            add(side, 'Суставы',
                'Узел привода: %s (двигатель + редуктор + подшипники + вал + '
                'фланцы + корпус)' % label, 'привод', masses,
                '%s, A05, A13' % src, 'низкая (параметрическая оценка)',
                'нет; раздельная оценка мотор/редуктор/подшипники/вал не '
                'выполнена (нужна схема); тормоз и крепёж — отдельные '
                'неизвестные строки',
                'точка сустава (допущение A13)',
                [(side + '_' + JKEY[key][1], joint_pos[(side, key)], 1.0)])

        # вспомогательные
        path = 0.0
        chain = [joint_pos[(side, k)] for k in
                 ('j3', 'j2', 'j1', 'elbow', 'ru', 'w1', 'w2')]
        for a, b in zip(chain, chain[1:]):
            path += math.dist(a, b)
        path += math.dist(joint_pos[(side, 'w2')], p_palm['center'])
        path += max(p_palm['dims']) / 2.0
        cable_len = path * CABLE_SLACK
        cbl = {sid: CABLE_SPEC[sid] * cable_len
               for sid, _ru2, _t, _rho, _lab in SCEN}
        c_shoulder = mesh_by_name(side, link_name['shoulder'][side])['center']
        c_forearm = mesh_by_name(side, link_name['forearm'][side])['center']
        add(side, 'Вспомогательные', 'Кабели и жгуты',
            'кабели', cbl, 'A14', 'низкая',
            'нет; разъёмы и датчики — отдельная неизвестная строка',
            'трасса %.2f м; половина в плече, половина в предплечье (допущение)'
            % cable_len,
            [(link_name['shoulder'][side], c_shoulder, 0.5),
             (link_name['forearm'][side], c_forearm, 0.5)])
        add(side, 'Вспомогательные', 'Разъёмы, датчики, кронштейны, кожухи',
            'неизвестно', None, 'A15', 'нет данных',
            'не входит в другие строки', 'неизвестно', None)
        add(side, 'Вспомогательные', 'Тормоза суставов (наличие не определено)',
            'тормоз', None, 'A16', 'нет данных', 'нет', 'неизвестно', None)
        add(side, 'Вспомогательные', 'Крепёж (суставы, звенья, кисть)',
            'крепёж', None, 'A17', 'нет данных', 'нет', 'неизвестно', None)

    # ---- итоги
    def group_mass(side, sid, group):
        return sum(r['mass'][sid] for r in rows
                   if r['side'] == side and r['group'] == group
                   and r['mass'] is not None and r['mass'][sid] is not None)

    def total_mass(side, sid):
        return sum(group_mass(side, sid, g) for g in GROUPS)

    totals = {(s, sid): total_mass(s, sid) for s in SIDES
              for sid, _ru2, _t, _rho, _lab in SCEN}
    for s in SIDES:
        for sid, _ru2, _t, _rho, _lab in SCEN:
            parts = sum(group_mass(s, sid, g) for g in GROUPS)
            assert abs(parts - totals[(s, sid)]) < 1e-9

    # отображаемые итоги: сумма округлённых групп (чтобы суммы в таблицах
    # сходились с итогами при печати с двумя знаками)
    disp_parts = {}
    disp_tot = {}
    for s in SIDES:
        for sid, _ru2, _t, _rho, _lab in SCEN:
            dp = [round(group_mass(s, sid, g), 2) for g in GROUPS]
            disp_parts[(s, sid)] = dp
            disp_tot[(s, sid)] = round(sum(dp), 2)

    unknown_rows = [r for r in rows if r['mass'] is None]

    # ---- моменты собственного веса (поза файла, 0°)
    def tau_sw(side, sid, key):
        _k, _suf, ax, _lbl = JKEY[key]
        n = AXIS[ax]
        T = joint_pos[(side, key)]
        sub = subtree[(side, key)]
        s = 0.0
        for r in rows:
            if r['side'] != side or r['mass'] is None or not r['com_pts']:
                continue
            m = r['mass'][sid]
            for anchor, pos, frac in r['com_pts']:
                if anchor not in sub:
                    continue
                s += m * frac * G * (-(pos[1] - T[1]) * n[0] +
                                     (pos[0] - T[0]) * n[1])
        return s

    # ---- запись arm_mass_budget.csv
    with open(OUT_CSV, 'w', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        w.writerow(['side', 'group', 'component', 'part_class',
                    'mass_lean_kg', 'mass_nominal_kg', 'mass_heavy_kg',
                    'basis', 'confidence', 'included_in_other', 'com_note'])
        for r in rows:
            w.writerow([r['side'], r['group'], r['component'], r['part']] +
                       ['' if (r['mass'] is None or r['mass'][sid] is None)
                        else f2(r['mass'][sid])
                        for sid in ('lean', 'nominal', 'heavy')] +
                       [r['basis'], r['confidence'], r['included'], r['com_note']])

    # ---- запись arm_mass_budget_moments.csv
    scen_ids = ('lean', 'nominal', 'heavy')
    note_m = ('частично: без механизмов/приводов пальцев, тормозов и крепежа; '
              'j2/локоть/w1 — условный класс привода')
    with open(OUT_MOM, 'w', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        w.writerow(['side', 'scenario', 'joint', 'axis',
                    'tau_selfweight_Nm', 'tau_load_A_K1_Nm',
                    'tau_load_C85_K1_Nm', 'tau_total_A_conditional_Nm', 'note'])
        for side in SIDES:
            for sid in scen_ids:
                for key, _suf, ax, label in JOINTS:
                    sw = tau_sw(side, sid, key)
                    la = loadA[(side, label)]
                    lc = loadC[(side, label)]
                    w.writerow([side, sid, label, ax, f1(sw), f1(la), f1(lc),
                                f1(sw + la), note_m])

    # ---- запись arm_mass_budget_assumptions.csv
    assumptions = [
        ('A01', 'Материал и толщина оболочек звеньев',
         'Al 2 / Al 3 / сталь 3 мм', 'мм',
         'исходные геометрические сценарии этапа 2 (arm_mass_scenarios.md)',
         'допущение, не измерение', 'низкая',
         'все массы оболочек', 'прочность/жёсткость не проверяются'),
        ('A02', 'Модель оболочки: закрытая коробка по bbox',
         'm = rho*(abc - (a-2t)(b-2t)(c-2t))', 'кг',
         'формула этапа 2; все 6 стенок, включая торцы',
         'геометрическая модель', 'средняя',
         'оболочки звеньев и пальцев',
         'рёбра/перегородки учитываются отдельно (A04)'),
        ('A03', 'COM оболочек и внутренней структуры — центр bbox',
         'центр габарита', 'м',
         'приближение; максимум смещения origin от центра bbox на этапе 2 — 32 мм',
         'допущение', 'средняя', 'моменты собственного веса',
         'реальный COM неизвестен'),
        ('A04', 'Внутренняя структура звена: перегородки + рёбра',
         'lean 1+0; nominal 2+2; heavy 3+3 (t ребра x1.5)', 'шт',
         'конструктивное допущение; высота ребра 0.6 внутреннего габарита',
         'допущение', 'низкая', 'масса звеньев', 'реальная конструкция неизвестна'),
        ('A05', 'Удельная масса узла привода',
         '0.015 / 0.030 / 0.060', 'кг на 1 Н*м статического момента',
         'порядок величины по общим данным об интегральных приводах; НЕ выбор оборудования',
         'инженерная оценка', 'низкая', 'масса приводов',
         'требует проверки по выбранной схеме привода'),
        ('A06', 'Расчётный момент j1',
         'max(|A K1|, |C85 K1|) из аудита', 'Н*м',
         'arm_joint_moments.csv', 'измерено по геометрии', 'средняя',
         'масса привода j1', 'хват K3t даёт до +30%'),
        ('A07', 'Расчётный момент j3',
         'max(|A K1|, |C85 K1|) из аудита', 'Н*м',
         'arm_joint_moments.csv', 'измерено по геометрии', 'средняя',
         'масса привода j3', 'в покое мал, в вылете 85° максимален'),
        ('A08', 'j2: статический момент в тестовых позах = 0',
         'класс j1 (условно)', 'Н*м',
         'ось Z вертикальна; гравитационный момент нулевой; динамика не задана',
         'допущение', 'низкая', 'масса привода j2', 'требуется реальная поза/динамика'),
        ('A09', 'Локоть: статический момент в тестовых позах = 0',
         'класс j3 (условно)', 'Н*м',
         'ось Z вертикальна в протестированных позах; при наклоне j1 локоть нагружается',
         'допущение', 'низкая', 'масса привода локтя', 'требуется поза подъёма'),
        ('A10', 'запястье-1: статический момент в тестовых позах = 0',
         'класс w2 (условно)', 'Н*м',
         'ось Z; динамика/трение не заданы', 'допущение', 'низкая',
         'масса привода w1', 'требуется поза/динамика'),
        ('A11', 'Лучелоктевой сустав: расчётный момент',
         '20', 'Н*м',
         'аудит даёт 1 Н*м (недостаточно для размерного класса); принято условно',
         'допущение', 'низкая', 'масса привода ru', 'требуется схема привода'),
        ('A12', 'w2: расчётный момент по хвату K1',
         '20.0 / 15.4 (L/R)', 'Н*м',
         'arm_joint_moments.csv, сценарий A; при K3t — до 267.9/242.0',
         'измерено по геометрии', 'средняя', 'масса привода w2',
         'хват за кончики пальцев меняет класс узла'),
        ('A13', 'Привод расположен в точке сустава (COM = фрейм сустава)',
         'точка сустава', 'м', 'компоновка неизвестна', 'допущение', 'низкая',
         'моменты собственного веса', 'реальные моторы могут быть смещены'),
        ('A14', 'Кабели: длина трассы x1.5; удельная масса',
         '0.08 / 0.15 / 0.30', 'кг/м',
         'длина по кинематической цепи; удельная масса — оценка по практике',
         'инженерная оценка', 'низкая', 'масса вспомогательных',
         'разъёмы не включены'),
        ('A15', 'Кисть: механизмы, приводы пальцев, поверхности, датчики',
         'не оценены', 'кг', 'схема захвата в данных не задана',
         'нет данных', 'нет данных', 'полнота бюджета',
         'кисть тяжелее суммы оболочек на неизвестную величину'),
        ('A16', 'Тормоза суставов', 'не оценены', 'кг',
         'наличие в архитектуре не определено', 'нет данных', 'нет данных',
         'полнота бюджета', 'при удержании груза 200 кг вопрос критичен'),
        ('A17', 'Крепёж', 'не оценён', 'кг',
         'количество и размеры соединений неизвестны', 'нет данных', 'нет данных',
         'полнота бюджета', 'ожидается малая доля, но не подтверждено'),
        ('A18', 'Stored-массы файла не используются',
         'плотности 571–2808 кг/м3', 'кг/м3',
         'происхождение не установлено; плотности неоднородны и не соответствуют металлам',
         'проверено по дампу', 'достоверно (о непригодности)', '—',
         'массы плеча и предплечья идентичны внутри руки — placeholder'),
        ('A19', 'Распределение полезной нагрузки',
         '100 / 100', 'кг', '200 кг на две руки — допущение аудита',
         'допущение', 'средняя', 'моменты нагрузки',
         'несимметрия 60/140 рассмотрена в аудите'),
        ('A20', 'Поза расчёта моментов собственного веса',
         'покой 0 град. (поза файла)', 'град.',
         'единственная поза с проверенными фреймами; поза подъёма не задана',
         'датум аудита', 'средняя', 'все моменты SW',
         'в других позах моменты меняются (см. §8 отчёта)'),
        ('A21', 'Прочность, жёсткость, устойчивость',
         'не проверялись', '—', 'вне рамок массового бюджета',
         'ограничение', '—', '—', 'тонкостенные оболочки не подтверждены расчётом'),
        ('A22', 'Геометрия звеньев — bbox-прокси',
         'боксы', 'м', 'все звенья руки — боксы (proxy)',
         'факт модели', 'средняя', 'оболочки', 'реальная геометрия может отличаться'),
        ('A23', 'Знаки моментов', 'по правилу правой руки', '—',
         'как в аудите; для привода важна величина', 'соглашение', '—',
         'моменты', 'знак зависит от позы и стороны'),
        ('A24', 'Габаритный флаг приводов',
         'качественно', '—',
         'класс узлов по крайним позам существенно крупнее поперечника звеньев 93–100 мм',
         'качественная оценка', 'низкая', 'компоновка',
         'не измерение; требует проверки после выбора схемы'),
        ('A25', 'Запасы, динамика, трение, КПД',
         'не учитывались', '—', 'статическая модель', 'ограничение', '—',
         'моменты', 'не нормативный расчёт'),
    ]
    with open(OUT_ASM, 'w', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        w.writerow(['id', 'assumption', 'value', 'unit', 'basis',
                    'source_quality', 'confidence', 'affects', 'note'])
        for a in assumptions:
            w.writerow(a)

    # ---- проверки и данные для отчёта
    scen_ids = ('lean', 'nominal', 'heavy')
    tmax = max(s[2] for s in SCEN)
    shell_domain = all(2.0 * tmax < min(p['dims'])
                       for s in SIDES for p in meshes[s])
    sha_ok = None
    sha_full = None
    if os.path.isfile(BLEND):
        sha_full = hash_file(BLEND)
        sha_ok = (sha_full == BLEND_SHA_EXPECTED)

    checks = [ref_msg]
    checks.append('состав мешей: 18 на руку, кисть 15 мешей — ок')
    checks.append('оболочная формула применима ко всем 18 мешам во всех '
                  'сценариях (2t < min габарита): %s' % ('да' if shell_domain else 'НЕТ'))
    checks.append('сумма групп = итог (внутренняя арифметика; в таблицах — '
                  'округление до 2 знаков, сумма округлённых) — ок')
    if sha_ok is None:
        checks.append('SHA256 .blend: файл не найден — не проверен')
    else:
        checks.append('SHA256 .blend: %s — %s' % (
            sha_full, 'совпадает с эталоном' if sha_ok else 'НЕ СОВПАДАЕТ'))

    poseC = {}
    grip = {}
    with open(MOM, encoding='utf-8') as f:
        for row in csv.DictReader(f):
            if row['scenario'] == 'C' and row['grip'] == 'K1':
                poseC[(row['side'], row['pose'], row['joint'])] = \
                    float(row['tau_Nm'])
            if row['scenario'] == 'A' and row['pose'] == 'rest 0°':
                grip[(row['side'], row['grip'], row['joint'])] = \
                    float(row['tau_Nm'])

    # ---- Markdown
    L = []
    L.append('# Предварительный массовый бюджет рук (arm_mass_budget_v1)')
    L.append('')
    L.append('Продолжение `arm_geometry_load_audit.md` и `arm_mass_scenarios.md`. '
             'Бюджет собран по геометрии из `arm_dump.json` (Blender не открывался), '
             'моменты нагрузки — из `arm_joint_moments.csv`. Исходный `.blend` не '
             'изменялся, Git-коммиты не создавались.')
    L.append('')
    L.append('> ⚠️ Это **предварительный** бюджет: оболочки — bbox-прокси, приводы — '
             'параметрическая оценка (не выбор оборудования), часть компонентов '
             'неизвестна (см. §8). Итоги — условные, не «полная масса руки».')
    L.append('')
    L.append('## 0. Краткая сводка')
    L.append('')
    rows_t = []
    for sid in scen_ids:
        lv = disp_tot[('left', sid)]
        rv = disp_tot[('right', sid)]
        rows_t.append([SCEN_LABEL[sid], f2(lv), f2(rv), f2(round(lv + rv, 2))])
    L.extend(tbl(['сценарий', 'левая рука, кг', 'правая рука, кг',
                  'обе руки, кг'], rows_t))
    L.append('')
    L.append('Полнота: итоги включают оболочки звеньев, внутреннюю структуру '
             '(параметрически), оболочки кисти, узлы приводов (параметрически) и '
             'кабели. **Не включены**: механизмы и приводы пальцев, тормоза, '
             'крепёж, разъёмы/датчики/кожухи — их масса неизвестна (§8).')
    L.append('')
    L.append('Главные источники неопределённости: (1) класс приводов — зависит от '
             'рабочей позы и схемы передачи (A05–A12); (2) кисть — схема захвата не '
             'задана (A15); (3) внутренняя структура звеньев (A04).')
    L.append('')
    L.append('## 1. Состояние исходных материалов')
    L.append('')
    L.append('- `arm_mass_model_review.md` / `.csv` в `arm_load_audit/` '
             '**отсутствуют** (проверено листингом каталога); содержание обзора '
             'модели масс включено в §2 этого отчёта.')
    L.append('- Проверки расчёта:')
    for c in checks:
        L.append('  - %s' % c)
    L.append('')
    L.append('## 2. Что входит в уже рассчитанные массы оболочек (этап 2)')
    L.append('')
    L.append('Этап 2 (`arm_mass_scenarios.*`) считает **только** закрытые коробчатые '
             'оболочки 18 мешей каждой руки по габаритам bbox. Торцы входят в '
             'оболочку (все 6 стенок формулы), рёбра и суставные узлы — нет.')
    L.append('')
    comp_rows = [
        ['Оболочки звеньев (плечо, предплечье, ладонь)', 'да',
         'закрытая коробка по bbox; толщина 2/3 мм (Al 2, Al 3, сталь 3)',
         'габариты, сценарии материала', 'реальная геометрия, рёбра'],
        ['Торцевые крышки/фланцы звеньев', 'частично',
         'входят в закрытую коробку как две стенки',
         '—', 'отдельные фланцы/бобышки не заданы'],
        ['Рёбра жёсткости, перегородки', 'нет', '—', '—',
         'конфигурация внутренней структуры'],
        ['Корпуса суставов', 'нет', '—', '—', 'компоновка суставов'],
        ['Кисть: ладонь', 'да', 'закрытая коробка по bbox ладони', 'габарит',
         'внутренний каркас кисти'],
        ['Кисть: пальцы', 'да', 'оболочки 15 мешей (фаланги)', 'габариты',
         'механизмы, приводы, поверхности'],
        ['Электродвигатели', 'нет', '—', '—', 'тип, момент, компоновка'],
        ['Редукторы', 'нет', '—', '—', 'тип, передаточное число'],
        ['Подшипники', 'нет', '—', '—', 'схема опор'],
        ['Крепёж', 'нет', '—', '—', 'количество и размеры соединений'],
        ['Кабели и жгуты', 'нет', '—', '—', 'трасса, сечения'],
    ]
    L.extend(tbl(['компонент', 'включён', 'способ', 'известные данные',
                  'недостающее'], comp_rows))
    L.append('')
    L.append('Проверка формулы: единицы — метры/кг/кг·м⁻³; двойного счёта '
             'поверхностей нет (каждая стенка входит один раз); сплошной объём не '
             'приравнивается оболочке — при `2t ≥ min` формула вырождается в '
             'сплошное тело, но для всех 18 мешей во всех сценариях `2t < min`.')
    L.append('')
    L.append('Proxy-объекты: все 18 мешей руки — боксы (bbox-прокси). В расчёт '
             '**не попали**: collision-меши (исключены сознательно), dummy-линки '
             '(`shoulder_3_link`, `shoulder_2_link`, `radioulnar_link`, '
             '`wrist_pitch_link` — без мешей), фреймы суставов (empty). '
             'Вне руки: `torso_link`, `base_link`, `head_link`, колёса и др.')
    L.append('')
    L.append('Stored-массы (поле `mass`) **не использованы**: их происхождение не '
             'установлено, расчётные плотности по объёму мешей неоднородны '
             '(≈571–2808 кг/м³, у плеча и предплечья массы идентичны внутри руки) — '
             'признак placeholder-значений (A18).')
    L.append('')
    L.append('## 3. Метод массового бюджета')
    L.append('')
    L.append('Сценарии (A01, A05, A04, A14):')
    L.append('')
    srows = []
    for sid, ru2, t, rho, lab in SCEN:
        srows.append([ru2, lab, f1(t * 1000), '%s' % f2(SPEC_MASS[sid]),
                      RIB_CFG[sid]['bulk'], RIB_CFG[sid]['ribs'],
                      f2(CABLE_SPEC[sid])])
    L.extend(tbl(['сценарий', 'оболочки', 't, мм', 'приводы, кг/(Н·м)',
                  'перегородок', 'рёбер', 'кабели, кг/м'], srows))
    L.append('')
    L.append('Расчёт: масса оболочки — закрытая коробка; внутренняя структура — '
             'перегородки (площадь сечения × t) и продольные рёбра (длина × 0.6 '
             'внутреннего габарита × t ребра); приводы — `m = k · T*`, где `T*` — '
             'расчётный статический момент сустава (аудит, K1, max по позам A/C), '
             '`k` — удельная масса (A05); кабели — длина трассы по кинематической '
             'цепи × 1.5 × удельная масса; COM приводов — точка сустава (A13), '
             'оболочек — центр bbox (A03).')
    L.append('')
    L.append('Моменты: `τ = m·g·(−r_y·n_x + r_x·n_y)`, `g = 9.81 м/с²`, поза файла '
             '0° (A20); оси — мировые X/Y/Z как в аудите. Для осей Z (j2, локоть, '
             'w1) гравитационный момент в этой позе строго нулевой — это не '
             '«малое значение», а следствие вертикальности оси.')
    L.append('')
    L.append('## 4. Массовый бюджет по группам')
    L.append('')
    gr = []
    for sid in scen_ids:
        r = [SCEN_LABEL[sid]]
        for side in SIDES:
            dp = disp_parts[(side, sid)]
            r.extend(f2(x) for x in dp)
            r.append(f2(disp_tot[(side, sid)]))
        gr.append(r)
    L.extend(tbl(['сценарий', 'звенья L', 'кисть L', 'суставы L', 'вспом. L',
                  'итого L', 'звенья R', 'кисть R', 'суставы R', 'вспом. R',
                  'итого R'], gr))
    L.append('')
    L.append('Доли приводов в итоге, %:')
    for sid in scen_ids:
        vals = []
        for side in SIDES:
            sh = 100.0 * group_mass(side, sid, 'Суставы') / totals[(side, sid)]
            vals.append('%s — %s' % (SIDE_RU[side], f1(sh)))
        L.append('- %s: %s' % (SCEN_LABEL[sid], ', '.join(vals)))
    L.append('')
    L.append('Доли учтённой кисти (оболочки ладони и пальцев, без неизвестных '
             'механизмов) в итоге, %:')
    for sid in scen_ids:
        vals = []
        for side in SIDES:
            sh = 100.0 * group_mass(side, sid, 'Кисть') / totals[(side, sid)]
            vals.append('%s — %s' % (SIDE_RU[side], f1(sh)))
        L.append('- %s: %s' % (SCEN_LABEL[sid], ', '.join(vals)))
    L.append('')
    L.append('Детализация по компонентам (номинальный сценарий), кг:')
    L.append('')
    det = []
    for r in rows:
        if r['side'] != 'left' or r['mass'] is None or r['mass']['nominal'] is None:
            continue
        rr = next((x for x in rows if x['side'] == 'right' and
                   x['group'] == r['group'] and x['component'] == r['component'] and
                   x['mass'] is not None), None)
        det.append([r['group'], r['component'], f2(r['mass']['nominal']),
                    f2(rr['mass']['nominal']) if rr else '—', r['basis']])
    L.extend(tbl(['группа', 'компонент', 'L, кг', 'R, кг', 'основание'], det))
    L.append('')
    L.append('Итоги по компонентам (все сценарии) — в `arm_mass_budget.csv`; '
             'неизвестные строки — в §8.')
    L.append('')

    L.append('## 5. Моменты собственного веса')
    L.append('')
    for side in SIDES:
        L.append('### %s рука, Н·м (поза файла 0°)' % SIDE_RU[side].capitalize())
        L.append('')
        swrows = []
        for key, _suf, ax, label in JOINTS:
            swrows.append([label] + [f1(tau_sw(side, sid, key))
                                     for sid in scen_ids])
        L.extend(tbl(['сустав', 'облегчённый', 'номинальный', 'тяжёлый'], swrows))
        L.append('')
    L.append('Знак — по правилу правой руки (как в аудите). Для осей Z (j2, '
             'локоть, w1) момент строго нулевой в этой позе: сила тяжести '
             'параллельна оси (A20). Каждая масса учтена только у суставов, '
             'которые являются для неё proximal в кинематической цепи '
             '(дистальные массы — по поддеревьям суставов).')
    L.append('')
    L.append('### Итоговые моменты: собственный вес + нагрузка 100 кг (K1, покой)')
    L.append('')
    tt = []
    for key, _suf, ax, label in JOINTS:
        r = [label]
        for side in SIDES:
            for sid in scen_ids:
                r.append(f1(tau_sw(side, sid, key) + loadA[(side, label)]))
        tt.append(r)
    L.extend(tbl(['сустав', 'L облегч.', 'L номин.', 'L тяж.',
                  'R облегч.', 'R номин.', 'R тяж.'], tt))
    L.append('')
    L.append('Итоги **условные**: не включают механизмы/приводы пальцев, тормоза, '
             'крепёж, разъёмы/датчики/кожухи; для j2/локтя/w1 масса привода '
             'принята по условному классу (A08–A10). Отдельные значения — в '
             '`arm_mass_budget_moments.csv`.')
    L.append('')
    L.append('## 6. Внешняя нагрузка 200 кг')
    L.append('')
    L.append('Из аудита (сценарий A, точка K1, 100 кг на кисть — допущение A19):')
    L.append('')
    lr = []
    for key, _suf, _ax, label in JOINTS:
        lr.append([label, f1(loadA[('left', label)]), f1(loadA[('right', label)]),
                   f1(loadC[('left', label)]), f1(loadC[('right', label)])])
    L.extend(tbl(['сустав', 'A покой L', 'A покой R', 'C 85° L', 'C 85° R'], lr))
    L.append('')
    L.append('Момент нагрузки доминирует над собственным весом: даже в тяжёлом '
             'сценарии вклад рук — единицы–десятки процентов от нагрузки '
             '(для j1: см. §7). Несимметрия 60/140 кг меняет момент плеча примерно '
             'в 0.6/1.4 раза (аудит §10.4). Хват за кончики пальцев (K3t) '
             'увеличивает j1 примерно на 30%, а w2 — более чем на порядок '
             '(аудит §10.2–10.3).')
    L.append('')
    L.append('## 7. Чувствительность и неопределённость')
    L.append('')
    L.append('### 7.1. Удельная масса приводов (A05) — главный рычаг')
    L.append('')
    Tsum = {side: sum(design_T[(side, k)] for k, *_ in JOINTS) for side in SIDES}
    sr = []
    for k in (0.010, 0.020, 0.030, 0.040, 0.060, 0.080):
        sr.append([f3(k), f1(Tsum['left'] * k), f1(Tsum['right'] * k)])
    L.extend(tbl(['k, кг/(Н·м)', 'приводы L, кг', 'приводы R, кг'], sr))
    L.append('')
    L.append('Сумма расчётных моментов по суставам: L %s Н·м, R %s Н·м. '
             'Сценарные значения k = 0.015/0.03/0.06; таблица показывает '
             'диапазон 0.01–0.08. Это **не** выбор оборудования, а параметр '
             'чувствительности.' % (f1(Tsum['left']), f1(Tsum['right'])))
    L.append('')
    L.append('### 7.2. Рабочая поза (A20) — второй рычаг')
    L.append('')
    L.append('Момент j1 от груза при наклоне выпрямленной руки вниз '
             '(только нагрузка 100 кг, K1):')
    L.append('')
    ar = []
    for ang in (0, 30, 45, 60, 75, 90):
        c = math.cos(math.radians(ang))
        ar.append(['%d°' % ang, f1(abs(loadA[('left', lbl['j1'])]) * c),
                   f1(abs(loadA[('right', lbl['j1'])]) * c)])
    L.extend(tbl(['угол руки ниже горизонта', 'модуль τ_j1 L, Н·м',
                  'модуль τ_j1 R, Н·м'], ar))
    L.append('')
    L.append('Момент j3 от груза при вылете вперёд (j2, из аудита, K1): '
             '0° — %s, 30° — %s, 60° — %s, 85° — %s Н·м (левая; правая: %s / '
             '%s / %s / %s). Если рабочая поза — согнутая/опущенная рука, класс '
             'приводов и их масса падают в разы; это допустимо только с новым '
             'расчётом моментов.'
             % (f1(loadA[('left', lbl['j3'])]),
                f1(poseC[('left', '30°', lbl['j3'])]),
                f1(poseC[('left', '60°', lbl['j3'])]),
                f1(loadC[('left', lbl['j3'])]),
                f1(loadA[('right', lbl['j3'])]),
                f1(poseC[('right', '30°', lbl['j3'])]),
                f1(poseC[('right', '60°', lbl['j3'])]),
                f1(loadC[('right', lbl['j3'])])))
    L.append('')
    L.append('### 7.3. Точка захвата (K1 → K3t)')
    L.append('')
    w2 = {(s, g): abs(grip[(s, g, lbl['w2'])])
          for s in SIDES for g in ('K1', 'K2', 'K3', 'K3t')}
    L.append('w2: %s/%s Н·м (K1) → %s/%s (K2) → %s/%s (K3) → %s/%s (K3t). '
             'Класс привода запястья зависит от хвата сильнее, чем от массы руки.'
             % (f1(w2[('left', 'K1')]), f1(w2[('right', 'K1')]),
                f1(w2[('left', 'K2')]), f1(w2[('right', 'K2')]),
                f1(w2[('left', 'K3')]), f1(w2[('right', 'K3')]),
                f1(w2[('left', 'K3t')]), f1(w2[('right', 'K3t')])))
    L.append('')
    L.append('## 8. Неизвестные параметры')
    L.append('')
    need = {
        'Механизм пальцев': 'кинематическая схема, тип передачи, материалы',
        'Приводы и редукторы пальцев': 'тип, момент, размещение (ладонь/предплечье)',
        'Захватные поверхности': 'конструкция, материалы, датчики',
        'Разъёмы': 'состав и количество разъёмов/датчиков/кожухов',
        'Тормоза': 'требования удержания при отключении, тип',
        'Крепёж': 'количество и размеры болтовых соединений',
    }
    unk = []
    seen = set()
    for r in unknown_rows:
        what = '—'
        for k, v in need.items():
            if r['component'].startswith(k) or k in r['component']:
                what = v
                break
        key = (r['group'], r['component'], what)
        if key in seen:
            continue
        seen.add(key)
        unk.append([r['group'], r['component'], r['basis'], what])
    L.extend(tbl(['группа', 'компонент', 'ссылка', 'что нужно для оценки'], unk))
    L.append('')
    L.append('Пока эти компоненты не заданы, итоговые массы и моменты — '
             '**нижняя граница учтённого**, а не полная масса руки.')
    L.append('')
    L.append('## 9. Ограничения')
    L.append('')
    L.append('- Оболочки — bbox-прокси; реальная геометрия, материал и толщины '
             'не заданы (A01, A22).')
    L.append('- Прочность, жёсткость, устойчивость и соединения **не проверялись**; '
             'из массового бюджета они не следуют (A21).')
    L.append('- Приводы оценены параметрически; КПД, динамика, трение и запасы '
             'не учитывались (A05, A25).')
    L.append('- Моменты — только поза файла 0°; поза подъёма не задана (A20).')
    L.append('- Stored-массы файла не использованы (A18).')
    L.append('- Кисть, тормоза, крепёж, разъёмы/датчики/кожухи в итоги не входят '
             '(A15–A17).')
    L.append('- Габаритный флаг A24: класс приводов по крайним позам, по общим '
             'данным, существенно крупнее поперечника звеньев 93–100 мм; '
             'это качественная оценка, требующая проверки компоновки.')
    L.append('')
    L.append('## 10. Рекомендации следующего этапа')
    L.append('')
    L.append('1. Зафиксировать рабочую позу/цикл подъёма (какие моменты считать '
             'расчётными) — от этого зависит класс приводов и масса.')
    L.append('2. Определить схему привода (тип передачи, размещение, тормоза) — '
             'заменить параметрическую оценку A05 конкретными массами.')
    L.append('3. Определить схему кисти и приводов пальцев — сейчас это главный '
             'неизвестный блок.')
    L.append('4. Задать материалы, толщины и внутреннюю структуру звеньев (A01, A04).')
    L.append('5. После появления данных — пересчёт этим же скриптом и отдельный '
             'расчёт прочности/жёсткости (вне рамок бюджета).')
    L.append('')
    L.append('## 11. Файлы и воспроизводимость')
    L.append('')
    L.append('- `arm_mass_budget.py` — расчёт (читает `../arm_dump.json`, '
             '`../arm_joint_moments.csv`, `../arm_mass_scenarios.csv`).')
    L.append('- `arm_mass_budget.csv` — построчный бюджет компонентов.')
    L.append('- `arm_mass_budget_moments.csv` — моменты по суставам, сценариям, '
             'сторонам.')
    L.append('- `arm_mass_budget_assumptions.csv` — допущения и источники.')
    L.append('')
    if sha_ok is None:
        L.append('SHA256 `.blend`: файл не найден, проверка не выполнена.')
    else:
        L.append('SHA256 `.blend`: `%s` — %s.' % (
            sha_full, 'совпадает с эталоном' if sha_ok else '**НЕ СОВПАДАЕТ**'))
    L.append('Исходные отчёты и `.blend` не изменялись; Git-коммиты не создавались.')
    L.append('')

    with open(OUT_MD, 'w', encoding='utf-8') as f:
        f.write('\n'.join(L) + '\n')

    print('Записано:')
    for p in (OUT_CSV, OUT_MOM, OUT_ASM, OUT_MD):
        print('  %s' % os.path.basename(p))
    print('')
    print('Итоги (условно), кг:')
    for sid in scen_ids:
        print('  %-9s L %6.2f  R %6.2f  обе %7.2f' % (
            sid, totals[('left', sid)], totals[('right', sid)],
            totals[('left', sid)] + totals[('right', sid)]))
    print('')
    print('Проверки:')
    for c in checks:
        print('  ' + c)


if __name__ == '__main__':
    main()
