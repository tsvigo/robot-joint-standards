#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
arm_mass_scenarios.py — ВАРИАНТЫ оценки массы звеньев руки и моментов
собственного веса (продолжение arm_geometry_load_audit.py, разделы 14–15).

Материал и толщины звеньев в .blend НЕ заданы, поэтому реальную массу вывести
нельзя. Скрипт строит параметрические варианты «материал × толщина стенки»
на фактических габаритах звеньев (bbox) и показывает, как меняется момент
собственного веса в каждом суставе:

    коробчатая оболочка:  m = ρ·(a·b·c − (a−2t)(b−2t)(c−2t)),  2t < min(a,b,c)
    сплошное тело:        m = ρ·a·b·c  (граница сверху; если 2t ≥ min — то же)

где (a, b, c) — габариты звена (bbox, м), t — толщина стенки (м), ρ — плотность
(алюминий 2700, сталь 7850 кг/м³).

Читает:  arm_dump.json              (рядом со скриптом; Blender не нужен)
Сверяет: arm_joint_moments.csv      (строка scenario=SW_stored — контроль
                                    сходимости с аудитом; при отсутствии файла
                                    проверка пропускается с предупреждением)
Пишет:   arm_mass_scenarios.md, arm_mass_scenarios.csv

Запуск:  python3 arm_mass_scenarios.py
"""
import csv
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
DUMP_JSON = os.path.join(HERE, 'arm_dump.json')
MOM_CSV = os.path.join(HERE, 'arm_joint_moments.csv')
OUT_MD = os.path.join(HERE, 'arm_mass_scenarios.md')
OUT_CSV = os.path.join(HERE, 'arm_mass_scenarios.csv')

G = 9.81
SIDES = ('left', 'right')
SIDE_RU = {'left': 'левая', 'right': 'правая'}

# суставы: ключ, суффикс empty, ось (robot_joints.md), подпись как в аудите
JOINTS = [
    ('j3', 'shoulder_joint_empty_3', 'Y', 'j3 (плечо-3, ось Y)'),
    ('j2', 'shoulder_joint_empty_2', 'Z', 'j2 (плечо-2, ось Z)'),
    ('j1', 'shoulder_joint_empty_1', 'X', 'j1 (плечо-1, ось X)'),
    ('elbow', 'elbow_joint_empty', 'Z', 'локоть (ось Z)'),
    ('ru', 'proximal_radioulnar_joint_empty', 'Y', 'лучелоктевой (ось Y)'),
    ('w1', 'wrist_joint_1_empty', 'Z', 'запястье-1 (ось Z)'),
    ('w2', 'wrist_joint_2_empty', 'X', 'запястье-2 (ось X)'),
]
AXIS = {'X': (1.0, 0.0, 0.0), 'Y': (0.0, 1.0, 0.0), 'Z': (0.0, 0.0, 1.0)}

# варианты: (короткое имя, полное имя, режим, толщина м, плотность кг/м³)
VARIANTS = [
    ('Al 2', 'Оболочка: алюминий 2 мм', 'shell', 0.002, 2700.0),
    ('Al 3', 'Оболочка: алюминий 3 мм', 'shell', 0.003, 2700.0),
    ('Al 5', 'Оболочка: алюминий 5 мм', 'shell', 0.005, 2700.0),
    ('Ст 2', 'Оболочка: сталь 2 мм', 'shell', 0.002, 7850.0),
    ('Ст 3', 'Оболочка: сталь 3 мм', 'shell', 0.003, 7850.0),
    ('Ст 5', 'Оболочка: сталь 5 мм', 'shell', 0.005, 7850.0),
    ('Al спл.', 'Сплошной алюминий — верхняя граница для металла', 'solid', None, 2700.0),
    ('Ст спл.', 'Сплошная сталь — абсолютный верх', 'solid', None, 7850.0),
]


def m4_apply(M, v):
    return tuple(M[i][0] * v[0] + M[i][1] * v[1] + M[i][2] * v[2] + M[i][3] for i in range(3))


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


def f4(x):
    return '%.4f' % x


def tbl(headers, rows):
    out = ['| ' + ' | '.join(headers) + ' |', '|' + '|'.join(['---'] * len(headers)) + '|']
    for r in rows:
        out.append('| ' + ' | '.join(str(c) for c in r) + ' |')
    return out


def box_shell_mass(a, b, c, t, rho):
    """Масса коробчатой оболочки толщины t; при 2t >= min-габарита — сплошное тело."""
    if t is None or 2.0 * t >= min(a, b, c):
        return rho * a * b * c
    return rho * (a * b * c - (a - 2.0 * t) * (b - 2.0 * t) * (c - 2.0 * t))


def main():
    if not os.path.isfile(DUMP_JSON):
        sys.exit('ОШИБКА: нет %s (положите дамп аудита рядом со скриптом)' % DUMP_JSON)
    with open(DUMP_JSON, encoding='utf-8') as f:
        dump = json.load(f)
    O = dump['objects']

    def desc(nm):
        seen = [nm]
        i = 0
        while i < len(seen):
            for c in O[seen[i]]['children']:
                seen.append(c)
            i += 1
        return seen

    # --- меши руки: поддерево shoulder_joint_empty_3, без collision-оболочек ---
    meshes = {}
    for side in SIDES:
        lst = []
        for nm in desc(side + '_shoulder_joint_empty_3'):
            r = O[nm]
            if r['type'] != 'MESH' or '_collision' in nm:
                continue
            dm = tuple(r['bbox_world_max'][i] - r['bbox_world_min'][i] for i in range(3))
            bctr = tuple((r['bbox_world_max'][i] + r['bbox_world_min'][i]) / 2.0 for i in range(3))
            lst.append({'name': nm, 'dims': dm, 'cw_origin': rec_T(r), 'cw_center': bctr,
                        'stored': r['props'].get('mass')})
        lst.sort(key=lambda x: x['name'])
        meshes[side] = lst
    hand_set = {side: set(desc(side + '_palm_link')) for side in SIDES}

    # --- суставы: точка и ось (поза файла — 0°, фреймы identity) ---
    joints = {}
    for side in SIDES:
        for key, suf, ax, label in JOINTS:
            joints[(side, key)] = {'T': rec_T(O[side + '_' + suf]), 'n': AXIS[ax],
                                   'suf': suf, 'label': label}

    def tau_side(side, key, m_of, com_field):
        j = joints[(side, key)]
        ds = set(desc(side + '_' + j['suf']))
        s = 0.0
        for p in meshes[side]:
            if p['name'] not in ds:
                continue
            m = m_of(p)
            cw = p[com_field]
            s += m * G * (-(cw[1] - j['T'][1]) * j['n'][0] + (cw[0] - j['T'][0]) * j['n'][1])
        return s

    # --- контроль сходимости: stored-массы, COM=origin — как в аудите §14 ---
    def m_stored(p):
        m = p['stored']
        return float(m) if isinstance(m, (int, float)) else 0.0

    check_msgs = []
    val_max = None
    if os.path.isfile(MOM_CSV):
        ref = {}
        with open(MOM_CSV, encoding='utf-8') as f:
            for row in csv.DictReader(f):
                if row['scenario'] == 'SW_stored':
                    ref[(row['side'], row['joint'])] = float(row['tau_Nm'])
        val_max = 0.0
        for side in SIDES:
            for key, suf, ax, label in JOINTS:
                mine = tau_side(side, key, m_stored, 'cw_origin')
                refv = ref.get((side, label))
                if refv is None:
                    check_msgs.append('нет строки SW_stored для %s / %s' % (side, label))
                    continue
                val_max = max(val_max, abs(mine - refv))
        check_msgs.insert(0, 'сверка SW_stored: макс |Δτ| = %s Н·м (допуск 0.02)' % f4(val_max))
        if val_max > 0.02:
            check_msgs.insert(0, 'ПРОВАЛ СВЕРКИ: расхождение с аудитом больше допуска')
    else:
        check_msgs.append('внимание: %s не найден — сверка с аудитом пропущена' % os.path.basename(MOM_CSV))

    # --- влияние выбора COM (origin по файлу vs центр габарита) на stored-массах ---
    dcom = 0.0
    dcom_pt = None
    for side in SIDES:
        for key, suf, ax, label in JOINTS:
            d = abs(tau_side(side, key, m_stored, 'cw_origin') - tau_side(side, key, m_stored, 'cw_center'))
            if d > dcom:
                dcom = d
                dcom_pt = (side, label)
    max_oc = max(max(abs(p['cw_origin'][i] - p['cw_center'][i]) for i in range(3))
                 for side in SIDES for p in meshes[side])

    # --- массы по вариантам ---
    vmass = {}
    for side in SIDES:
        for vshort, vfull, kind, t, rho in VARIANTS:
            for p in meshes[side]:
                a, b, c = p['dims']
                if kind == 'solid':
                    m = rho * a * b * c
                else:
                    m = box_shell_mass(a, b, c, t, rho)
                vmass[(side, vshort, p['name'])] = m

    def m_variant(side, vshort):
        return lambda p: vmass[(side, vshort, p['name'])]

    def arm_mass(side, m_of):
        return sum(m_of(p) for p in meshes[side])

    def hand_mass(side, m_of):
        return sum(m_of(p) for p in meshes[side] if p['name'] in hand_set[side])

    n_stored = {side: sum(1 for p in meshes[side] if isinstance(p['stored'], (int, float)))
                for side in SIDES}

    # --- моменты нагрузки (сценарий A, K1) из CSV аудита — для долей ---
    load = {}
    if os.path.isfile(MOM_CSV):
        with open(MOM_CSV, encoding='utf-8') as f:
            for row in csv.DictReader(f):
                if (row['scenario'] == 'A' and row['pose'] == 'rest 0°' and row['grip'] == 'K1'):
                    load[(row['side'], row['joint'])] = float(row['tau_Nm'])

    # ---------------- CSV: массы по звеньям и вариантам ----------------
    cols = ['side', 'item', 'dims_mm', 'stored_kg'] + [v[0] + '_kg' for v in VARIANTS]
    rows = []
    for side in SIDES:
        for p in meshes[side]:
            dm = '%.0f×%.0f×%.0f' % (p['dims'][0] * 1000, p['dims'][1] * 1000, p['dims'][2] * 1000)
            r = {'side': side, 'item': p['name'], 'dims_mm': dm,
                 'stored_kg': f4(p['stored']) if isinstance(p['stored'], (int, float)) else ''}
            for vshort, vfull, kind, t, rho in VARIANTS:
                r[vshort + '_kg'] = f4(vmass[(side, vshort, p['name'])])
            rows.append(r)
    with open(OUT_CSV, 'w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        for r in rows:
            w.writerow(r)

    # ---------------- MD ----------------
    L = []
    L.append('# Варианты массы звеньев руки и моментов собственного веса')
    L.append('')
    L.append('Продолжение `arm_geometry_load_audit.md` (разделы 14–15). Материал и толщины в `.blend` '
             'не заданы, поэтому вместо одной «оценки» — параметрические **варианты конструкции**: '
             'оболочка постоянной толщины из алюминия/стали на фактических габаритах (bbox) звеньев '
             'и сплошные тела как границы сверху.')
    L.append('')
    L.append('> ⚠️ Это **не оценка реальной модели**. Габариты — bbox proxy-боксов, COM — по допущению '
             '(см. §2), внутренние приводы/крепёж/кабели не учтены. Достоверность — 🔴 (варианты).')
    L.append('')
    L.append('## 1. Метод')
    L.append('')
    L.append('Коробчатая оболочка толщины `t`: `m = ρ·(a·b·c − (a−2t)(b−2t)(c−2t))` при `2t < min(a,b,c)`, '
             'иначе сплошное тело. Плотности: алюминий 2700 кг/м³, сталь 7850 кг/м³. Момент собственного '
             'веса звена: `τ = m·g·(−(cw_y−T_y)·n_x + (cw_x−T_x)·n_y)`, COM звена — центр его габаритного '
             'бокса; `T`, `n` — точка/ось сустава (как в аудите). Оси Z-суставов (j2, локоть, запястье-1) '
             'при вертикальной гравитации дают τ = 0.')
    L.append('')
    L.append('## 2. Контроль сходимости с аудитом')
    L.append('')
    for m in check_msgs:
        L.append('- %s' % m)
    L.append('- Влияние выбора точки COM: пересчёт stored-масс «origin по файлу» ↔ «центр габарита» '
             'даёт расхождение до %s Н·м (%s). Принято: вариантные массы — COM в центре габарита.'
             % (f2(dcom), '%s, %s' % (SIDE_RU[dcom_pt[0]], dcom_pt[1]) if dcom_pt else '—'))
    L.append('- Максимальное смещение origin от центра габарита по всем звеньям руки: %s м '
             '(звенья с большим смещением — цепочки пальцев, точка вращения на суставе).' % f3(max_oc))
    L.append('- Отличие сумм масс от аудита §15: здесь учтены и меши большого пальца (в аудите они '
             'не попали в сумму по префиксу имени); значения в файле — UNVERIFIED в обоих случаях.')
    L.append('')
    L.append('## 3. Масса руки и кисти по вариантам, кг')
    L.append('')
    rows = []
    for vshort, vfull, kind, t, rho in VARIANTS:
        rows.append([vfull, f2(arm_mass('left', m_variant('left', vshort))),
                     f2(arm_mass('right', m_variant('right', vshort))),
                     f2(hand_mass('left', m_variant('left', vshort))),
                     f2(hand_mass('right', m_variant('right', vshort)))])
    rows.append(['— справка: массы из файла (%d/%d звеньев)' % (n_stored['left'], len(meshes['left'])),
                 f2(arm_mass('left', m_stored)), f2(arm_mass('right', m_stored)),
                 f2(hand_mass('left', m_stored)), f2(hand_mass('right', m_stored))])
    L.extend(tbl(['вариант', 'рука L', 'рука R', 'кисть L', 'кисть R'], rows))
    L.append('')
    L.append('«Кисть» — всё поддерево ладони (ладонь + 15 мешей пальцев и большого пальца), «рука» — '
             '18 мешей целиком. Для масштаба: 100-кг груз — это в десятки раз больше массы кисти '
             'при любом варианте.')
    L.append('')
    L.append('## 4. Момент собственного веса по вариантам, Н·м')
    L.append('')
    for side in SIDES:
        L.append('### %s рука' % SIDE_RU[side].capitalize())
        L.append('')
        rows = []
        for key, suf, ax, label in JOINTS:
            r = [label]
            for vshort, vfull, kind, t, rho in VARIANTS:
                r.append(f1(tau_side(side, key, m_variant(side, vshort), 'cw_center')))
            rows.append(r)
        L.extend(tbl(['сустав'] + [v[0] for v in VARIANTS], rows))
        L.append('')
    L.append('Доли от момента нагрузки (сценарий A, K1, 100 кг):')
    L.append('')
    rows = []
    for key, suf, ax, label in JOINTS:
        if ax == 'Z':
            continue
        r = [label]
        for side in SIDES:
            lv = load.get((side, label))
            if lv is None or abs(lv) < 5.0:
                r.append('—')
                continue
            lo = abs(tau_side(side, key, m_variant(side, 'Al 2'), 'cw_center'))
            hi = abs(tau_side(side, key, m_variant(side, 'Ст спл.'), 'cw_center'))
            r.append('%s…%s' % (f1(100.0 * lo / abs(lv)), f1(100.0 * hi / abs(lv))))
        rows.append(r)
    L.extend(tbl(['сустав', 'L, %', 'R, %'], rows))
    L.append('')
    L.append('Диапазон дан от лёгкого варианта (Al 2 мм) до абсолютной границы (сплошная сталь); '
             'реальные конструкции — между ними.')
    L.append('')
    L.append('## 5. Выводы и что нужно для точного числа')
    L.append('')
    L.append('- Для плечевых и локтевых суставов собственный вес рук остаётся второстепенным: даже '
             'сплошная сталь даёт единицы–десятки процентов от момента груза, а оболочечные варианты — '
             'ещё меньше. Вывод аудита «нагрузка доминирует» устойчив к массе рук.')
    L.append('- Для кистевого сустава (запястье-2) при хвате за ладонь (K1) момент нагрузки ≈ 20 Н·м — '
             'того же порядка, что и вклад тяжёлых вариантов кисти; масса кисти здесь значима.')
    L.append('- Точную массу (и, значит, точный собственный вес) можно получить только из конструкции. '
             'Нужны: материал звеньев, толщина стенок/рёбра, заполнение (приводы внутри — их масса), '
             'крепёж и кабели, реальные COM (в файле COM ≈ origin — placeholder).')
    L.append('- Пересчёт после получения данных: подставить плотность и толщину (или готовые массы) — '
             'скрипт `arm_mass_scenarios.py` считает всё за секунды; дальше те же числа входят в момент '
             'сустава вместе с нагрузкой.')
    L.append('')
    L.append('## 6. Файлы')
    L.append('')
    L.append('- `arm_mass_scenarios.py` — этот расчёт (читает `arm_dump.json`, Blender не нужен);')
    L.append('- `arm_mass_scenarios.csv` — массы всех 36 мешей рук по вариантам;')
    L.append('- `arm_geometry_load_audit.md` — базовый аудит (геометрия, моменты нагрузки, SW по файлу).')
    L.append('')
    with open(OUT_MD, 'w', encoding='utf-8') as f:
        f.write('\n'.join(L) + '\n')

    # ---------------- консоль ----------------
    print(check_msgs[0])
    if val_max is not None and val_max > 0.02:
        print('ОШИБКА: расхождение с аудитом — разобраться до использования результатов')
        sys.exit(2)
    print('Масса руки, кг:  вариант   L      R    | кисть L  кисть R')
    for vshort, vfull, kind, t, rho in VARIANTS:
        print('  %-8s %6s %6s | %7s %7s' % (vshort,
              f2(arm_mass('left', m_variant('left', vshort))),
              f2(arm_mass('right', m_variant('right', vshort))),
              f2(hand_mass('left', m_variant('left', vshort))),
              f2(hand_mass('right', m_variant('right', vshort)))))
    print('  %-8s %6s %6s | %7s %7s' % ('файл',
          f2(arm_mass('left', m_stored)), f2(arm_mass('right', m_stored)),
          f2(hand_mass('left', m_stored)), f2(hand_mass('right', m_stored))))
    for key, suf, ax, label in JOINTS:
        if ax == 'Z':
            continue
        print('τ_SW %s: ' % label + '  '.join(
            '%s L %s R %s' % (v[0],
                              f1(tau_side('left', key, m_variant('left', v[0]), 'cw_center')),
                              f1(tau_side('right', key, m_variant('right', v[0]), 'cw_center')))
            for v in (VARIANTS[0], VARIANTS[1], VARIANTS[4], VARIANTS[-1])))
    print('ГОТОВО: %s, %s' % (os.path.basename(OUT_MD), os.path.basename(OUT_CSV)))


if __name__ == '__main__':
    main()
